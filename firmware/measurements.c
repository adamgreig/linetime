#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

#include "ublox.h"
#include "microsd.h"

#include "measurements.h"

/***************************************************************** CONSTANTS */
/* High speed timer frequency */
#define TIMER_FREQ (100000000)
#define NS_PER_TICK (1000000000 / TIMER_FREQ)
/*****************************************************************************/


/**************************************************************** PROTOTYPES */
static void measurements_adcs_init(void);
static void measurements_timers_init(void);
static void measurements_error(const char* err);
static void measurements_gpt2_cb(GPTDriver* gptd);
static void measurements_adc_cb(ADCDriver *adcp, adcsample_t* buf, size_t n);
static void measurements_handle_zc(void);
static void measurements_handle_pps(void);
static struct ublox_utc time_capture_get_utc(gptcnt_t pps_ts);
/*****************************************************************************/

/******************************************************************* STRUCTS */
/* Hold the measured information about a mains cycle */
struct mains_cycle_measurement {
    gptcnt_t zc_timestamp;
    gptcnt_t adc_timestamp;
    gptcnt_t pps_timestamp;
};

struct time_capture_item {
    gptcnt_t pps_timestamp;
    struct ublox_utc utc;
};
/*****************************************************************************/


/************************************************************ STATIC GLOBALS */
/* Store a recent mains bias reading */
static volatile adcsample_t mains_bias;
/*****************************************************************************/


/******************************************************************* BUFFERS */
/* Store the DMA'd ADC samples from the mains waveform.
 * Note you have to update the size of the buffer in struct mains_waveform
 * to 1/8 * MAINS_WAVE_BUFLEN if you change it here.
 */
#define MAINS_WAVE_BUFLEN (1024)
static adcsample_t mains_wave_buf[MAINS_WAVE_BUFLEN];

/* Store PPS timestamps and corresponding UTC times. */
#define TIME_BUF_SIZE (4)
static struct time_capture_item time_capture_buf[TIME_BUF_SIZE] = {{0}};
static size_t time_capture_buf_idx = 0;
static mutex_t time_capture_buf_mtx;
static gptcnt_t time_capture_pps_timestamp;
static binary_semaphore_t time_capture_pps_bs;

/* Store the memory pool and mailbox for the cycle queue. */
#define CYCLE_QUEUE_SIZE (128)
static memory_pool_t cycle_queue_pool;
static uint8_t cycle_queue_pool_buf[CYCLE_QUEUE_SIZE
                                    * sizeof(struct mains_cycle_measurement)]
    __attribute__((aligned(sizeof(stkalign_t))));
static mailbox_t cycle_queue_mbox;
static msg_t cycle_queue_mbox_buf[CYCLE_QUEUE_SIZE]
    __attribute__((aligned(sizeof(stkalign_t))));

/* Store the memory pool and mailbox for the waveform queue. */
#define WAVE_QUEUE_SIZE (16)
static memory_pool_t wave_queue_pool;
static uint8_t wave_queue_pool_buf[WAVE_QUEUE_SIZE
                                   * sizeof(struct mains_waveform)]
    __attribute__((aligned(sizeof(stkalign_t))));
static mailbox_t wave_queue_mbox;
static msg_t wave_queue_mbox_buf[WAVE_QUEUE_SIZE]
    __attribute__((aligned(sizeof(stkalign_t))));
/*****************************************************************************/


/************************************************************ ERROR HANDLING */
static void measurements_error(const char* err)
{
    /* TODO error handling */
    chSysHalt(err);
}
/*****************************************************************************/


/******************************************************************* THREADS */
/* Time capture thread.
 * Associates PPS timestamps with UTC times, and provides a blocking interface
 * to find out what UTC time a given PPS timestamp was.
 */
static THD_WORKING_AREA(time_capture_thd_wa, 512);
static THD_FUNCTION(time_capture_thd, arg)
{
    (void)arg;

    msg_t pps_rv, utc_rv;
    gptcnt_t pps_ts;
    struct ublox_utc pps_utc;

    while(true) {

        /* Wait for a new PPS */
        pps_rv = chBSemWaitTimeout(&time_capture_pps_bs, MS2ST(1001));
        if(pps_rv != MSG_OK) {
            /* Probably the PPS has stopped. We'll just try again without
             * touching the time capture buffer.
             */
            continue;
        }

        /* Hold buffer lock while we wait for UTC */
        chMtxLock(&time_capture_buf_mtx);

        pps_ts = time_capture_pps_timestamp;

        /* Now wait for corresponding UTC */
        utc_rv = chBSemWaitTimeout(&ublox_last_utc_bs, MS2ST(500));
        if(utc_rv != MSG_OK) {
            /* Probably we lost lock or UTC is not fully resolved despite PPS.
             * Release the lock but don't update the buffer.
             */
            chMtxUnlock(&time_capture_buf_mtx);
            continue;
        }
        pps_utc = ublox_last_utc;

        time_capture_buf[time_capture_buf_idx].pps_timestamp = pps_ts;
        time_capture_buf[time_capture_buf_idx].utc = pps_utc;
        time_capture_buf_idx = (time_capture_buf_idx + 1) % TIME_BUF_SIZE;

        /* Unlock buffer now we've had the next UTC */
        chMtxUnlock(&time_capture_buf_mtx);
    }
}

static struct ublox_utc time_capture_get_utc(gptcnt_t pps_ts) {
    size_t i;
    struct ublox_utc utc = { .valid = false };

    /* Wait 1ms to ensure any very-recent PPS has already caused the
     * capture thread to take the buffer lock.
     */
    chThdSleepMilliseconds(1);

    chMtxLock(&time_capture_buf_mtx);
    for(i=0; i<TIME_BUF_SIZE; i++) {
        if(time_capture_buf[i].pps_timestamp == pps_ts) {
            utc = time_capture_buf[i].utc;
            break;
        }
    }
    chMtxUnlock(&time_capture_buf_mtx);

    return utc;
}

/* Cycle queue processing thread.
 * Each incoming cycle consists of 3 timestamps:
 * the captured time of the zero crossing, the associated time of the
 * most recent PPS, and the current ADC sample number.
 * We need to resolve this cycle's UTC time, period, and RMS voltage,
 * then submit it for upload/storage.
 */
static THD_WORKING_AREA(cycle_queue_thd_wa, 256);
static THD_FUNCTION(cycle_queue_thd, arg)
{
    (void)arg;

    msg_t mbox_r;
    struct mains_cycle_measurement *measurement = NULL;

    /* Store information about the previous measurement processed */
    bool prev_measurement_valid = false;
    struct mains_cycle_measurement prev_measurement = {0};

    /* Store a completed cycle for submission to further storage/upload. */
    struct mains_cycle cycle;

    /* Store the UTC time associated with a cycle's PPS timestamp */
    struct ublox_utc cycle_utc;

    while(true) {
        /* Wait for a message about a new cycle measurement */
        mbox_r = chMBFetch(&cycle_queue_mbox,
                           (msg_t*)&measurement, TIME_INFINITE);
        if(mbox_r != MSG_OK || measurement == NULL) {
            chSysHalt("C1");
            continue;
        }

        /* If this reading is within 5ms of the previous good reading, it's
         * not a real mains cycle, so immediately discard it to prevent
         * spamming.
         */
        if(prev_measurement_valid &&
           (measurement->zc_timestamp - prev_measurement.zc_timestamp)
           < (NS_PER_TICK * 5000))
        {
            chSysHalt("C2");
            chPoolFree(&cycle_queue_pool, measurement);
            continue;
        }

        /* If we don't have valid UTC time, discard this reading.
         * In theory we could try and work it out later, but it's not worth
         * it - we expect to just always have valid UTC except at startup.
         */
        if(time_capture_get_utc(measurement->pps_timestamp).valid == false) {
            prev_measurement_valid = false;
            chPoolFree(&cycle_queue_pool, measurement);
            continue;
        }

        /* If the previous measurement wasn't valid, but we do have a resolved
         * time for this measurement, we'll store this
         * as a valid previous measurement and not process further.
         */
        if(prev_measurement_valid == false) {
            prev_measurement = *measurement;
            prev_measurement_valid = true;
            chPoolFree(&cycle_queue_pool, measurement);
            continue;
        }

        /* Now we're primarily concerned with the previous measurement -
         * we can now work out its period and its RMS. The new measurement
         * is only useful for finding the period - we'll otherwise store it
         * now and process it next time.
         * The `cycle` we now fill out refers to `prev_measurement`.
         */

        /* Get the UTC time corresponding to the previous cycle.
         * It really should be valid - or we wouldn't have stored it -
         * but we'll check just in case.
         */
        cycle_utc = time_capture_get_utc(prev_measurement.pps_timestamp);
        if(cycle_utc.valid == false) {
            chSysHalt("C5");
            prev_measurement_valid = false;
            chPoolFree(&cycle_queue_pool, measurement);
            continue;
        }

        /* Copy valid UTC time into this cycle */
        cycle.utc_year = cycle_utc.year;
        cycle.utc_month = cycle_utc.month;
        cycle.utc_day = cycle_utc.day;
        cycle.utc_hour = cycle_utc.hour;
        cycle.utc_minute = cycle_utc.minute;
        cycle.utc_second = cycle_utc.second;

        /* Compute nanoseconds since PPS */
        cycle.nanoseconds = (prev_measurement.zc_timestamp
                             - prev_measurement.pps_timestamp) * NS_PER_TICK;

        /* Compute period */
        cycle.period_ns = (measurement->zc_timestamp
                           - prev_measurement.zc_timestamp) * NS_PER_TICK;

        /* Compute RMS */
        size_t i;
        double sum_squares = 0.0;
        size_t start_idx = prev_measurement.adc_timestamp;
        size_t buflen = MAINS_WAVE_BUFLEN;
        size_t waveform_len = (measurement->adc_timestamp
                               - prev_measurement.adc_timestamp) % buflen;
        uint16_t _mains_bias = mains_bias;
        for(i=start_idx; i!=(start_idx+waveform_len)%buflen; i=(i+1)%buflen) {
            double sample = mains_wave_buf[i] - _mains_bias;
            sum_squares += sample * sample;
        }
        double rms = sqrt(sum_squares / waveform_len);
        cycle.rms = (uint16_t)(rms * (1<<7));

        /* Submit this cycle to the SD card heap */
        microsd_log(TAG_MEASUREMENT_ZC, sizeof(cycle), &cycle);

        /* Store this measurement for use next time */
        prev_measurement = *measurement;
        prev_measurement_valid = true;

        /* Remove this measurement from the memory pool */
        chPoolFree(&cycle_queue_pool, measurement);
    }
}

/* Waveform queue processing thread.
 * We just need to resolve the UTC time for each waveform and then
 * submit it for upload/storage.
 */
static THD_WORKING_AREA(wave_queue_thd_wa, 256);
static THD_FUNCTION(wave_queue_thd, arg)
{
    (void)arg;

    msg_t mbox_r;
    struct mains_waveform *waveform = NULL;
    struct ublox_utc waveform_utc = {0};

    while(true) {
        /* Wait for a message about a new waveform */
        mbox_r = chMBFetch(&wave_queue_mbox, (msg_t*)&waveform, TIME_INFINITE);
        if(mbox_r != MSG_OK || waveform == NULL) {
            chSysHalt("W1");
            continue;
        }

        /* Fetch the UTC time corresponding to this timestamp.
         * Will return a UTC with valid=false if we don't know,
         * will block up to half a second if we need to wait and see.
         */
        waveform_utc = time_capture_get_utc(waveform->_pps_timestamp);

        /* If we don't currently have valid UTC time, discard this reading.
         * In theory we could try and work it out later, but it's not worth
         * it - we expect to just always have valid UTC except at startup.
         */
        if(waveform_utc.valid == false) {
            /*chSysHalt("W2");*/
            palSetLine(LINE_LED_RED);
            chPoolFree(&wave_queue_pool, waveform);
            continue;
        } else {
            palClearLine(LINE_LED_RED);
        }

        /* Copy valid UTC time into this waveform */
        waveform->utc_year = waveform_utc.year;
        waveform->utc_month = waveform_utc.month;
        waveform->utc_day = waveform_utc.day;
        waveform->utc_hour = waveform_utc.hour;
        waveform->utc_minute = waveform_utc.minute;
        waveform->utc_second = waveform_utc.second;

        /* Submit that waveform to the SD card heap */
        microsd_log(TAG_MEASUREMENT_WAVE,
                    sizeof(struct mains_waveform), waveform);

        /* Remove that waveform from the memory pool */
        chPoolFree(&wave_queue_pool, waveform);
    }
}

/* Mains bias reading thread.
 * Just reads the mains bias on a regular basis, so that the waveform
 * reading can be properly zeroed.
 */
static THD_WORKING_AREA(mains_bias_thd_wa, 128);
static THD_FUNCTION(mains_bias_thd, arg)
{
    (void)arg;

    static const ADCConversionGroup adc2_grp = {
        .circular = false,
        .num_channels = 1,
        .end_cb = NULL,
        .error_cb = NULL,
        .cr1 = 0,
        .cr2 = ADC_CR2_SWSTART,
        .smpr1 = ADC_SMPR1_SMP_AN13(ADC_SAMPLE_480),
        .smpr2 = 0,
        .sqr1 = 0,
        .sqr2 = 0,
        .sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN13),
    };

    while(true) {
        adcsample_t reading;
        msg_t rv = adcConvert(&ADCD2, &adc2_grp, &reading, 1);

        if(rv != MSG_OK) {
            measurements_error("error reading mains bias");
        }

        mains_bias = reading;

        chThdSleepMilliseconds(100);
    }
}
/*****************************************************************************/


/******************************************************** CALLBACK FUNCTIONS */
static void measurements_adc_cb(ADCDriver *adcp, adcsample_t* buf, size_t n)
{
    (void)adcp;
    struct mains_waveform *waveform;

    /* Capture current ticks since last PPS, as early as possible */
    gptcnt_t pps_timestamp = GPTD2.tim->CCR[1];
    uint32_t delta_ticks = (GPTD2.tim->CNT) - pps_timestamp;

    /* Check buffers are the right length */
    chDbgAssert(n == MAINS_WAVE_BUFLEN/2,
                "Buffer incorrect length");
    chDbgAssert(n/4 == sizeof(waveform->waveform)/sizeof(waveform->waveform[0]),
                "Buffer incorrect size for struct mains_waveform");

    /* Allocate memory in the queue pool for this waveform buffer */
    chSysLockFromISR();
    waveform = chPoolAllocI(&wave_queue_pool);
    chSysUnlockFromISR();
    if(waveform == NULL) {
        return;
    }

    /* Grab a copy of the mains bias so it doesn't change inside this buffer */
    int32_t _mains_bias = (int32_t)mains_bias;

    /* Subsample the waveform (we run higher speed/res for good RMS accuracy,
     * but don't want that much data on the server really).
     */
    size_t i;
    for(i=0; i<n/4; i++) {
        int8_t samp = ((int32_t)buf[i*4] - _mains_bias) >> 4;
        waveform->waveform[i] = samp;
    }

    /* Save the nanoseconds-since-PPS for the end-of-buffer sample */
    waveform->nanoseconds = delta_ticks * NS_PER_TICK;

    /* Save the current PPS timestamp for later reconstruction of UTC */
    waveform->_pps_timestamp = pps_timestamp;

    /* Let the processing thread know about this buffer */
    chSysLockFromISR();
    msg_t rv = chMBPostI(&wave_queue_mbox, (intptr_t)waveform);
    if(rv != MSG_OK) {
        chPoolFreeI(&wave_queue_pool, waveform);
    }
    chSysUnlockFromISR();
}

/* Handle TIM2 input capture events */
static void measurements_gpt2_cb(GPTDriver* gptd)
{
    (void)gptd;

    /* Amazingly, ChibiOS sets SR to 0, so there's no way to tell which
     * channel caused this interrupt to fire. So we'll do this stupid thing.
     */
    static uint32_t prev_ccr1 = 0, prev_ccr2 = 0;

    /* Handle PPS */
    if(GPTD2.tim->CCR[1] != prev_ccr2) {
        prev_ccr2 = GPTD2.tim->CCR[1];
        measurements_handle_pps();
    }

    /* Handle mains zero crossing */
    if(GPTD2.tim->CCR[0] != prev_ccr1) {
        prev_ccr1 = GPTD2.tim->CCR[0];
        measurements_handle_zc();
    }
}

static void measurements_handle_zc()
{
    /* Grab relevant timestamps */
    struct mains_cycle_measurement *cycle;
    gptcnt_t zc_ts = GPTD2.tim->CCR[0];
    gptcnt_t pps_ts = GPTD2.tim->CCR[1];
    gptcnt_t adc_ts = GPTD9.tim->CNT;

    /* Allocate memory in the queue pool for them */
    chSysLockFromISR();
    cycle = chPoolAllocI(&cycle_queue_pool);
    if(cycle == NULL) {
        chSysUnlockFromISR();
        return;
    }

    /* Save them to the queue pool */
    cycle->zc_timestamp = zc_ts;
    cycle->adc_timestamp = adc_ts;
    cycle->pps_timestamp = pps_ts;

    /* Let the queue processor know about them */
    msg_t rv = chMBPostI(&cycle_queue_mbox, (intptr_t)cycle);
    if(rv != MSG_OK) {
        chPoolFreeI(&cycle_queue_pool, cycle);
    }
    chSysUnlockFromISR();
}

static void measurements_handle_pps()
{
    /* Save the captured timer count at the PPS moment. */
    time_capture_pps_timestamp = GPTD2.tim->CCR[1];

    /* Signal new timestamp the time capture thread. */
    chSysLockFromISR();
    chBSemSignalI(&time_capture_pps_bs);
    chSysUnlockFromISR();
}
/*****************************************************************************/


/************************************************************ INIT FUNCTIONS */
static void measurements_timers_init()
{
    /* Configure GPT2 to precisely time the GPS PPS and the mains
     * zero-crossing events. It runs at full speed 100MHz.
     * We interrupt on both CC1 (mains zc) and CC2 (PPS).
     */
    static const GPTConfig gpt2_cfg = {
        .frequency = TIMER_FREQ,
        .callback = measurements_gpt2_cb,
        .cr2 = 0,
        .dier = 0,
    };
    gptStart(&GPTD2, &gpt2_cfg);
    /* Set CC1 and CC2 to input capture mode on TI1 and TI2. */
    GPTD2.tim->CCMR1 = STM32_TIM_CCMR1_CC1S(1) | STM32_TIM_CCMR1_CC2S(1);
    /* Enable CC1 and CC2, non-inverted rising-edge. */
    GPTD2.tim->CCER = STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC2E;
    /* Have to set this here rather than in gpt2_cfg as gptStart clears it.. */
    GPTD2.tim->DIER = STM32_TIM_DIER_CC1IE | STM32_TIM_DIER_CC2IE;
    gptStartContinuous(&GPTD2, 0xFFFFFFFF);

    /* Configure GPT9 to run off GPT3's TRGO,
     * counting how many ADC samples have been captured,
     * resetting at the same period as the ADC buffer size.
     */
    static const GPTConfig gpt9_cfg = {
        .frequency = STM32_TIMCLK2, /* to get a prescaler of 0 */
        .callback = NULL,
        .cr2 = 0,
        .dier = 0,
    };
    /* Trigger from TIM3 TRGO, and clock from that trigger */
    gptStart(&GPTD9, &gpt9_cfg);
    GPTD9.tim->SMCR =   STM32_TIM_SMCR_TS(1) | STM32_TIM_SMCR_SMS(7);
    gptStartContinuous(&GPTD9, MAINS_WAVE_BUFLEN);

    /* Configure GPT3 for 100kHz clock,
     * generating TRGO on update at 4kHz to clock TIM9 (counting),
     * generating CC4 also at 4kHz to clock ADC1.
     * Note ADC1 will begin sampling after this timer is started.
     */
    static const GPTConfig gpt3_cfg = {
        .frequency = 100000,
        .callback = NULL,
        .cr2 = STM32_TIM_CR2_MMS(2),
        .dier = 0,
    };
    gptStart(&GPTD3, &gpt3_cfg);
    GPTD3.tim->CCR[3] = 1;
    GPTD3.tim->CCER = STM32_TIM_CCER_CC4E;
    GPTD3.tim->CCMR2 = STM32_TIM_CCMR2_OC4M(6);
    gptStartContinuous(&GPTD3, 24);
}

static void measurements_adcs_init()
{
    /* ADC1: Sample mains waveform continuously, clocked by TIM3 CC4. */
    static const ADCConversionGroup adc1_grp = {
        .circular = true,
        .num_channels = 1,
        .end_cb = measurements_adc_cb,
        .error_cb = NULL,
        .cr1 = 0,
        .cr2 = ADC_CR2_EXTEN_RISING | ADC_CR2_EXTSEL_SRC(6),
        .smpr1 = ADC_SMPR1_SMP_AN12(ADC_SAMPLE_144),
        .smpr2 = 0,
        .sqr1 = 0,
        .sqr2 = 0,
        .sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN12),
    };

    adcStart(&ADCD1, NULL);
    adcStart(&ADCD2, NULL);

    /* Start the ADC1 conversions. Since it's clocked by TIM3 CC4, it won't
     * actually being converting until the timer is started.
     */
    adcStartConversion(&ADCD1, &adc1_grp, mains_wave_buf, MAINS_WAVE_BUFLEN);
}

void measurements_init()
{

    /* Initialise the time capture buffer */
    chMtxObjectInit(&time_capture_buf_mtx);
    chBSemObjectInit(&time_capture_pps_bs, false);

    /* Initialise the cycle memory pool and mailbox */
    chPoolObjectInit(&cycle_queue_pool,
                     sizeof(struct mains_cycle_measurement), NULL);
    chPoolLoadArray(&cycle_queue_pool, cycle_queue_pool_buf, CYCLE_QUEUE_SIZE);
    chMBObjectInit(&cycle_queue_mbox, cycle_queue_mbox_buf, CYCLE_QUEUE_SIZE);

    /* Initialise the waveform memory pool and mailbox */
    chPoolObjectInit(&wave_queue_pool, sizeof(struct mains_waveform), NULL);
    chPoolLoadArray(&wave_queue_pool, wave_queue_pool_buf, WAVE_QUEUE_SIZE);
    chMBObjectInit(&wave_queue_mbox, wave_queue_mbox_buf, WAVE_QUEUE_SIZE);

    /* Start threads to process queues and submit items onwards */
    chThdCreateStatic(time_capture_thd_wa, sizeof(time_capture_thd_wa),
                      NORMALPRIO+10, time_capture_thd, NULL);
    chThdCreateStatic(cycle_queue_thd_wa, sizeof(cycle_queue_thd_wa),
                      NORMALPRIO+9, cycle_queue_thd, NULL);
    chThdCreateStatic(wave_queue_thd_wa, sizeof(wave_queue_thd_wa),
                      NORMALPRIO+8, wave_queue_thd, NULL);

    /* Wait for a PVT packet to ensure the threads are ready to process
     * their queues before starting measurements.
     */
    chBSemWait(&ublox_last_utc_bs);

    /* Start up the ADCs */
    measurements_adcs_init();

    /* Kick it all off */
    measurements_timers_init();

    /* Start a thread to constantly update the mains bias value */
    chThdCreateStatic(mains_bias_thd_wa, sizeof(mains_bias_thd_wa),
                      NORMALPRIO, mains_bias_thd, NULL);
}
/*****************************************************************************/
