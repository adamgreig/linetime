#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

#include "ublox.h"
#include "measurements.h"
#include "microsd.h"

static void measurements_adcs_init(void);
static void measurements_timers_init(void);
static adcsample_t measurements_read_mains_bias(void);
static void measurements_error(const char* err);
static void measurements_gpt2_cb(GPTDriver* gptd);
static void measurements_adc_cb(ADCDriver *adcp, adcsample_t* buf, size_t n);
static void measurements_handle_zc(void);
static void measurements_handle_pps(void);
static void cycle_queue_submit(struct mains_cycle *cycle);
static void wave_queue_submit(struct mains_waveform *wave);

/* Store the DMA'd ADC samples from the mains waveform.
 * Note you have to update the size of the buffer in struct mains_waveform
 * to 0.05 * MAINS_WAVE_BUFLEN if you change it here.
 */
#define MAINS_WAVE_BUFLEN (5120)
static adcsample_t mains_wave_buf[MAINS_WAVE_BUFLEN];

/* High speed timer frequency */
#define TIMER_FREQ (96000000)

/* Hold the measured information about a mains cycle */
static struct {
    /* Record whether this measurement has been set-up yet or not. */
    bool timestamps_valid;

    /* ADC sample counter value at time of ZC */
    gptcnt_t adc_timestamp;

    /* 96MHz timer value when ZC at start of cycle detected */
    gptcnt_t zc_timestamp;

    /* UTC value corresponding to zc_timestamp, if available */
    bool zc_utc_available;
    uint16_t zc_utc_week;
    uint64_t zc_utc_tow_sub_ms;
} prev_mains_cycle;

/* Store an output struct mains_cycle, filled in by the ZC ISR */
static struct mains_cycle mains_cycle_out;

/* Store the TIM2 count at the last PPS, and the corresponding UTC time */
static volatile struct {
    gptcnt_t timestamp;
    bool utc_valid;
    uint16_t utc_week;
    uint64_t utc_tow_sub_ms;
} prev_pps;

/* Store a recent mains bias reading */
static volatile adcsample_t mains_bias;

static void measurements_error(const char* err)
{
    /* TODO error handling */
    chSysHalt(err);
}

/* Read and return the MAINS_BIAS voltage */
static adcsample_t measurements_read_mains_bias()
{
    adcsample_t result;

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

    msg_t rv = adcConvert(&ADCD2, &adc2_grp, &result, 1);

    if(rv != MSG_OK) {
        measurements_error("reading mains bias");
    }

    return result;
}

static void measurements_timers_init()
{
    /* Configure GPT2 to precisely time the GPS PPS and the mains
     * zero-crossing events. It runs at full speed 96MHz.
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
        .frequency = 192000000 /* to get a prescaler of 0 */,
        .callback = NULL,
        .cr2 = 0,
        .dier = 0,
    };
    /* Trigger from TIM3 TRGO, and clock from that trigger */
    gptStart(&GPTD9, &gpt9_cfg);
    GPTD9.tim->SMCR =   STM32_TIM_SMCR_TS(1) | STM32_TIM_SMCR_SMS(7);
    gptStartContinuous(&GPTD9, MAINS_WAVE_BUFLEN);

    /* Configure GPT3 for 1MHz clock,
     * generating TRGO on update at 50kHz to clock TIM9 (counting),
     * generating CC4 also at 50kHz to clock ADC1.
     * Note ADC1 will begin sampling after this timer is started.
     */
    static const GPTConfig gpt3_cfg = {
        .frequency = 1000000,
        .callback = NULL,
        .cr2 = STM32_TIM_CR2_MMS(2),
        .dier = 0,
    };
    gptStart(&GPTD3, &gpt3_cfg);
    GPTD3.tim->CCR[3] = 1;
    GPTD3.tim->CCER = STM32_TIM_CCER_CC4E;
    GPTD3.tim->CCMR2 = STM32_TIM_CCMR2_OC4M(6);
    gptStartContinuous(&GPTD3, 19);

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

static void measurements_adc_cb(ADCDriver *adcp, adcsample_t* buf, size_t n)
{
    (void)adcp;

    struct mains_waveform waveform;

    /* Get current timestamp and most recent PPS timestamp */
    uint64_t delta_ticks    = (GPTD2.tim->CNT) - prev_pps.timestamp;
    uint64_t delta_sub_ms   = (delta_ticks<<32) / (TIMER_FREQ / 1000);
    waveform.utc_tow_sub_ms = prev_pps.utc_tow_sub_ms + delta_sub_ms;
    waveform.utc_week       = prev_pps.utc_week;

    /* Check buffers are the right length */
    chDbgAssert(n == MAINS_WAVE_BUFLEN/2,
                "Buffer incorrect length");
    chDbgAssert(n/10 == sizeof(waveform.waveform)/sizeof(int16_t),
                "Buffer too long for struct mains_waveform");

    /* Grab a copy of the mains bias so it doesn't change inside this buffer */
    adcsample_t _mains_bias = mains_bias;

    /* Subsample the waveform (we run high speed for good RMS accuracy,
     * but don't want that much data on the server really
     */
    size_t i;
    for(i=0; i<n/10; i++) {
        waveform.waveform[i] = (int16_t)buf[i*10] - _mains_bias;
    }

    wave_queue_submit(&waveform);
}

static void measurements_handle_zc()
{
    /* Grab timestamps corresponding to the current ZC */
    uint32_t zc_ts = GPTD2.tim->CCR[0];
    uint32_t adc_ts = GPTD9.tim->CNT;
    uint32_t pps_ts = prev_pps.timestamp;

    /* Bodge filter. Quit early if we're within 5ms of the last interrupt,
     * since it won't be a real transition.
     */
    if(prev_mains_cycle.timestamps_valid &&
       zc_ts - prev_mains_cycle.zc_timestamp < (TIMER_FREQ/200))
    {
        return;
    }

    /* Quit early if we don't have a valid previous measurement,
     * or if we don't currently have UTC available.
     * The previous measurement will sort itself out next ZC,
     * and UTC valid will sort itself out in the background.
     */
    if(!prev_mains_cycle.timestamps_valid   ||
       !prev_pps.utc_valid                  ||
       pps_ts != prev_pps.timestamp)
    {
        prev_mains_cycle.adc_timestamp = adc_ts;
        prev_mains_cycle.zc_timestamp = zc_ts;
        prev_mains_cycle.timestamps_valid = true;
        prev_mains_cycle.zc_utc_available = false;
        /* TODO warning */
        return;
    }

    /* Find the UTC time corresponding to the current ZC */
    uint64_t delta_ticks   = zc_ts - prev_pps.timestamp;
    uint64_t delta_sub_ms  = (delta_ticks<<32) / (TIMER_FREQ / 1000);
    uint64_t zc_utc_tow_sub_ms = prev_pps.utc_tow_sub_ms + delta_sub_ms;

    mains_cycle_out.utc_week = prev_mains_cycle.zc_utc_week;
    mains_cycle_out.utc_tow_sub_ms = prev_mains_cycle.zc_utc_tow_sub_ms;

    /* Compute the frequency of the previous waveform. */
    /* TODO: Consider checking for timer overflow beyond two 22s periods,
     * possibly by computing period just based on the UTC timestamps.
     */
    double period = zc_ts - prev_mains_cycle.zc_timestamp;
    mains_cycle_out.frequency = (double)TIMER_FREQ / period;

    /* Compute the RMS voltage of the previous waveform. */
    /* TODO: This wraps around much sooner than the timer will, in the event
     * of low-frequency cycles. Detect and handle gracefully.
     */
/* TODO this crashes, waveform_len ends up ginormous, fix me */
#if 0
    size_t i;
    int64_t sum_squares = 0;
    size_t waveform_len = adc_ts - prev_mains_cycle.adc_timestamp;
    size_t start_idx = prev_mains_cycle.adc_timestamp;
    uint16_t _mains_bias = mains_bias;
    if(start_idx + waveform_len < MAINS_WAVE_BUFLEN) {
        for(i=0; i<waveform_len; i++) {
            int16_t sample = mains_wave_buf[start_idx + i] - _mains_bias;
            sum_squares += sample * sample;
        }
    } else {
        for(i=0; i<MAINS_WAVE_BUFLEN - start_idx; i++) {
            int16_t sample = mains_wave_buf[start_idx + i] - _mains_bias;
            sum_squares += sample * sample;
        }
        for(i=0; i<waveform_len - MAINS_WAVE_BUFLEN; i++) {
            int16_t sample = mains_wave_buf[i] - _mains_bias;
            sum_squares += sample * sample;
        }
    }
    mains_cycle_out.rms = sqrt((double)sum_squares / (double)waveform_len);
#endif
    mains_cycle_out.rms = 0;

    cycle_queue_submit(&mains_cycle_out);

    /* Save the current waveform details for use as the previous waveform
     * next time around.
     */
    prev_mains_cycle.adc_timestamp = adc_ts;
    prev_mains_cycle.zc_timestamp = zc_ts;
    prev_mains_cycle.timestamps_valid = true;

    prev_mains_cycle.zc_utc_week = prev_pps.utc_week;
    prev_mains_cycle.zc_utc_tow_sub_ms = zc_utc_tow_sub_ms;
    prev_mains_cycle.zc_utc_available = true;
}

static void measurements_handle_pps()
{
    /* Save the timer counter, and if there has been a valid timepulse
     * message, save the UTC moment that corresponds to this timepulse.
     *
     * Note that in theory this should be an atomic read but it's a pain
     * to lock from inside the ISR context and the uBlox sends the
     * timepulse messages well before the actual pulse,
     * so we shouldn't ever be doing this at the same time...
     */
    prev_pps.timestamp = GPTD2.tim->CCR[1];
    if(ublox_upcoming_tp_time.valid) {
        prev_pps.utc_valid = true;
        prev_pps.utc_tow_sub_ms = ublox_upcoming_tp_time.tow_sub_ms;
        prev_pps.utc_week = ublox_upcoming_tp_time.week;

        /* We "consume" the PPS TP message validity here.
         * No other pulse should use this UTC instant.
         */
        ublox_upcoming_tp_time.valid = false;
    } else {
        prev_pps.utc_valid = false;
        prev_pps.utc_tow_sub_ms = 0;
        prev_pps.utc_week = 0;
    }
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
        GPTD2.tim->SR &= ~STM32_TIM_SR_CC2IF;
    }

    /* Handle mains zero crossing */
    if(GPTD2.tim->CCR[0] != prev_ccr1) {
        prev_ccr1 = GPTD2.tim->CCR[0];
        measurements_handle_zc();
        GPTD2.tim->SR &= ~STM32_TIM_SR_CC1IF;
    }

}

static THD_WORKING_AREA(mains_bias_thd_wa, 128);
static THD_FUNCTION(mains_bias_thd, arg)
{
    (void)arg;
    while(true) {
        mains_bias = measurements_read_mains_bias();
        chThdSleepMilliseconds(100);
    }
}

#define CYCLE_QUEUE_SIZE (64)
static memory_pool_t cycle_queue_pool;
static uint8_t cycle_queue_pool_buf[CYCLE_QUEUE_SIZE*sizeof(struct mains_cycle)]
    __attribute__((aligned(sizeof(stkalign_t))))
    __attribute__((section(".MAIN_STACK_RAM")));
static mailbox_t cycle_queue_mbox;
static msg_t cycle_queue_mbox_buf[CYCLE_QUEUE_SIZE]
    __attribute__((aligned(sizeof(stkalign_t))))
    __attribute__((section(".MAIN_STACK_RAM")));
static THD_WORKING_AREA(cycle_queue_thd_wa, 256);
static THD_FUNCTION(cycle_queue_thd, arg)
{
    (void)arg;
    chPoolObjectInit(&cycle_queue_pool, sizeof(struct mains_cycle), NULL);
    chPoolLoadArray(&cycle_queue_pool, cycle_queue_pool_buf, CYCLE_QUEUE_SIZE);
    chMBObjectInit(&cycle_queue_mbox, cycle_queue_mbox_buf, CYCLE_QUEUE_SIZE);

    msg_t mbox_r;
    void* mbox_p;

    while(true) {
        mbox_r = chMBFetch(&cycle_queue_mbox, (msg_t*)&mbox_p, TIME_INFINITE);
        if(mbox_r != MSG_OK || mbox_p == 0) {
            continue;
        }

        microsd_log(TAG_MEASUREMENT_ZC, sizeof(struct mains_cycle), mbox_p);

        chPoolFree(&cycle_queue_pool, mbox_p);
    }
}

static void cycle_queue_submit(struct mains_cycle *cycle)
{
    chSysLockFromISR();
    void* msg = chPoolAllocI(&cycle_queue_pool);
    if(msg == NULL) {
        return;
    }
    chSysUnlockFromISR();

    memcpy(msg, cycle, sizeof(struct mains_cycle));

    chSysLockFromISR();
    msg_t rv = chMBPostI(&cycle_queue_mbox, (intptr_t)msg);
    if(rv != MSG_OK) {
        chPoolFreeI(&cycle_queue_pool, msg);
    }
    chSysUnlockFromISR();
}

#define WAVE_QUEUE_SIZE (64)
static memory_pool_t wave_queue_pool;
static uint8_t wave_queue_pool_buf[WAVE_QUEUE_SIZE*sizeof(struct mains_waveform)]
    __attribute__((aligned(sizeof(stkalign_t))))
    __attribute__((section(".MAIN_STACK_RAM")));
static mailbox_t wave_queue_mbox;
static msg_t wave_queue_mbox_buf[WAVE_QUEUE_SIZE]
    __attribute__((aligned(sizeof(stkalign_t))))
    __attribute__((section(".MAIN_STACK_RAM")));
static THD_WORKING_AREA(wave_queue_thd_wa, 256);
static THD_FUNCTION(wave_queue_thd, arg)
{
    (void)arg;
    chPoolObjectInit(&wave_queue_pool, sizeof(struct mains_waveform), NULL);
    chPoolLoadArray(&wave_queue_pool, wave_queue_pool_buf, WAVE_QUEUE_SIZE);
    chMBObjectInit(&wave_queue_mbox, wave_queue_mbox_buf, WAVE_QUEUE_SIZE);

    msg_t mbox_r;
    void* mbox_p;

    while(true) {
        mbox_r = chMBFetch(&wave_queue_mbox, (msg_t*)&mbox_p, TIME_INFINITE);
        if(mbox_r != MSG_OK || mbox_p == 0) {
            continue;
        }

        microsd_log(TAG_MEASUREMENT_WAVE, sizeof(struct mains_waveform), mbox_p);

        chPoolFree(&wave_queue_pool, mbox_p);
    }
}

static void wave_queue_submit(struct mains_waveform *wave)
{
    chSysLockFromISR();
    void* msg = chPoolAllocI(&wave_queue_pool);
    if(msg == NULL) {
        return;
    }
    chSysUnlockFromISR();

    memcpy(msg, wave, sizeof(struct mains_waveform));

    chSysLockFromISR();
    msg_t rv = chMBPostI(&wave_queue_mbox, (intptr_t)msg);
    if(rv != MSG_OK) {
        chPoolFreeI(&wave_queue_pool, msg);
    }
    chSysUnlockFromISR();
}

void measurements_init()
{
    /* Ensure we don't try and compute deltas on the first measurement.. */
    prev_mains_cycle.timestamps_valid = false;

    /* Ensure we don't start reporting times until we know them. */
    prev_pps.utc_valid = false;

    /* Start threads to submit items onwards */
    chThdCreateStatic(cycle_queue_thd_wa, sizeof(cycle_queue_thd_wa),
                      NORMALPRIO, cycle_queue_thd, NULL);
    chThdCreateStatic(wave_queue_thd_wa, sizeof(wave_queue_thd_wa),
                      NORMALPRIO, wave_queue_thd, NULL);

    /* Start up the ADCs */
    measurements_adcs_init();

    /* Kick it all off */
    measurements_timers_init();

    /* Start a thread to constantly update the mains bias value */
    chThdCreateStatic(mains_bias_thd_wa, sizeof(mains_bias_thd_wa),
                      NORMALPRIO, mains_bias_thd, NULL);
}
