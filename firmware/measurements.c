#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

#include "ublox.h"
#include "measurements.h"

static void measurements_adcs_init(void);
static void measurements_timers_init(void);
static adcsample_t measurements_read_mains_bias(void);
static void measurements_error(const char* err);
static void measurements_gpt2_int(GPTDriver* gptd);
static void measurements_handle_zc(void);
static void measurements_handle_pps(void);

/* Store the DMA'd ADC samples from the mains waveform */
#define MAINS_WAVE_BUFLEN (8192)
static adcsample_t mains_wave_buf[MAINS_WAVE_BUFLEN];

/* High speed timer frequency */
#define TIMER_FREQ (192000000)

/* Hold the measured information about a mains cycle */
static struct {
    /* Record whether this measurement has been set-up yet or not. */
    bool timestamps_valid;

    /* ADC sample counter value at time of ZC */
    gptcnt_t adc_timestamp;

    /* 192MHz timer value when ZC at start of cycle detected */
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
     * zero-crossing events. It runs at full speed 192MHz.
     * We interrupt on both CC1 (mains zc) and CC2 (PPS).
     */
    static const GPTConfig gpt2_cfg = {
        .frequency = TIMER_FREQ,
        .callback = measurements_gpt2_int,
        .cr2 = 0,
        .dier = STM32_TIM_DIER_CC1IE | STM32_TIM_DIER_CC2IE,
    };
    /* Set CC1 and CC2 to input capture mode on TI1 and TI2. */
    GPTD2.tim->CCMR1 = STM32_TIM_CCMR1_CC1S(1) | STM32_TIM_CCMR1_CC2S(1);
    /* Enable CC1 and CC2, non-inverted rising-edge. */
    GPTD2.tim->CCER = STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC2E;
    gptStart(&GPTD2, &gpt2_cfg);
    gptStartContinuous(&GPTD2, 0xFFFFFFFF);

    /* Configure GPT9 to run off GPT3's TRGO,
     * counting how many ADC samples have been captured,
     * resetting at the same period as the ADC buffer size.
     */
    static const GPTConfig gpt9_cfg = {
        .frequency = 1000000 /* not actually used */,
        .callback = NULL,
        .cr2 = 0,
        .dier = 0,
    };
    /* Trigger from TIM3 TRGO, and clock from that trigger */
    GPTD9.tim->SMCR =   STM32_TIM_SMCR_TS(1) | STM32_TIM_SMCR_SMS(7);
    gptStart(&GPTD9, &gpt9_cfg);
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
    GPTD3.tim->CCR[3] = 20;
    gptStart(&GPTD3, &gpt3_cfg);
    gptStartContinuous(&GPTD3, 20);

}

static void measurements_adcs_init()
{
    /* ADC1: Sample mains waveform continuously, clocked by TIM3 CC4. */
    /* TODO: enable half-full and full callbacks,
     *       and in them get a UTC timestamp and
     *       subsample the buffer 1/10, sticking
     *       results somewhere to be uploaded.
     */
    static const ADCConversionGroup adc1_grp = {
        .circular = true,
        .num_channels = 1,
        .end_cb = NULL,
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

static void measurements_handle_zc()
{
    /* Grab timestamps corresponding to the current ZC */
    uint32_t zc_ts = GPTD2.tim->CCR[0];
    uint32_t adc_ts = GPTD9.tim->CNT;
    uint32_t pps_ts = prev_pps.timestamp;

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

    /* Compute the frequency of the previous waveform. */
    /* TODO: Consider checking for timer overflow beyond 22s period,
     * possibly by computing period just based on the UTC timestamps.
     */
    double period = zc_ts - prev_mains_cycle.zc_timestamp;
    mains_cycle_out.frequency = (double)TIMER_FREQ / period;

    /* Compute the RMS voltage of the previous waveform. */
    /* TODO: This wraps around much sooner than the timer will, in the event
     * of low-frequency cycles. Detect and handle gracefully.
     */
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

    /* TODO: submit the filled-in mains_cycle_out for queueing. */

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
static void measurements_gpt2_int(GPTDriver* gptd)
{
    (void)gptd;

    /* Handle PPS */
    if(GPTD2.tim->SR & STM32_TIM_SR_CC2IF) {
        measurements_handle_pps();
        GPTD2.tim->SR &= ~STM32_TIM_SR_CC2IF;
    }

    /* Handle mains zero crossing */
    if(GPTD2.tim->SR & STM32_TIM_SR_CC1IF) {
        measurements_handle_zc();
        GPTD2.tim->SR &= ~STM32_TIM_SR_CC1IF;
    }

}

static THD_WORKING_AREA(mains_bias_thd_wa, 128);
static THD_FUNCTION(mains_bias_thd, arg) {
    (void)arg;
    while(true) {
        mains_bias = measurements_read_mains_bias();
        chThdSleepMilliseconds(100);
    }
}

void measurements_init()
{
    /* Ensure we don't try and compute deltas on the first measurement.. */
    prev_mains_cycle.timestamps_valid = false;

    /* Ensure we don't start reporting times until we know them. */
    prev_pps.utc_valid = false;

    /* Start up the ADCs */
    measurements_adcs_init();

    /* Kick it all off */
    measurements_timers_init();

    /* Start a thread to constantly update the mains bias value */
    chThdCreateStatic(mains_bias_thd_wa, sizeof(mains_bias_thd_wa),
                      NORMALPRIO, mains_bias_thd, NULL);
}
