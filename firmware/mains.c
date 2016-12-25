#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "ch.h"
#include "hal.h"

#include "mains.h"
#include "ublox.h"

struct mains_cycle {
    /* 192MHz timer value when ZC at start of cycle detected */
    gptcnt_t zc_timestamp;

    /* 192MHz timer value at time of PPS pulse most recently preceding ZC */
    gptcnt_t pps_timestamp;

    /* ADC sample counter value at time of ZC */
    gptcnt_t adc_timestamp;

    /* Period until next ZC, in cycles of 192MHz timer */
    uint32_t period;

    /* ADC readings until next ZC (at most 2048 of them, expect around 1000) */
    adcsample_t waveform[2048];
    gptcnt_t waveform_len;

    /* Recent ADC reading of mains bias voltage */
    adcsample_t mains_bias;
};

/* Store information on the current and previous mains cycles */
static struct mains_cycle current_mains_cycle, prev_mains_cycle;

static void mains_adcs_init(void);
static void mains_timers_init(void);
static adcsample_t mains_read_bias(void);
static void mains_error(const char* err);
static void mains_gpt2_int(GPTDriver* gptd);

/* Store the DMA'd ADC samples from the mains waveform */
#define MAINS_WAVE_BUFLEN (4096)
static adcsample_t mains_wave_buf[MAINS_WAVE_BUFLEN];

/* Store the TIM2 count at the last PPS */
static gptcnt_t pps_timestamp;

/* Store a recent mains bias reading */
static adcsample_t mains_bias;

static void mains_error(const char* err)
{
    /* TODO error handling */
    chSysHalt(err);
}

/* Read and return the MAINS_BIAS voltage */
static adcsample_t mains_read_bias()
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
        mains_error("reading mains bias");
    }

    return result;
}

static void mains_timers_init()
{
    /* Configure GPT2 to precisely time the GPS PPS and the mains
     * zero-crossing events. It runs at full speed 192MHz.
     * We interrupt on both CC1 (mains zc) and CC2 (PPS).
     */
    static const GPTConfig gpt2_cfg = {
        .frequency = 192000000,
        .callback = mains_gpt2_int,
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

static void mains_adcs_init()
{
    /* ADC1: Sample mains waveform continuously, clocked by TIM3 CC4. */
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
     * actually convert until the timer is started.
     */
    adcStartConversion(&ADCD1, &adc1_grp, mains_wave_buf, MAINS_WAVE_BUFLEN);
}

/* Handle TIM2 input capture events */
static void mains_gpt2_int(GPTDriver* gptd)
{
    (void)gptd;

    /* Handle mains zero crossing */
    if(GPTD2.tim->SR & STM32_TIM_SR_CC1IF) {
        /* Record current timestamps */
        current_mains_cycle.adc_timestamp = GPTD9.tim->CNT;
        current_mains_cycle.zc_timestamp = GPTD2.tim->CCR[0];
        current_mains_cycle.pps_timestamp = pps_timestamp;

        /* Update period and ADC samples of previous cycle */
        prev_mains_cycle.period =
            current_mains_cycle.zc_timestamp - prev_mains_cycle.zc_timestamp;
        prev_mains_cycle.waveform_len =
            current_mains_cycle.adc_timestamp - prev_mains_cycle.adc_timestamp;
        prev_mains_cycle.waveform_len %= MAINS_WAVE_BUFLEN;

        /* Copy waveform of previous cycle if it will fit */
        prev_mains_cycle.mains_bias = mains_bias;
        size_t start = prev_mains_cycle.adc_timestamp;
        size_t end = start + prev_mains_cycle.waveform_len;
        size_t max_len = sizeof(prev_mains_cycle.waveform)/sizeof(adcsample_t);
        if(prev_mains_cycle.waveform_len < max_len) {
            if(end < MAINS_WAVE_BUFLEN) {
                size_t len = end - start;
                memcpy(prev_mains_cycle.waveform, &mains_wave_buf[start], len);
            } else {
                size_t len1 = MAINS_WAVE_BUFLEN - start;
                memcpy(prev_mains_cycle.waveform, &mains_wave_buf[start], len1);
                size_t len2 = end - MAINS_WAVE_BUFLEN;
                memcpy(&prev_mains_cycle.waveform[len1], mains_wave_buf, len2);
            }
        } else {
            prev_mains_cycle.waveform_len = 0;
        }

        /* Submit the now-complete previous cycle for processing */
        /* TODO: send the mains_cycle struct somewhere and:
         * * resolve its timestamp
         * * compute the RMS
         * * compute the frequency
         * * subsample the waveform
         * * add to transmission queue
         */

        /* Copy relevant bits of current_mains into prev_mains */
        prev_mains_cycle.adc_timestamp = current_mains_cycle.adc_timestamp;
        prev_mains_cycle.pps_timestamp = current_mains_cycle.pps_timestamp;
        prev_mains_cycle.zc_timestamp = current_mains_cycle.zc_timestamp;
    }

    /* Handle PPS */
    if(GPTD2.tim->SR & STM32_TIM_SR_CC2IF) {
        /* Just save the timer value, when the next TP message comes in
         * we'll use it to resolve the UTC time this corresponded to.
         */
        pps_timestamp = GPTD2.tim->CCR[1];
    }
}

static THD_WORKING_AREA(mains_bias_thd_wa, 128);
static THD_FUNCTION(mains_bias_thd, arg) {
    (void)arg;
    while(true) {
        mains_bias = mains_read_bias();
        chThdSleepMilliseconds(100);
    }
}

void mains_init()
{
    /* Start up the ADCs */
    mains_adcs_init();

    /* Kick it all off */
    mains_timers_init();

    /* Start a thread to constantly update the mains bias value */
    chThdCreateStatic(mains_bias_thd_wa, sizeof(mains_bias_thd_wa),
                      NORMALPRIO, mains_bias_thd, NULL);
}
