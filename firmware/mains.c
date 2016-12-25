#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "ch.h"
#include "hal.h"

#include "mains.h"


static void mains_adcs_init(void);
static void mains_timers_init(void);
static adcsample_t mains_read_bias(void);
static void mains_error(const char* err);

/* Store the DMA'd ADC samples from the mains waveform */
#define MAINS_WAVE_BUFLEN (8192)
static adcsample_t mains_wave_buf[MAINS_WAVE_BUFLEN];

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
        .callback = NULL,
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

    /* Start the ADC conversions. Since it's clocked by TIM3 CC4, it won't
     * actually convert until the timer is started.
     */
    adcStartConversion(&ADCD1, &adc1_grp, mains_wave_buf, MAINS_WAVE_BUFLEN);
}

void mains_init()
{
    /* Start up the ADCs */
    mains_adcs_init();

    /* Kick it all off */
    mains_timers_init();

    /* uh, whatever */
    mains_read_bias();
}
