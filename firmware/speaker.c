#include "ch.h"
#include "hal.h"

#include "speaker.h"

void speaker_beep(void);

#define DAC_BUFFER_SIZE 360

/*
 * DAC test buffer (sine wave).
 */
static const dacsample_t dac_buffer[DAC_BUFFER_SIZE] = {
(  2047>>2)+64,( 2082>>2)+64,( 2118>>2)+64,( 2154>>2)+64,( 2189>>2)+64,( 2225>>2)+64,( 2260>>2)+64,( 2296>>2)+64,( 2331>>2)+64,( 2367>>2)+64,( 2402>>2)+64,( 2437>>2)+64,
(  2472>>2)+64,( 2507>>2)+64,( 2542>>2)+64,( 2576>>2)+64,( 2611>>2)+64,( 2645>>2)+64,( 2679>>2)+64,( 2713>>2)+64,( 2747>>2)+64,( 2780>>2)+64,( 2813>>2)+64,( 2846>>2)+64,
(  2879>>2)+64,( 2912>>2)+64,( 2944>>2)+64,( 2976>>2)+64,( 3008>>2)+64,( 3039>>2)+64,( 3070>>2)+64,( 3101>>2)+64,( 3131>>2)+64,( 3161>>2)+64,( 3191>>2)+64,( 3221>>2)+64,
(  3250>>2)+64,( 3278>>2)+64,( 3307>>2)+64,( 3335>>2)+64,( 3362>>2)+64,( 3389>>2)+64,( 3416>>2)+64,( 3443>>2)+64,( 3468>>2)+64,( 3494>>2)+64,( 3519>>2)+64,( 3544>>2)+64,
(  3568>>2)+64,( 3591>>2)+64,( 3615>>2)+64,( 3637>>2)+64,( 3660>>2)+64,( 3681>>2)+64,( 3703>>2)+64,( 3723>>2)+64,( 3744>>2)+64,( 3763>>2)+64,( 3782>>2)+64,( 3801>>2)+64,
(  3819>>2)+64,( 3837>>2)+64,( 3854>>2)+64,( 3870>>2)+64,( 3886>>2)+64,( 3902>>2)+64,( 3917>>2)+64,( 3931>>2)+64,( 3944>>2)+64,( 3958>>2)+64,( 3970>>2)+64,( 3982>>2)+64,
(  3993>>2)+64,( 4004>>2)+64,( 4014>>2)+64,( 4024>>2)+64,( 4033>>2)+64,( 4041>>2)+64,( 4049>>2)+64,( 4056>>2)+64,( 4062>>2)+64,( 4068>>2)+64,( 4074>>2)+64,( 4078>>2)+64,
(  4082>>2)+64,( 4086>>2)+64,( 4089>>2)+64,( 4091>>2)+64,( 4092>>2)+64,( 4093>>2)+64,( 4094>>2)+64,( 4093>>2)+64,( 4092>>2)+64,( 4091>>2)+64,( 4089>>2)+64,( 4086>>2)+64,
(  4082>>2)+64,( 4078>>2)+64,( 4074>>2)+64,( 4068>>2)+64,( 4062>>2)+64,( 4056>>2)+64,( 4049>>2)+64,( 4041>>2)+64,( 4033>>2)+64,( 4024>>2)+64,( 4014>>2)+64,( 4004>>2)+64,
(  3993>>2)+64,( 3982>>2)+64,( 3970>>2)+64,( 3958>>2)+64,( 3944>>2)+64,( 3931>>2)+64,( 3917>>2)+64,( 3902>>2)+64,( 3886>>2)+64,( 3870>>2)+64,( 3854>>2)+64,( 3837>>2)+64,
(  3819>>2)+64,( 3801>>2)+64,( 3782>>2)+64,( 3763>>2)+64,( 3744>>2)+64,( 3723>>2)+64,( 3703>>2)+64,( 3681>>2)+64,( 3660>>2)+64,( 3637>>2)+64,( 3615>>2)+64,( 3591>>2)+64,
(  3568>>2)+64,( 3544>>2)+64,( 3519>>2)+64,( 3494>>2)+64,( 3468>>2)+64,( 3443>>2)+64,( 3416>>2)+64,( 3389>>2)+64,( 3362>>2)+64,( 3335>>2)+64,( 3307>>2)+64,( 3278>>2)+64,
(  3250>>2)+64,( 3221>>2)+64,( 3191>>2)+64,( 3161>>2)+64,( 3131>>2)+64,( 3101>>2)+64,( 3070>>2)+64,( 3039>>2)+64,( 3008>>2)+64,( 2976>>2)+64,( 2944>>2)+64,( 2912>>2)+64,
(  2879>>2)+64,( 2846>>2)+64,( 2813>>2)+64,( 2780>>2)+64,( 2747>>2)+64,( 2713>>2)+64,( 2679>>2)+64,( 2645>>2)+64,( 2611>>2)+64,( 2576>>2)+64,( 2542>>2)+64,( 2507>>2)+64,
(  2472>>2)+64,( 2437>>2)+64,( 2402>>2)+64,( 2367>>2)+64,( 2331>>2)+64,( 2296>>2)+64,( 2260>>2)+64,( 2225>>2)+64,( 2189>>2)+64,( 2154>>2)+64,( 2118>>2)+64,( 2082>>2)+64,
(  2047>>2)+64,( 2012>>2)+64,( 1976>>2)+64,( 1940>>2)+64,( 1905>>2)+64,( 1869>>2)+64,( 1834>>2)+64,( 1798>>2)+64,( 1763>>2)+64,( 1727>>2)+64,( 1692>>2)+64,( 1657>>2)+64,
(  1622>>2)+64,( 1587>>2)+64,( 1552>>2)+64,( 1518>>2)+64,( 1483>>2)+64,( 1449>>2)+64,( 1415>>2)+64,( 1381>>2)+64,( 1347>>2)+64,( 1314>>2)+64,( 1281>>2)+64,( 1248>>2)+64,
(  1215>>2)+64,( 1182>>2)+64,( 1150>>2)+64,( 1118>>2)+64,( 1086>>2)+64,( 1055>>2)+64,( 1024>>2)+64,(  993>>2)+64,(  963>>2)+64,(  933>>2)+64,(  903>>2)+64,(  873>>2)+64,
(   844>>2)+64,(  816>>2)+64,(  787>>2)+64,(  759>>2)+64,(  732>>2)+64,(  705>>2)+64,(  678>>2)+64,(  651>>2)+64,(  626>>2)+64,(  600>>2)+64,(  575>>2)+64,(  550>>2)+64,
(   526>>2)+64,(  503>>2)+64,(  479>>2)+64,(  457>>2)+64,(  434>>2)+64,(  413>>2)+64,(  391>>2)+64,(  371>>2)+64,(  350>>2)+64,(  331>>2)+64,(  312>>2)+64,(  293>>2)+64,
(   275>>2)+64,(  257>>2)+64,(  240>>2)+64,(  224>>2)+64,(  208>>2)+64,(  192>>2)+64,(  177>>2)+64,(  163>>2)+64,(  150>>2)+64,(  136>>2)+64,(  124>>2)+64,(  112>>2)+64,
(   101>>2)+64,(   90>>2)+64,(   80>>2)+64,(   70>>2)+64,(   61>>2)+64,(   53>>2)+64,(   45>>2)+64,(   38>>2)+64,(   32>>2)+64,(   26>>2)+64,(   20>>2)+64,(   16>>2)+64,
(    12>>2)+64,(    8>>2)+64,(    5>>2)+64,(    3>>2)+64,(    2>>2)+64,(    1>>2)+64,(    0>>2)+64,(    1>>2)+64,(    2>>2)+64,(    3>>2)+64,(    5>>2)+64,(    8>>2)+64,
(    12>>2)+64,(   16>>2)+64,(   20>>2)+64,(   26>>2)+64,(   32>>2)+64,(   38>>2)+64,(   45>>2)+64,(   53>>2)+64,(   61>>2)+64,(   70>>2)+64,(   80>>2)+64,(   90>>2)+64,
(   101>>2)+64,(  112>>2)+64,(  124>>2)+64,(  136>>2)+64,(  150>>2)+64,(  163>>2)+64,(  177>>2)+64,(  192>>2)+64,(  208>>2)+64,(  224>>2)+64,(  240>>2)+64,(  257>>2)+64,
(   275>>2)+64,(  293>>2)+64,(  312>>2)+64,(  331>>2)+64,(  350>>2)+64,(  371>>2)+64,(  391>>2)+64,(  413>>2)+64,(  434>>2)+64,(  457>>2)+64,(  479>>2)+64,(  503>>2)+64,
(   526>>2)+64,(  550>>2)+64,(  575>>2)+64,(  600>>2)+64,(  626>>2)+64,(  651>>2)+64,(  678>>2)+64,(  705>>2)+64,(  732>>2)+64,(  759>>2)+64,(  787>>2)+64,(  816>>2)+64,
(   844>>2)+64,(  873>>2)+64,(  903>>2)+64,(  933>>2)+64,(  963>>2)+64,(  993>>2)+64,( 1024>>2)+64,( 1055>>2)+64,( 1086>>2)+64,( 1118>>2)+64,( 1150>>2)+64,( 1182>>2)+64,
(  1215>>2)+64,( 1248>>2)+64,( 1281>>2)+64,( 1314>>2)+64,( 1347>>2)+64,( 1381>>2)+64,( 1415>>2)+64,( 1449>>2)+64,( 1483>>2)+64,( 1518>>2)+64,( 1552>>2)+64,( 1587>>2)+64,
(  1622>>2)+64,( 1657>>2)+64,( 1692>>2)+64,( 1727>>2)+64,( 1763>>2)+64,( 1798>>2)+64,( 1834>>2)+64,( 1869>>2)+64,( 1905>>2)+64,( 1940>>2)+64,( 1976>>2)+64,( 2012>>2)+64,
};

static void end_cb(DACDriver *dacp, const dacsample_t *buffer, size_t n) {
    (void)dacp; (void)buffer; (void)n;
    palToggleLine(LINE_LED_GRN);
}

static void tim_cb(GPTDriver *gptp) {
    (void)gptp;
    palToggleLine(LINE_LED_YLW);
}

static void error_cb(DACDriver *dacp, dacerror_t err) {
    (void)dacp;
    (void)err;
    chSysHalt("DAC failure");
}

static const DACConfig dac_cfg = {
    .init = 2047,
    .datamode = DAC_DHRM_12BIT_RIGHT,
};

static const DACConversionGroup dac_grp_cfg = {
    .num_channels = 1,
    .end_cb = end_cb,
    .error_cb = error_cb,
    .trigger = DAC_TRG(0),
};

static const GPTConfig gpt_cfg = {
    .frequency = 10000000,
    .callback = tim_cb,
    .cr2 = TIM_CR2_MMS_1,
    .dier = 0,
};

void speaker_beep()
{
    (void)dac_cfg;
    (void)dac_grp_cfg;
    (void)gpt_cfg;
    (void)tim_cb;
    (void)error_cb;
    (void)end_cb;


    palSetLine(LINE_SOUNDER_SD);

    dacStartConversion(&DACD2, &dac_grp_cfg, dac_buffer, DAC_BUFFER_SIZE);
    gptStartContinuous(&GPTD6, 20);

    chThdSleep(TIME_INFINITE);
}

void speaker_init()
{
    gptStart(&GPTD6, &gpt_cfg);
    dacStart(&DACD2, &dac_cfg);

    speaker_beep();
}
