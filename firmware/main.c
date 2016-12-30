#include "ch.h"
#include "hal.h"

#include "ublox.h"
#include "cs2100.h"
#include "measurements.h"

int main(void)
{
    halInit();
    chSysInit();

    ublox_init(&SD2);
    cs2100_configure(&I2CD3);
    cs2100_set_pll();

    /*measurements_init();*/

    while(true) {
        palSetLine(LINE_LED_GRN);
        chThdSleepMilliseconds(500);
        palClearLine(LINE_LED_GRN);
        chThdSleepMilliseconds(500);
    }

    while(true) {
        chThdSleep(TIME_INFINITE);
    }
}
