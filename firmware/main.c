#include "ch.h"
#include "hal.h"

#include "ublox.h"
#include "cs2100.h"
#include "measurements.h"
#include "microsd.h"

int main(void)
{
    halInit();
    chSysInit();

    /* Bring up the ublox, including the 1MHz TIMEPULSE output */
    ublox_init(&SD2);

    /* Set up the CS2100 to provide the 26MHz GPS-disciplined clock */
    cs2100_configure(&I2CD3);

    /* Swap the PLL to run off the 26MHz clock */
    cs2100_set_pll();

    /* Start up the µSD card, ready for logging */
    microsd_init();
    chThdSleepMilliseconds(2000);

    /* Wait until GPS lock before starting mains measurements */
    while(!ublox_last_utc.valid) {
        palSetLine(LINE_LED_YLW);
        chThdSleepMilliseconds(500);
        palClearLine(LINE_LED_YLW);
        chBSemWaitTimeout(&ublox_last_utc_bs, MS2ST(500));
    }

    /* Start taking mains measurements */
    measurements_init();

    while(true) {
        palSetLine(LINE_LED_GRN);
        chThdSleepMilliseconds(500);
        palClearLine(LINE_LED_GRN);
        chThdSleepMilliseconds(500);
    }
}

/* The clock security system calls this if HSE goes away.
 * Best bet is probably to reset everything.
 */
void NMI_Handler(void) {
    NVIC_SystemReset();
}
