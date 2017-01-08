#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

#include "ublox.h"
#include "cs2100.h"
#include "measurements.h"
#include "microsd.h"
#include "lcd.h"
#include "speaker.h"
#include "gui.h"
#include "network.h"


int main(void)
{
    /* Everything we put in SRAM1+SRAM2 will need to be DMA'd,
     * and everything else gets put in DTCM, so there's really
     * no point having the cache turned on.
     */
    SCB_DisableDCache();

    halInit();
    chSysInit();

    /* Bring up the ublox, including the 1MHz TIMEPULSE output */
    ublox_init(&SD2);

    /* Set up the CS2100 to provide the 26MHz GPS-disciplined clock */
    cs2100_configure(&I2CD3);

    /* Swap the PLL to run off the 26MHz clock */
    cs2100_set_pll();

    /* Turn on the screen and speaker */
    /*lcd_init();*/
    /*speaker_init();*/

    /* Start GUI */
    /*gui_init();*/

    /* Start up the network */
    network_init();
    chThdSleep(TIME_INFINITE);

    /* Start up the µSD card, ready for logging */
    microsd_init();

    /* Wait until GPS lock before starting mains measurements */
    while(!ublox_last_utc.valid) {
        palSetLine(LINE_LED_YLW);
        chThdSleepMilliseconds(500);
        palClearLine(LINE_LED_YLW);
        chThdSleepMilliseconds(500);
    }

    /* Start taking mains measurements */
    measurements_init();

    /* Blink green LED at 1PPS, locked to top of second if possible */
    event_listener_t pps_listener;
    chEvtRegister(&measurements_pps_evt, &pps_listener, 0);
    while(true) {
        chEvtWaitOneTimeout(EVENT_MASK(0), MS2ST(2000));
        palSetLine(LINE_LED_GRN);
        chThdSleepMilliseconds(200);
        palClearLine(LINE_LED_GRN);
    }
}

/* The clock security system calls this if HSE goes away.
 * Best bet is probably to reset everything.
 */
void NMI_Handler(void) {
    NVIC_SystemReset();
}
