#include "ch.h"
#include "hal.h"

#include "ublox.h"

int main(void)
{
    halInit();
    chSysInit();

    ublox_init(&SD2);

    while(true) {
        chThdSleep(TIME_INFINITE);
    }
}
