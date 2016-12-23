#include "ch.h"
#include "hal.h"

int main(void)
{
    halInit();
    chSysInit();

    while(true) {
        chThdSleep(TIME_INFINITE);
    }
}
