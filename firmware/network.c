#include <stdint.h>

#include "ch.h"
#include "hal.h"

#include "lwipthread.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"

#include "network.h"
#include "rom.h"

static THD_WORKING_AREA(network_thd_wa, 2048);
static THD_FUNCTION(network_thd, arg)
{
    (void)arg;
    chRegSetThreadName("network");

    /* Read our MAC address */
    uint8_t mac_addr[6];
    rom_get_eui48(mac_addr);

    /* Start up LwIP */
    lwipthread_opts_t lwipopts = {
        .macaddress = mac_addr,
        .address = 0x00000000,
        .netmask = 0xFFFFFFFF,
        .gateway = 0x00000000,
    };
    lwipInit(&lwipopts);

    chThdSleep(TIME_INFINITE);
}

void network_init()
{
    chThdCreateStatic(network_thd_wa, sizeof(network_thd_wa), NORMALPRIO,
                      network_thd, NULL);
}
