#include <string.h>

#include "hal.h"

#define MANAGE_CACHE 0
#define DISABLE_CACHE 0
#define USE_MPU 1
#define MAC_ALLOW_ERRORS 1
#define APP_CHECK_PACKETS 1

void gpio_init(void);
void rcc_reset(void);
void rcc_start(void);
void mac_init(void);
void phy_reset(void);
void phy_init(void);
bool phy_poll_link(void);
uint32_t smi_read(uint32_t reg);
void smi_write(uint32_t reg, uint32_t val);
void send_packet(void* packet, size_t len);
void read_packet(void);
void release_packet(void);
void send_ping_reply(void);
void send_arp_reply(void);
void send_udp_reply(uint32_t val);
void check_packet_error(void);

static uint32_t rxcount;
static uint32_t pingcount;

#define htons(x) (((x&0xFF00)>>8)|((x&0x00FF)<<8))
#define ntohs(x) htons(x)

struct mac_header {
    uint8_t dst[6], src[6];
    uint16_t ethertype;
} __attribute__((packed));

struct arp_packet {
    struct mac_header mac_header;
    uint16_t htype, ptype;
    uint8_t hlen, plen;
    uint16_t oper;
    uint8_t sha[6];
    uint8_t spa[4];
    uint8_t tha[6];
    uint8_t tpa[4];
} __attribute__((packed));

struct arp_packet arp_announce = {
    .mac_header = {
        .dst = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .src = {0x56, 0x54, 0x9f, 0x08, 0x87, 0x1d},
        .ethertype = 0x0608,
    },
    .htype = 0x0100,
    .ptype = 0x0008,
    .hlen = 6,
    .plen = 4,
    .oper = 0x0100,
    .sha = {0x56, 0x54, 0x9f, 0x08, 0x87, 0x1d},
    .spa = {192, 168, 2, 202},
    .tha = {0, 0, 0, 0, 0, 0},
    .tpa = {192, 168, 2, 202},
};

struct arp_packet arp_reply = {
    .mac_header = {
        .dst = {0, 0, 0, 0, 0, 0},
        .src = {0x56, 0x54, 0x9f, 0x08, 0x87, 0x1d},
        .ethertype = 0x0608,
    },
    .htype = 0x0100,
    .ptype = 0x0008,
    .hlen = 6,
    .plen = 4,
    .oper = 0x0200,
    .sha = {0x56, 0x54, 0x9f, 0x08, 0x87, 0x1d},
    .spa = {192, 168, 2, 202},
    .tha = {0, 0, 0, 0, 0, 0},
    .tpa = {0, 0, 0, 0},
};

struct ip4_header {
    uint8_t vers_ihl;
    uint8_t dscp_ecn;
    uint16_t len;
    uint16_t id;
    uint16_t flags_frag;
    uint8_t ttl;
    uint8_t proto;
    uint16_t crc;
    uint8_t src[4];
    uint8_t dst[4];
} __attribute__((packed));

struct udp_header {
    uint16_t src_port, dst_port, len, crc;
} __attribute__((packed));

struct icmp_header {
    uint8_t type, code;
    uint16_t checksum;
    uint32_t data;
} __attribute__((packed));

struct icmp_ping {
    struct mac_header mac_header;
    struct ip4_header ip4_header;
    struct icmp_header icmp_header;
    uint8_t payload[128];
} __attribute__((packed));

struct icmp_ping ping_request = {0};
struct icmp_ping ping_response = {
    .mac_header = {
        .dst = {0x60, 0xa4, 0x4c, 0x5f, 0x45, 0x6f},
        .src = {0x56, 0x54, 0x9f, 0x08, 0x87, 0x1d},
        .ethertype = 0x0008,
    },
    .ip4_header = {
        .vers_ihl = (4<<4) | 5,
        .dscp_ecn = 0,
        .len = 0, /* filled in at runtime */
        .id = 0,
        .flags_frag = (1<<6),
        .ttl = 255,
        .proto = 1,
        .crc = 0, /* filled in by the stm32 */
        .src = {192, 168, 2, 202},
        .dst = {192, 168, 2, 2},
    },
    .icmp_header = {
        .type = 0,
        .code = 0,
        .checksum = 0, /* filled in by the stm32 */
        .data = 0, /* filled in at runtime */
    },
    .payload = {0}, /* filled in at runtime */
};

struct udp_ping {
    struct mac_header mac_header;
    struct ip4_header ip4_header;
    struct udp_header udp_header;
    uint32_t data;
} __attribute__((packed));

struct udp_ping udp_response = {
    .mac_header = {
        .dst = {0x60, 0xa4, 0x4c, 0x5f, 0x45, 0x6f},
        .src = {0x56, 0x54, 0x9f, 0x08, 0x87, 0x1d},
        .ethertype = 0x0008,
    },
    .ip4_header = {
        .vers_ihl = (4<<4) | 5,
        .dscp_ecn = 0,
        .len = htons(46),
        .id = 0,
        .flags_frag = (1<<6),
        .ttl = 255,
        .proto = 17,
        .crc = 0, /* filled in by the stm32 */
        .src = {192, 168, 2, 202},
        .dst = {192, 168, 2, 2},
    },
    .udp_header = {
        .src_port = 0x8223,
        .dst_port = 0x8223,
        .len = htons(12),
        .crc = 0 /* filled in by the stm32 */,
    },
    .data = 0,
};


/* Transmit and receive descriptors and buffers */
uint32_t tbuf[2][2048/4] __attribute__((aligned(4))) __attribute__((section(".eth")));
uint32_t rbuf[2][2048/4] __attribute__((aligned(4))) __attribute__((section(".eth")));

struct tdes {
    volatile uint32_t tdes0, tdes1, tdes2, tdes3;
};
struct tdes td[2] __attribute__((aligned(4))) __attribute__((section(".eth")));
struct tdes *tdptr;

struct rdes {
    volatile uint32_t rdes0, rdes1, rdes2, rdes3;
};
struct rdes rd[2] __attribute__((aligned(4))) __attribute__((section(".eth")));
struct rdes *rdptr;


void gpio_init()
{
    /* Enable compensation cell */
    SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD;

    /* Wait for compensation cell to become ready */
    while(!(SYSCFG->CMPCR & SYSCFG_CMPCR_READY)) {}

    /*
     * All AF:
     * GPIOA 1, 2, 7
     * GPIOB 13
     * GPIOC 1, 4, 5
     * GPIOG 11, 13, 14
     *
     * Output:
     * GPIOB 7, 15
     */
    GPIOA->MODER |= GPIO_MODER_MODER1_1 |
                    GPIO_MODER_MODER2_1 |
                    GPIO_MODER_MODER7_1;
    GPIOB->MODER |= GPIO_MODER_MODER7_0 |
                    GPIO_MODER_MODER13_1|
                    GPIO_MODER_MODER15_0;
    GPIOC->MODER |= GPIO_MODER_MODER1_1 |
                    GPIO_MODER_MODER4_1 |
                    GPIO_MODER_MODER5_1;
    GPIOG->MODER |= GPIO_MODER_MODER11_1 |
                    GPIO_MODER_MODER13_1 |
                    GPIO_MODER_MODER14_1;

    /* All push-pull, no need to change anything in OTYPER */

    /* All high speed */
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_1 |
                      GPIO_OSPEEDER_OSPEEDR2_1 |
                      GPIO_OSPEEDER_OSPEEDR7_1;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1 |
                      GPIO_OSPEEDER_OSPEEDR13_1|
                      GPIO_OSPEEDER_OSPEEDR15_1;
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_1 |
                      GPIO_OSPEEDER_OSPEEDR4_1 |
                      GPIO_OSPEEDER_OSPEEDR5_1;
    GPIOG->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11_1 |
                      GPIO_OSPEEDER_OSPEEDR13_1 |
                      GPIO_OSPEEDER_OSPEEDR14_1;

    /* All floating, no need to change anything in PUPDR */
    /* All outputting 0 to start, no change in ODR */

    /* Set AF11 */
    GPIOA->AFRL |= (11<<4)  | (11<<8)  | (11<<28);
    GPIOB->AFRH |= (11<<20);
    GPIOC->AFRL |= (11<<4)  | (11<<16) | (11<<20);
    GPIOG->AFRH |= (11<<12) | (11<<20) | (11<<24);
}

void rcc_reset()
{
    /* Reset GPIOs */
    RCC->AHB1RSTR = RCC_AHB1RSTR_GPIOARST |
                    RCC_AHB1RSTR_GPIOBRST |
                    RCC_AHB1RSTR_GPIOCRST |
                    RCC_AHB1RSTR_GPIOGRST;

    RCC->AHB1RSTR = 0;

    /* Reset MAC */
    RCC->AHB1RSTR = RCC_AHB1RSTR_ETHMACRST;

    /* Select RMII mode in SYSCFG */
    SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;

    RCC->AHB1RSTR = 0;
}

void rcc_start()
{
    /* Enable GPIO clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIOGEN;

    /* Enable MAC clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACRXEN |
                    RCC_AHB1ENR_ETHMACTXEN |
                    RCC_AHB1ENR_ETHMACEN;
}

void mac_init()
{
    /* Reset ETH DMA */
    ETH->DMABMR |= ETH_DMABMR_SR;
    while(ETH->DMABMR & ETH_DMABMR_SR) {}

    /* Set MAC address 56:54:9f:08:87:1d */
    ETH->MACA0HR = (0x1d << 8) | (0x87);
    ETH->MACA0LR = (0x08 << 24) | (0x9f << 16) | (0x54 << 8) | (0x56);

    /* Just enable RX and TX for now.
     * We'll set link speed and duplex after the link comes up.
     */
    ETH->MACCR = ETH_MACCR_RE | ETH_MACCR_TE;
#if MAC_ALLOW_ERRORS
    ETH->MACFFR = ETH_MACFFR_RA;
#endif

    /* Set up DMA descriptors in chain mode */
    td[0].tdes0 = (1<<20);
    td[1].tdes0 = (1<<20);
    td[0].tdes1 = sizeof(tbuf[0]);
    td[1].tdes1 = sizeof(tbuf[1]);
    td[0].tdes2 = (intptr_t)tbuf[0];
    td[1].tdes2 = (intptr_t)tbuf[1];
    td[0].tdes3 = (intptr_t)&td[1];
    td[1].tdes3 = (intptr_t)&td[0];
    rd[0].rdes0 = (1<<31);
    rd[1].rdes0 = (1<<31);
    rd[0].rdes1 = sizeof(rbuf[0]) | (1<<14);
    rd[1].rdes1 = sizeof(rbuf[1]) | (1<<14);
    rd[0].rdes2 = (intptr_t)rbuf[0];
    rd[1].rdes2 = (intptr_t)rbuf[1];
    rd[0].rdes3 = (intptr_t)&rd[1];
    rd[1].rdes3 = (intptr_t)&rd[0];
    rdptr = &rd[0];
    tdptr = &td[0];
    __DMB();
#if MANAGE_CACHE
    SCB_CleanDCache();
#endif
    ETH->DMATDLAR = (intptr_t)&td[0];
    ETH->DMARDLAR = (intptr_t)&rd[0];

    /* Set DMA interrupts */
    ETH->DMAIER = ETH_DMAIER_NISE | ETH_DMAIER_RIE | ETH_DMAIER_TIE;

    /* Set DMA bus mode */
    ETH->DMABMR = ETH_DMABMR_AAB | ETH_DMABMR_PBL_1Beat;

    /* Flush TX FIFO and ready to set OMR */
    ETH->DMAOMR = ETH_DMAOMR_FTF;
    while(ETH->DMAOMR & ETH_DMAOMR_FTF) {}

    /* Set operation mode and start DMA */
    ETH->DMAOMR = ETH_DMAOMR_RSF | ETH_DMAOMR_TSF |
#if MAC_ALLOW_ERRORS
                  ETH_DMAOMR_FEF |
#endif
                  ETH_DMAOMR_ST | ETH_DMAOMR_SR;
}

uint32_t smi_read(uint32_t reg)
{
    /* Use PHY address 00000, set register address,
     * set clock to HCLK/102, start read operation */
    ETH->MACMIIAR = (reg << 6) |
                    ETH_MACMIIAR_CR_Div102 |
                    ETH_MACMIIAR_MB;

    /* Wait for read to complete */
    while(ETH->MACMIIAR & ETH_MACMIIAR_MB) {}

    /* Return resulting data */
    return ETH->MACMIIDR;
}

void smi_write(uint32_t reg, uint32_t val)
{
    /* Set the value to write */
    ETH->MACMIIDR = val;

    /* Use PHY address 00000, set register address,
     * set clock to HCLK/102, start write operation */
    ETH->MACMIIAR = (reg << 6) |
                     ETH_MACMIIAR_CR_Div102 |
                     ETH_MACMIIAR_MW |
                     ETH_MACMIIAR_MB;

    /* Wait for write to complete */
    while(ETH->MACMIIAR & ETH_MACMIIAR_MB) {}
}

void phy_reset()
{
    /* Hold RESET low for a few ms then set high to bring out of reset */
    GPIOB->BSRR.H.clear = (1<<15);
    for(volatile uint32_t i=0; i<1920000; i++) asm("nop");
    GPIOB->BSRR.H.set = (1<<15);
    for(volatile uint32_t i=0; i<1920000; i++) asm("nop");

    /* Software reset */
    smi_write(0x00, (1<<15));
    while(smi_read(0x00) & (1<<15)) {}
}

void phy_init()
{
    /* Power up with auto negotiation */
    smi_write(0x00, (1<<12));
}

bool phy_poll_link()
{
    uint32_t bsr, bcr, lpa;
    bsr = smi_read(0x01);
    bcr = smi_read(0x00);
    lpa = smi_read(0x05);
    (void)bcr;
    (void)lpa;

    /* No link if no auto negotiate, what century even is this */
    if(!(bcr & (1<<12))) {
        return false;
    }

    /* No link if link is down. Obviously */
    if(!(bsr & (1<<2))) {
        return false;
    }

    /* No link if remote fault */
    if(bsr & (1<<4)) {
        return false;
    }

    /* No link if autoneg incomplete */
    if(!(bsr & (1<<5))) {
        return false;
    }

    /* No link if other side can't do 100Mbps full duplex */
    if(!(lpa & (1<<8))) {
        return false;
    }

    /* Otherwise, great. */
    ETH->MACCR |= ETH_MACCR_FES | ETH_MACCR_DM;
    return true;
}

void send_packet(void* packet, size_t len)
{
    /* Wait for a descriptor to become available */
    while(tdptr->tdes0 & (1<<31)) {
        tdptr = (struct tdes*)tdptr->tdes3;
        __DMB();
#if MANAGE_CACHE
        SCB_InvalidateDCache();
#endif
    }

    /* Copy packet into descriptor */
    memcpy((void*)tdptr->tdes2, packet, len);
    tdptr->tdes1 = len;

    /* Set ownership to DMA, enable completion interrupt, this is the first
     * and last segment of this frame, enable all checksum completion,
     * and our descriptors are chained.
     */
    tdptr->tdes0 = (1<<31) | (1<<30) | (1<<29) |
                   (1<<28) | (3<<22) | (1<<20);

    __DMB();
#if MANAGE_CACHE
    SCB_CleanDCache();
#endif

    /* If DMA has stopped, tell it to restart */
    if((ETH->DMASR & ETH_DMASR_TPS) == ETH_DMASR_TPS_Suspended) {
        ETH->DMASR = ETH_DMASR_TBUS;
        ETH->DMATPDR = 0xFFFFFFFF;
    }
}

void read_packet()
{
    /* Wait for a descriptor to become available */
    while(rdptr->rdes0 & (1<<31)) {
        rdptr = (struct rdes*)rdptr->rdes3;

    __DMB();
#if MANAGE_CACHE
        SCB_CleanInvalidateDCache();
#endif

    }
}

void release_packet()
{
    rdptr->rdes0 = (1<<31);

    __DMB();
#if MANAGE_CACHE
    SCB_CleanDCache();
#endif

    if((ETH->DMASR & ETH_DMASR_RPS) == ETH_DMASR_RPS_Suspended) {
        ETH->DMASR = ETH_DMASR_RBUS;
        ETH->DMARPDR = 0xFFFFFFFF;
    }
}

void send_ping_reply()
{
    struct icmp_ping* ping = (struct icmp_ping*)rdptr->rdes2;
    ping_response.icmp_header.data = ping->icmp_header.data;
    ping_response.ip4_header.len = ping->ip4_header.len;
    size_t len = ntohs(ping->ip4_header.len);
    memcpy(ping_response.payload, ping->payload, len - 28);
    release_packet();
    send_packet(&ping_response, len + sizeof(struct mac_header));
}

void send_arp_reply()
{
    struct arp_packet* request = (struct arp_packet*)rdptr->rdes2;
    memcpy(&arp_reply.mac_header.dst, &request->mac_header.src, 6);
    memcpy(&arp_reply.tha, &request->sha, 6);
    memcpy(&arp_reply.tpa, &request->spa, 4);
    release_packet();
    send_packet(&arp_reply, sizeof(struct arp_packet));
}

void send_udp_reply(uint32_t val)
{
    (void)val;
#if 0
    struct udp_ping* ping = (struct udp_ping*)rdptr->rdes2;
    memcpy(&udp_response.mac_header.dst, &ping->mac_header.src, 6);
    memcpy(&udp_response.ip4_header.dst, &ping->ip4_header.src, 4);
    udp_response.ip4_header.len = ping->ip4_header.len;
    udp_response.udp_header.src_port = ping->udp_header.src_port;
    udp_response.udp_header.dst_port = ping->udp_header.dst_port;
    udp_response.udp_header.len = ping->udp_header.len;
    udp_response.data = ping->data;
#else
#if 0
    udp_response.data = val;
#else
    struct udp_ping* ping = (struct udp_ping*)rdptr->rdes2;
    udp_response.data = ping->data;
    (void)val;
#endif
#endif
    release_packet();
    send_packet(&udp_response, sizeof(struct udp_ping));
}

void check_packet_error(void)
{
    /* check crc error */
    if(rdptr->rdes0 & (1<<1)) {
        asm("nop");
    } else {
        asm("nop");
    }
}

int main(void)
{
#if USE_MPU
    MPU->RNR = 7;
    MPU->RBAR = 0x2007C000;
    MPU->RASR =
        (3<<24) |       /* (AP) Access permission: RW, RW */
        (4<<19) |       /* (TEX) outer cache policy: no cache */
        (0<<16) |       /* (B) inner cache policy (1/2): no cache */
        (0<<17) |       /* (C) inner cache policy (2/2): no cache */
        (1<<18) |       /* (S) shared */
        (13<<1) |       /* (SIZE) 2^(13+1) = 2^14 = 16K bytes */
        (1<<0);         /* (ENABLE) */
    MPU->CTRL = (1<<2) | (1<<0);
    SCB_CleanInvalidateDCache();
#endif

#if DISABLE_CACHE
    SCB_DisableDCache();
#endif

    rcc_reset();
    rcc_start();

    gpio_init();

    phy_reset();
    phy_init();

    mac_init();

    while(phy_poll_link() == false) {}

    rxcount = pingcount = 0;

    send_packet(&arp_announce, sizeof(arp_announce));

    while(true) {
        read_packet();

        uint16_t ethertype = ((struct mac_header*)rdptr->rdes2)->ethertype;

        if(ethertype == 0x0608) {
            send_arp_reply();
        } else if(ethertype == 0x0008) {
            struct icmp_ping* iping = (struct icmp_ping*)rdptr->rdes2;
            struct udp_ping* uping = (struct udp_ping*)rdptr->rdes2;
            if(iping->ip4_header.proto == 1
#if APP_CHECK_PACKETS
                && iping->icmp_header.type == 8
                && iping->icmp_header.code == 0
#endif
               )
            {
                pingcount++;
                send_ping_reply();
            } else if(uping->ip4_header.proto == 17
#if APP_CHECK_PACKETS
                       && uping->udp_header.dst_port == htons(9090)
#endif
                     )
            {
                rxcount++;
                check_packet_error();
                send_udp_reply(rxcount);
            } else {
                release_packet();
            }
        } else {
            release_packet();
        }
    }
}

/* Make hard faults give a useful stack trace in gdb */
void **HARDFAULT_PSP;
register void *stack_pointer asm("sp");
void HardFault_Handler(void) {
    asm("mrs %0, psp" : "=r"(HARDFAULT_PSP) ::);
    stack_pointer = HARDFAULT_PSP;
    while(1);
}
