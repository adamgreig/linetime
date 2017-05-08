#include <string.h>
#include <stdint.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

#include "lcd.h"

/******************************************************************* GLOBALS */
EVENTSOURCE_DECL(lcd_frame_evt);
uint8_t (*lcd_framebuf)[320];
/*****************************************************************************/

/**************************************************************** PROTOTYPES */
static void backlight_init(void);
static void backlight_set(uint8_t brightness);

static void ili9342_init(void);
static void ili9342_write_index(uint8_t index);
static void ili9342_write_data(uint8_t data);

static void ltdc_init(void);
static void ltdc_layer1_init(void);
/*****************************************************************************/

/************************************************************ STATIC GLOBALS */
/* Colour lookup table.
 * Each entry is KRGB, where K is the colour key.
 */
#if 0
static const uint32_t clut[] = {
    0x00000000,     /* Black    */
    0x01FFFFFF,     /* White    */
    0x02FF0000,     /* Red      */
    0x0300FF00,     /* Green    */
    0x040000FF,     /* Blue     */
    0x05FFFF00,     /* Yellow   */
    0x06FF00FF,     /* Purple   */
    0x0700FFFF,     /* Cyan     */
};
static void clut_init(void) {}
#else
static uint32_t clut[256];
static void clut_init(void)
{
    int r=0, g=0, b=0;
    for(int i=0; i<256; i++) {
        if(r<250) r += 3;
        else if(g<250) g+= 3;
        else b += 3;
        clut[i] = (i<<24)|(r<<16)|(g<<8)|b;
    }
}
#endif
/*****************************************************************************/

/******************************************************************* STRUCTS */
/*****************************************************************************/

/******************************************************************* BUFFERS */
/* Our main frame buffers. They're huge.
 * We put them into SRAM1 because
 * a) they won't fit in DTCM
 * b) we can turn off cache and DMA directly
 * Framebufs 1 and 2 swap around each frame to do double buffering.
 * framebuf_front points to the memory currently being drawn to the LCD,
 * framebuf_back points to the memory area to draw to.
 */
static uint8_t framebuf_1[240][320]
    __attribute__((section(".sram1")))
    __attribute__((aligned(4)));
static uint8_t framebuf_2[240][320]
    __attribute__((section(".sram1")))
    __attribute__((aligned(4)));
uint8_t (*framebuf_front)[320];
uint8_t (*framebuf_back)[320];
/*****************************************************************************/

/******************************************************* BACKLIGHT FUNCTIONS */
/* Set up the backlight PWM for 0-100% brightness */
static void backlight_init()
{
    static const PWMConfig pwm_cfg = {
        .frequency = 100000,
        .period = 100,
        .callback = NULL,
        .channels = {
            {
                .mode = PWM_OUTPUT_ACTIVE_HIGH,
                .callback = NULL,
            },
        },
    };

    pwmStart(&PWMD4, &pwm_cfg);
}

static void backlight_set(uint8_t brightness)
{
    pwmEnableChannel(&PWMD4, 0, brightness);
}
/*****************************************************************************/

/************************************************************ LTDC FUNCTIONS */
static void ltdc_init(void)
{
    /* Reset and enable the LTDC */
    rccResetLTDC();
    rccEnableLTDC(false);

    /* Set up synchronisation parameters */
    const uint16_t hsync = 10;
    const uint16_t vsync = 2;
    const uint16_t hbp = 20;
    const uint16_t vbp = 2;
    const uint16_t haw = 320;
    const uint16_t vah = 240;
    const uint16_t hfp = 10;
    const uint16_t vfp = 4;

    LTDC->SSCR = ((hsync-1)<<16)             | (vsync-1);
    LTDC->BPCR = ((hsync+hbp-1)<<16)         | (vsync+vbp-1);
    LTDC->AWCR = ((hsync+hbp+haw-1)<<16)     | (vsync+vbp+vah-1);
    LTDC->TWCR = ((hsync+hbp+haw+hfp-1)<<16) | (vsync+vbp+vah+vfp-1);

    /* Interrupt on final line, just before vfp period starts. */
    LTDC->LIPCR = vsync+vbp+vah;
    LTDC->IER = LTDC_IER_LIE;
    nvicEnableVector(STM32_LTDC_EV_NUMBER, 95);

    ltdc_layer1_init();

    /* Reload shadow registers */
    LTDC->SRCR |= LTDC_SRCR_IMR;
    while(LTDC->SRCR & LTDC_SRCR_IMR);

    /* Enable LTDC */
    LTDC->GCR = LTDC_GCR_LTDCEN;
}

static void ltdc_layer1_init(void)
{
    /* Horizontal Position */
    const uint16_t whstart = 0 + 10 + 20;
    const uint16_t whstop  = whstart + 320 - 1;
    LTDC_Layer1->WHPCR = (whstop<<16) | (whstart);

    /* Vertical Position */
    const uint16_t wvstart = 0 + 2 + 2;
    const uint16_t wvstop = wvstart + 240 - 1;
    LTDC_Layer1->WVPCR = (wvstop<<16) | (wvstart);

    /* Pixel Format: L8 (for use with CLUT) */
    LTDC_Layer1->PFCR = 0x5;

    /* Constant Alpha: 255 */
    LTDC_Layer1->CACR = 255;

    /* Frame Buffer */
    LTDC_Layer1->CFBAR = (intptr_t)framebuf_front;

    /* Frame Buffer pitch and line length */
    const uint16_t pitch = 320;
    const uint16_t length = 320 + 3;
    LTDC_Layer1->CFBLR = (pitch<<16) | (length);

    /* Frame Buffer line count */
    const uint16_t lines = 240;
    LTDC_Layer1->CFBLNR = lines;

    /* Write the colour lookup table */
    clut_init();
    for(size_t i=0; i<sizeof(clut)/4; i++) {
        LTDC_Layer1->CLUTWR = clut[i];
    }

    /* Enable the layer with CLUT */
    LTDC_Layer1->CR = LTDC_LxCR_CLUTEN | LTDC_LxCR_LEN;
}

/* Interrupt handler swaps framebuf_front and framebuf_back */
CH_IRQ_HANDLER(STM32_LTDC_EV_HANDLER) {
    CH_IRQ_PROLOGUE();

#if 0
    /* Swap buffers around */
    if(framebuf_front == framebuf_1) {
        framebuf_front = framebuf_2;
        framebuf_back  = framebuf_1;
    } else {
        framebuf_front = framebuf_1;
        framebuf_back  = framebuf_2;
    }

    lcd_framebuf = framebuf_back;
#endif

    /* Set LTDC to new buffer and reload shadow registers */
    LTDC_Layer1->CFBAR = (intptr_t)framebuf_front;
    LTDC->SRCR |= LTDC_SRCR_IMR;

    /* Signal our semaphore that the buffers have swapped */
    chSysLockFromISR();
    chEvtBroadcastI(&lcd_frame_evt);
    chSysUnlockFromISR();

    /* Clear interrupt flag */
    LTDC->ICR = LTDC_ICR_CLIF;

    CH_IRQ_EPILOGUE();
}
/*****************************************************************************/

/********************************************************* ILI9342 FUNCTIONS */
static void ili9342_init()
{
    static const SPIConfig spi_cfg = {
        .end_cb = NULL,
        .ssport = GPIOE,
        .sspad = 4,
        .cr1 = SPI_CR1_BR_2,
        .cr2 = 0,
    };

    spiStart(&SPID4, &spi_cfg);

    /* Hardware reset */
    palClearLine(LINE_LCD_RST);
    chThdSleepMilliseconds(20);
    palSetLine(LINE_LCD_RST);
    chThdSleepMilliseconds(120);

    spiSelect(&SPID4);

    /* Software reset */
    ili9342_write_index(0x01);
    chThdSleepMilliseconds(20);

    /* Display off */
    ili9342_write_index(0x28);

    /* Enable extended commands. */
    ili9342_write_index(0xC8);
        ili9342_write_data(0xFF);
        ili9342_write_data(0x93);
        ili9342_write_data(0x42);

    /* Memory access control */
    ili9342_write_index(0x36);
        ili9342_write_data(0x18);

    /* Pixel format */
    ili9342_write_index(0x3A);
        ili9342_write_data(0x66);

    /* RGB */
    ili9342_write_index(0xB0);
        ili9342_write_data(0x40);

    /* Display Function Control */
    ili9342_write_index(0xB6);
        ili9342_write_data(0x0A);
        ili9342_write_data(0x82);
        ili9342_write_data(0x27);
        ili9342_write_data(0x02);

    /* Interface control */
    ili9342_write_index(0xF6);
        ili9342_write_data(0x01);
        ili9342_write_data(0x00);
        ili9342_write_data(0x06);

    /* Power control */
    ili9342_write_index(0xC0);
        ili9342_write_data(0x10);
        ili9342_write_data(0x10);
    ili9342_write_index(0xc1);
        ili9342_write_data(0x36);

    /* VCOM */
    ili9342_write_index(0xc5);
        ili9342_write_data(0xc3);

    /* Gamma */
    ili9342_write_index(0xE0);
        ili9342_write_data(0x00);
        ili9342_write_data(0x05);
        ili9342_write_data(0x08);
        ili9342_write_data(0x02);
        ili9342_write_data(0x1A);
        ili9342_write_data(0x0C);
        ili9342_write_data(0x42);
        ili9342_write_data(0x7A);
        ili9342_write_data(0x54);
        ili9342_write_data(0x08);
        ili9342_write_data(0x0D);
        ili9342_write_data(0x0C);
        ili9342_write_data(0x23);
        ili9342_write_data(0x25);
        ili9342_write_data(0x0F);
    ili9342_write_index(0xE1);
        ili9342_write_data(0x00);
        ili9342_write_data(0x29);
        ili9342_write_data(0x2F);
        ili9342_write_data(0x03);
        ili9342_write_data(0x0F);
        ili9342_write_data(0x05);
        ili9342_write_data(0x42);
        ili9342_write_data(0x55);
        ili9342_write_data(0x53);
        ili9342_write_data(0x06);
        ili9342_write_data(0x0F);
        ili9342_write_data(0x0C);
        ili9342_write_data(0x38);
        ili9342_write_data(0x3A);
        ili9342_write_data(0x0F);

    /* Sleep out */
    ili9342_write_index(0x11);
    chThdSleepMilliseconds(120);

    /* Display on */
    ili9342_write_index(0x29);

    spiUnselect(&SPID4);
}

static void ili9342_write_index(uint8_t index)
{
    palClearLine(LINE_LCD_DCX);
    spiPolledExchange(&SPID4, index);
}

static void ili9342_write_data(uint8_t data)
{
    palSetLine(LINE_LCD_DCX);
    spiPolledExchange(&SPID4, data);
}
/*****************************************************************************/

/********************************************************** PUBLIC FUNCTIONS */
void lcd_init()
{
    memset(framebuf_1, 0, sizeof(framebuf_1));
    memset(framebuf_2, 0, sizeof(framebuf_2));

    framebuf_front = framebuf_1;
    framebuf_back = framebuf_2;
#if 1
    lcd_framebuf = framebuf_2;
#else
    lcd_framebuf = framebuf_1;
#endif

    backlight_init();
    backlight_set(0);

    ltdc_init();

    ili9342_init();

    chThdSleepMilliseconds(100);
    backlight_set(100);
}
/*****************************************************************************/
