#include <string.h>
#include <stdint.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

#include "lcd.h"

/***************************************************************** CONSTANTS */
/*****************************************************************************/

/**************************************************************** PROTOTYPES */
void backlight_init(void);
void backlight_set(uint8_t brightness);

void ili9342_init(void);
void ili9342_write_index(uint8_t index);
void ili9342_write_data(uint8_t data);

void ltdc_init(void);
void ltdc_layer1_init(void);
/*****************************************************************************/

/************************************************************ STATIC GLOBALS */
/* Colour lookup table.
 * Each entry is KRGB, where K is the colour key.
 */
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
#define BLACK   (0)
#define WHITE   (1)
#define RED     (2)
#define GREEN   (3)
#define BLUE    (4)
#define YELLOW  (5)
#define PURPLE  (6)
#define CYAN    (7)
/*****************************************************************************/

/******************************************************************* STRUCTS */
/*****************************************************************************/

/******************************************************************* BUFFERS */
/* Our main frame buffer. It's huge.
 * We put it into SRAM1 because
 * a) it won't fit in DTCM
 * b) we can turn off cache and DMA directly
 */
static uint8_t framebuf[240][320]
    __attribute__((section(".sram1")))
    __attribute__((aligned(4)));
/*****************************************************************************/

/******************************************************* BACKLIGHT FUNCTIONS */
/* Set up the backlight PWM for 0-100% brightness */
void backlight_init()
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

void backlight_set(uint8_t brightness)
{
    pwmEnableChannel(&PWMD4, 0, brightness);
}
/*****************************************************************************/

/************************************************************ LTDC FUNCTIONS */
void ltdc_init(void)
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

    ltdc_layer1_init();

    /* Reload shadow registers */
    LTDC->SRCR |= LTDC_SRCR_IMR;
    while(LTDC->SRCR & LTDC_SRCR_IMR);

    /* Enable LTDC */
    LTDC->GCR = LTDC_GCR_LTDCEN;
}

void ltdc_layer1_init(void)
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
    LTDC_Layer1->CFBAR = (intptr_t)framebuf;

    /* Frame Buffer pitch and line length */
    const uint16_t pitch = 320;
    const uint16_t length = 320 + 3;
    LTDC_Layer1->CFBLR = (pitch<<16) | (length);

    /* Frame Buffer line count */
    const uint16_t lines = 240;
    LTDC_Layer1->CFBLNR = lines;

    /* Write the colour lookup table */
    for(size_t i=0; i<sizeof(clut)/4; i++) {
        LTDC_Layer1->CLUTWR = clut[i];
    }

    /* Enable the layer with CLUT */
    LTDC_Layer1->CR = LTDC_LxCR_CLUTEN | LTDC_LxCR_LEN;
}
/*****************************************************************************/

/********************************************************* ILI9342 FUNCTIONS */
void ili9342_init()
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
        ili9342_write_data(0x01);

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

void ili9342_write_index(uint8_t index)
{
    palClearLine(LINE_LCD_DCX);
    spiPolledExchange(&SPID4, index);
}

void ili9342_write_data(uint8_t data)
{
    palSetLine(LINE_LCD_DCX);
    spiPolledExchange(&SPID4, data);
}
/*****************************************************************************/

/********************************************************** PUBLIC FUNCTIONS */
void lcd_init()
{
    /* Disable DCACHE so framebuf isn't corrupted.
     * Everything else is in the DTCM (uncachable) anyway.
     */
    SCB_DisableDCache();
    memset(framebuf, 0, sizeof(framebuf));
    for(size_t y=0; y<240; y++) {
        framebuf[y][0] = WHITE;
    }
    for(size_t x=1; x<319; x++) {
        framebuf[0][x] = WHITE;
        for(size_t y=1; y<239; y++) {
            if(x>320/2)
                if(y>240/2)
                    framebuf[y][x] = RED;
                else
                    framebuf[y][x] = GREEN;
            else
                if(y>240/2)
                    framebuf[y][x] = BLUE;
                else
                    framebuf[y][x] = YELLOW;
        }
        framebuf[239][x] = WHITE;
    }
    for(size_t y=0; y<240; y++) {
        framebuf[y][319] = WHITE;
    }
    for(size_t x=0; x<10; x++) {
        for(size_t y=0; y<10; y++) {
            framebuf[y][x] = WHITE;
        }
    }

    backlight_init();
    backlight_set(0);

    ltdc_init();

    ili9342_init();

    chThdSleepMilliseconds(100);
    backlight_set(100);
}
/*****************************************************************************/
