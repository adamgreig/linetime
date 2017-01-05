#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H

#include <string.h>

/* Huge frame buffer. */
static uint8_t framebuf[320*240*2]
    __attribute__((section(".sram1")))
    __attribute__((aligned(sizeof(stkalign_t))));

static const ltdcConfig driverCfg = {
    .width = 320,
    .height = 240,
    .hsync = 10,
    .vsync = 2,
    .hbackporch = 20,
    .vbackporch = 2,
    .hfrontporch = 10,
    .vfrontporch = 4,
    .syncflags = 0,
    .bgcolor = 0x000000,

    .bglayer = {
        /* Frame */
        .frame = (void*)framebuf,
        .width = 320,
        .height = 240,
        .pitch = 320 * LTDC_PIXELBYTES,
        .fmt = LTDC_PIXELFORMAT,

        /* Window */
        .x = 0,
        .y = 0,
        .cx = 320,
        .cy = 240,
        .defcolor = LTDC_COLOR_FUCHSIA,
        .keycolor = 0x980088,
        .blending = LTDC_BLEND_FIX1_FIX2,
        .palette = NULL,
        .palettelen = 0,
        .alpha = 0xFF,
        .layerflags = LTDC_LEF_ENABLE,
    },

    .fglayer = LTDC_UNUSED_LAYER_CONFIG,
};

static const SPIConfig spi_cfg = {
    .end_cb = NULL,
    .ssport = GPIOE,
    .sspad = 4,
    .cr1 = SPI_CR1_BR_2,
    .cr2 = 0,
};

static const PWMConfig pwm_cfg = {
    .frequency = 100000,
    .period = 100,
    .callback = NULL,
    .channels = {
        {
            .mode = PWM_OUTPUT_ACTIVE_HIGH,
            .callback = NULL,
        },
        {
            .mode = PWM_OUTPUT_DISABLED,
            .callback = NULL,
        },
        {
            .mode = PWM_OUTPUT_DISABLED,
            .callback = NULL,
        },
        {
            .mode = PWM_OUTPUT_DISABLED,
            .callback = NULL,
        },
    },
};

static void write_index(uint8_t index)
{
    palClearLine(LINE_LCD_DCX);
    spiPolledExchange(&SPID4, index);
}

static void write_data(uint8_t data)
{
    palSetLine(LINE_LCD_DCX);
    spiPolledExchange(&SPID4, data);
}

static void init_ili9342(void)
{
    /* Hardware reset */
    palClearLine(LINE_LCD_RST);
    gfxSleepMilliseconds(20);
    palSetLine(LINE_LCD_RST);
    gfxSleepMilliseconds(120);

    spiSelect(&SPID4);

    /* Software reset */
    write_index(0x01);
    gfxSleepMilliseconds(20);

    /* Enable extended commands. */
    write_index(0xC8);
    write_data(0xFF);
    write_data(0x93);
    write_data(0x42);

    /* Display off */
    write_index(0x28);

    /* Memory access control */
    write_index(0x36);
    write_data(0xc8);

    /* Pixel format */
    write_index(0x3A);
    write_data(0x66);

    /* RGB */
    write_index(0xB0);
    write_data(0x40);

    /* Display Waveform Cycle 1 (in Normal Mode) */
    /* Controls frame rate */
    write_index(0xB1);
    write_data(0x00);
    write_data(0x1B);

    /* Display Function Control */
    write_index(0xB6);
    write_data(0x0A);
    write_data(0x82);
    write_data(0x27);
    write_data(0x01);

    /* Interface control */
    write_index(0xF6);
    write_data(0x01);
    write_data(0x00);
    write_data(0x06);

    /* Power control */
    write_index(0xC0);
    write_data(0x10);
    write_data(0x10);
    write_index(0xc1);
    write_data(0x36);

    /* VCOM */
    write_index(0xc5);
    write_data(0xc3);
    write_data(0x3c);
    write_index(0xc7);
    write_data(0xc0);

    /* Gamma */
    write_index(0xf2);
    write_data(0x08);
    write_index(0x26);
    write_data(0x01);
    write_index(0xE0);
    write_data(0x00);
    write_data(0x05);
    write_data(0x08);
    write_data(0x02);
    write_data(0x1A);
    write_data(0x0C);
    write_data(0x42);
    write_data(0x7A);
    write_data(0x54);
    write_data(0x08);
    write_data(0x0D);
    write_data(0x0C);
    write_data(0x23);
    write_data(0x25);
    write_data(0x0F);
    write_index(0xE1);
    write_data(0x00);
    write_data(0x29);
    write_data(0x2F);
    write_data(0x03);
    write_data(0x0F);
    write_data(0x05);
    write_data(0x42);
    write_data(0x55);
    write_data(0x53);
    write_data(0x06);
    write_data(0x0F);
    write_data(0x0C);
    write_data(0x38);
    write_data(0x3A);
    write_data(0x0F);

    /* Sleep out */
    write_index(0x11);
    gfxSleepMilliseconds(120);

    /* Display on */
    write_index(0x29);
    gfxSleepMilliseconds(120);

#if 0
    write_index(0x2a);
    write_data(0);
    write_data(0);
    write_data(320>>8);
    write_data(320&0xFF);
    write_index(0x2b);
    write_data(0);
    write_data(0);
    write_data(0);
    write_data(240);
    write_index(0x2c);
    for(size_t y=0; y<240; y++) {
        for(size_t x=0; x<320; x++) {
            write_data((~(x^y))&0xFF);
            write_data(y&0xFF);
            write_data((x^y)&0xFF);
        }
    }
#endif

    spiUnselect(&SPID4);
}

static GFXINLINE void init_board(GDisplay* g)
{
    /* Disable the DCACHE globally. We can't have it on the framebuffer for
     * the LCD, and so it's either this, manual cache management, or use the
     * MPU to disable it for a large region including the framebuffer.
     *
     * This isn't as bad as it looks since everything except the framebuffer
     * and the SD card write buffer fits into the DTCM which isn't cacheable
     * anyway.
     */
    SCB_DisableDCache();

    /* Not using multiple displays so set g->board to 0. */
    g->board = 0;

    /* Start the control interface SPI */
    spiStart(&SPID4, &spi_cfg);

    /* Start the backlight PWM. */
    pwmStart(&PWMD4, &pwm_cfg);
    pwmEnableChannel(&PWMD4, 0, 0);

    /* Clear the buffer so it's initially black instead of noise */
    memset(framebuf, 0, sizeof(framebuf));

    /* Set up the ILI9342 */
    init_ili9342();

}

static GFXINLINE void post_init_board(GDisplay* g)
{
    (void)g;
}

static GFXINLINE void set_backlight(GDisplay* g, uint8_t percent)
{
    (void)g;
    pwmEnableChannel(&PWMD4, 0, percent);
}

#endif /* _GDISP_LLD_BOARD_H */
