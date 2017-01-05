#include "ch.h"
#include "hal.h"

#include "lcd.h"

void lcd_backlight_init(void);

void lcd_backlight_init()
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

    pwmStart(&PWMD4, &pwm_cfg);
    lcd_backlight_set(50);
}

void lcd_backlight_set(uint8_t brightness)
{
    pwmEnableChannel(&PWMD4, 0, brightness);
}

void lcd_init()
{
    lcd_backlight_init();
}
