#include "app.h"

#include <stdint.h>

#include "display.h"
#include "display_pcd8544.h"
#include "pcd8544.h"
#include "pcd8544_transport.h"

static pcd8544_transport_t lcd_transport;
static pcd8544_t lcd;
static display_t display;

static void draw_demo(void)
{
    for (uint8_t x = 0U; x < DISPLAY_WIDTH; ++x)
    {
        display_set_pixel(&display, x, 0U, true);
        display_set_pixel(
            &display,
            x,
            DISPLAY_HEIGHT - 1U,
            true
        );
    }

    for (uint8_t y = 0U; y < DISPLAY_HEIGHT; ++y)
    {
        display_set_pixel(&display, 0U, y, true);
        display_set_pixel(
            &display,
            DISPLAY_WIDTH - 1U,
            y,
            true
        );
    }

    for (uint8_t x = 0U; x < DISPLAY_WIDTH; ++x)
    {
        uint8_t y1 = (uint8_t)(
            ((uint16_t)x * (DISPLAY_HEIGHT - 1U)) /
            (DISPLAY_WIDTH - 1U)
        );

        uint8_t y2 = (uint8_t)(
            (DISPLAY_HEIGHT - 1U) - y1
        );

        display_set_pixel(&display, x, y1, true);
        display_set_pixel(&display, x, y2, true);
    }
}

bool app_init(app_t *app)
{
    (void)app;

    pcd8544_transport_stm32_init(&lcd_transport);

    if (!pcd8544_init(&lcd, &lcd_transport))
    {
        return false;
    }

    display_init(&display, false);

    draw_demo();

    if (!display_pcd8544_flush(&display, &lcd))
    {
        return false;
    }

    return true;
}

void app_run(app_t *app)
{
    (void)app;
}