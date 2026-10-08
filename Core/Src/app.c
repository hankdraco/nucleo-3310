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
    display_draw_line(&display, 0, 0, 83, 0);
    display_draw_line(&display, 83, 0, 83, 47);
    display_draw_line(&display, 83, 47, 0, 47);
    display_draw_line(&display, 0, 47, 0, 0);

    display_draw_line(&display, 0, 0, 83, 47);
    display_draw_line(&display, 83, 0, 0, 47);
}

bool app_init(app_t *app)
{
    (void)app;

    pcd8544_transport_stm32_init(&lcd_transport);

    if (!pcd8544_init(&lcd, &lcd_transport))
    {
        return false;
    }

    display_config_t config =
    {
        .orientation =
            DISPLAY_ORIENTATION_FLIP_X |
            DISPLAY_ORIENTATION_FLIP_Y
    };

    display_init(&display, &config);

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