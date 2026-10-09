#include "app.h"

#include <stdint.h>
#include <math.h>

#include "display.h"
#include "display_pcd8544.h"
#include "pcd8544.h"
#include "pcd8544_transport.h"

#define APP_RENDER_INTERVAL_SECONDS (1.0f / 30.0f)
#define APP_TWO_PI_RAD              6.28318530718f
#define APP_LINE_RADIUS_PIXELS      20.0f

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

static bool app_render_and_flush(const app_t *app)
{
    const int16_t center_x = (int16_t)(DISPLAY_WIDTH / 2U);
    const int16_t center_y = (int16_t)(DISPLAY_HEIGHT / 2U);

    float angle = app->line.angle_rad;

    int16_t endpoint_x = (int16_t)lroundf(
        (float)center_x +
        APP_LINE_RADIUS_PIXELS * cosf(angle)
    );

    int16_t endpoint_y = (int16_t)lroundf(
        (float)center_y +
        APP_LINE_RADIUS_PIXELS * sinf(angle)
    );

    display_clear(&display);

    if (!display_draw_line(
            &display,
            center_x,
            center_y,
            endpoint_x,
            endpoint_y))
    {
        return false;
    }

    return display_pcd8544_flush(&display, &lcd);
}

bool app_init(app_t *app)
{
    app->render_accumulator_seconds = 0.0f;

    app->line.angle_rad = 0.0f;
    app->line.angular_velocity_rad_s = APP_TWO_PI_RAD / 60.0f;

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

    if (!display_pcd8544_flush(&display, &lcd))
    {
        return false;
    }

    return true;
}

bool app_run(app_t *app, float dt_seconds)
{
    if (dt_seconds > 0.0f)
    {
        /*
         * Update animation state.
         */
        app->line.angle_rad +=
            app->line.angular_velocity_rad_s * dt_seconds;

        /*
         * Keep angle in the range [0, 2*pi).
         */
        if (app->line.angle_rad >= APP_TWO_PI_RAD ||
            app->line.angle_rad < 0.0f)
        {
            app->line.angle_rad =
                fmodf(app->line.angle_rad, APP_TWO_PI_RAD);

            if (app->line.angle_rad < 0.0f)
            {
                app->line.angle_rad += APP_TWO_PI_RAD;
            }
        }

        /*
         * Accumulate elapsed time for rendering.
         */
        app->render_accumulator_seconds += dt_seconds;
    }

    /*
     * No frame is due yet.
     */
    if (app->render_accumulator_seconds <
        APP_RENDER_INTERVAL_SECONDS)
    {
        return true;
    }

    /*
     * Discard missed frame intervals while preserving the remainder.
     * We render only the latest animation state.
     */
    app->render_accumulator_seconds -=
        floorf(
            app->render_accumulator_seconds /
            APP_RENDER_INTERVAL_SECONDS
        ) * APP_RENDER_INTERVAL_SECONDS;

    return app_render_and_flush(app);
}