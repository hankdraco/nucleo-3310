#include <math.h>
#include <stddef.h>

#include "app.h"
#include "depth_buffer.h"
#include "display.h"
#include "display_pcd8544.h"
#include "matrix4.h"
#include "pcd8544.h"
#include "pcd8544_transport.h"
#include "scene3d.h"
#include "scene_loader.h"

#define APP_RENDER_INTERVAL_SECONDS (1.0f / 30.0f)

static pcd8544_transport_t lcd_transport;
static pcd8544_t lcd;
static display_t display;

/*
 * Static storage: the depth buffer must not live on the render stack.
 */
static depth_buffer_t scene_depth_buffer;

static bool app_render_and_flush(const app_t *app)
{
    display_clear(&display);

    matrix4_t view_matrix;

    if (!camera3d_build_view_matrix(
            &app->camera,
            &view_matrix))
    {
        return false;
    }

    if (!scene3d_render(
            &app->scene,
            &display,
            &scene_depth_buffer,
            &view_matrix,
            &app->projection))
    {
        return false;
    }

    return display_pcd8544_flush(&display, &lcd);
}

bool app_init(app_t *app)
{
    if (app == NULL)
    {
        return false;
    }

    app->render_accumulator_seconds = 0.0f;

    /*
     * Load the scene, its camera and its projection.
     * The application does not know which models are instantiated.
     */
    if (!scene_loader_load(
            &app->scene,
            &app->camera,
            &app->projection))
    {
        return false;
    }

    /*
     * Hardware initialization remains the application's responsibility.
     */
    pcd8544_transport_stm32_init(&lcd_transport);

    if (!pcd8544_init(&lcd, &lcd_transport))
    {
        return false;
    }

    const display_config_t config =
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
    if (app == NULL ||
        !isfinite(dt_seconds) ||
        dt_seconds < 0.0f ||
        app->scene.instance_count > SCENE3D_MAX_INSTANCES)
    {
        return false;
    }

    if (dt_seconds > 0.0f)
    {
        /*
         * Update scene state independently of rendering frequency.
         */
        if (!scene3d_update(&app->scene, dt_seconds))
        {
            return false;
        }

        app->render_accumulator_seconds += dt_seconds;

        if (!isfinite(app->render_accumulator_seconds))
        {
            return false;
        }
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
     * Preserve the remainder and render the latest scene state.
     */
    app->render_accumulator_seconds -=
        floorf(
            app->render_accumulator_seconds /
            APP_RENDER_INTERVAL_SECONDS
        ) * APP_RENDER_INTERVAL_SECONDS;

    return app_render_and_flush(app);
}