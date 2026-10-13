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

#define APP_TWO_PI_RAD                 (6.28318530718f)
#define APP_CAMERA_ORBIT_SPEED_RAD_S   (0.15f)

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
    * Initialize a temporary horizontal camera orbit around the look-at target.
    */
    if (!app->camera.target_enabled)
    {
        return false;
    }

    const float offset_x =
        app->camera.position.x - app->camera.target.x;

    const float offset_z =
        app->camera.position.z - app->camera.target.z;

    app->camera_orbit_radius = hypotf(offset_x, offset_z);

    if (!isfinite(app->camera_orbit_radius) ||
        app->camera_orbit_radius <= 0.0f)
    {
        return false;
    }

    /*
    * Preserve the current camera position as the starting point.
    */
    app->camera_orbit_phase_rad = atan2f(offset_x, offset_z);

    app->camera_orbit_height_offset =
        app->camera.position.y - app->camera.target.y;

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

        /*
         * Temporary camera orbit controller.
         * Keep the look-at target fixed and move the camera
         * around it in the world XZ plane.
         */
        const float advanced_phase =
            app->camera_orbit_phase_rad +
            APP_CAMERA_ORBIT_SPEED_RAD_S * dt_seconds;

        if (!isfinite(advanced_phase))
        {
            return false;
        }

        const float next_phase =
            fmodf(advanced_phase, APP_TWO_PI_RAD);

        if (!isfinite(next_phase))
        {
            return false;
        }

        const float camera_x =
            app->camera.target.x +
            app->camera_orbit_radius * sinf(next_phase);

        const float camera_z =
            app->camera.target.z +
            app->camera_orbit_radius * cosf(next_phase);

        const float camera_y =
            app->camera.target.y +
            app->camera_orbit_height_offset;

        if (!isfinite(camera_x) ||
            !isfinite(camera_y) ||
            !isfinite(camera_z))
        {
            return false;
        }

        app->camera_orbit_phase_rad = next_phase;

        app->camera.position.x = camera_x;
        app->camera.position.y = camera_y;
        app->camera.position.z = camera_z;
        //

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