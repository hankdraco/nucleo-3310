#include "app.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "display.h"
#include "display_pcd8544.h"
#include "pcd8544.h"
#include "pcd8544_transport.h"

#include "geometry3d.h"
#include "projection.h"
#include "matrix4.h"
#include "cube_model.h"
#include "renderer3d.h"
#include "scene3d.h"

#define APP_RENDER_INTERVAL_SECONDS       (1.0f / 30.0f)
#define APP_TWO_PI_RAD                    6.28318530718f

#define APP_CUBE_INITIAL_ANGLE_X_RAD      0.55f
#define APP_CUBE_INITIAL_ANGLE_Y_RAD      0.65f

#define APP_CUBE_ANGULAR_VELOCITY_X_RAD_S 0.55f
#define APP_CUBE_ANGULAR_VELOCITY_Y_RAD_S 0.85f

#define APP_CUBE_SCALE_X       1.0f
#define APP_CUBE_SCALE_Y       0.3f
#define APP_CUBE_SCALE_Z       1.0f

#define APP_CUBE_POSITION_X    0.0f
#define APP_CUBE_POSITION_Y    0.0f
#define APP_CUBE_POSITION_Z    0.0f

#define APP_CUBE_INSTANCE_INDEX 0U

#define APP_NEAR_CUBE_INDEX 0U
#define APP_FAR_CUBE_INDEX  1U

static pcd8544_transport_t lcd_transport;
static pcd8544_t lcd;
static display_t display;

static const projection_perspective_t cube_projection =
{
    .focal_length = 48.0f,
    .near_plane = 0.1f,
    .center_x = (float)(DISPLAY_WIDTH / 2U),
    .center_y = (float)(DISPLAY_HEIGHT / 2U)
};

static void app_update_angle(
    float *angle_rad,
    float angular_velocity_rad_s,
    float dt_seconds)
{
    *angle_rad += angular_velocity_rad_s * dt_seconds;

    *angle_rad = fmodf(*angle_rad, APP_TWO_PI_RAD);

    if (*angle_rad < 0.0f)
    {
        *angle_rad += APP_TWO_PI_RAD;
    }
}

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
            &view_matrix,
            &cube_projection))
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

    if (!camera3d_init(&app->camera))
    {
        return false;
    }

    /*
    * Preserve the original camera position:
    * (0, 0, -5), looking along positive Z.
    */
    app->camera.position.z = -5.0f;

    app->render_accumulator_seconds = 0.0f;

    scene3d_init(&app->scene);

    for (size_t i = 0U; i < SCENE3D_MAX_INSTANCES; ++i)
    {
        app->animations[i].angular_velocity_x_rad_s = 0.0f;
        app->animations[i].angular_velocity_y_rad_s = 0.0f;
        app->animations[i].angular_velocity_z_rad_s = 0.0f;
    }

    // instances definitions
    const geometry3d_instance_t near_cube =
    {
        .model = &geometry3d_cube_model,
        .transform =
        {
            .position = { .x = -1.1f, .y = 0.0f, .z = -1.0f },
            .rotation_rad = { .x = 0.55f, .y = 0.65f, .z = 0.0f },
            //.scale = { .x = 0.6f, .y = 0.6f, .z = 0.6f }
            .scale = { .x = 0.6f, .y = 2.6f, .z = 0.6f }
        }
    };

    const geometry3d_instance_t far_cube =
    {
        .model = &geometry3d_cube_model,
        .transform =
        {
            //.position = { .x = 1.1f, .y = 0.15f, .z = 1.2f },
            .position = { .x = 5.1f, .y = 0.15f, .z = 1.2f },
            .rotation_rad = { .x = 0.25f, .y = -0.4f, .z = 0.2f },
            //.scale = { .x = 0.4f, .y = 0.4f, .z = 0.4f }
            .scale = { .x = 1.4f, .y = 1.4f, .z = 1.4f }
        }
    };

    if (!scene3d_add_instance(&app->scene, &near_cube))
    {
        return false;
    }

    if (!scene3d_add_instance(&app->scene, &far_cube))
    {
        return false;
    }

    /* Animation velocities for the near cube. */
    app->animations[APP_NEAR_CUBE_INDEX].angular_velocity_x_rad_s = 0.55f;
    app->animations[APP_NEAR_CUBE_INDEX].angular_velocity_y_rad_s = 0.85f;
    app->animations[APP_NEAR_CUBE_INDEX].angular_velocity_z_rad_s = 0.15f;

    /* Animation velocities for the far cube. */
    app->animations[APP_FAR_CUBE_INDEX].angular_velocity_x_rad_s = -0.35f;
    app->animations[APP_FAR_CUBE_INDEX].angular_velocity_y_rad_s = 0.55f;
    app->animations[APP_FAR_CUBE_INDEX].angular_velocity_z_rad_s = 0.65f;

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
    if (app == NULL ||
        dt_seconds < 0.0f ||
        app->scene.instance_count > SCENE3D_MAX_INSTANCES)
    {
        return false;
    }

    geometry3d_instance_t *cube =
        scene3d_get_instance(
            &app->scene,
            APP_CUBE_INSTANCE_INDEX);

    if (cube == NULL)
    {
        return false;
    }

    if (dt_seconds > 0.0f)
    {
        /*
         * Update the animation independently of rendering.
         */
        for (size_t i = 0U; i < app->scene.instance_count; ++i)
        {
            geometry3d_instance_t *instance =
                scene3d_get_instance(&app->scene, i);

            if (instance == NULL)
            {
                return false;
            }

            const app_instance_animation_t *animation =
                &app->animations[i];

            app_update_angle(
                &instance->transform.rotation_rad.x,
                animation->angular_velocity_x_rad_s,
                dt_seconds);

            app_update_angle(
                &instance->transform.rotation_rad.y,
                animation->angular_velocity_y_rad_s,
                dt_seconds);

            app_update_angle(
                &instance->transform.rotation_rad.z,
                animation->angular_velocity_z_rad_s,
                dt_seconds);
        }

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
     * Render only the latest animation state.
     */
    app->render_accumulator_seconds -=
        floorf(
            app->render_accumulator_seconds /
            APP_RENDER_INTERVAL_SECONDS
        ) * APP_RENDER_INTERVAL_SECONDS;

    return app_render_and_flush(app);
}