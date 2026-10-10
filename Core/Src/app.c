#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "app.h"
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
#include "depth_buffer.h"
#include "modifier3d.h"

#define APP_RENDER_INTERVAL_SECONDS       (1.0f / 30.0f)
#define APP_PI_RAD                        (3.14159265358979323846f)

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

static depth_buffer_t scene_depth_buffer;

static const projection_perspective_t cube_projection =
{
    .focal_length = 48.0f,
    .near_plane = 0.1f,
    .center_x = (float)(DISPLAY_WIDTH / 2U),
    .center_y = (float)(DISPLAY_HEIGHT / 2U)
};

static bool app_add_rotation_modifiers(
    app_t *app,
    size_t instance_index,
    float speed_x,
    float speed_y,
    float speed_z)
{
    if (app == NULL)
    {
        return false;
    }

    geometry3d_instance_t *instance =
        scene3d_get_instance(&app->scene, instance_index);

    if (instance == NULL)
    {
        return false;
    }

    const modifier3d_config_t rotation_x =
    {
        .target = MODIFIER3D_TARGET_ROTATION_X,
        .behavior = MODIFIER3D_BEHAVIOR_LINEAR_FORWARD,
        .min_value = -APP_PI_RAD,
        .max_value = APP_PI_RAD,
        .speed = speed_x
    };

    const modifier3d_config_t rotation_y =
    {
        .target = MODIFIER3D_TARGET_ROTATION_Y,
        .behavior = MODIFIER3D_BEHAVIOR_LINEAR_FORWARD,
        .min_value = -APP_PI_RAD,
        .max_value = APP_PI_RAD,
        .speed = speed_y
    };

    const modifier3d_config_t rotation_z =
    {
        .target = MODIFIER3D_TARGET_ROTATION_Z,
        .behavior = MODIFIER3D_BEHAVIOR_LINEAR_FORWARD,
        .min_value = -APP_PI_RAD,
        .max_value = APP_PI_RAD,
        .speed = speed_z
    };

    return modifier3d_add(instance, &rotation_x) &&
           modifier3d_add(instance, &rotation_y) &&
           modifier3d_add(instance, &rotation_z);
}

static bool app_add_pulsating_scale_modifiers(
    app_t *app,
    size_t instance_index)
{
    if (app == NULL)
    {
        return false;
    }

    geometry3d_instance_t *instance =
        scene3d_get_instance(&app->scene, instance_index);

    if (instance == NULL)
    {
        return false;
    }

    /*
     * The near cube starts at scale (0.6, 2.6, 0.6).
     *
     * All axes use the same normalized range and phase, so the
     * cube preserves its proportions while pulsating.
     */
    const modifier3d_config_t scale_x =
    {
        .target = MODIFIER3D_TARGET_SCALE_X,
        .behavior = MODIFIER3D_BEHAVIOR_ABS_SIN,
        .min_value = 0.4f,
        .max_value = 0.8f,
        .speed = 1.8f
    };

    const modifier3d_config_t scale_y =
    {
        .target = MODIFIER3D_TARGET_SCALE_Y,
        .behavior = MODIFIER3D_BEHAVIOR_ABS_SIN,
        .min_value = 1.7333333f,
        .max_value = 3.4666667f,
        .speed = 1.8f
    };

    const modifier3d_config_t scale_z =
    {
        .target = MODIFIER3D_TARGET_SCALE_Z,
        .behavior = MODIFIER3D_BEHAVIOR_ABS_SIN,
        .min_value = 0.4f,
        .max_value = 0.8f,
        .speed = 1.8f
    };

    return modifier3d_add(instance, &scale_x) &&
           modifier3d_add(instance, &scale_y) &&
           modifier3d_add(instance, &scale_z);
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
            &scene_depth_buffer,
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

    /* Test camera position. */
    app->camera.position.x = -4.0f;
    app->camera.position.z = -5.0f;

    /* Aim at the center of the near cube. */
    const geometry3d_vector3_t target =
    {
        .x = -1.1f,
        .y = 0.0f,
        .z = -1.0f
    };

    if (!camera3d_set_target(&app->camera, &target))
    {
        return false;
    }

    app->render_accumulator_seconds = 0.0f;

    scene3d_init(&app->scene);

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

    if (!app_add_rotation_modifiers(
            app,
            0U,
            0.55f,
            0.85f,
            0.15f))
    {
        return false;
    }

    if (!app_add_rotation_modifiers(
            app,
            1U,
            -0.35f,
            0.55f,
            0.65f))
    {
        return false;
    }

    if (!app_add_pulsating_scale_modifiers(app, 0U))
    {
        return false;
    }

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
        !isfinite(dt_seconds) ||
        dt_seconds < 0.0f ||
        app->scene.instance_count > SCENE3D_MAX_INSTANCES)
    {
        return false;
    }

    if (dt_seconds > 0.0f)
    {
        /*
         * Update instance state independently of rendering.
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
     * Preserve the accumulator remainder while rendering only
     * the latest scene state.
     */
    app->render_accumulator_seconds -=
        floorf(
            app->render_accumulator_seconds /
            APP_RENDER_INTERVAL_SECONDS
        ) * APP_RENDER_INTERVAL_SECONDS;

    return app_render_and_flush(app);
}