
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

#define APP_MAX_MODEL_VERTICES 64U

static pcd8544_transport_t lcd_transport;
static pcd8544_t lcd;
static display_t display;

static const projection_perspective_t cube_projection =
{
    .focal_length = 48.0f,
    .camera_distance = 6.0f - 1.0f,
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
    const geometry3d_instance_t *instance = &app->cube;
    const geometry3d_model_t *model = instance->model;

    int16_t screen_x[APP_MAX_MODEL_VERTICES];
    int16_t screen_y[APP_MAX_MODEL_VERTICES];
    
    
    /*
     * Build individual transformation matrices.
     */
    matrix4_t rotation_x;
    matrix4_t rotation_y;
    matrix4_t rotation_z;

    matrix4_t scale;
    matrix4_t translation;

    matrix4_t rotation_yx;
    matrix4_t rotation_zyx;
    matrix4_t rotated_scaled;
    matrix4_t transform;

    matrix4_set_rotation_x(
        &rotation_x,
        app->cube.transform.rotation_rad.x);

    matrix4_set_rotation_y(
        &rotation_y,
        app->cube.transform.rotation_rad.y);

    matrix4_set_rotation_z(
        &rotation_z,
        app->cube.transform.rotation_rad.z);

    matrix4_set_scale(
        &scale,
        app->cube.transform.scale.x,
        app->cube.transform.scale.y,
        app->cube.transform.scale.z);

    matrix4_set_translation(
        &translation,
        app->cube.transform.position.x,
        app->cube.transform.position.y,
        app->cube.transform.position.z);

    /*
     * Rotation order: Y, then X, then Z.
     */
    if (!matrix4_multiply(
            &rotation_yx,
            &rotation_x,
            &rotation_y))
    {
        return false;
    }

    if (!matrix4_multiply(
            &rotation_zyx,
            &rotation_z,
            &rotation_yx))
    {
        return false;
    }

    /*
     * Scale first, then rotate.
     */
    if (!matrix4_multiply(
            &rotated_scaled,
            &rotation_zyx,
            &scale))
    {
        return false;
    }

    /*
     * Translate last.
     *
     * M = T * Rz * Rx * Ry * S
     */
    if (!matrix4_multiply(
            &transform,
            &translation,
            &rotated_scaled))
    {
        return false;
    }

    /*
     * Transform and project all vertices.
     */
    for (size_t i = 0U; i < model->vertex_count; ++i)
    {
        geometry3d_vertex_t vertex;

        if (!geometry3d_transform_vertex(
                &transform,
                &model->vertices[i],
                &vertex))
        {
            return false;
        }

        if (!projection_perspective_project(
                &cube_projection,
                &vertex,
                &screen_x[i],
                &screen_y[i]))
        {
            return false;
        }
    }

    /*
     * Clear the previous frame.
     */
    display_clear(&display);

    /*
     * Draw all cube edges.
     */
    for (size_t i = 0U; i < model->edge_count; ++i)
    {
        const geometry3d_edge_t *edge = &model->edges[i];

        if (!display_draw_line(
                &display,
                screen_x[edge->start],
                screen_y[edge->start],
                screen_x[edge->end],
                screen_y[edge->end]))
        {
            return false;
        }
    }

    /*
     * Transfer the completed canvas to the LCD.
     */
    return display_pcd8544_flush(&display, &lcd);
}

bool app_init(app_t *app)
{
    if (app == NULL)
    {
        return false;
    }

    app->render_accumulator_seconds = 0.0f;

    /* Select the model used by this instance. */
    app->cube.model = &geometry3d_cube_model;

    if (!geometry3d_model_is_valid(app->cube.model) ||
        app->cube.model->vertex_count > APP_MAX_MODEL_VERTICES)
    {
        return false;
    }

    /* Initial position in world coordinates. */
    app->cube.transform.position.x = APP_CUBE_POSITION_X;
    app->cube.transform.position.y = APP_CUBE_POSITION_Y;
    app->cube.transform.position.z = APP_CUBE_POSITION_Z;

    /* Initial rotation in radians. */
    app->cube.transform.rotation_rad.x =
        APP_CUBE_INITIAL_ANGLE_X_RAD;

    app->cube.transform.rotation_rad.y =
        APP_CUBE_INITIAL_ANGLE_Y_RAD;

    app->cube.transform.rotation_rad.z = 0.35f;

    /* Initial scale. */
    app->cube.transform.scale.x = APP_CUBE_SCALE_X;
    app->cube.transform.scale.y = APP_CUBE_SCALE_Y;
    app->cube.transform.scale.z = APP_CUBE_SCALE_Z;

    /* Animation velocities. */
    app->cube_animation.angular_velocity_x_rad_s =
        APP_CUBE_ANGULAR_VELOCITY_X_RAD_S;

    app->cube_animation.angular_velocity_y_rad_s =
        APP_CUBE_ANGULAR_VELOCITY_Y_RAD_S;

    app->cube_animation.angular_velocity_z_rad_s = 0.0f;

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
    if (app == NULL || dt_seconds < 0.0f)
    {
        return false;
    }

    if (dt_seconds > 0.0f)
    {
        /*
         * Update the animation independently of rendering.
         */
        app_update_angle(
            &app->cube.transform.rotation_rad.x,
            app->cube_animation.angular_velocity_x_rad_s,
            dt_seconds);

        app_update_angle(
            &app->cube.transform.rotation_rad.y,
            app->cube_animation.angular_velocity_y_rad_s,
            dt_seconds);

        app_update_angle(
            &app->cube.transform.rotation_rad.z,
            app->cube_animation.angular_velocity_z_rad_s,
            dt_seconds);

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