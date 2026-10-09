
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

#define APP_RENDER_INTERVAL_SECONDS       (1.0f / 30.0f)
#define APP_TWO_PI_RAD                    6.28318530718f

#define APP_CUBE_INITIAL_ANGLE_X_RAD      0.55f
#define APP_CUBE_INITIAL_ANGLE_Y_RAD      0.65f

#define APP_CUBE_ANGULAR_VELOCITY_X_RAD_S 0.55f
#define APP_CUBE_ANGULAR_VELOCITY_Y_RAD_S 0.85f

#define APP_CUBE_SCALE_X       1.0f
#define APP_CUBE_SCALE_Y       0.3f
#define APP_CUBE_SCALE_Z       1.0f

#define APP_CUBE_POSITION_X    0.9f
#define APP_CUBE_POSITION_Y    0.0f
#define APP_CUBE_POSITION_Z    0.0f

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

static const geometry3d_cube_t cube_model =
{
    .vertices =
    {
        { -1.0f, -1.0f, -1.0f },
        {  1.0f, -1.0f, -1.0f },
        {  1.0f,  1.0f, -1.0f },
        { -1.0f,  1.0f, -1.0f },

        { -1.0f, -1.0f,  1.0f },
        {  1.0f, -1.0f,  1.0f },
        {  1.0f,  1.0f,  1.0f },
        { -1.0f,  1.0f,  1.0f }
    },
    .edges =
    {
        /* Face at z = -1 */
        { 0U, 1U },
        { 1U, 2U },
        { 2U, 3U },
        { 3U, 0U },

        /* Face at z = +1 */
        { 4U, 5U },
        { 5U, 6U },
        { 6U, 7U },
        { 7U, 4U },

        /* Connect both faces */
        { 0U, 4U },
        { 1U, 5U },
        { 2U, 6U },
        { 3U, 7U }
    }
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
    int16_t screen_x[GEOMETRY3D_CUBE_VERTEX_COUNT];
    int16_t screen_y[GEOMETRY3D_CUBE_VERTEX_COUNT];

    
    /*
     * Build individual transformation matrices.
     */
    matrix4_t rotation_x;
    matrix4_t rotation_y;
    matrix4_t rotation;
    matrix4_t scale;
    matrix4_t translation;
    matrix4_t rotated_scaled;
    matrix4_t transform;

    matrix4_set_rotation_x(
        &rotation_x,
        app->cube.angle_x_rad);

    matrix4_set_rotation_y(
        &rotation_y,
        app->cube.angle_y_rad);

    matrix4_set_scale(
        &scale,
        APP_CUBE_SCALE_X,
        APP_CUBE_SCALE_Y,
        APP_CUBE_SCALE_Z);

    matrix4_set_translation(
        &translation,
        APP_CUBE_POSITION_X,
        APP_CUBE_POSITION_Y,
        APP_CUBE_POSITION_Z);

    /*
     * Compose rotation: first Y, then X.
     */
    if (!matrix4_multiply(
            &rotation,
            &rotation_x,
            &rotation_y))
    {
        return false;
    }

    /*
     * Apply scale first, then rotation.
     */
    if (!matrix4_multiply(
            &rotated_scaled,
            &rotation,
            &scale))
    {
        return false;
    }

    /*
     * Apply translation last.
     *
     * Final transform: T * Rx * Ry * S
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
    for (size_t i = 0U;
         i < GEOMETRY3D_CUBE_VERTEX_COUNT;
         ++i)
    {
        geometry3d_vertex_t vertex;

        if (!geometry3d_transform_vertex(
                &transform,
                &cube_model.vertices[i],
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
    for (size_t i = 0U;
         i < GEOMETRY3D_CUBE_EDGE_COUNT;
         ++i)
    {
        const geometry3d_edge_t *edge = &cube_model.edges[i];

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

    app->cube.angle_x_rad = APP_CUBE_INITIAL_ANGLE_X_RAD;
    app->cube.angular_velocity_x_rad_s =
        APP_CUBE_ANGULAR_VELOCITY_X_RAD_S;

    app->cube.angle_y_rad = APP_CUBE_INITIAL_ANGLE_Y_RAD;
    app->cube.angular_velocity_y_rad_s =
        APP_CUBE_ANGULAR_VELOCITY_Y_RAD_S;

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
            &app->cube.angle_x_rad,
            app->cube.angular_velocity_x_rad_s,
            dt_seconds);

        app_update_angle(
            &app->cube.angle_y_rad,
            app->cube.angular_velocity_y_rad_s,
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