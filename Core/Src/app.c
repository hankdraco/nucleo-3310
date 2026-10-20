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
#include "stm32g4xx_hal.h"
#include "telemetry_protocol.h"
#include "telemetry_transport.h"

#define APP_RENDER_INTERVAL_SECONDS (1.0f / 30.0f)
#define APP_TELEMETRY_INTERVAL_MS 1000U

static pcd8544_transport_t lcd_transport;
static pcd8544_t lcd;
static display_t display;

/*
 * Static storage: the depth buffer must not live on the render stack.
 */
static depth_buffer_t scene_depth_buffer;

static bool app_is_active_car(const car_modifier_context_t *context)
{
    return context != NULL &&
           context->state != CAR_MODIFIER_STATE_FREE &&
           context->state != CAR_MODIFIER_STATE_SPAWN;
}

static size_t app_find_first_active_car(const app_t *app)
{
    for (size_t i = 0U; i < app->scene.agent_count; ++i)
    {
        if (app_is_active_car(&app->car_contexts[i])) return i;
    }
    return (size_t)-1;
}

static void app_capture_car_states(app_t *app)
{
    for (size_t i = 0U; i < APP_CAR_COUNT; ++i)
    {
        app->previous_car_states[i] = app->car_contexts[i].state;
    }
}

static bool app_update_follow_camera(app_t *app, float dt_seconds)
{
    if (!app->camera_controller.follow_mode) return true;

    bool snap_to_vehicle = false;
    if (app->followed_agent_index != (size_t)-1)
    {
        const size_t followed = app->followed_agent_index;
        if (followed >= app->scene.agent_count ||
            !app_is_active_car(&app->car_contexts[followed]))
        {
            app->followed_agent_index = (size_t)-1;
        }
    }

    if (app->followed_agent_index == (size_t)-1)
    {
        for (size_t i = 0U; i < app->scene.agent_count; ++i)
        {
            if (app->previous_car_states[i] == CAR_MODIFIER_STATE_SPAWN &&
                app_is_active_car(&app->car_contexts[i]))
            {
                app->followed_agent_index = i;
                snap_to_vehicle = true;
                break;
            }
        }
    }

    if (app->followed_agent_index != (size_t)-1)
    {
        const geometry3d_instance_t *agent =
            &app->scene.agent_instances[app->followed_agent_index];
        if (!camera_controller_follow_vehicle(
                &app->camera_controller,
                &app->camera,
                &agent->transform.position,
                agent->transform.rotation_rad.y,
                dt_seconds,
                snap_to_vehicle))
        {
            return false;
        }
    }

    app_capture_car_states(app);
    return true;
}

static uint32_t app_read_cycle_counter(void)
{
    return DWT->CYCCNT;
}

static bool app_render_and_flush(
    const app_t *app,
    telemetry_sample_t *sample)
{
    sample->render_start_cycles = app_read_cycle_counter();
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

    sample->render_end_cycles = app_read_cycle_counter();

    if (!display_pcd8544_flush(&display, &lcd))
    {
        return false;
    }

    sample->lcd_end_cycles = app_read_cycle_counter();
    return true;
}

static bool app_send_telemetry_batch(app_t *app, uint32_t now_ms)
{
    if (app->telemetry_sample_count == 0U ||
        (uint32_t)(now_ms - app->telemetry_batch_start_ms) <
            APP_TELEMETRY_INTERVAL_MS ||
        telemetry_transport_is_busy())
    {
        return true;
    }

    size_t frame_size = 0U;
    if (!telemetry_protocol_encode(
            SystemCoreClock,
            app->telemetry_samples,
            app->telemetry_sample_count,
            app->telemetry_frame,
            sizeof(app->telemetry_frame),
            &frame_size))
    {
        return false;
    }

    if (!telemetry_transport_send(app->telemetry_frame, frame_size))
    {
        return true;
    }

    app->telemetry_sample_count = 0U;
    app->telemetry_batch_start_ms = now_ms;
    return true;
}

bool app_init(app_t *app)
{
    if (app == NULL)
    {
        return false;
    }

    app->render_accumulator_seconds = 0.0f;
    app->telemetry_batch_start_ms = HAL_GetTick();
    app->telemetry_next_sequence = 0U;
    app->telemetry_sample_count = 0U;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    app->car_environment = (car_modifier_environment_t){
        .city_half_extent_x=160.0f,.city_half_extent_z=160.0f,
        .street_half_width=12.8f,.lane_edge_offset=7.0f,
        .minimum_speed_units_per_second=12.0f,.maximum_speed_units_per_second=18.0f,
        .random_state=0x91E10DA5U
    };

    /*
     * Load the scene, its camera and its projection.
     * The application does not know which models are instantiated.
     */
    if (!scene_loader_load(
            &app->scene,
            &app->camera,
            &app->projection,
            app->car_contexts,
            APP_CAR_COUNT,
            &app->car_environment))
    {
        return false;
    }

    /*
    * Initialize camera control from the initial scene view.
    * The controller preserves the current orientation
    * and switches the camera to manual orientation mode.
    */
    if (!camera_controller_init(
            &app->camera_controller,
            &app->camera))
    {
        return false;
    }
    app->followed_agent_index = (size_t)-1;
    app->camera_mode_key_was_down = false;
    app_capture_car_states(app);
    if (!camera_demo_init(&app->camera_demo, &app->camera) ||
        !camera_controller_set_demo_mode(
            &app->camera_controller, &app->camera, true))
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

bool app_run(
    app_t *app,
    float dt_seconds,
    const input_snapshot_t *input)
{
    if (app == NULL ||
        input == NULL ||
        !isfinite(dt_seconds) ||
        dt_seconds < 0.0f ||
        app->scene.instance_count > SCENE3D_MAX_INSTANCES ||
        app->scene.agent_count > SCENE3D_MAX_AGENTS ||
        app->scene.instance_count + app->scene.agent_count > SCENE3D_MAX_INSTANCES)
    {
        return false;
    }

    const bool camera_mode_key_down =
        (input->key_mask & INPUT_KEY_CAMERA_MODE) != 0U;
    if (camera_mode_key_down && !app->camera_mode_key_was_down)
    {
        if (app->camera_controller.follow_mode)
        {
            if (!camera_demo_init(&app->camera_demo, &app->camera) ||
                !camera_controller_set_demo_mode(
                    &app->camera_controller, &app->camera, true))
            {
                return false;
            }
            app->followed_agent_index = (size_t)-1;
        }
        else if (app->camera_controller.demo_mode)
        {
            if (!camera_controller_set_demo_mode(
                    &app->camera_controller, &app->camera, false))
            {
                return false;
            }
        }
        else
        {
            if (!camera_controller_set_follow_mode(
                    &app->camera_controller, &app->camera, true))
            {
                return false;
            }
            app->followed_agent_index = app_find_first_active_car(app);
            app_capture_car_states(app);
        }
    }
    app->camera_mode_key_was_down = camera_mode_key_down;

    /*
     * Process input on every application update,
     * independently of rendering frequency.
     */
    if (!camera_controller_update(
            &app->camera_controller,
            &app->camera,
            input,
            dt_seconds))
    {
        return false;
    }

    app->render_accumulator_seconds += dt_seconds;
    if (!isfinite(app->render_accumulator_seconds))
    {
        return false;
    }

    if (app->render_accumulator_seconds <
        APP_RENDER_INTERVAL_SECONDS)
    {
        return true;
    }

    /* Advance the scene only for a frame that will be rendered. */
    const float simulation_dt_seconds = app->render_accumulator_seconds;
    telemetry_sample_t sample = {0};
    sample.update_start_cycles = app_read_cycle_counter();
    if (!scene3d_update(&app->scene, simulation_dt_seconds))
    {
        return false;
    }
    sample.update_end_cycles = app_read_cycle_counter();

    app->render_accumulator_seconds -=
        floorf(
            app->render_accumulator_seconds /
            APP_RENDER_INTERVAL_SECONDS
        ) * APP_RENDER_INTERVAL_SECONDS;

    if (app->camera_controller.demo_mode)
    {
        if (!camera_demo_update(
                &app->camera_demo,
                &app->camera,
                simulation_dt_seconds))
        {
            return false;
        }
    }
    else if (!app_update_follow_camera(app, simulation_dt_seconds))
    {
        return false;
    }

    sample.sequence = app->telemetry_next_sequence++;
    if (!app_render_and_flush(app, &sample))
    {
        return false;
    }

    if (app->telemetry_sample_count < TELEMETRY_PROTOCOL_MAX_SAMPLES)
    {
        app->telemetry_samples[app->telemetry_sample_count++] = sample;
    }

    return app_send_telemetry_batch(app, HAL_GetTick());
}
