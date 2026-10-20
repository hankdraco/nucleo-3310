#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Modules/engine3d/camera3d.h"
#include "Modules/engine3d/projection.h"
#include "Modules/engine3d/scene3d.h"
#include "App/camera_controller.h"
#include "App/camera_demo.h"
#include "Modules/traffic/car_modifier.h"
#include "Protocols/input/input_types.h"
#include "Protocols/telemetry/telemetry_protocol.h"

#define APP_CAR_COUNT 10U

typedef struct
{
    float render_accumulator_seconds;

    uint32_t telemetry_batch_start_ms;
    uint16_t telemetry_next_sequence;
    size_t telemetry_sample_count;
    telemetry_sample_t telemetry_samples[TELEMETRY_PROTOCOL_MAX_SAMPLES];
    uint8_t telemetry_frame[TELEMETRY_PROTOCOL_MAX_FRAME_SIZE];

    scene3d_t scene;
    camera3d_t camera;
    camera_controller_t camera_controller;
    camera_demo_t camera_demo;
    size_t followed_agent_index;
    bool camera_mode_key_was_down;
    car_modifier_state_t previous_car_states[APP_CAR_COUNT];

    /* Shared environment and stable per-car state referenced by scene modifiers. */
    car_modifier_environment_t car_environment;
    car_modifier_context_t car_contexts[APP_CAR_COUNT];
    projection_perspective_t projection;
} app_t;

bool app_init(app_t *app);

bool app_run(
    app_t *app,
    float dt_seconds,
    const input_snapshot_t *input);

#endif /* APP_H */
