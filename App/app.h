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

#define APP_CAR_COUNT 10U /**< Number of autonomous cars created by the demo. */

/** Top-level application state, including scene, camera and telemetry buffers. */
typedef struct
{
    float render_accumulator_seconds; /**< Time accumulated toward the next rendered frame. */

    uint32_t telemetry_batch_start_ms; /**< Tick when the current telemetry batch began. */
    uint16_t telemetry_next_sequence; /**< Sequence assigned to the next telemetry sample. */
    size_t telemetry_sample_count; /**< Number of samples currently buffered. */
    telemetry_sample_t telemetry_samples[TELEMETRY_PROTOCOL_MAX_SAMPLES]; /**< Pending raw timing samples. */
    uint8_t telemetry_frame[TELEMETRY_PROTOCOL_MAX_FRAME_SIZE]; /**< Encoded frame storage for asynchronous transmission. */

    scene3d_t scene; /**< Scene instances and autonomous agents. */
    camera3d_t camera; /**< Active view camera. */
    camera_controller_t camera_controller; /**< User and vehicle-follow camera control. */
    camera_demo_t camera_demo; /**< Progress through the demo camera path. */
    size_t followed_agent_index; /**< Agent currently followed by the camera. */
    bool camera_mode_key_was_down; /**< Previous camera-mode key state for edge detection. */
    car_modifier_state_t previous_car_states[APP_CAR_COUNT]; /**< Previous states used to detect car respawns. */

    /* Shared environment and stable per-car state referenced by scene modifiers. */
    car_modifier_environment_t car_environment; /**< Shared world configuration for all car modifiers. */
    car_modifier_context_t car_contexts[APP_CAR_COUNT]; /**< Stable, caller-owned context for each car. */
    projection_perspective_t projection; /**< Perspective projection used by the renderer. */
} app_t;

/** Initialize the scene, camera, rendering state and telemetry state.
 * @param app Application storage to initialize.
 * @return true on success; false if initialization fails or app is NULL.
 */
bool app_init(app_t *app);

/** Process one application iteration and render when the frame is due.
 * @param app Initialized application state.
 * @param dt_seconds Elapsed time since the previous loop iteration.
 * @param input Input snapshot consumed for this iteration.
 * @return true on success; false if an application subsystem reports failure.
 */
bool app_run(
    app_t *app,
    float dt_seconds,
    const input_snapshot_t *input);

#endif /* APP_H */
