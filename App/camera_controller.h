#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>

#include "Modules/engine3d/camera3d.h"
#include "Protocols/input/input_types.h"

#define CAMERA_CONTROLLER_DEFAULT_MOVE_SPEED 8.0f /**< Default free-fly movement speed in world units/s. */
#define CAMERA_CONTROLLER_DEFAULT_MOUSE_SENSITIVITY 0.005f /**< Default camera rotation per mouse count, in radians. */

/* 89 degrees, leaving a margin from +/-90 degrees. */
#define CAMERA_CONTROLLER_MAX_PITCH_RAD 1.55334306f /**< Pitch limit that keeps the view away from vertical. */

/** Mutable state used to translate input into camera movement and mode changes. */
typedef struct
{
    float yaw_rad; /**< Current manual yaw angle in radians. */
    float pitch_rad; /**< Current manual pitch angle in radians. */

    float move_speed_units_per_second; /**< Free-fly movement speed. */
    float mouse_sensitivity_rad_per_count; /**< Rotation applied per mouse count. */

    bool follow_mode; /**< True while tracking a moving car. */
    bool demo_mode; /**< True while traversing the cinematic path. */
    bool follow_camera_initialized; /**< True after the follow view has been positioned. */
    bool initialized; /**< True after controller initialization. */

} camera_controller_t;

/** Initialize from the camera's current orientation.
 * If look-at mode is active, preserve its orientation before enabling manual control.
 * @param controller Controller state to initialize.
 * @param camera Camera supplying the initial pose.
 * @return true on success; false for invalid pointers or camera state.
 */
bool camera_controller_init(
    camera_controller_t *controller,
    camera3d_t *camera);

/** Apply one input snapshot to the camera.
 * Mouse deltas rotate the view; key states move it. Delta time scales movement,
 * but does not scale the already accumulated mouse deltas.
 * @param controller Initialized controller state.
 * @param camera Camera to update.
 * @param input Input state to apply.
 * @param dt_seconds Elapsed time in seconds.
 * @return true on success; false for invalid input or camera failure.
 */
bool camera_controller_update(
    camera_controller_t *controller,
    camera3d_t *camera,
    const input_snapshot_t *input,
    float dt_seconds);

/** Enable or disable vehicle-follow mode.
 * @param controller Controller state.
 * @param camera Camera to configure.
 * @param enabled Whether follow mode should be enabled.
 * @return true on success; false if arguments or mode transition are invalid.
 */
bool camera_controller_set_follow_mode(
    camera_controller_t *controller,
    camera3d_t *camera,
    bool enabled);

/** Switch between the demo path and manually controlled free-fly mode.
 * @param controller Controller state.
 * @param camera Camera to configure.
 * @param enabled Whether demo mode should be enabled.
 * @return true on success; false if arguments or mode transition are invalid.
 */
bool camera_controller_set_demo_mode(
    camera_controller_t *controller,
    camera3d_t *camera,
    bool enabled);

/** Position the camera above and behind a vehicle, looking ahead along its route.
 * @param controller Controller state.
 * @param camera Camera to move.
 * @param vehicle_position Vehicle world position.
 * @param vehicle_yaw_rad Vehicle heading in radians.
 * @param dt_seconds Elapsed time in seconds for smoothing.
 * @param snap_to_vehicle If true, place the camera immediately without smoothing.
 * @return true on success; false for invalid arguments or camera failure.
 */
bool camera_controller_follow_vehicle(
    camera_controller_t *controller,
    camera3d_t *camera,
    const geometry3d_vector3_t *vehicle_position,
    float vehicle_yaw_rad,
    float dt_seconds,
    bool snap_to_vehicle);

#endif /* CAMERA_CONTROLLER_H */
