#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>

#include "camera3d.h"
#include "input_types.h"

#define CAMERA_CONTROLLER_DEFAULT_MOVE_SPEED 8.0f
#define CAMERA_CONTROLLER_DEFAULT_MOUSE_SENSITIVITY 0.005f

/* 89 degrees, leaving a margin from +/-90 degrees. */
#define CAMERA_CONTROLLER_MAX_PITCH_RAD 1.55334306f

typedef struct
{
    float yaw_rad;
    float pitch_rad;

    float move_speed_units_per_second;
    float mouse_sensitivity_rad_per_count;

    bool follow_mode;
    bool demo_mode;
    bool follow_camera_initialized;
    bool initialized;

} camera_controller_t;

/*
 * Initialize from the camera's current orientation.
 *
 * If the camera is in look-at mode, its current
 * orientation is preserved before switching to
 * manual orientation.
 */
bool camera_controller_init(
    camera_controller_t *controller,
    camera3d_t *camera);

/*
 * Apply one input snapshot to the camera.
 *
 * Mouse deltas rotate the camera.
 * Key state moves the camera.
 * dt_seconds controls movement distance, not
 * the interpretation of mouse deltas.
 */
bool camera_controller_update(
    camera_controller_t *controller,
    camera3d_t *camera,
    const input_snapshot_t *input,
    float dt_seconds);

/* Enable or disable vehicle-follow mode. */
bool camera_controller_set_follow_mode(
    camera_controller_t *controller,
    camera3d_t *camera,
    bool enabled);

/* Switch between the camera path and manually controlled free-fly mode. */
bool camera_controller_set_demo_mode(
    camera_controller_t *controller,
    camera3d_t *camera,
    bool enabled);

/* Position the camera above and behind a vehicle, looking ahead along its route. */
bool camera_controller_follow_vehicle(
    camera_controller_t *controller,
    camera3d_t *camera,
    const geometry3d_vector3_t *vehicle_position,
    float vehicle_yaw_rad,
    float dt_seconds,
    bool snap_to_vehicle);

#endif /* CAMERA_CONTROLLER_H */
