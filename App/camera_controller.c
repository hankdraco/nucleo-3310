#include "App/camera_controller.h"

#include <math.h>
#include <stdint.h>

#define CAMERA_CONTROLLER_TWO_PI 6.28318530717958647692f
#define CAMERA_CONTROLLER_FOLLOW_DISTANCE_UNITS (8.0f)
#define CAMERA_CONTROLLER_FOLLOW_HEIGHT_UNITS (6.0f)
#define CAMERA_CONTROLLER_LOOK_AHEAD_UNITS (10.0f)
#define CAMERA_CONTROLLER_FOLLOW_SMOOTHING (6.0f)

static float wrap_yaw(float yaw_rad)
{
    return remainderf(yaw_rad, CAMERA_CONTROLLER_TWO_PI);
}

static float clamp_pitch(float pitch_rad)
{
    if (pitch_rad > CAMERA_CONTROLLER_MAX_PITCH_RAD)
    {
        return CAMERA_CONTROLLER_MAX_PITCH_RAD;
    }

    if (pitch_rad < -CAMERA_CONTROLLER_MAX_PITCH_RAD)
    {
        return -CAMERA_CONTROLLER_MAX_PITCH_RAD;
    }

    return pitch_rad;
}

static bool is_finite_vector(const geometry3d_vector3_t *vector)
{
    return vector != NULL &&
           isfinite(vector->x) &&
           isfinite(vector->y) &&
           isfinite(vector->z);
}

bool camera_controller_init(
    camera_controller_t *controller,
    camera3d_t *camera)
{
    if (controller == NULL || camera == NULL)
    {
        return false;
    }

    if (!is_finite_vector(&camera->position))
    {
        return false;
    }

    /*
     * Convert look-at orientation into manual orientation
     * before the controller takes ownership of the camera.
     */
    if (camera->target_enabled)
    {
        if (!camera3d_clear_target(camera))
        {
            return false;
        }
    }

    if (!isfinite(camera->rotation_rad.x) ||
        !isfinite(camera->rotation_rad.y))
    {
        return false;
    }

    camera_controller_t initialized_controller =
    {
        .yaw_rad = wrap_yaw(camera->rotation_rad.y),
        .pitch_rad = clamp_pitch(camera->rotation_rad.x),
        .move_speed_units_per_second =
            CAMERA_CONTROLLER_DEFAULT_MOVE_SPEED,
        .mouse_sensitivity_rad_per_count =
            CAMERA_CONTROLLER_DEFAULT_MOUSE_SENSITIVITY,
        .initialized = true
    };

    /*
     * This controller supports yaw/pitch, not camera roll.
     * Our current look-at camera already has zero roll.
     */
    camera->rotation_rad.x = initialized_controller.pitch_rad;
    camera->rotation_rad.y = initialized_controller.yaw_rad;
    camera->rotation_rad.z = 0.0f;

    *controller = initialized_controller;

    return true;
}

bool camera_controller_update(
    camera_controller_t *controller,
    camera3d_t *camera,
    const input_snapshot_t *input,
    float dt_seconds)
{
    if (controller == NULL ||
        camera == NULL ||
        input == NULL ||
        !controller->initialized)
    {
        return false;
    }

    if (controller->follow_mode || controller->demo_mode)
    {
        return isfinite(dt_seconds) && dt_seconds >= 0.0f;
    }

    if (camera->target_enabled)
    {
        /*
         * Once initialized, the controller requires
         * manual camera orientation.
         */
        return false;
    }

    if (!isfinite(dt_seconds) ||
        dt_seconds < 0.0f ||
        !is_finite_vector(&camera->position))
    {
        return false;
    }

    if (!isfinite(controller->yaw_rad) ||
        !isfinite(controller->pitch_rad) ||
        !isfinite(controller->move_speed_units_per_second) ||
        !isfinite(controller->mouse_sensitivity_rad_per_count) ||
        controller->move_speed_units_per_second < 0.0f ||
        controller->mouse_sensitivity_rad_per_count < 0.0f)
    {
        return false;
    }

    /*
     * Work on local copies first. If a calculation fails,
     * neither the controller nor the camera is partially
     * updated.
     */
    float yaw_rad = wrap_yaw(
        controller->yaw_rad +
        (float)input->mouse_dx *
        controller->mouse_sensitivity_rad_per_count);

    float pitch_rad = clamp_pitch(
        controller->pitch_rad +
        (float)input->mouse_dy *
        controller->mouse_sensitivity_rad_per_count);

    if (!isfinite(yaw_rad) || !isfinite(pitch_rad))
    {
        return false;
    }

    const int forward_axis =
        ((input->key_mask & INPUT_KEY_FORWARD) != 0U ? 1 : 0) -
        ((input->key_mask & INPUT_KEY_BACKWARD) != 0U ? 1 : 0);

    const int strafe_axis =
        ((input->key_mask & INPUT_KEY_STRAFE_RIGHT) != 0U ? 1 : 0) -
        ((input->key_mask & INPUT_KEY_STRAFE_LEFT) != 0U ? 1 : 0);

    const int vertical_axis =
        ((input->key_mask & INPUT_KEY_MOVE_UP) != 0U ? 1 : 0) -
        ((input->key_mask & INPUT_KEY_MOVE_DOWN) != 0U ? 1 : 0);

    /*
     * Camera forward vector derived from yaw and pitch.
     *
     * The camera looks along +Z when yaw and pitch
     * are both zero.
     */
    const float cos_pitch = cosf(pitch_rad);
    const float sin_yaw = sinf(yaw_rad);
    const float cos_yaw = cosf(yaw_rad);
    const float sin_pitch = sinf(pitch_rad);

    const float forward_x = sin_yaw * cos_pitch;
    const float forward_y = -sin_pitch;
    const float forward_z = cos_yaw * cos_pitch;

    /*
     * Horizontal strafe-right vector.
     */
    const float right_x = cos_yaw;
    const float right_z = -sin_yaw;

    float move_x =
        (float)forward_axis * forward_x +
        (float)strafe_axis * right_x;

    float move_y =
        (float)forward_axis * forward_y +
        (float)vertical_axis;

    float move_z =
        (float)forward_axis * forward_z +
        (float)strafe_axis * right_z;

    /*
     * Normalize combined movement so pressing two
     * directions diagonally does not increase speed.
     */
    const float move_length = sqrtf(
        move_x * move_x +
        move_y * move_y +
        move_z * move_z);

    if (!isfinite(move_length))
    {
        return false;
    }

    geometry3d_vector3_t new_position = camera->position;

    if (move_length > 0.0f && dt_seconds > 0.0f)
    {
        const float distance =
            controller->move_speed_units_per_second *
            dt_seconds;

        if (!isfinite(distance))
        {
            return false;
        }

        const float scale = distance / move_length;

        move_x *= scale;
        move_y *= scale;
        move_z *= scale;

        new_position.x += move_x;
        new_position.y += move_y;
        new_position.z += move_z;

        if (!is_finite_vector(&new_position))
        {
            return false;
        }
    }

    /*
     * Commit the update only after all calculations
     * have passed validation.
     */
    controller->yaw_rad = yaw_rad;
    controller->pitch_rad = pitch_rad;

    camera->position = new_position;
    camera->rotation_rad.x = pitch_rad;
    camera->rotation_rad.y = yaw_rad;
    camera->rotation_rad.z = 0.0f;

    return true;
}

bool camera_controller_set_follow_mode(
    camera_controller_t *controller,
    camera3d_t *camera,
    bool enabled)
{
    if (controller == NULL || camera == NULL || !controller->initialized)
    {
        return false;
    }

    if (enabled == controller->follow_mode) return true;

    if (enabled)
    {
        if (camera->target_enabled && !camera3d_clear_target(camera))
        {
            return false;
        }
        controller->demo_mode = false;
        controller->follow_mode = true;
        controller->follow_camera_initialized = false;
        return true;
    }

    if (camera->target_enabled && !camera3d_clear_target(camera))
    {
        return false;
    }
    controller->follow_mode = false;
    controller->demo_mode = false;
    controller->follow_camera_initialized = false;
    controller->yaw_rad = wrap_yaw(camera->rotation_rad.y);
    controller->pitch_rad = clamp_pitch(camera->rotation_rad.x);
    return true;
}

bool camera_controller_set_demo_mode(
    camera_controller_t *controller,
    camera3d_t *camera,
    bool enabled)
{
    if (controller == NULL || camera == NULL || !controller->initialized)
    {
        return false;
    }

    if (enabled == controller->demo_mode)
    {
        return true;
    }

    if (enabled)
    {
        controller->follow_mode = false;
        controller->demo_mode = true;
        controller->follow_camera_initialized = false;
        return true;
    }

    if (camera->target_enabled && !camera3d_clear_target(camera))
    {
        return false;
    }

    controller->demo_mode = false;
    controller->yaw_rad = wrap_yaw(camera->rotation_rad.y);
    controller->pitch_rad = clamp_pitch(camera->rotation_rad.x);
    return true;
}

bool camera_controller_follow_vehicle(
    camera_controller_t *controller,
    camera3d_t *camera,
    const geometry3d_vector3_t *vehicle_position,
    float vehicle_yaw_rad,
    float dt_seconds,
    bool snap_to_vehicle)
{
    if (controller == NULL || camera == NULL || vehicle_position == NULL ||
        !controller->initialized || !controller->follow_mode ||
        !is_finite_vector(vehicle_position) || !isfinite(vehicle_yaw_rad) ||
        !isfinite(dt_seconds) || dt_seconds < 0.0f)
    {
        return false;
    }

    const float forward_x = sinf(vehicle_yaw_rad);
    const float forward_z = cosf(vehicle_yaw_rad);
    geometry3d_vector3_t desired_position =
    {
        .x = vehicle_position->x -
             forward_x * CAMERA_CONTROLLER_FOLLOW_DISTANCE_UNITS,
        .y = vehicle_position->y + CAMERA_CONTROLLER_FOLLOW_HEIGHT_UNITS,
        .z = vehicle_position->z -
             forward_z * CAMERA_CONTROLLER_FOLLOW_DISTANCE_UNITS
    };
    geometry3d_vector3_t target =
    {
        .x = vehicle_position->x +
             forward_x * CAMERA_CONTROLLER_LOOK_AHEAD_UNITS,
        .y = vehicle_position->y + 1.0f,
        .z = vehicle_position->z +
             forward_z * CAMERA_CONTROLLER_LOOK_AHEAD_UNITS
    };

    if (!is_finite_vector(&desired_position) || !is_finite_vector(&target))
    {
        return false;
    }

    if (snap_to_vehicle || !controller->follow_camera_initialized)
    {
        camera->position = desired_position;
    }
    else if (dt_seconds > 0.0f)
    {
        float blend = CAMERA_CONTROLLER_FOLLOW_SMOOTHING * dt_seconds;
        if (blend > 1.0f) blend = 1.0f;
        camera->position.x += (desired_position.x - camera->position.x) * blend;
        camera->position.y += (desired_position.y - camera->position.y) * blend;
        camera->position.z += (desired_position.z - camera->position.z) * blend;
    }

    if (!camera3d_set_target(camera, &target)) return false;
    controller->follow_camera_initialized = true;
    controller->yaw_rad = camera->rotation_rad.y;
    controller->pitch_rad = camera->rotation_rad.x;
    return true;
}
