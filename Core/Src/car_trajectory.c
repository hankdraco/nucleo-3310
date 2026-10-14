#include "car_trajectory.h"

#include <math.h>
#include <stddef.h>

#define CAR_TRAJECTORY_HALF_PI_RAD 1.57079632679489661923f

static bool car_trajectory_vec3_is_finite(
    const car_trajectory_vec3_t *value)
{
    return value != NULL &&
           isfinite(value->x) &&
           isfinite(value->y) &&
           isfinite(value->z);
}

static bool car_trajectory_pose_is_finite(
    const car_trajectory_pose_t *pose)
{
    return pose != NULL &&
           car_trajectory_vec3_is_finite(&pose->position) &&
           isfinite(pose->yaw_rad);
}

static float car_trajectory_direction_sign(
    car_turn_direction_t direction)
{
    return (direction == CAR_TURN_DIRECTION_RIGHT) ? 1.0f : -1.0f;
}

/* Re-evaluate from the captured start pose rather than accumulating deltas. */
static bool car_trajectory_calculate_turn_pose(
    const car_trajectory_t *trajectory,
    float progress_rad,
    car_trajectory_pose_t *pose)
{
    if (trajectory == NULL || pose == NULL ||
        !isfinite(progress_rad) ||
        progress_rad < 0.0f ||
        progress_rad > CAR_TRAJECTORY_HALF_PI_RAD)
    {
        return false;
    }

    const float signed_angle =
        car_trajectory_direction_sign(trajectory->turn_direction) *
        progress_rad;
    const float cosine = cosf(signed_angle);
    const float sine = sinf(signed_angle);

    const float offset_x =
        trajectory->turn_start_pose.position.x - trajectory->turn_pivot.x;
    const float offset_z =
        trajectory->turn_start_pose.position.z - trajectory->turn_pivot.z;

    car_trajectory_pose_t result = trajectory->turn_start_pose;

    /* Rotation about world +Y, matching the project's matrix4 Y rotation. */
    result.position.x = trajectory->turn_pivot.x +
                        cosine * offset_x + sine * offset_z;
    result.position.z = trajectory->turn_pivot.z -
                        sine * offset_x + cosine * offset_z;
    result.yaw_rad = trajectory->turn_start_pose.yaw_rad + signed_angle;

    if (!car_trajectory_pose_is_finite(&result))
    {
        return false;
    }

    *pose = result;
    return true;
}

static bool car_trajectory_move_straight(
    car_trajectory_t *trajectory,
    float dt_seconds)
{
    if (trajectory == NULL ||
        !isfinite(dt_seconds) || dt_seconds < 0.0f ||
        !isfinite(trajectory->speed_units_per_second) ||
        trajectory->speed_units_per_second <= 0.0f ||
        !car_trajectory_pose_is_finite(&trajectory->pose))
    {
        return false;
    }

    const float distance = trajectory->speed_units_per_second * dt_seconds;
    const float delta_x = sinf(trajectory->pose.yaw_rad) * distance;
    const float delta_z = cosf(trajectory->pose.yaw_rad) * distance;

    car_trajectory_pose_t result = trajectory->pose;
    result.position.x += delta_x;
    result.position.z += delta_z;

    if (!car_trajectory_pose_is_finite(&result))
    {
        return false;
    }

    trajectory->pose = result;
    return true;
}

bool car_trajectory_init(
    car_trajectory_t *trajectory,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second)
{
    if (trajectory == NULL ||
        !car_trajectory_pose_is_finite(initial_pose) ||
        !isfinite(speed_units_per_second) ||
        speed_units_per_second <= 0.0f)
    {
        return false;
    }

    car_trajectory_t result = {0};
    result.pose = *initial_pose;
    result.speed_units_per_second = speed_units_per_second;
    result.phase = CAR_TRAJECTORY_STRAIGHT;
    result.turn_start_pose = *initial_pose;
    result.turn_direction = CAR_TURN_DIRECTION_RIGHT;

    *trajectory = result;
    return true;
}

bool car_trajectory_begin_turn(
    car_trajectory_t *trajectory,
    const car_trajectory_vec3_t *pivot,
    car_turn_direction_t direction)
{
    if (trajectory == NULL ||
        !car_trajectory_vec3_is_finite(pivot) ||
        !car_trajectory_pose_is_finite(&trajectory->pose) ||
        !isfinite(trajectory->speed_units_per_second) ||
        trajectory->speed_units_per_second <= 0.0f ||
        trajectory->phase != CAR_TRAJECTORY_STRAIGHT ||
        (direction != CAR_TURN_DIRECTION_LEFT &&
         direction != CAR_TURN_DIRECTION_RIGHT))
    {
        return false;
    }

    const float offset_x = trajectory->pose.position.x - pivot->x;
    const float offset_z = trajectory->pose.position.z - pivot->z;
    const float radius = sqrtf(offset_x * offset_x + offset_z * offset_z);

    if (!isfinite(radius) || radius <= 0.0f)
    {
        return false;
    }

    const float angular_speed =
        trajectory->speed_units_per_second / radius;

    if (!isfinite(angular_speed) || angular_speed <= 0.0f)
    {
        return false;
    }

    car_trajectory_t result = *trajectory;
    result.phase = CAR_TRAJECTORY_TURNING;
    result.turn_start_pose = trajectory->pose;
    result.turn_pivot = *pivot;
    result.turn_radius = radius;
    result.turn_progress_rad = 0.0f;
    result.turn_angular_speed_rad_per_second = angular_speed;
    result.turn_direction = direction;

    *trajectory = result;
    return true;
}

bool car_trajectory_update(
    car_trajectory_t *trajectory,
    float dt_seconds)
{
    if (trajectory == NULL ||
        !isfinite(dt_seconds) || dt_seconds < 0.0f ||
        !car_trajectory_pose_is_finite(&trajectory->pose) ||
        !isfinite(trajectory->speed_units_per_second) ||
        trajectory->speed_units_per_second <= 0.0f ||
        (trajectory->phase != CAR_TRAJECTORY_STRAIGHT &&
         trajectory->phase != CAR_TRAJECTORY_TURNING))
    {
        return false;
    }

    if (trajectory->phase == CAR_TRAJECTORY_STRAIGHT)
    {
        car_trajectory_t result = *trajectory;

        if (!car_trajectory_move_straight(&result, dt_seconds))
        {
            return false;
        }

        *trajectory = result;
        return true;
    }

    if (!isfinite(trajectory->turn_radius) ||
        trajectory->turn_radius <= 0.0f ||
        !isfinite(trajectory->turn_progress_rad) ||
        trajectory->turn_progress_rad < 0.0f ||
        trajectory->turn_progress_rad > CAR_TRAJECTORY_HALF_PI_RAD ||
        !isfinite(trajectory->turn_angular_speed_rad_per_second) ||
        trajectory->turn_angular_speed_rad_per_second <= 0.0f ||
        (trajectory->turn_direction != CAR_TURN_DIRECTION_LEFT &&
         trajectory->turn_direction != CAR_TURN_DIRECTION_RIGHT) ||
        !car_trajectory_pose_is_finite(&trajectory->turn_start_pose) ||
        !car_trajectory_vec3_is_finite(&trajectory->turn_pivot))
    {
        return false;
    }

    /* Work on a copy so any detected invalid result leaves the caller's state intact. */
    car_trajectory_t result = *trajectory;
    float remaining_dt = dt_seconds;

    while (remaining_dt > 0.0f && result.phase == CAR_TRAJECTORY_TURNING)
    {
        const float remaining_angle =
            CAR_TRAJECTORY_HALF_PI_RAD - result.turn_progress_rad;
        const float time_to_finish =
            remaining_angle / result.turn_angular_speed_rad_per_second;

        if (!isfinite(time_to_finish) || time_to_finish < 0.0f)
        {
            return false;
        }

        const float requested_angle =
            result.turn_angular_speed_rad_per_second * remaining_dt;

        if (requested_angle < remaining_angle)
        {
            result.turn_progress_rad += requested_angle;

            if (!car_trajectory_calculate_turn_pose(
                    &result, result.turn_progress_rad, &result.pose))
            {
                return false;
            }

            remaining_dt = 0.0f;
        }
        else
        {
            /* Clamp to exactly 90 degrees, then retain any leftover time. */
            result.turn_progress_rad = CAR_TRAJECTORY_HALF_PI_RAD;

            if (!car_trajectory_calculate_turn_pose(
                    &result, CAR_TRAJECTORY_HALF_PI_RAD, &result.pose))
            {
                return false;
            }

            remaining_dt -= time_to_finish;
            result.phase = CAR_TRAJECTORY_STRAIGHT;
            result.turn_angular_speed_rad_per_second = 0.0f;
        }
    }

    if (result.phase == CAR_TRAJECTORY_STRAIGHT && remaining_dt > 0.0f)
    {
        if (!car_trajectory_move_straight(&result, remaining_dt))
        {
            return false;
        }
    }

    *trajectory = result;
    return true;
}
