#ifndef CAR_TRAJECTORY_H
#define CAR_TRAJECTORY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Portable kinematic model for one vehicle.
 *
 * The vehicle moves in the world XZ plane. Y is preserved, yaw is rotation
 * around world +Y, and yaw == 0 means forward along local/world +Z.
 * Positive yaw turns the vehicle from +Z toward +X.
 *
 * This module knows nothing about the scene, rendering, STM32, or randomness.
 */

typedef struct
{
    float x;
    float y;
    float z;
} car_trajectory_vec3_t;

typedef struct
{
    car_trajectory_vec3_t position;
    float yaw_rad;
} car_trajectory_pose_t;

typedef enum
{
    CAR_TRAJECTORY_STRAIGHT = 0,
    CAR_TRAJECTORY_TURNING = 1
} car_trajectory_phase_t;

typedef enum
{
    CAR_TURN_DIRECTION_LEFT = -1,
    CAR_TURN_DIRECTION_RIGHT = 1
} car_turn_direction_t;

typedef struct
{
    car_trajectory_pose_t pose;
    float speed_units_per_second;
    car_trajectory_phase_t phase;

    /* Snapshots used to evaluate the circular arc without accumulated drift. */
    car_trajectory_pose_t turn_start_pose;
    car_trajectory_vec3_t turn_pivot;
    float turn_radius;
    float turn_progress_rad;
    float turn_angular_speed_rad_per_second;
    car_turn_direction_t turn_direction;
} car_trajectory_t;

/* Initializes a vehicle travelling straight at a constant positive speed. */
bool car_trajectory_init(
    car_trajectory_t *trajectory,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second);

/*
 * Starts a quarter-turn around a vertical pivot.
 *
 * The radius is calculated from the vehicle reference point to the pivot in
 * the XZ plane. Therefore angular_speed = linear_speed / radius, preserving
 * the same linear speed as straight travel. The initial pose must already be
 * tangent to the chosen circle; street/lane geometry is responsible for that.
 */
bool car_trajectory_begin_turn(
    car_trajectory_t *trajectory,
    const car_trajectory_vec3_t *pivot,
    car_turn_direction_t direction);

/*
 * Advances the trajectory by dt_seconds.
 * If the turn completes during this update, any remaining time is applied to
 * straight travel in the new direction. Invalid input leaves the state intact.
 */
bool car_trajectory_update(
    car_trajectory_t *trajectory,
    float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* CAR_TRAJECTORY_H */
