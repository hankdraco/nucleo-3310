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

/** Position vector used by the portable vehicle trajectory model. */
typedef struct
{
    float x; /**< World X coordinate. */
    float y; /**< World Y coordinate, preserved during planar motion. */
    float z; /**< World Z coordinate. */
} car_trajectory_vec3_t;

/** Vehicle position and heading at one instant. */
typedef struct
{
    car_trajectory_vec3_t position; /**< Vehicle reference position. */
    float yaw_rad; /**< Heading around +Y; zero points along +Z. */
} car_trajectory_pose_t;

/** Current motion mode of the trajectory. */
typedef enum
{
    CAR_TRAJECTORY_STRAIGHT = 0, /**< Moving along a straight segment. */
    CAR_TRAJECTORY_TURNING = 1 /**< Following a circular turn arc. */
} car_trajectory_phase_t;

/** Direction of a quarter-turn around its pivot. */
typedef enum
{
    CAR_TURN_DIRECTION_LEFT = -1, /**< Turn counterclockwise when viewed from above. */
    CAR_TURN_DIRECTION_RIGHT = 1 /**< Turn clockwise when viewed from above. */
} car_turn_direction_t;

/** Mutable kinematic state for a car moving straight or along a circular turn. */
typedef struct
{
    car_trajectory_pose_t pose; /**< Current position and heading. */
    float speed_units_per_second; /**< Nonnegative linear speed. */
    car_trajectory_phase_t phase; /**< Straight travel or active turn. */

    car_trajectory_pose_t turn_start_pose; /**< Pose at the beginning of the active turn. */
    car_trajectory_vec3_t turn_pivot; /**< Fixed center of the turn arc. */
    float turn_radius; /**< Arc radius measured from the vehicle reference point. */
    float turn_progress_rad; /**< Angular progress since the turn began. */
    float turn_angular_speed_rad_per_second; /**< Signed angular velocity of the active turn. */
    car_turn_direction_t turn_direction; /**< Left or right turn direction. */
} car_trajectory_t;

/** Initialize straight motion at a constant positive speed.
 * @param trajectory State to initialize.
 * @param initial_pose Starting position and heading.
 * @param speed_units_per_second Initial linear speed; must be positive.
 * @return true on success; false for invalid arguments or speed.
 */
bool car_trajectory_init(
    car_trajectory_t *trajectory,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second);

/** Start a quarter-turn around a vertical pivot.
 * Radius is measured from the current reference point; the initial pose must
 * be tangent to the arc. Linear speed is preserved by angular_speed = speed/radius.
 * @param trajectory Initialized trajectory to modify.
 * @param pivot Center of the turn circle.
 * @param direction Left or right turn direction.
 * @return true on success; false if state or pivot is invalid.
 */
bool car_trajectory_begin_turn(
    car_trajectory_t *trajectory,
    const car_trajectory_vec3_t *pivot,
    car_turn_direction_t direction);

/** Advance motion by a time step, continuing straight after a completed turn.
 * Any time remaining after the quarter-turn is applied to the new straight path.
 * Invalid input leaves the state unchanged.
 * @param trajectory Trajectory to advance.
 * @param dt_seconds Elapsed time in seconds; must be finite and nonnegative.
 * @return true on success; false for invalid state or time step.
 */
bool car_trajectory_update(
    car_trajectory_t *trajectory,
    float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* CAR_TRAJECTORY_H */
