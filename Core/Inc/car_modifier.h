#ifndef CAR_MODIFIER_H
#define CAR_MODIFIER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "car_trajectory.h"
#include "geometry3d.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CAR_MODIFIER_STATE_FREE = 0,
    CAR_MODIFIER_STATE_APPROACH_TURN,
    CAR_MODIFIER_STATE_TURNING,
    CAR_MODIFIER_STATE_DEPARTING,
    CAR_MODIFIER_STATE_SPAWN
} car_modifier_state_t;

typedef enum
{
    CAR_MODIFIER_SAFETY_STATE_CLEAR = 0,
    CAR_MODIFIER_SAFETY_STATE_YIELDING,
    CAR_MODIFIER_SAFETY_STATE_STOPPED
} car_modifier_safety_state_t;

typedef struct car_modifier_context car_modifier_context_t;

/*
 * Shared world data for all cars. Each car context holds only a pointer to
 * this object; the random generator state is shared and deterministic.
 */
typedef struct
{
    float city_half_extent_x;
    float city_half_extent_z;
    float street_half_width;
    float lane_edge_offset;
    float minimum_speed_units_per_second;
    float maximum_speed_units_per_second;
    uint32_t random_state;
    geometry3d_instance_t *agent_instances;
    car_modifier_context_t *agent_contexts;
    size_t *agent_count;
} car_modifier_environment_t;

/*
 * Caller-owned runtime state for one car custom modifier.
 * The route FSM is deliberately separate from the trajectory's motion math.
 */
typedef enum
{
    CAR_MANEUVER_STRAIGHT = 0,
    CAR_MANEUVER_LEFT,
    CAR_MANEUVER_RIGHT
} car_maneuver_t;

struct car_modifier_context
{
    car_trajectory_t trajectory;
    car_modifier_state_t state;
    car_modifier_safety_state_t safety_state;
    float approach_distance_remaining;
    float turn_radius;
    car_maneuver_t maneuver;
    car_modifier_environment_t *environment;
};

/* Prepare a context for its first retryable SPAWN update. */
bool car_modifier_context_init_spawn(
    car_modifier_context_t *context,
    car_modifier_environment_t *environment);

/* Initialize a car without a route FSM (straight trajectory only). */
bool car_modifier_context_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second);

/*
 * Initialize a car approaching a maneuver from any heading.
 * distance_to_maneuver is the straight approach distance. Turns capture a
 * fixed pivot at the maneuver point; straight-through ignores turn_radius.
 */
bool car_modifier_route_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second,
    float distance_to_maneuver,
    car_maneuver_t maneuver,
    float turn_radius);

/*
 * Custom modifier callback. The route FSM consumes dt across state boundaries,
 * then synchronizes the trajectory pose into the renderable instance.
 * On failure, neither context nor instance transform is changed.
 */
bool car_modifier_update(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context);

/* First agent-update phase. The safety FSM is currently an inert scaffold. */
bool car_modifier_safety_update(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context);

#ifdef __cplusplus
}
#endif

#endif /* CAR_MODIFIER_H */
