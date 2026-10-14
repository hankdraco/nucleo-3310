#ifndef CAR_MODIFIER_H
#define CAR_MODIFIER_H

#include <stdbool.h>

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
    CAR_MODIFIER_STATE_DEPARTING
} car_modifier_state_t;

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

typedef struct
{
    car_trajectory_t trajectory;
    car_modifier_state_t state;
    float approach_distance_remaining;
    float turn_radius;
    car_maneuver_t maneuver;
} car_modifier_context_t;

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

#ifdef __cplusplus
}
#endif

#endif /* CAR_MODIFIER_H */
