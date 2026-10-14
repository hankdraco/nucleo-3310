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
typedef struct
{
    car_trajectory_t trajectory;
    car_modifier_state_t state;

    /* Route configuration for the demo's +Z approach and 90-degree turn. */
    car_trajectory_vec3_t turn_pivot;
    car_turn_direction_t turn_direction;
} car_modifier_context_t;

/* Initialize a car without a route FSM (straight trajectory only). */
bool car_modifier_context_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second);

/*
 * Initialize a car approaching a quarter-turn from +Z.
 * The pivot's Z coordinate is the point where the turn begins.
 * For a right turn the pivot must be to the +X side; for a left turn, -X.
 */
bool car_modifier_route_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second,
    const car_trajectory_vec3_t *turn_pivot,
    car_turn_direction_t turn_direction);

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
