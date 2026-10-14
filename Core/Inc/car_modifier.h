#ifndef CAR_MODIFIER_H
#define CAR_MODIFIER_H

#include <stdbool.h>

#include "car_trajectory.h"
#include "geometry3d.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Caller-owned runtime state for one car custom modifier.
 *
 * This context intentionally owns behavior state rather than a pointer to a
 * geometry instance. The scene copies instances, while this context remains
 * at a stable caller-provided address. Future car FSM state can be added here.
 */
typedef struct
{
    car_trajectory_t trajectory;
} car_modifier_context_t;

/* Initialize the persistent state used by the car custom modifier. */
bool car_modifier_context_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second);

/*
 * Custom modifier callback: advance the trajectory and synchronize the
 * resulting position and world-Y yaw into the renderable instance.
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
