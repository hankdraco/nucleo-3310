#ifndef CAR_TRAJECTORY_INSTANCE_H
#define CAR_TRAJECTORY_INSTANCE_H

#include <stdbool.h>

#include "car_trajectory.h"
#include "geometry3d.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Copies a trajectory's current pose into a geometry instance transform.
 * The trajectory remains the owner of motion; the instance is its renderable
 * representation. Invalid input leaves the instance unchanged.
 */
bool car_trajectory_apply_pose(
    const car_trajectory_t *trajectory,
    geometry3d_instance_t *instance);

#ifdef __cplusplus
}
#endif

#endif /* CAR_TRAJECTORY_INSTANCE_H */
