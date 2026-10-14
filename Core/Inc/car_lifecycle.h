#ifndef CAR_LIFECYCLE_H
#define CAR_LIFECYCLE_H

#include <stdbool.h>

#include "car_trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Returns true when the vehicle's reference position is inside the city plan.
 *
 * The city plan is an axis-aligned rectangle centered at the world origin in
 * the XZ plane. The caller supplies its already-scaled half-extents in world
 * units. Positions on the boundary are considered inside.
 *
 * Invalid positions or invalid extents return false.
 */
bool car_lifecycle_is_inside_city_plan(
    const car_trajectory_pose_t *pose,
    float half_extent_x,
    float half_extent_z);

#ifdef __cplusplus
}
#endif

#endif /* CAR_LIFECYCLE_H */
