#include <math.h>
#include <stddef.h>

#include "car_lifecycle.h"

bool car_lifecycle_is_inside_city_plan(
    const car_trajectory_pose_t *pose,
    float half_extent_x,
    float half_extent_z)
{
    if (pose == NULL ||
        !isfinite(pose->position.x) ||
        !isfinite(pose->position.z) ||
        !isfinite(half_extent_x) ||
        !isfinite(half_extent_z) ||
        half_extent_x <= 0.0f ||
        half_extent_z <= 0.0f)
    {
        return false;
    }

    return pose->position.x >= -half_extent_x &&
           pose->position.x <= half_extent_x &&
           pose->position.z >= -half_extent_z &&
           pose->position.z <= half_extent_z;
}
