#include "car_trajectory_instance.h"

#include <math.h>
#include <stddef.h>

bool car_trajectory_apply_pose(
    const car_trajectory_t *trajectory,
    geometry3d_instance_t *instance)
{
    if (trajectory == NULL || instance == NULL ||
        !isfinite(trajectory->pose.position.x) ||
        !isfinite(trajectory->pose.position.y) ||
        !isfinite(trajectory->pose.position.z) ||
        !isfinite(trajectory->pose.yaw_rad))
    {
        return false;
    }

    instance->transform.position.x = trajectory->pose.position.x;
    instance->transform.position.y = trajectory->pose.position.y;
    instance->transform.position.z = trajectory->pose.position.z;
    instance->transform.rotation_rad.y = trajectory->pose.yaw_rad;

    return true;
}
