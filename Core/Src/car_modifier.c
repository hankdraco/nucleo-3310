#include "car_modifier.h"

#include <math.h>
#include <stddef.h>

bool car_modifier_context_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second)
{
    if (context == NULL)
    {
        return false;
    }

    return car_trajectory_init(
        &context->trajectory,
        initial_pose,
        speed_units_per_second);
}

bool car_modifier_update(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context)
{
    if (instance == NULL ||
        context == NULL ||
        !isfinite(dt_seconds) ||
        dt_seconds < 0.0f)
    {
        return false;
    }

    car_modifier_context_t *car_context = context;

    /*
     * Advance a copy first. If trajectory evaluation fails, the caller-owned
     * state and renderable transform remain unchanged.
     */
    car_trajectory_t next_trajectory = car_context->trajectory;

    if (!car_trajectory_update(&next_trajectory, dt_seconds))
    {
        return false;
    }

    const car_trajectory_pose_t *pose = &next_trajectory.pose;

    if (!isfinite(pose->position.x) ||
        !isfinite(pose->position.y) ||
        !isfinite(pose->position.z) ||
        !isfinite(pose->yaw_rad))
    {
        return false;
    }

    instance->transform.position.x = pose->position.x;
    instance->transform.position.y = pose->position.y;
    instance->transform.position.z = pose->position.z;
    instance->transform.rotation_rad.y = pose->yaw_rad;

    car_context->trajectory = next_trajectory;

    return true;
}
