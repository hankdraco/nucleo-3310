#include "car_vehicle.h"

#include <stddef.h>

#include "car_trajectory_instance.h"

bool car_vehicle_init(
    car_vehicle_t *vehicle,
    geometry3d_instance_t *instance,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second)
{
    if (vehicle == NULL || instance == NULL || initial_pose == NULL)
    {
        return false;
    }

    car_vehicle_t candidate = {0};
    geometry3d_instance_t instance_candidate = *instance;

    if (!car_trajectory_init(
            &candidate.trajectory,
            initial_pose,
            speed_units_per_second) ||
        !car_trajectory_apply_pose(
            &candidate.trajectory,
            &instance_candidate))
    {
        return false;
    }

    candidate.instance = instance;
    candidate.initialized = true;

    *instance = instance_candidate;
    *vehicle = candidate;

    return true;
}

bool car_vehicle_update(
    car_vehicle_t *vehicle,
    float dt_seconds)
{
    if (vehicle == NULL ||
        !vehicle->initialized ||
        vehicle->instance == NULL)
    {
        return false;
    }

    car_trajectory_t trajectory_candidate = vehicle->trajectory;
    geometry3d_instance_t instance_candidate = *vehicle->instance;

    if (!car_trajectory_update(&trajectory_candidate, dt_seconds) ||
        !car_trajectory_apply_pose(
            &trajectory_candidate,
            &instance_candidate))
    {
        return false;
    }

    *vehicle->instance = instance_candidate;
    vehicle->trajectory = trajectory_candidate;

    return true;
}

bool car_vehicle_begin_turn(
    car_vehicle_t *vehicle,
    const car_trajectory_vec3_t *pivot,
    car_turn_direction_t direction)
{
    if (vehicle == NULL ||
        !vehicle->initialized ||
        vehicle->instance == NULL)
    {
        return false;
    }

    return car_trajectory_begin_turn(
        &vehicle->trajectory,
        pivot,
        direction);
}
