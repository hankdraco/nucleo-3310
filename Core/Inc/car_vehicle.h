#ifndef CAR_VEHICLE_H
#define CAR_VEHICLE_H

#include <stdbool.h>

#include "car_trajectory.h"
#include "geometry3d.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Couples one kinematic trajectory to its renderable geometry instance.
 * The scene owns the instance; this controller only retains a pointer to it.
 */
typedef struct
{
    car_trajectory_t trajectory;
    geometry3d_instance_t *instance;
    bool initialized;
} car_vehicle_t;

bool car_vehicle_init(
    car_vehicle_t *vehicle,
    geometry3d_instance_t *instance,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second);

/* Advances the trajectory and synchronizes its pose to the geometry instance. */
bool car_vehicle_update(
    car_vehicle_t *vehicle,
    float dt_seconds);

/* Begins a quarter-turn around the supplied world-space pivot. */
bool car_vehicle_begin_turn(
    car_vehicle_t *vehicle,
    const car_trajectory_vec3_t *pivot,
    car_turn_direction_t direction);

#ifdef __cplusplus
}
#endif

#endif /* CAR_VEHICLE_H */
