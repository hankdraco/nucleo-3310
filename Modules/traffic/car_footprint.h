#ifndef CAR_FOOTPRINT_H
#define CAR_FOOTPRINT_H

#include <stdbool.h>

/* Half extents of the current cube car proxy, in world units. */
#define CAR_FOOTPRINT_HALF_WIDTH_UNITS (1.25f)
#define CAR_FOOTPRINT_HALF_LENGTH_UNITS (2.25f)
#define CAR_FOOTPRINT_SPAWN_CLEARANCE_UNITS (0.5f)

typedef struct
{
    float center_x;
    float center_z;
    float yaw_rad;
    float half_width;
    float half_length;
} car_footprint_t;

/* Oriented rectangle overlap on the world's XZ plane. Touching counts as overlap. */
bool car_footprints_overlap(
    const car_footprint_t *a,
    const car_footprint_t *b,
    float clearance);

#endif /* CAR_FOOTPRINT_H */
