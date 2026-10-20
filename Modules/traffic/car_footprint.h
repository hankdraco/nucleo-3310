#ifndef CAR_FOOTPRINT_H
#define CAR_FOOTPRINT_H

#include <stdbool.h>

/* Half extents of the current cube car proxy, in world units. */
#define CAR_FOOTPRINT_HALF_WIDTH_UNITS (1.25f) /**< Half-width of the current car proxy, in world units. */
#define CAR_FOOTPRINT_HALF_LENGTH_UNITS (2.25f) /**< Half-length of the current car proxy, in world units. */
#define CAR_FOOTPRINT_SPAWN_CLEARANCE_UNITS (0.5f) /**< Extra gap required when validating a spawn point. */

/** Oriented rectangular car footprint projected onto the XZ ground plane. */
typedef struct
{
    float center_x; /**< Footprint center X coordinate. */
    float center_z; /**< Footprint center Z coordinate. */
    float yaw_rad; /**< Heading around the world Y axis, in radians. */
    float half_width; /**< Half extent along the vehicle's local width axis. */
    float half_length; /**< Half extent along the vehicle's local forward axis. */
} car_footprint_t;

/** Test overlap between two oriented rectangles using the separating-axis theorem.
 * Touching edges count as overlap.
 * @param a First footprint.
 * @param b Second footprint.
 * @param clearance Additional separation required between footprints.
 * @return true if the rectangles overlap or are within clearance.
 */
bool car_footprints_overlap(
    const car_footprint_t *a,
    const car_footprint_t *b,
    float clearance);

#endif /* CAR_FOOTPRINT_H */
