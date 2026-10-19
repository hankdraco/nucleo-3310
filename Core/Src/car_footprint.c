#include "car_footprint.h"

#include <math.h>
#include <stddef.h>

static bool car_footprint_valid(const car_footprint_t *f)
{
    return f != NULL && isfinite(f->center_x) && isfinite(f->center_z) &&
           isfinite(f->yaw_rad) && isfinite(f->half_width) &&
           isfinite(f->half_length) && f->half_width > 0.0f &&
           f->half_length > 0.0f;
}

static bool car_footprint_separated_on_axis(
    const car_footprint_t *a,
    const car_footprint_t *b,
    float axis_x,
    float axis_z,
    float clearance)
{
    const float a_forward_x = sinf(a->yaw_rad);
    const float a_forward_z = cosf(a->yaw_rad);
    const float a_side_x = cosf(a->yaw_rad);
    const float a_side_z = -sinf(a->yaw_rad);
    const float b_forward_x = sinf(b->yaw_rad);
    const float b_forward_z = cosf(b->yaw_rad);
    const float b_side_x = cosf(b->yaw_rad);
    const float b_side_z = -sinf(b->yaw_rad);
    const float a_radius = a->half_length * fabsf(a_forward_x * axis_x + a_forward_z * axis_z) +
                           a->half_width * fabsf(a_side_x * axis_x + a_side_z * axis_z);
    const float b_radius = b->half_length * fabsf(b_forward_x * axis_x + b_forward_z * axis_z) +
                           b->half_width * fabsf(b_side_x * axis_x + b_side_z * axis_z);
    const float distance = fabsf((b->center_x - a->center_x) * axis_x +
                                 (b->center_z - a->center_z) * axis_z);
    return distance > a_radius + b_radius + clearance;
}

bool car_footprints_overlap(
    const car_footprint_t *a,
    const car_footprint_t *b,
    float clearance)
{
    if (!car_footprint_valid(a) || !car_footprint_valid(b) ||
        !isfinite(clearance) || clearance < 0.0f)
    {
        return false;
    }

    return !car_footprint_separated_on_axis(a, b, sinf(a->yaw_rad), cosf(a->yaw_rad), clearance) &&
           !car_footprint_separated_on_axis(a, b, cosf(a->yaw_rad), -sinf(a->yaw_rad), clearance) &&
           !car_footprint_separated_on_axis(a, b, sinf(b->yaw_rad), cosf(b->yaw_rad), clearance) &&
           !car_footprint_separated_on_axis(a, b, cosf(b->yaw_rad), -sinf(b->yaw_rad), clearance);
}
