
#include "geometry3d.h"

#include <math.h>

geometry3d_vertex_t geometry3d_rotate_y(
    const geometry3d_vertex_t *vertex,
    float angle_rad)
{
    const float sin_angle = sinf(angle_rad);
    const float cos_angle = cosf(angle_rad);

    geometry3d_vertex_t result =
    {
        .x = vertex->x * cos_angle + vertex->z * sin_angle,
        .y = vertex->y,
        .z = -vertex->x * sin_angle + vertex->z * cos_angle
    };

    return result;
}

geometry3d_vertex_t geometry3d_rotate_x(
    const geometry3d_vertex_t *vertex,
    float angle_rad)
{
    const float sin_angle = sinf(angle_rad);
    const float cos_angle = cosf(angle_rad);

    geometry3d_vertex_t result =
    {
        .x = vertex->x,
        .y = vertex->y * cos_angle - vertex->z * sin_angle,
        .z = vertex->y * sin_angle + vertex->z * cos_angle
    };

    return result;
}