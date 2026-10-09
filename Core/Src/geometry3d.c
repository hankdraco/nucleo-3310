
#include <stddef.h>

#include "geometry3d.h"

bool geometry3d_transform_vertex(
    const matrix4_t *transform,
    const geometry3d_vertex_t *vertex,
    geometry3d_vertex_t *result)
{
    if (transform == NULL ||
        vertex == NULL ||
        result == NULL)
    {
        return false;
    }

    geometry3d_vertex_t transformed =
    {
        .x = transform->m[0][0] * vertex->x
           + transform->m[0][1] * vertex->y
           + transform->m[0][2] * vertex->z
           + transform->m[0][3],

        .y = transform->m[1][0] * vertex->x
           + transform->m[1][1] * vertex->y
           + transform->m[1][2] * vertex->z
           + transform->m[1][3],

        .z = transform->m[2][0] * vertex->x
           + transform->m[2][1] * vertex->y
           + transform->m[2][2] * vertex->z
           + transform->m[2][3]
    };

    *result = transformed;

    return true;
}