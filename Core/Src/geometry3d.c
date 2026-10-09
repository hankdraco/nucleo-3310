
#include <stddef.h>

#include "geometry3d.h"


bool geometry3d_model_is_valid(
    const geometry3d_model_t *model)
{
    if (model == NULL ||
        model->vertices == NULL ||
        model->vertex_count == 0U)
    {
        return false;
    }

    /*
     * Edge indices are uint16_t.
     */
    if (model->vertex_count >
        (size_t)UINT16_MAX + 1U)
    {
        return false;
    }

    if (model->edge_count > 0U &&
        model->edges == NULL)
    {
        return false;
    }

    for (size_t i = 0U; i < model->edge_count; ++i)
    {
        const geometry3d_edge_t *edge = &model->edges[i];

        if (edge->start >= model->vertex_count ||
            edge->end >= model->vertex_count)
        {
            return false;
        }
    }

    return true;
}

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