#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "Modules/engine3d/geometry3d.h"

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
     * Vertex indices are uint16_t.
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

    if (model->triangle_count > 0U &&
        model->triangles == NULL)
    {
        return false;
    }

    /*
     * Validate edge vertex indices.
     */
    for (size_t i = 0U; i < model->edge_count; ++i)
    {
        const geometry3d_edge_t *edge = &model->edges[i];

        if (edge->start >= model->vertex_count ||
            edge->end >= model->vertex_count)
        {
            return false;
        }
    }

    /*
     * Validate triangle vertex indices.
     */
    for (size_t i = 0U; i < model->triangle_count; ++i)
    {
        const geometry3d_triangle_t *triangle =
            &model->triangles[i];

        if (triangle->a >= model->vertex_count ||
            triangle->b >= model->vertex_count ||
            triangle->c >= model->vertex_count)
        {
            return false;
        }
    }

    return true;
}

bool geometry3d_build_transform_matrix(
    const geometry3d_transform_t *transform,
    matrix4_t *result)
{
    if (transform == NULL || result == NULL)
    {
        return false;
    }

    if (!isfinite(transform->position.x) ||
        !isfinite(transform->position.y) ||
        !isfinite(transform->position.z) ||
        !isfinite(transform->rotation_rad.x) ||
        !isfinite(transform->rotation_rad.y) ||
        !isfinite(transform->rotation_rad.z) ||
        !isfinite(transform->scale.x) ||
        !isfinite(transform->scale.y) ||
        !isfinite(transform->scale.z))
    {
        return false;
    }

    matrix4_t rotation_x;
    matrix4_t rotation_y;
    matrix4_t rotation_z;
    matrix4_t scale;
    matrix4_t translation;

    matrix4_t rotation_yx;
    matrix4_t rotation_zyx;
    matrix4_t rotated_scaled;
    matrix4_t temporary;

    matrix4_set_rotation_x(
        &rotation_x,
        transform->rotation_rad.x);

    matrix4_set_rotation_y(
        &rotation_y,
        transform->rotation_rad.y);

    matrix4_set_rotation_z(
        &rotation_z,
        transform->rotation_rad.z);

    matrix4_set_scale(
        &scale,
        transform->scale.x,
        transform->scale.y,
        transform->scale.z);

    matrix4_set_translation(
        &translation,
        transform->position.x,
        transform->position.y,
        transform->position.z);

    /*
     * M = T * Rz * Rx * Ry * S
     */
    if (!matrix4_multiply(
            &rotation_yx,
            &rotation_x,
            &rotation_y))
    {
        return false;
    }

    if (!matrix4_multiply(
            &rotation_zyx,
            &rotation_z,
            &rotation_yx))
    {
        return false;
    }

    if (!matrix4_multiply(
            &rotated_scaled,
            &rotation_zyx,
            &scale))
    {
        return false;
    }

    if (!matrix4_multiply(
            &temporary,
            &translation,
            &rotated_scaled))
    {
        return false;
    }

    for (size_t row = 0U; row < MATRIX4_SIZE; ++row)
    {
        for (size_t column = 0U;
             column < MATRIX4_SIZE;
             ++column)
        {
            if (!isfinite(temporary.m[row][column]))
            {
                return false;
            }
        }
    }

    *result = temporary;

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