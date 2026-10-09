
#include "renderer3d.h"

#include <stddef.h>
#include <stdint.h>

#include "matrix4.h"

bool renderer3d_draw_instance(
    display_t *canvas,
    const geometry3d_instance_t *instance,
    const projection_perspective_t *projection)
{
    if (canvas == NULL ||
        instance == NULL ||
        instance->model == NULL ||
        projection == NULL)
    {
        return false;
    }

    const geometry3d_model_t *model = instance->model;

    if (model->vertices == NULL ||
        model->vertex_count == 0U ||
        model->vertex_count > RENDERER3D_MAX_VERTICES ||
        (model->edge_count > 0U && model->edges == NULL))
    {
        return false;
    }

    int16_t screen_x[RENDERER3D_MAX_VERTICES];
    int16_t screen_y[RENDERER3D_MAX_VERTICES];

    /*
     * Build the model transformation matrices.
     */
    matrix4_t rotation_x;
    matrix4_t rotation_y;
    matrix4_t rotation_z;

    matrix4_t scale;
    matrix4_t translation;

    matrix4_t rotation_yx;
    matrix4_t rotation_zyx;
    matrix4_t rotated_scaled;
    matrix4_t transform;

    matrix4_set_rotation_x(
        &rotation_x,
        instance->transform.rotation_rad.x);

    matrix4_set_rotation_y(
        &rotation_y,
        instance->transform.rotation_rad.y);

    matrix4_set_rotation_z(
        &rotation_z,
        instance->transform.rotation_rad.z);

    matrix4_set_scale(
        &scale,
        instance->transform.scale.x,
        instance->transform.scale.y,
        instance->transform.scale.z);

    matrix4_set_translation(
        &translation,
        instance->transform.position.x,
        instance->transform.position.y,
        instance->transform.position.z);

    /*
     * Column vectors:
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
            &transform,
            &translation,
            &rotated_scaled))
    {
        return false;
    }

    /*
     * Transform and project each vertex once.
     */
    for (size_t i = 0U; i < model->vertex_count; ++i)
    {
        geometry3d_vertex_t vertex;

        if (!geometry3d_transform_vertex(
                &transform,
                &model->vertices[i],
                &vertex))
        {
            return false;
        }

        if (!projection_perspective_project(
                projection,
                &vertex,
                &screen_x[i],
                &screen_y[i]))
        {
            return false;
        }
    }

    /*
     * Draw all edges using the projected vertices.
     */
    for (size_t i = 0U; i < model->edge_count; ++i)
    {
        const geometry3d_edge_t *edge = &model->edges[i];

        if (edge->start >= model->vertex_count ||
            edge->end >= model->vertex_count)
        {
            return false;
        }

        if (!display_draw_line(
                canvas,
                screen_x[edge->start],
                screen_y[edge->start],
                screen_x[edge->end],
                screen_y[edge->end]))
        {
            return false;
        }
    }

    return true;
}