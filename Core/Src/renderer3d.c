#include "renderer3d.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "matrix4.h"

static bool renderer3d_clip_segment_to_near_plane(
    const projection_perspective_t *projection,
    geometry3d_vertex_t *start,
    geometry3d_vertex_t *end,
    bool *visible)
{
    if (projection == NULL ||
        start == NULL ||
        end == NULL ||
        visible == NULL ||
        !isfinite(projection->near_plane) ||
        projection->near_plane <= 0.0f)
    {
        return false;
    }

    const float start_depth = start->z;
    const float end_depth = end->z;

    if (!isfinite(start_depth) || !isfinite(end_depth))
    {
        return false;
    }

    const bool start_inside =
        start_depth >= projection->near_plane;

    const bool end_inside =
        end_depth >= projection->near_plane;

    /*
     * Both endpoints are behind the near plane:
     * the entire segment is invisible.
     */
    if (!start_inside && !end_inside)
    {
        *visible = false;
        return true;
    }

    /*
     * Both endpoints are in front of the near plane.
     */
    if (start_inside && end_inside)
    {
        *visible = true;
        return true;
    }

    /*
     * The segment crosses the near plane.
     * Find the intersection by linear interpolation.
     */
    const float t =
        (projection->near_plane - start_depth) /
        (end_depth - start_depth);

    geometry3d_vertex_t intersection =
    {
        .x = start->x + t * (end->x - start->x),
        .y = start->y + t * (end->y - start->y),

        /*
         * Place the intersection just inside the visible
         * half-space to avoid floating-point boundary errors.
         */
        .z = projection->near_plane
    };

    if (!start_inside)
    {
        *start = intersection;
    }
    else
    {
        *end = intersection;
    }

    *visible = true;
    return true;
}

static bool renderer3d_clip_test(
    float p,
    float q,
    float *u_min,
    float *u_max)
{
    if (p == 0.0f)
    {
        return q >= 0.0f;
    }

    const float ratio = q / p;

    if (p < 0.0f)
    {
        if (ratio > *u_max)
        {
            return false;
        }

        if (ratio > *u_min)
        {
            *u_min = ratio;
        }
    }
    else
    {
        if (ratio < *u_min)
        {
            return false;
        }

        if (ratio < *u_max)
        {
            *u_max = ratio;
        }
    }

    return true;
}

static bool renderer3d_clip_line_to_canvas(
    float *x0,
    float *y0,
    float *x1,
    float *y1)
{
    /*
     * Liang-Barsky line clipping.
     * Clip against [0, width - 1] x [0, height - 1].
     */
    if (!isfinite(*x0) ||
        !isfinite(*y0) ||
        !isfinite(*x1) ||
        !isfinite(*y1))
    {
        return false;
    }

    const float original_x0 = *x0;
    const float original_y0 = *y0;

    const float dx = *x1 - *x0;
    const float dy = *y1 - *y0;

    const float xmin = 0.0f;
    const float ymin = 0.0f;
    const float xmax = (float)(DISPLAY_WIDTH - 1U);
    const float ymax = (float)(DISPLAY_HEIGHT - 1U);

    float u_min = 0.0f;
    float u_max = 1.0f;

    if (!renderer3d_clip_test(-dx, *x0 - xmin, &u_min, &u_max) ||
        !renderer3d_clip_test( dx, xmax - *x0, &u_min, &u_max) ||
        !renderer3d_clip_test(-dy, *y0 - ymin, &u_min, &u_max) ||
        !renderer3d_clip_test( dy, ymax - *y0, &u_min, &u_max))
    {
        return false;
    }

    *x0 = original_x0 + u_min * dx;
    *y0 = original_y0 + u_min * dy;

    *x1 = original_x0 + u_max * dx;
    *y1 = original_y0 + u_max * dy;

    /*
     * Clamp tiny floating-point boundary deviations.
     */
    if (*x0 < xmin) *x0 = xmin;
    if (*x0 > xmax) *x0 = xmax;
    if (*x1 < xmin) *x1 = xmin;
    if (*x1 > xmax) *x1 = xmax;

    if (*y0 < ymin) *y0 = ymin;
    if (*y0 > ymax) *y0 = ymax;
    if (*y1 < ymin) *y1 = ymin;
    if (*y1 > ymax) *y1 = ymax;

    return true;
}

bool renderer3d_draw_instance(
    display_t *canvas,
    const geometry3d_instance_t *instance,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection)
{
    if (canvas == NULL ||
        instance == NULL ||
        instance->model == NULL ||
        view_matrix == NULL ||
        projection == NULL)
    {
        return false;
    }

    if (!isfinite(projection->focal_length) ||
        !isfinite(projection->near_plane) ||
        !isfinite(projection->center_x) ||
        !isfinite(projection->center_y) ||
        projection->focal_length <= 0.0f ||
        projection->near_plane <= 0.0f)
    {
        return false;
    }

    const geometry3d_model_t *model = instance->model;

    if (!geometry3d_model_is_valid(model) ||
        model->vertex_count > RENDERER3D_MAX_VERTICES)
    {
        return false;
    }

    /*
     * Build the model transformation:
     *
     * M = T * Rz * Rx * Ry * S
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

    matrix4_t camera_transform;

    if (!matrix4_multiply(
            &camera_transform,
            view_matrix,
            &transform))
    {
        return false;
    }

    /*
     * Transform each model vertex once.
     */
    geometry3d_vertex_t transformed_vertices[
        RENDERER3D_MAX_VERTICES];

    for (size_t i = 0U; i < model->vertex_count; ++i)
    {
        if (!geometry3d_transform_vertex(
                &camera_transform,
                &model->vertices[i],
                &transformed_vertices[i]))
        {
            return false;
        }
    }

    /*
     * Clip, project and draw every edge independently.
     */
    for (size_t i = 0U; i < model->edge_count; ++i)
    {
        const geometry3d_edge_t *edge = &model->edges[i];

        geometry3d_vertex_t start =
            transformed_vertices[edge->start];

        geometry3d_vertex_t end =
            transformed_vertices[edge->end];

        bool visible;

        if (!renderer3d_clip_segment_to_near_plane(
                projection,
                &start,
                &end,
                &visible))
        {
            return false;
        }

        /*
         * An edge entirely behind the near plane is not an error.
         */
        if (!visible)
        {
            continue;
        }

        float x0;
        float y0;
        float x1;
        float y1;

        if (!projection_perspective_project_float(
                projection, &start, &x0, &y0) ||
            !projection_perspective_project_float(
                projection, &end, &x1, &y1))
        {
            return false;
        }

        /*
         * An edge can be in front of the camera but entirely
         * outside the viewport. Such an edge is simply skipped.
         */
        if (!renderer3d_clip_line_to_canvas(
                &x0, &y0, &x1, &y1))
        {
            continue;
        }

        if (!display_draw_line(
                canvas,
                (int16_t)lroundf(x0),
                (int16_t)lroundf(y0),
                (int16_t)lroundf(x1),
                (int16_t)lroundf(y1)))
        {
            return false;
        }
    }

    return true;
}