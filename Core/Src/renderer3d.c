#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "renderer3d.h"
#include "matrix4.h"
#include "rasterizer3d.h"

#define RENDERER3D_EDGE_DEPTH_BIAS_RELATIVE (0.01f)

static bool renderer3d_transform_instance_vertices(
    const geometry3d_instance_t *instance,
    const matrix4_t *world_matrix,
    const matrix4_t *view_matrix,
    geometry3d_vertex_t transformed_vertices[
        RENDERER3D_MAX_VERTICES])
{
    if (instance == NULL ||
        instance->model == NULL ||
        world_matrix == NULL ||
        view_matrix == NULL ||
        transformed_vertices == NULL)
    {
        return false;
    }

    const geometry3d_model_t *model = instance->model;

    if (!geometry3d_model_is_valid(model) ||
        model->vertex_count > RENDERER3D_MAX_VERTICES)
    {
        return false;
    }

    matrix4_t camera_transform;

    /*
     * The scene has already computed model-to-world, including
     * every ancestor. Apply the world-to-camera transformation.
     */
    if (!matrix4_multiply(
            &camera_transform,
            view_matrix,
            world_matrix))
    {
        return false;
    }

    for (size_t i = 0U; i < model->vertex_count; ++i)
    {
        if (!geometry3d_transform_vertex(
                &camera_transform,
                &model->vertices[i],
                &transformed_vertices[i]))
        {
            return false;
        }

        if (!isfinite(transformed_vertices[i].x) ||
            !isfinite(transformed_vertices[i].y) ||
            !isfinite(transformed_vertices[i].z))
        {
            return false;
        }
    }

    return true;
}

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

static float renderer3d_screen_line_parameter(
    float px,
    float py,
    float x0,
    float y0,
    float x1,
    float y1)
{
    const float dx = x1 - x0;
    const float dy = y1 - y0;

    /*
     * Use the dominant axis to avoid division by a very
     * small component of the line direction.
     */
    if (fabsf(dx) >= fabsf(dy))
    {
        if (dx != 0.0f)
        {
            return (px - x0) / dx;
        }
    }

    if (dy != 0.0f)
    {
        return (py - y0) / dy;
    }

    return 0.0f;
}

static bool renderer3d_draw_depth_tested_line(
    display_t *canvas,
    const depth_buffer_t *depth_buffer,
    int16_t x0,
    int16_t y0,
    int16_t x1,
    int16_t y1,
    float inverse_depth0,
    float inverse_depth1)
{
    if (canvas == NULL ||
        depth_buffer == NULL ||
        !isfinite(inverse_depth0) ||
        !isfinite(inverse_depth1) ||
        inverse_depth0 <= 0.0f ||
        inverse_depth1 <= 0.0f)
    {
        return false;
    }

    int x = x0;
    int y = y0;

    const int absolute_dx =
        (x1 >= x0) ? (int)x1 - (int)x0 : (int)x0 - (int)x1;

    const int absolute_dy =
        (y1 >= y0) ? (int)y1 - (int)y0 : (int)y0 - (int)y1;

    const int dx = absolute_dx;
    const int dy = -absolute_dy;

    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;

    int error = dx + dy;
    const int total_steps =
        (absolute_dx > absolute_dy)
            ? absolute_dx
            : absolute_dy;

    int step = 0;

    for (;;)
    {
        const float t = (total_steps > 0)
            ? (float)step / (float)total_steps
            : 0.0f;

        /*
         * Perspective-correct interpolation along the projected
         * line: reciprocal depth is linear in screen space.
         */
        const float inverse_depth =
            inverse_depth0 +
            t * (inverse_depth1 - inverse_depth0);

        /*
        * Bias the edge slightly toward the camera during visibility
        * testing only. Do not modify the depth buffer itself.
        */
        const float biased_inverse_depth =
            inverse_depth *
            (1.0f + RENDERER3D_EDGE_DEPTH_BIAS_RELATIVE);

        if (depth_buffer_is_visible(
                depth_buffer,
                (int16_t)x,
                (int16_t)y,
                biased_inverse_depth))
        {
            if (!display_set_pixel(
                    canvas,
                    (uint8_t)x,
                    (uint8_t)y,
                    true))
            {
                return false;
            }
        }

        if (x == x1 && y == y1)
        {
            break;
        }

        const int error_twice = 2 * error;

        if (error_twice >= dy)
        {
            error += dy;
            x += sx;
        }

        if (error_twice <= dx)
        {
            error += dx;
            y += sy;
        }

        ++step;
    }

    return true;
}

bool renderer3d_rasterize_instance(
    depth_buffer_t *depth_buffer,
    const geometry3d_instance_t *instance,
    const matrix4_t *world_matrix,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection)
{
    if (depth_buffer == NULL ||
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

    geometry3d_vertex_t transformed_vertices[
        RENDERER3D_MAX_VERTICES];

    if (!renderer3d_transform_instance_vertices(
        instance,
        world_matrix,
        view_matrix,
        transformed_vertices))
    {
        return false;
    }

    /*
     * A model without triangles contributes no depth samples.
     * Its edges can still be rendered by the edge pass.
     */
    for (size_t i = 0U; i < model->triangle_count; ++i)
    {
        const geometry3d_triangle_t *triangle =
            &model->triangles[i];

        if (!rasterizer3d_rasterize_triangle(
                depth_buffer,
                projection,
                &transformed_vertices[triangle->a],
                &transformed_vertices[triangle->b],
                &transformed_vertices[triangle->c]))
        {
            return false;
        }
    }

    return true;
}

bool renderer3d_draw_instance(
    display_t *canvas,
    const depth_buffer_t *depth_buffer,
    const geometry3d_instance_t *instance,
    const matrix4_t *world_matrix,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection)
{
    if (canvas == NULL ||
        depth_buffer == NULL ||
        instance == NULL ||
        instance->model == NULL ||
        world_matrix == NULL ||
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

    /*
     * Transform all model vertices into camera space.
     *
     * world_matrix already includes the local transformations
     * of this instance and all its ancestors.
     */
    geometry3d_vertex_t transformed_vertices[
        RENDERER3D_MAX_VERTICES];

    if (!renderer3d_transform_instance_vertices(
            instance,
            world_matrix,
            view_matrix,
            transformed_vertices))
    {
        return false;
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
         * Preserve the original projected segment so we can
         * calculate the correct depth at clipped screen endpoints.
         */
        const float projected_x0 = x0;
        const float projected_y0 = y0;
        const float projected_x1 = x1;
        const float projected_y1 = y1;

        const float inverse_depth_start = 1.0f / start.z;
        const float inverse_depth_end = 1.0f / end.z;

        if (!isfinite(inverse_depth_start) ||
            !isfinite(inverse_depth_end) ||
            inverse_depth_start <= 0.0f ||
            inverse_depth_end <= 0.0f)
        {
            return false;
        }

        /*
         * Clip the projected line to the viewport.
         */
        if (!renderer3d_clip_line_to_canvas(
                &x0, &y0, &x1, &y1))
        {
            continue;
        }

        const int16_t screen_x0 = (int16_t)lroundf(x0);
        const int16_t screen_y0 = (int16_t)lroundf(y0);
        const int16_t screen_x1 = (int16_t)lroundf(x1);
        const int16_t screen_y1 = (int16_t)lroundf(y1);

        float clipped_inverse_depth0;
        float clipped_inverse_depth1;

        /*
         * If both endpoints round to the same pixel, use the
         * nearest endpoint depth for this single-pixel segment.
         */
        if (screen_x0 == screen_x1 &&
            screen_y0 == screen_y1)
        {
            clipped_inverse_depth0 =
                fmaxf(inverse_depth_start, inverse_depth_end);

            clipped_inverse_depth1 = clipped_inverse_depth0;
        }
        else
        {
            float clipped_t0 = renderer3d_screen_line_parameter(
                (float)screen_x0,
                (float)screen_y0,
                projected_x0,
                projected_y0,
                projected_x1,
                projected_y1);

            float clipped_t1 = renderer3d_screen_line_parameter(
                (float)screen_x1,
                (float)screen_y1,
                projected_x0,
                projected_y0,
                projected_x1,
                projected_y1);

            /*
             * Rounding the clipped coordinates can move a point
             * slightly beyond the original segment.
             */
            if (clipped_t0 < 0.0f) clipped_t0 = 0.0f;
            if (clipped_t0 > 1.0f) clipped_t0 = 1.0f;
            if (clipped_t1 < 0.0f) clipped_t1 = 0.0f;
            if (clipped_t1 > 1.0f) clipped_t1 = 1.0f;

            clipped_inverse_depth0 =
                inverse_depth_start +
                clipped_t0 *
                (inverse_depth_end - inverse_depth_start);

            clipped_inverse_depth1 =
                inverse_depth_start +
                clipped_t1 *
                (inverse_depth_end - inverse_depth_start);
        }

        if (!renderer3d_draw_depth_tested_line(
                canvas,
                depth_buffer,
                screen_x0,
                screen_y0,
                screen_x1,
                screen_y1,
                clipped_inverse_depth0,
                clipped_inverse_depth1))
        {
            return false;
        }
    }

    return true;
}