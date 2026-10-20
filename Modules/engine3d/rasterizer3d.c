#include "Modules/engine3d/rasterizer3d.h"

#include <math.h>
#include <stddef.h>

#define RASTERIZER3D_INSIDE_EPSILON (1.0e-5f)

typedef struct
{
    float x;
    float y;
    float inverse_depth;
} rasterizer3d_screen_vertex_t;

static bool rasterizer3d_vertex_is_finite(
    const geometry3d_vertex_t *vertex)
{
    return vertex != NULL &&
           isfinite(vertex->x) &&
           isfinite(vertex->y) &&
           isfinite(vertex->z);
}

/*
 * Clip a triangle against camera-space z >= near_plane.
 *
 * A clipped triangle can produce a polygon with three or four
 * vertices. The output retains the polygon's original winding.
 */
static bool rasterizer3d_clip_triangle(
    const geometry3d_vertex_t input[3],
    float near_plane,
    geometry3d_vertex_t output[4],
    size_t *output_count)
{
    if (input == NULL ||
        output == NULL ||
        output_count == NULL ||
        !isfinite(near_plane) ||
        near_plane <= 0.0f)
    {
        return false;
    }

    size_t count = 0U;

    for (size_t i = 0U; i < 3U; ++i)
    {
        const geometry3d_vertex_t *start = &input[i];
        const geometry3d_vertex_t *end = &input[(i + 1U) % 3U];

        const bool start_inside = start->z >= near_plane;
        const bool end_inside = end->z >= near_plane;

        if (start_inside && end_inside)
        {
            /* Both endpoints are inside: retain the end vertex. */
            output[count++] = *end;
        }
        else if (start_inside && !end_inside)
        {
            /* Leaving the visible half-space: retain intersection. */
            const float t =
                (near_plane - start->z) / (end->z - start->z);

            output[count++] = (geometry3d_vertex_t)
            {
                .x = start->x + t * (end->x - start->x),
                .y = start->y + t * (end->y - start->y),
                .z = near_plane
            };
        }
        else if (!start_inside && end_inside)
        {
            /* Entering the visible half-space: retain intersection. */
            const float t =
                (near_plane - start->z) / (end->z - start->z);

            output[count++] = (geometry3d_vertex_t)
            {
                .x = start->x + t * (end->x - start->x),
                .y = start->y + t * (end->y - start->y),
                .z = near_plane
            };

            /*
             * If end lies exactly on the plane, the intersection
             * already represents it. Avoid adding a duplicate.
             */
            if (end->z > near_plane)
            {
                output[count++] = *end;
            }
        }

        /* If both endpoints are outside, this edge adds nothing. */
    }

    *output_count = count;
    return true;
}

/*
 * Signed 2D edge function.
 * Its value is twice the signed area of triangle (a, b, p).
 */
static float rasterizer3d_edge_function(
    float ax,
    float ay,
    float bx,
    float by,
    float px,
    float py)
{
    return (bx - ax) * (py - ay) -
           (by - ay) * (px - ax);
}

/*
 * Rasterize a triangle after projection.
 *
 * Reciprocal depth is interpolated linearly in screen space.
 */
static bool rasterizer3d_rasterize_projected_triangle(
    depth_buffer_t *depth_buffer,
    const rasterizer3d_screen_vertex_t *a,
    const rasterizer3d_screen_vertex_t *b,
    const rasterizer3d_screen_vertex_t *c)
{
    if (depth_buffer == NULL ||
        a == NULL ||
        b == NULL ||
        c == NULL)
    {
        return false;
    }

    const float area = rasterizer3d_edge_function(
        a->x, a->y,
        b->x, b->y,
        c->x, c->y);

    if (!isfinite(area))
    {
        return false;
    }

    /*
     * Degenerate triangles cover no area.
     * They are valid geometry, so simply skip them.
     */
    if (area == 0.0f)
    {
        return true;
    }

    const float min_x = fmaxf(
        0.0f,
        ceilf(fminf(a->x, fminf(b->x, c->x))));

    const float max_x = fminf(
        (float)(DISPLAY_WIDTH - 1U),
        floorf(fmaxf(a->x, fmaxf(b->x, c->x))));

    const float min_y = fmaxf(
        0.0f,
        ceilf(fminf(a->y, fminf(b->y, c->y))));

    const float max_y = fminf(
        (float)(DISPLAY_HEIGHT - 1U),
        floorf(fmaxf(a->y, fmaxf(b->y, c->y))));

    /*
     * Clamp the bounding box before converting to integers.
     * Projected vertices may lie far outside the viewport.
     */
    if (min_x > max_x || min_y > max_y)
    {
        return true;
    }

    const int x_start = (int)min_x;
    const int x_end = (int)max_x;
    const int y_start = (int)min_y;
    const int y_end = (int)max_y;

    for (int y = y_start; y <= y_end; ++y)
    {
        for (int x = x_start; x <= x_end; ++x)
        {
            /*
             * Sample at integer screen coordinates to align with
             * the pixel coordinates used by our line renderer.
             */
            const float px = (float)x;
            const float py = (float)y;

            const float w0 = rasterizer3d_edge_function(
                b->x, b->y,
                c->x, c->y,
                px, py) / area;

            const float w1 = rasterizer3d_edge_function(
                c->x, c->y,
                a->x, a->y,
                px, py) / area;

            const float w2 = rasterizer3d_edge_function(
                a->x, a->y,
                b->x, b->y,
                px, py) / area;

            /*
             * Dividing each signed edge value by signed area
             * makes the test independent of triangle winding.
             */
            if (w0 < -RASTERIZER3D_INSIDE_EPSILON ||
                w1 < -RASTERIZER3D_INSIDE_EPSILON ||
                w2 < -RASTERIZER3D_INSIDE_EPSILON)
            {
                continue;
            }

            /*
             * Perspective-correct depth interpolation:
             *
             * 1/z = w0*(1/z0) + w1*(1/z1) + w2*(1/z2)
             *
             * Larger inverse depth means a nearer surface.
             */
            const float inverse_depth =
                w0 * a->inverse_depth +
                w1 * b->inverse_depth +
                w2 * c->inverse_depth;

            if (isfinite(inverse_depth) &&
                inverse_depth > 0.0f)
            {
                /*
                 * A false return simply means that the sample
                 * was invalid or an existing surface was nearer.
                 * Neither case is a rasterization failure.
                 */
                (void)depth_buffer_write_if_nearer(
                    depth_buffer,
                    (int16_t)x,
                    (int16_t)y,
                    inverse_depth);
            }
        }
    }

    return true;
}

static bool rasterizer3d_project_vertex(
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *vertex,
    rasterizer3d_screen_vertex_t *result)
{
    if (projection == NULL ||
        vertex == NULL ||
        result == NULL ||
        vertex->z <= 0.0f)
    {
        return false;
    }

    if (!projection_perspective_project_float(
            projection,
            vertex,
            &result->x,
            &result->y))
    {
        return false;
    }

    result->inverse_depth = 1.0f / vertex->z;

    return isfinite(result->inverse_depth) &&
           result->inverse_depth > 0.0f;
}

bool rasterizer3d_rasterize_triangle(
    depth_buffer_t *depth_buffer,
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *a,
    const geometry3d_vertex_t *b,
    const geometry3d_vertex_t *c)
{
    if (depth_buffer == NULL ||
        projection == NULL ||
        !rasterizer3d_vertex_is_finite(a) ||
        !rasterizer3d_vertex_is_finite(b) ||
        !rasterizer3d_vertex_is_finite(c) ||
        !isfinite(projection->focal_length) ||
        !isfinite(projection->near_plane) ||
        !isfinite(projection->center_x) ||
        !isfinite(projection->center_y) ||
        projection->focal_length <= 0.0f ||
        projection->near_plane <= 0.0f)
    {
        return false;
    }

    const geometry3d_vertex_t input[3] =
    {
        *a, *b, *c
    };

    geometry3d_vertex_t clipped[4];
    size_t clipped_count = 0U;

    if (!rasterizer3d_clip_triangle(
            input,
            projection->near_plane,
            clipped,
            &clipped_count))
    {
        return false;
    }

    /*
     * No part of this triangle is in front of the near plane.
     */
    if (clipped_count < 3U)
    {
        return true;
    }

    rasterizer3d_screen_vertex_t projected[4];

    for (size_t i = 0U; i < clipped_count; ++i)
    {
        if (!rasterizer3d_project_vertex(
                projection,
                &clipped[i],
                &projected[i]))
        {
            return false;
        }
    }

    /*
     * A clipped quadrilateral is split into two triangles.
     * A triangle is processed as a single triangle.
     */
    for (size_t i = 1U; i + 1U < clipped_count; ++i)
    {
        if (!rasterizer3d_rasterize_projected_triangle(
                depth_buffer,
                &projected[0],
                &projected[i],
                &projected[i + 1U]))
        {
            return false;
        }
    }

    return true;
}