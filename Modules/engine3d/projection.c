#include <math.h>
#include <float.h>

#include "Modules/engine3d/projection.h"

bool projection_orthographic_project(
    const projection_orthographic_t *projection,
    const geometry3d_vertex_t *vertex,
    int16_t *screen_x,
    int16_t *screen_y)
{
    if (projection == 0 ||
        vertex == 0 ||
        screen_x == 0 ||
        screen_y == 0)
    {
        return false;
    }

    const float x = projection->center_x
                  + projection->scale * vertex->x;

    const float y = projection->center_y
                  - projection->scale * vertex->y;

    *screen_x = (int16_t)lroundf(x);
    *screen_y = (int16_t)lroundf(y);

    return true;
}

bool projection_perspective_project(
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *vertex,
    int16_t *screen_x,
    int16_t *screen_y)
{
    if (screen_x == NULL || screen_y == NULL)
    {
        return false;
    }

    float x;
    float y;

    if (!projection_perspective_project_float(
            projection, vertex, &x, &y))
    {
        return false;
    }

    if (x < (float)INT16_MIN ||
        x > (float)INT16_MAX ||
        y < (float)INT16_MIN ||
        y > (float)INT16_MAX)
    {
        return false;
    }

    *screen_x = (int16_t)lroundf(x);
    *screen_y = (int16_t)lroundf(y);

    return true;
}

bool projection_perspective_project_float(
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *vertex,
    float *screen_x,
    float *screen_y)
{
    if (projection == NULL ||
        vertex == NULL ||
        screen_x == NULL ||
        screen_y == NULL)
    {
        return false;
    }

    if (!isfinite(projection->focal_length) ||
        !isfinite(projection->near_plane) ||
        !isfinite(projection->center_x) ||
        !isfinite(projection->center_y) ||
        !isfinite(vertex->x) ||
        !isfinite(vertex->y) ||
        !isfinite(vertex->z) ||
        projection->focal_length <= 0.0f ||
        projection->near_plane <= 0.0f)
    {
        return false;
    }

    float depth = vertex->z;

    const float tolerance =
        8.0f * FLT_EPSILON *
        fmaxf(1.0f, fabsf(vertex->z));

    if (!isfinite(depth) ||
        depth < projection->near_plane - tolerance)
    {
        return false;
    }

    /*
     * Compensate for small floating-point errors at the near plane.
     */
    if (depth < projection->near_plane)
    {
        depth = projection->near_plane;
    }

    const float x =
        projection->center_x +
        projection->focal_length * vertex->x / depth;

    const float y =
        projection->center_y -
        projection->focal_length * vertex->y / depth;

    if (!isfinite(x) || !isfinite(y))
    {
        return false;
    }

    *screen_x = x;
    *screen_y = y;

    return true;
}