
#include "projection.h"

#include <math.h>

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
    if (projection == 0 ||
        vertex == 0 ||
        screen_x == 0 ||
        screen_y == 0)
    {
        return false;
    }

    if (projection->focal_length <= 0.0f ||
        projection->camera_distance <= 0.0f)
    {
        return false;
    }

    /*
     * Camera is at (0, 0, -camera_distance)
     * and looks toward positive Z.
     */
    const float camera_depth =
        projection->camera_distance + vertex->z;

    /*
     * Points at or behind the camera cannot be projected.
     */
    if (camera_depth <= 0.0f)
    {
        return false;
    }

    const float x =
        projection->center_x +
        projection->focal_length * vertex->x / camera_depth;

    const float y =
        projection->center_y -
        projection->focal_length * vertex->y / camera_depth;

    *screen_x = (int16_t)lroundf(x);
    *screen_y = (int16_t)lroundf(y);

    return true;
}