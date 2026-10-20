#include "Modules/engine3d/depth_buffer.h"

#include <math.h>
#include <stddef.h>

#define DEPTH_BUFFER_VISIBILITY_EPSILON (1.0e-3f)

static bool depth_buffer_coordinates_are_valid(
    int16_t x,
    int16_t y)
{
    return x >= 0 &&
           y >= 0 &&
           x < (int16_t)DISPLAY_WIDTH &&
           y < (int16_t)DISPLAY_HEIGHT;
}

void depth_buffer_clear(depth_buffer_t *buffer)
{
    if (buffer == NULL)
    {
        return;
    }

    for (size_t y = 0U; y < DISPLAY_HEIGHT; ++y)
    {
        for (size_t x = 0U; x < DISPLAY_WIDTH; ++x)
        {
            buffer->inverse_depth[y][x] = 0.0f;
        }
    }
}

bool depth_buffer_write_if_nearer(
    depth_buffer_t *buffer,
    int16_t x,
    int16_t y,
    float inverse_depth)
{
    if (buffer == NULL ||
        !depth_buffer_coordinates_are_valid(x, y) ||
        !isfinite(inverse_depth) ||
        inverse_depth <= 0.0f)
    {
        return false;
    }

    float *stored_depth =
        &buffer->inverse_depth[(size_t)y][(size_t)x];

    if (inverse_depth > *stored_depth)
    {
        *stored_depth = inverse_depth;
        return true;
    }

    return false;
}

bool depth_buffer_is_visible(
    const depth_buffer_t *buffer,
    int16_t x,
    int16_t y,
    float inverse_depth)
{
    if (buffer == NULL ||
        !depth_buffer_coordinates_are_valid(x, y) ||
        !isfinite(inverse_depth) ||
        inverse_depth <= 0.0f)
    {
        return false;
    }

    const float stored_depth =
        buffer->inverse_depth[(size_t)y][(size_t)x];

    /*
     * No surface has been rendered at this pixel yet.
     */
    if (stored_depth <= 0.0f)
    {
        return true;
    }

    /*
     * Larger inverse depth means nearer to the camera.
     * Treat tiny differences as equal to avoid losing visible
     * edges because of floating-point and rasterization errors.
     */
    const float tolerance =
        DEPTH_BUFFER_VISIBILITY_EPSILON *
        fmaxf(
            1.0f,
            fmaxf(inverse_depth, stored_depth));

    return inverse_depth + tolerance >= stored_depth;
}