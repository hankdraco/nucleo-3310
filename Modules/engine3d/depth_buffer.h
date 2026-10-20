#ifndef DEPTH_BUFFER_H
#define DEPTH_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#include "Modules/engine3d/display.h"

/** Per-pixel reciprocal-depth storage for visibility tests. */
typedef struct
{
    /*
     * Reciprocal camera-space depth, 1 / z.
     * Zero means that no surface has been written.
     * Larger positive values represent nearer surfaces.
     */
    float inverse_depth[DISPLAY_HEIGHT][DISPLAY_WIDTH];
} depth_buffer_t;

/** Clear the buffer so every pixel is initially uncovered.
 * @param buffer Buffer to clear.
 */
void depth_buffer_clear(depth_buffer_t *buffer);

/** Store a sample if it is nearer than the existing surface.
 * @param buffer Buffer to update.
 * @param x Pixel column.
 * @param y Pixel row.
 * @param inverse_depth Positive reciprocal camera-space depth (1/z).
 * @return true if written; false if invalid, out of bounds or farther away.
 */
bool depth_buffer_write_if_nearer(
    depth_buffer_t *buffer,
    int16_t x,
    int16_t y,
    float inverse_depth);

/** Test visibility against the stored surface, with a small precision tolerance.
 * @param buffer Buffer containing the nearest surface at each pixel.
 * @param x Pixel column.
 * @param y Pixel row.
 * @param inverse_depth Sample reciprocal depth (1/z).
 * @return true if visible or the pixel is empty; false otherwise.
 */
bool depth_buffer_is_visible(
    const depth_buffer_t *buffer,
    int16_t x,
    int16_t y,
    float inverse_depth);

#endif /* DEPTH_BUFFER_H */
