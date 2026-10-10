#ifndef DEPTH_BUFFER_H
#define DEPTH_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#include "display.h"

typedef struct
{
    /*
     * Reciprocal camera-space depth, 1 / z.
     * Zero means that no surface has been written.
     * Larger positive values represent nearer surfaces.
     */
    float inverse_depth[DISPLAY_HEIGHT][DISPLAY_WIDTH];
} depth_buffer_t;

/* Initialize all pixels as having no surface in front of them. */
void depth_buffer_clear(depth_buffer_t *buffer);

/*
 * Store a sample if it is nearer than the existing surface.
 *
 * Returns true if the sample was written.
 * Returns false if it was invalid or not nearer.
 */
bool depth_buffer_write_if_nearer(
    depth_buffer_t *buffer,
    int16_t x,
    int16_t y,
    float inverse_depth);

/*
 * Test whether a sample is visible against the stored surface.
 * A small tolerance is used to accommodate rasterization precision.
 */
bool depth_buffer_is_visible(
    const depth_buffer_t *buffer,
    int16_t x,
    int16_t y,
    float inverse_depth);

#endif /* DEPTH_BUFFER_H */