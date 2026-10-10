#ifndef RASTERIZER3D_H
#define RASTERIZER3D_H

#include <stdbool.h>

#include "depth_buffer.h"
#include "geometry3d.h"
#include "projection.h"

/*
 * Rasterize one triangle in camera-space coordinates.
 *
 * The triangle is clipped against the near plane, projected onto
 * the display, and used to update the depth buffer.
 *
 * Does not modify the display framebuffer.
 */
bool rasterizer3d_rasterize_triangle(
    depth_buffer_t *depth_buffer,
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *a,
    const geometry3d_vertex_t *b,
    const geometry3d_vertex_t *c);

#endif /* RASTERIZER3D_H */