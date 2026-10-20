#ifndef RASTERIZER3D_H
#define RASTERIZER3D_H

#include <stdbool.h>

#include "Modules/engine3d/depth_buffer.h"
#include "Modules/engine3d/geometry3d.h"
#include "Modules/engine3d/projection.h"

/** Rasterize one camera-space triangle into the depth buffer.
 * Clips against the near plane and does not modify the framebuffer.
 * @param depth_buffer Depth target to update.
 * @param projection Perspective projection parameters.
 * @param a First camera-space vertex.
 * @param b Second camera-space vertex.
 * @param c Third camera-space vertex.
 * @return true on success; false for invalid arguments or geometry.
 */
bool rasterizer3d_rasterize_triangle(
    depth_buffer_t *depth_buffer,
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *a,
    const geometry3d_vertex_t *b,
    const geometry3d_vertex_t *c);

#endif /* RASTERIZER3D_H */
