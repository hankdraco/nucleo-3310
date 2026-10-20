#ifndef RENDERER3D_H
#define RENDERER3D_H

#include <stdbool.h>

#include "Modules/engine3d/display.h"
#include "Modules/engine3d/geometry3d.h"
#include "Modules/engine3d/projection.h"
#include "Modules/engine3d/matrix4.h"
#include "Modules/engine3d/depth_buffer.h"

/*
 * Maximum number of vertices supported by this renderer.
 */
#define RENDERER3D_MAX_VERTICES 64U

bool renderer3d_rasterize_instance(
    depth_buffer_t *depth_buffer,
    const geometry3d_instance_t *instance,
    const matrix4_t *world_matrix,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection);

bool renderer3d_draw_instance(
    display_t *canvas,
    const depth_buffer_t *depth_buffer,
    const geometry3d_instance_t *instance,
    const matrix4_t *world_matrix,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection);

#endif /* RENDERER3D_H */