#ifndef RENDERER3D_H
#define RENDERER3D_H

#include <stdbool.h>

#include "Modules/engine3d/display.h"
#include "Modules/engine3d/geometry3d.h"
#include "Modules/engine3d/projection.h"
#include "Modules/engine3d/matrix4.h"
#include "Modules/engine3d/depth_buffer.h"

#define RENDERER3D_MAX_VERTICES 64U /**< Largest model vertex count supported by renderer scratch storage. */

/** Rasterize an instance's filled triangles into the depth buffer.
 * @param depth_buffer Depth target to update.
 * @param instance Model instance to rasterize.
 * @param world_matrix Local-to-world transform.
 * @param view_matrix World-to-camera transform.
 * @param projection Perspective projection parameters.
 * @return true on success; false for invalid models, transforms or projection.
 */
bool renderer3d_rasterize_instance(
    depth_buffer_t *depth_buffer,
    const geometry3d_instance_t *instance,
    const matrix4_t *world_matrix,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection);

/** Draw visible model edges into the canvas using the depth buffer.
 * @param canvas Framebuffer to draw into.
 * @param depth_buffer Visibility data produced by rasterizing the scene.
 * @param instance Model instance to draw.
 * @param world_matrix Local-to-world transform.
 * @param view_matrix World-to-camera transform.
 * @param projection Perspective projection parameters.
 * @return true on success; false for invalid arguments or model data.
 */
bool renderer3d_draw_instance(
    display_t *canvas,
    const depth_buffer_t *depth_buffer,
    const geometry3d_instance_t *instance,
    const matrix4_t *world_matrix,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection);

#endif /* RENDERER3D_H */
