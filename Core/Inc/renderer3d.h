
#ifndef RENDERER3D_H
#define RENDERER3D_H

#include <stdbool.h>

#include "display.h"
#include "geometry3d.h"
#include "projection.h"

/*
 * Maximum number of vertices supported by this renderer.
 */
#define RENDERER3D_MAX_VERTICES 64U

/*
 * Draw one object instance onto an existing canvas.
 *
 * Does not clear the canvas or flush it to hardware.
 */
bool renderer3d_draw_instance(
    display_t *canvas,
    const geometry3d_instance_t *instance,
    const projection_perspective_t *projection);

#endif /* RENDERER3D_H */