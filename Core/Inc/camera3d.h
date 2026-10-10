#ifndef CAMERA3D_H
#define CAMERA3D_H

#include <stdbool.h>

#include "geometry3d.h"
#include "matrix4.h"

typedef struct
{
    geometry3d_vector3_t position;
    geometry3d_vector3_t rotation_rad;
} camera3d_t;

/* Initialize the camera at the world origin with zero rotation. */
bool camera3d_init(camera3d_t *camera);

/*
 * Build the world-to-camera view matrix.
 *
 * Camera rotation follows the engine's rotation order:
 * Rz * Rx * Ry.
 *
 * The resulting view matrix is its inverse, including translation.
 */
bool camera3d_build_view_matrix(
    const camera3d_t *camera,
    matrix4_t *view_matrix);

#endif /* CAMERA3D_H */