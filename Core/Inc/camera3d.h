#ifndef CAMERA3D_H
#define CAMERA3D_H

#include <stdbool.h>

#include "geometry3d.h"
#include "matrix4.h"

typedef struct
{
    geometry3d_vector3_t position;

    /* Used for manual orientation when target mode is disabled. */
    geometry3d_vector3_t rotation_rad;

    /* World-space point the camera looks at. */
    geometry3d_vector3_t target;

    bool target_enabled;
} camera3d_t;

/* Initialize the camera at the world origin with zero rotation. */
bool camera3d_init(camera3d_t *camera);

/*
 * Enable look-at mode.
 * The camera automatically aims at the specified world-space point.
 */
bool camera3d_set_target(
    camera3d_t *camera,
    const geometry3d_vector3_t *target);

/* Disable look-at mode and return to manual orientation. */
bool camera3d_clear_target(camera3d_t *camera);

/* Build the world-to-camera view matrix. */
bool camera3d_build_view_matrix(
    const camera3d_t *camera,
    matrix4_t *view_matrix);

#endif /* CAMERA3D_H */