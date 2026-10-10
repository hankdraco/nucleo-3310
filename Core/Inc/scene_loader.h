#ifndef SCENE_LOADER_H
#define SCENE_LOADER_H

#include <stdbool.h>

#include "camera3d.h"
#include "projection.h"
#include "scene3d.h"

/*
 * Load the default demonstration scene.
 *
 * Initializes the scene, camera and perspective projection.
 * Returns false if any part of the scene cannot be configured.
 */
bool scene_loader_load(
    scene3d_t *scene,
    camera3d_t *camera,
    projection_perspective_t *projection);

#endif /* SCENE_LOADER_H */