#ifndef SCENE_LOADER_H
#define SCENE_LOADER_H

#include <stdbool.h>
#include <stddef.h>

#include "Modules/engine3d/camera3d.h"
#include "Modules/engine3d/projection.h"
#include "Modules/engine3d/scene3d.h"
#include "Modules/traffic/car_modifier.h"

/*
 * Load the default demonstration scene.
 *
 * Initializes the scene, camera and perspective projection.
 * Returns false if any part of the scene cannot be configured.
 */
bool scene_loader_load(
    scene3d_t *scene,
    camera3d_t *camera,
    projection_perspective_t *projection,
    car_modifier_context_t *car_contexts,
    size_t car_count,
    car_modifier_environment_t *car_environment);

#endif /* SCENE_LOADER_H */