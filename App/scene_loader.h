#ifndef SCENE_LOADER_H
#define SCENE_LOADER_H

#include <stdbool.h>
#include <stddef.h>

#include "Modules/engine3d/camera3d.h"
#include "Modules/engine3d/projection.h"
#include "Modules/engine3d/scene3d.h"
#include "Modules/traffic/car_modifier.h"

/** Load the default demonstration scene and initialize its camera/projection.
 * @param scene Scene storage to initialize and populate.
 * @param camera Camera storage to initialize.
 * @param projection Perspective projection storage to initialize.
 * @param car_contexts Caller-owned context array with at least car_count items.
 * @param car_count Number of car agents to add to the scene.
 * @param car_environment Shared environment referenced by each car context.
 * @return true on success; false if an argument or scene setup is invalid.
 */
bool scene_loader_load(
    scene3d_t *scene,
    camera3d_t *camera,
    projection_perspective_t *projection,
    car_modifier_context_t *car_contexts,
    size_t car_count,
    car_modifier_environment_t *car_environment);

#endif /* SCENE_LOADER_H */
