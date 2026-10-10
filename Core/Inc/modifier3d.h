#ifndef MODIFIER3D_H
#define MODIFIER3D_H

#include <stdbool.h>

#include "geometry3d.h"
#include "modifier3d_types.h"

/*
 * Add a modifier to an instance.
 *
 * The modifier is initialized from the instance's current parameter
 * value. Adding another modifier targeting the same parameter fails.
 */
bool modifier3d_add(
    geometry3d_instance_t *instance,
    const modifier3d_config_t *config);

/*
 * Update all modifiers belonging to an instance.
 *
 * Modifiers incrementally mutate instance->transform.
 */
bool modifier3d_update_instance(
    geometry3d_instance_t *instance,
    float dt_seconds);

#endif /* MODIFIER3D_H */