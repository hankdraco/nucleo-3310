#ifndef MODIFIER3D_H
#define MODIFIER3D_H

#include <stdbool.h>

#include "geometry3d.h"
#include "modifier3d_types.h"

/*
 * Add a modifier to an instance.
 *
 * The modifier is initialized from the instance's current parameter
 * value. Adding another scalar modifier targeting the same parameter fails.
 *
 * CUSTOM modifiers use target MODIFIER3D_TARGET_ALL and require non-NULL
 * custom_fn and context. The callback receives the instance, dt and context.
 * Context memory is caller-owned and must remain valid for as long as any
 * instance or scene copy may invoke the modifier. Copies retain the pointer;
 * context data itself is not copied and no dynamic allocation is performed.
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