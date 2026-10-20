#ifndef SCENE3D_H
#define SCENE3D_H

#include <stdbool.h>
#include <stddef.h>

#include "Modules/engine3d/display.h"
#include "Modules/engine3d/geometry3d.h"
#include "Modules/engine3d/projection.h"
#include "Modules/engine3d/depth_buffer.h"

#define SCENE3D_MAX_INSTANCES 64U
#define SCENE3D_MAX_AGENTS 16U

#define SCENE3D_NO_PARENT_INDEX ((size_t)-1)

typedef struct
{
    /* Static scene instances, including scenery and non-agent animations. */
    geometry3d_instance_t instances[SCENE3D_MAX_INSTANCES];

    /*
     * Parent index for each instance.
     * SCENE3D_NO_PARENT_INDEX means that the instance is a root.
     */
    size_t parent_indices[SCENE3D_MAX_INSTANCES];

    size_t instance_count;

    /* Dynamic agents (currently cars), kept separate for simulation queries. */
    geometry3d_instance_t agent_instances[SCENE3D_MAX_AGENTS];
    size_t agent_count;
} scene3d_t;

void scene3d_init(scene3d_t *scene);

bool scene3d_add_instance(
    scene3d_t *scene,
    const geometry3d_instance_t *instance);

/* Add a root instance to the dynamic-agent collection. */
bool scene3d_add_agent_instance(
    scene3d_t *scene,
    const geometry3d_instance_t *instance);

geometry3d_instance_t *scene3d_get_instance(
    scene3d_t *scene,
    size_t index);

bool scene3d_render(
    const scene3d_t *scene,
    display_t *canvas,
    depth_buffer_t *depth_buffer,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection);

bool scene3d_update(
    scene3d_t *scene,
    float dt_seconds);

/*
 * Attach a child instance to a parent instance.
 * Rejects invalid indices, self-parenting and hierarchy cycles.
 */
bool scene3d_set_parent(
    scene3d_t *scene,
    size_t child_index,
    size_t parent_index);

/* Detach an instance, making it a root of the scene. */
bool scene3d_clear_parent(
    scene3d_t *scene,
    size_t child_index);

/*
 * Build the instance's local-to-world transformation.
 * Includes all ancestors up to the root.
 */
bool scene3d_build_world_transform(
    const scene3d_t *scene,
    size_t instance_index,
    matrix4_t *world_matrix);

#endif /* SCENE3D_H */
