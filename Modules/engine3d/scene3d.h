#ifndef SCENE3D_H
#define SCENE3D_H

#include <stdbool.h>
#include <stddef.h>

#include "Modules/engine3d/display.h"
#include "Modules/engine3d/geometry3d.h"
#include "Modules/engine3d/projection.h"
#include "Modules/engine3d/depth_buffer.h"

#define SCENE3D_MAX_INSTANCES 64U /**< Maximum number of static scene instances. */
#define SCENE3D_MAX_AGENTS 16U /**< Maximum number of dynamic agent instances. */

#define SCENE3D_NO_PARENT_INDEX ((size_t)-1) /**< Sentinel indicating that an instance has no parent. */

/** Fixed-capacity scene storage for static objects and dynamic agents. */
typedef struct
{
    /* Static scene instances, including scenery and non-agent animations. */
    geometry3d_instance_t instances[SCENE3D_MAX_INSTANCES]; /**< Static objects and non-agent animations. */

    /*
     * Parent index for each instance.
     * SCENE3D_NO_PARENT_INDEX means that the instance is a root.
     */
    size_t parent_indices[SCENE3D_MAX_INSTANCES]; /**< Parent index for each static instance. */

    size_t instance_count; /**< Number of initialized entries in instances. */

    /* Dynamic agents (currently cars), kept separate for simulation queries. */
    geometry3d_instance_t agent_instances[SCENE3D_MAX_AGENTS]; /**< Dynamic agents, separate from static objects. */
    size_t agent_count; /**< Number of initialized entries in agent_instances. */
} scene3d_t;

/** Initialize an empty scene and clear all instance/agent counts.
 * @param scene Scene storage to initialize.
 */
void scene3d_init(scene3d_t *scene);

/** Add a static scene instance, copied by value into scene storage.
 * @param scene Destination scene.
 * @param instance Instance to copy.
 * @return true on success; false if arguments are invalid or capacity is exhausted.
 */
bool scene3d_add_instance(
    scene3d_t *scene,
    const geometry3d_instance_t *instance);

/** Add a root instance to the dynamic-agent collection.
 * @param scene Destination scene.
 * @param instance Agent instance to copy.
 * @return true on success; false if arguments are invalid or capacity is exhausted.
 */
bool scene3d_add_agent_instance(
    scene3d_t *scene,
    const geometry3d_instance_t *instance);

/** Retrieve a static instance by index.
 * @param scene Scene to query.
 * @param index Zero-based static-instance index.
 * @return Mutable instance pointer, or NULL when index is invalid.
 */
geometry3d_instance_t *scene3d_get_instance(
    scene3d_t *scene,
    size_t index);

/** Render static instances and agents using the supplied view and projection.
 * @param scene Scene to render.
 * @param canvas Destination framebuffer.
 * @param depth_buffer Scratch depth buffer.
 * @param view_matrix World-to-camera transform.
 * @param projection Perspective projection parameters.
 * @return true on success; false if an instance or rendering operation fails.
 */
bool scene3d_render(
    const scene3d_t *scene,
    display_t *canvas,
    depth_buffer_t *depth_buffer,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection);

/** Run modifier pre-updates and updates for scene instances and agents.
 * @param scene Scene to update.
 * @param dt_seconds Elapsed simulation time.
 * @return true on success; false if a modifier callback fails.
 */
bool scene3d_update(
    scene3d_t *scene,
    float dt_seconds);

/** Attach a static child instance to a static parent instance.
 * Rejects invalid indices, self-parenting and hierarchy cycles.
 * @param scene Scene containing the instances.
 * @param child_index Child instance index.
 * @param parent_index Parent instance index.
 * @return true on success; false if indices or hierarchy are invalid.
 */
bool scene3d_set_parent(
    scene3d_t *scene,
    size_t child_index,
    size_t parent_index);

/** Detach a static instance, making it a scene root.
 * @param scene Scene containing the instance.
 * @param child_index Instance index to detach.
 * @return true on success; false if scene or index is invalid.
 */
bool scene3d_clear_parent(
    scene3d_t *scene,
    size_t child_index);

/** Build a static instance's local-to-world transform, including its ancestors.
 * @param scene Scene containing the instance.
 * @param instance_index Static instance index.
 * @param world_matrix Receives the combined transformation.
 * @return true on success; false for invalid index or hierarchy.
 */
bool scene3d_build_world_transform(
    const scene3d_t *scene,
    size_t instance_index,
    matrix4_t *world_matrix);

#endif /* SCENE3D_H */
