#include <stddef.h>
#include <math.h>

#include "Modules/engine3d/scene3d.h"
#include "Modules/engine3d/renderer3d.h"
#include "Modules/engine3d/modifier3d.h"

static bool scene3d_counts_are_valid(const scene3d_t *scene)
{
    return scene != NULL &&
           scene->instance_count <= SCENE3D_MAX_INSTANCES &&
           scene->agent_count <= SCENE3D_MAX_AGENTS &&
           scene->instance_count + scene->agent_count <= SCENE3D_MAX_INSTANCES;
}

static bool scene3d_instance_is_valid(const geometry3d_instance_t *instance)
{
    return instance != NULL &&
           geometry3d_model_is_valid(instance->model) &&
           instance->model->vertex_count <= RENDERER3D_MAX_VERTICES;
}

void scene3d_init(scene3d_t *scene)
{
    if (scene == NULL)
    {
        return;
    }

    scene->instance_count = 0U;
    scene->agent_count = 0U;

    for (size_t i = 0U; i < SCENE3D_MAX_INSTANCES; ++i)
    {
        scene->parent_indices[i] = SCENE3D_NO_PARENT_INDEX;
    }
}

bool scene3d_add_instance(
    scene3d_t *scene,
    const geometry3d_instance_t *instance)
{
    if (!scene3d_counts_are_valid(scene) || !scene3d_instance_is_valid(instance))
    {
        return false;
    }

    if (scene->instance_count + scene->agent_count >= SCENE3D_MAX_INSTANCES)
    {
        return false;
    }

    const size_t new_index = scene->instance_count;

    scene->instances[new_index] = *instance;
    scene->parent_indices[new_index] = SCENE3D_NO_PARENT_INDEX;

    ++scene->instance_count;

    return true;
}

bool scene3d_add_agent_instance(
    scene3d_t *scene,
    const geometry3d_instance_t *instance)
{
    if (!scene3d_counts_are_valid(scene) || !scene3d_instance_is_valid(instance) ||
        scene->agent_count >= SCENE3D_MAX_AGENTS ||
        scene->instance_count + scene->agent_count >= SCENE3D_MAX_INSTANCES)
    {
        return false;
    }

    scene->agent_instances[scene->agent_count++] = *instance;
    return true;
}

geometry3d_instance_t *scene3d_get_instance(
    scene3d_t *scene,
    size_t index)
{
    if (scene == NULL || index >= scene->instance_count)
    {
        return NULL;
    }

    return &scene->instances[index];
}

bool scene3d_update(
    scene3d_t *scene,
    float dt_seconds)
{
    if (scene == NULL ||
        !isfinite(dt_seconds) ||
        dt_seconds < 0.0f ||
        !scene3d_counts_are_valid(scene))
    {
        return false;
    }

    /*
     * Every instance owns its modifiers.
     * Modifiers incrementally update the instance's current transform.
     */
    /* Static scene instances are updated once and never enter agent phases. */
    for (size_t i = 0U; i < scene->instance_count; ++i)
    {
        if (!modifier3d_update_instance(
                &scene->instances[i],
                dt_seconds))
        {
            return false;
        }
    }

    /* Phase 1: all agents sense the same pre-movement scene state. */
    for (size_t i = 0U; i < scene->agent_count; ++i)
    {
        if (!modifier3d_pre_update_instance(&scene->agent_instances[i], dt_seconds))
        {
            return false;
        }
    }

    /* Phase 2: advance agent routes and transforms after sensing is complete. */
    for (size_t i = 0U; i < scene->agent_count; ++i)
    {
        if (!modifier3d_update_instance(&scene->agent_instances[i], dt_seconds))
        {
            return false;
        }
    }

    return true;
}

bool scene3d_render(
    const scene3d_t *scene,
    display_t *canvas,
    depth_buffer_t *depth_buffer,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection)
{
    if (scene == NULL ||
        canvas == NULL ||
        depth_buffer == NULL ||
        view_matrix == NULL ||
        projection == NULL ||
        !scene3d_counts_are_valid(scene))
    {
        return false;
    }

    depth_buffer_clear(depth_buffer);

    /*
     * PASS 1: rasterize every instance using its world transform.
     */
    for (size_t i = 0U; i < scene->instance_count; ++i)
    {
        matrix4_t world_matrix;

        if (!scene3d_build_world_transform(
                scene,
                i,
                &world_matrix))
        {
            return false;
        }

        if (!renderer3d_rasterize_instance(
                depth_buffer,
                &scene->instances[i],
                &world_matrix,
                view_matrix,
                projection))
        {
            return false;
        }
    }

    for (size_t i = 0U; i < scene->agent_count; ++i)
    {
        matrix4_t world_matrix;
        if (!geometry3d_build_transform_matrix(&scene->agent_instances[i].transform, &world_matrix) ||
            !renderer3d_rasterize_instance(depth_buffer, &scene->agent_instances[i],
                                           &world_matrix, view_matrix, projection))
        {
            return false;
        }
    }

    /*
     * PASS 2: draw all edges using the same transform hierarchy.
     */
    for (size_t i = 0U; i < scene->instance_count; ++i)
    {
        matrix4_t world_matrix;

        if (!scene3d_build_world_transform(
                scene,
                i,
                &world_matrix))
        {
            return false;
        }

        if (!renderer3d_draw_instance(
                canvas,
                depth_buffer,
                &scene->instances[i],
                &world_matrix,
                view_matrix,
                projection))
        {
            return false;
        }
    }

    for (size_t i = 0U; i < scene->agent_count; ++i)
    {
        matrix4_t world_matrix;
        if (!geometry3d_build_transform_matrix(&scene->agent_instances[i].transform, &world_matrix) ||
            !renderer3d_draw_instance(canvas, depth_buffer, &scene->agent_instances[i],
                                      &world_matrix, view_matrix, projection))
        {
            return false;
        }
    }

    return true;
}

bool scene3d_set_parent(
    scene3d_t *scene,
    size_t child_index,
    size_t parent_index)
{
    if (scene == NULL ||
        !scene3d_counts_are_valid(scene) ||
        child_index >= scene->instance_count ||
        parent_index >= scene->instance_count ||
        child_index == parent_index)
    {
        return false;
    }

    /*
     * Walk up the prospective parent's ancestry.
     * If we encounter the child, the new relationship would
     * create a cycle.
     */
    size_t current = parent_index;
    size_t steps = 0U;

    while (current != SCENE3D_NO_PARENT_INDEX)
    {
        if (current >= scene->instance_count ||
            current == child_index)
        {
            return false;
        }

        current = scene->parent_indices[current];

        ++steps;

        /*
         * Also detect a pre-existing malformed cycle.
         */
        if (steps > scene->instance_count)
        {
            return false;
        }
    }

    scene->parent_indices[child_index] = parent_index;

    return true;
}

bool scene3d_clear_parent(
    scene3d_t *scene,
    size_t child_index)
{
    if (scene == NULL ||
        !scene3d_counts_are_valid(scene) ||
        child_index >= scene->instance_count)
    {
        return false;
    }

    scene->parent_indices[child_index] = SCENE3D_NO_PARENT_INDEX;

    return true;
}

bool scene3d_build_world_transform(
    const scene3d_t *scene,
    size_t instance_index,
    matrix4_t *world_matrix)
{
    if (scene == NULL ||
        world_matrix == NULL ||
        !scene3d_counts_are_valid(scene) ||
        instance_index >= scene->instance_count)
    {
        return false;
    }

    /*
     * Collect the ancestry from the instance towards the root.
     * This fixed-size array avoids dynamic allocation and recursion.
     */
    size_t chain[SCENE3D_MAX_INSTANCES];
    size_t chain_count = 0U;
    size_t current = instance_index;

    while (current != SCENE3D_NO_PARENT_INDEX)
    {
        if (current >= scene->instance_count ||
            chain_count >= scene->instance_count)
        {
            /* Invalid parent index or hierarchy cycle. */
            return false;
        }

        chain[chain_count++] = current;
        current = scene->parent_indices[current];
    }

    matrix4_t accumulated;
    matrix4_set_identity(&accumulated);

    /*
     * The collected chain is child-to-root.
     * Apply transforms in reverse order: root-to-child.
     */
    while (chain_count > 0U)
    {
        const size_t index = chain[--chain_count];

        matrix4_t local_matrix;
        matrix4_t combined;

        if (!geometry3d_build_transform_matrix(
                &scene->instances[index].transform,
                &local_matrix))
        {
            return false;
        }

        /*
         * Column-vector convention:
         *
         * M_world(child) = M_world(parent) * M_local(child)
         */
        if (!matrix4_multiply(
                &combined,
                &accumulated,
                &local_matrix))
        {
            return false;
        }

        accumulated = combined;
    }

    *world_matrix = accumulated;
    return true;
}
