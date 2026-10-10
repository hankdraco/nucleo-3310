#include <stddef.h>

#include "scene3d.h"
#include "renderer3d.h"

void scene3d_init(scene3d_t *scene)
{
    if (scene == NULL)
    {
        return;
    }

    scene->instance_count = 0U;
}

bool scene3d_add_instance(
    scene3d_t *scene,
    const geometry3d_instance_t *instance)
{
    if (scene == NULL || instance == NULL)
    {
        return false;
    }

    if (scene->instance_count >= SCENE3D_MAX_INSTANCES)
    {
        return false;
    }

    if (!geometry3d_model_is_valid(instance->model) ||
        instance->model->vertex_count > RENDERER3D_MAX_VERTICES)
    {
        return false;
    }

    /*
     * Store a copy of the instance.
     * The model geometry itself remains shared.
     */
    scene->instances[scene->instance_count] = *instance;
    ++scene->instance_count;

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
        scene->instance_count > SCENE3D_MAX_INSTANCES)
    {
        return false;
    }

    /*
     * Start a new frame: discard depth samples from the previous one.
     */
    depth_buffer_clear(depth_buffer);

    /*
     * PASS 1:
     * Rasterize every instance's triangles before drawing any edges.
     */
    for (size_t i = 0U; i < scene->instance_count; ++i)
    {
        if (!renderer3d_rasterize_instance(
                depth_buffer,
                &scene->instances[i],
                view_matrix,
                projection))
        {
            return false;
        }
    }

    /*
     * PASS 2:
     * Preserve the existing wireframe rendering for now.
     * The depth buffer is populated but not yet used to hide edges.
     */
    for (size_t i = 0U; i < scene->instance_count; ++i)
    {
        if (!renderer3d_draw_instance(
                canvas,
                depth_buffer,
                &scene->instances[i],
                view_matrix,
                projection))
        {
            return false;
        }
    }

    return true;
}