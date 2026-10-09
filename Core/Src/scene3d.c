#include "scene3d.h"

#include <stddef.h>

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
    const projection_perspective_t *projection)
{
    if (scene == NULL ||
        canvas == NULL ||
        projection == NULL ||
        scene->instance_count > SCENE3D_MAX_INSTANCES)
    {
        return false;
    }

    for (size_t i = 0U; i < scene->instance_count; ++i)
    {
        if (!renderer3d_draw_instance(
                canvas,
                &scene->instances[i],
                projection))
        {
            return false;
        }
    }

    return true;
}