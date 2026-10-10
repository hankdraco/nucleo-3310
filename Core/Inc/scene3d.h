#ifndef SCENE3D_H
#define SCENE3D_H

#include <stdbool.h>
#include <stddef.h>

#include "display.h"
#include "geometry3d.h"
#include "projection.h"
#include "depth_buffer.h"

#define SCENE3D_MAX_INSTANCES 8U

typedef struct
{
    geometry3d_instance_t instances[SCENE3D_MAX_INSTANCES];
    size_t instance_count;
} scene3d_t;

void scene3d_init(scene3d_t *scene);

bool scene3d_add_instance(
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

#endif /* SCENE3D_H */