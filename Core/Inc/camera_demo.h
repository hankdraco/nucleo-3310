#ifndef CAMERA_DEMO_H
#define CAMERA_DEMO_H

#include <stdbool.h>

#include "camera3d.h"

typedef struct
{
    float elapsed_seconds;
    bool initialized;
} camera_demo_t;

/* Start the closed cinematic path at its first keyframe. */
bool camera_demo_init(camera_demo_t *demo, camera3d_t *camera);

/* Advance along the path and update camera position and look target. */
bool camera_demo_update(
    camera_demo_t *demo,
    camera3d_t *camera,
    float dt_seconds);

#endif /* CAMERA_DEMO_H */
