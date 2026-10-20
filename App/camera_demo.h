#ifndef CAMERA_DEMO_H
#define CAMERA_DEMO_H

#include <stdbool.h>

#include "Modules/engine3d/camera3d.h"

/** Runtime state for traversing the closed demo-camera path. */
typedef struct
{
    float elapsed_seconds; /**< Elapsed time since the current path cycle began. */
    bool initialized; /**< True after the first keyframe has been applied. */
} camera_demo_t;

/** Start the closed cinematic path at its first keyframe.
 * @param demo State to initialize.
 * @param camera Camera whose initial position and target will be set.
 * @return true on success; false for invalid pointers or camera failure.
 */
bool camera_demo_init(camera_demo_t *demo, camera3d_t *camera);

/** Advance along the path and update camera position and look target.
 * @param demo Initialized path state.
 * @param camera Camera to update.
 * @param dt_seconds Elapsed time in seconds.
 * @return true on success; false for invalid input or camera failure.
 */
bool camera_demo_update(
    camera_demo_t *demo,
    camera3d_t *camera,
    float dt_seconds);

#endif /* CAMERA_DEMO_H */
