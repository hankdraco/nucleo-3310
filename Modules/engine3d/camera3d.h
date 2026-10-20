#ifndef CAMERA3D_H
#define CAMERA3D_H

#include <stdbool.h>

#include "Modules/engine3d/geometry3d.h"
#include "Modules/engine3d/matrix4.h"

/** Camera pose and optional world-space look-at target. */
typedef struct
{
    geometry3d_vector3_t position; /**< Camera position in world coordinates. */

    /* Used for manual orientation when target mode is disabled. */
    geometry3d_vector3_t rotation_rad; /**< Manual Euler rotation, in radians. */

    /* World-space point the camera looks at. */
    geometry3d_vector3_t target; /**< World-space point viewed when target mode is enabled. */

    bool target_enabled; /**< Selects look-at orientation instead of manual rotation. */
} camera3d_t;

/** Initialize at the world origin with zero rotation.
 * @param camera Camera to initialize.
 * @return true on success; false if camera is NULL.
 */
bool camera3d_init(camera3d_t *camera);

/** Enable look-at mode toward a world-space point.
 * @param camera Camera to update.
 * @param target Point the camera should face.
 * @return true on success; false for invalid arguments or degenerate direction.
 */
bool camera3d_set_target(
    camera3d_t *camera,
    const geometry3d_vector3_t *target);

/** Disable look-at mode and use the stored manual rotation.
 * @param camera Camera to update.
 * @return true on success; false if camera is NULL.
 */
bool camera3d_clear_target(camera3d_t *camera);

/** Build the world-to-camera view matrix.
 * @param camera Camera pose to use.
 * @param view_matrix Destination world-to-camera matrix.
 * @return true on success; false for invalid arguments or orientation.
 */
bool camera3d_build_view_matrix(
    const camera3d_t *camera,
    matrix4_t *view_matrix);

#endif /* CAMERA3D_H */
