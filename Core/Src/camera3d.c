#include "camera3d.h"

#include <math.h>

bool camera3d_init(camera3d_t *camera)
{
    if (camera == NULL)
    {
        return false;
    }

    camera->position.x = 0.0f;
    camera->position.y = 0.0f;
    camera->position.z = 0.0f;

    camera->rotation_rad.x = 0.0f;
    camera->rotation_rad.y = 0.0f;
    camera->rotation_rad.z = 0.0f;

    return true;
}

bool camera3d_build_view_matrix(
    const camera3d_t *camera,
    matrix4_t *view_matrix)
{
    if ((camera == NULL) || (view_matrix == NULL))
    {
        return false;
    }

    if (!isfinite(camera->position.x) ||
        !isfinite(camera->position.y) ||
        !isfinite(camera->position.z) ||
        !isfinite(camera->rotation_rad.x) ||
        !isfinite(camera->rotation_rad.y) ||
        !isfinite(camera->rotation_rad.z))
    {
        return false;
    }

    matrix4_t translation;
    matrix4_t rotation_x_inverse;
    matrix4_t rotation_y_inverse;
    matrix4_t rotation_z_inverse;

    matrix4_t intermediate_1;
    matrix4_t intermediate_2;
    matrix4_t result;

    /*
     * Inverse translation: move the world by the negative
     * of the camera's world-space position.
     */
    matrix4_set_translation(
        &translation,
        -camera->position.x,
        -camera->position.y,
        -camera->position.z);

    /*
     * Inverse rotations: negate each camera angle.
     */
    matrix4_set_rotation_x(
        &rotation_x_inverse,
        -camera->rotation_rad.x);

    matrix4_set_rotation_y(
        &rotation_y_inverse,
        -camera->rotation_rad.y);

    matrix4_set_rotation_z(
        &rotation_z_inverse,
        -camera->rotation_rad.z);

    /*
     * Original camera orientation:
     *
     *     R = Rz * Rx * Ry
     *
     * Inverse orientation:
     *
     *     R^-1 = Ry^-1 * Rx^-1 * Rz^-1
     *
     * View matrix:
     *
     *     V = R^-1 * T^-1
     */
    if (!matrix4_multiply(
            &intermediate_1,
            &rotation_z_inverse,
            &translation))
    {
        return false;
    }

    if (!matrix4_multiply(
            &intermediate_2,
            &rotation_x_inverse,
            &intermediate_1))
    {
        return false;
    }

    if (!matrix4_multiply(
            &result,
            &rotation_y_inverse,
            &intermediate_2))
    {
        return false;
    }

    *view_matrix = result;

    return true;
}