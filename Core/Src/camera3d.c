#include "camera3d.h"

#include <math.h>
#include <stddef.h>

static bool camera3d_compute_look_at_rotation(
    const camera3d_t *camera,
    const geometry3d_vector3_t *target,
    geometry3d_vector3_t *rotation)
{
    if (camera == NULL || target == NULL || rotation == NULL)
    {
        return false;
    }

    if (!isfinite(camera->position.x) ||
        !isfinite(camera->position.y) ||
        !isfinite(camera->position.z) ||
        !isfinite(target->x) ||
        !isfinite(target->y) ||
        !isfinite(target->z))
    {
        return false;
    }

    const float dx = target->x - camera->position.x;
    const float dy = target->y - camera->position.y;
    const float dz = target->z - camera->position.z;

    if (!isfinite(dx) || !isfinite(dy) || !isfinite(dz))
    {
        return false;
    }

    const float horizontal_distance = hypotf(dx, dz);
    const float distance = hypotf(horizontal_distance, dy);

    /*
     * A camera cannot look at a target at its own position.
     */
    if (!isfinite(horizontal_distance) ||
        !isfinite(distance) ||
        distance <= 0.0f)
    {
        return false;
    }

    /*
     * Our camera looks along +Z, with +Y pointing upwards.
     *
     * A positive yaw points the camera towards +X.
     * A negative pitch points the camera towards +Y.
     */
    rotation->y = (horizontal_distance > 0.0f)
        ? atan2f(dx, dz)
        : 0.0f;

    rotation->x = -atan2f(dy, horizontal_distance);

    /*
     * Look-at does not introduce roll.
     */
    rotation->z = 0.0f;

    return isfinite(rotation->x) &&
           isfinite(rotation->y);
}

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

    camera->target.x = 0.0f;
    camera->target.y = 0.0f;
    camera->target.z = 0.0f;

    camera->target_enabled = false;

    return true;
}

bool camera3d_set_target(
    camera3d_t *camera,
    const geometry3d_vector3_t *target)
{
    if (camera == NULL || target == NULL)
    {
        return false;
    }

    /*
     * Validate the target before modifying the camera.
     */
    geometry3d_vector3_t rotation;

    if (!camera3d_compute_look_at_rotation(
            camera,
            target,
            &rotation))
    {
        return false;
    }

    camera->target = *target;
    camera->target_enabled = true;

    return true;
}

bool camera3d_clear_target(camera3d_t *camera)
{
    if (camera == NULL)
    {
        return false;
    }

    camera->target_enabled = false;

    return true;
}

bool camera3d_build_view_matrix(
    const camera3d_t *camera,
    matrix4_t *view_matrix)
{
    if (camera == NULL || view_matrix == NULL)
    {
        return false;
    }

    if (!isfinite(camera->position.x) ||
        !isfinite(camera->position.y) ||
        !isfinite(camera->position.z))
    {
        return false;
    }

    geometry3d_vector3_t rotation;

    if (camera->target_enabled)
    {
        /*
         * Recalculate orientation every time.
         * Moving the camera therefore keeps the same target in view.
         */
        if (!camera3d_compute_look_at_rotation(
                camera,
                &camera->target,
                &rotation))
        {
            return false;
        }
    }
    else
    {
        rotation = camera->rotation_rad;

        if (!isfinite(rotation.x) ||
            !isfinite(rotation.y) ||
            !isfinite(rotation.z))
        {
            return false;
        }
    }

    matrix4_t translation;
    matrix4_t rotation_x_inverse;
    matrix4_t rotation_y_inverse;
    matrix4_t rotation_z_inverse;

    matrix4_t intermediate_1;
    matrix4_t intermediate_2;
    matrix4_t result;

    /*
     * The view matrix uses the inverse camera transformation.
     */
    matrix4_set_translation(
        &translation,
        -camera->position.x,
        -camera->position.y,
        -camera->position.z);

    matrix4_set_rotation_x(
        &rotation_x_inverse,
        -rotation.x);

    matrix4_set_rotation_y(
        &rotation_y_inverse,
        -rotation.y);

    matrix4_set_rotation_z(
        &rotation_z_inverse,
        -rotation.z);

    /*
     * Camera orientation:
     *
     *     R = Rz * Rx * Ry
     *
     * Inverse orientation:
     *
     *     R^-1 = Ry^-1 * Rx^-1 * Rz^-1
     *
     * View:
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