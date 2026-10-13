#include "camera3d.h"

#include <stdbool.h>
#include <math.h>

#define CAMERA3D_TEST_TOLERANCE 0.0001f

static bool camera3d_test_transform_point(
    const matrix4_t *matrix,
    const geometry3d_vector3_t *point,
    geometry3d_vector3_t *result)
{
    if (matrix == NULL || point == NULL || result == NULL)
    {
        return false;
    }

    const float x =
        matrix->m[0][0] * point->x +
        matrix->m[0][1] * point->y +
        matrix->m[0][2] * point->z +
        matrix->m[0][3];

    const float y =
        matrix->m[1][0] * point->x +
        matrix->m[1][1] * point->y +
        matrix->m[1][2] * point->z +
        matrix->m[1][3];

    const float z =
        matrix->m[2][0] * point->x +
        matrix->m[2][1] * point->y +
        matrix->m[2][2] * point->z +
        matrix->m[2][3];

    const float w =
        matrix->m[3][0] * point->x +
        matrix->m[3][1] * point->y +
        matrix->m[3][2] * point->z +
        matrix->m[3][3];

    if (!isfinite(x) || !isfinite(y) ||
        !isfinite(z) || !isfinite(w) ||
        fabsf(w) < 1.0e-6f)
    {
        return false;
    }

    result->x = x / w;
    result->y = y / w;
    result->z = z / w;

    return isfinite(result->x) &&
           isfinite(result->y) &&
           isfinite(result->z);
}

static bool test_camera3d_clear_target_preserves_orientation(void)
{
    camera3d_t camera;

    if (!camera3d_init(&camera))
        return false;

    camera.position.x = 8.0f;
    camera.position.y = 2.0f;
    camera.position.z = -6.0f;

    const geometry3d_vector3_t target =
    {
        .x = 0.0f,
        .y = 2.4f,
        .z = 0.0f
    };

    if (!camera3d_set_target(&camera, &target))
        return false;

    if (!camera.target_enabled)
        return false;

    if (!camera3d_clear_target(&camera))
        return false;

    if (camera.target_enabled)
        return false;

    /*
     * Dla kierunku od (8, 2, -6) do (0, 2.4, 0):
     *
     * yaw   = atan2(-8, 6)
     * pitch = -atan2(0.4, 10)
     */
    const float expected_yaw = atan2f(-8.0f, 6.0f);
    const float expected_pitch = -atan2f(0.4f, 10.0f);

    if (fabsf(camera.rotation_rad.y - expected_yaw) >
        CAMERA3D_TEST_TOLERANCE)
    {
        return false;
    }

    if (fabsf(camera.rotation_rad.x - expected_pitch) >
        CAMERA3D_TEST_TOLERANCE)
    {
        return false;
    }

    if (fabsf(camera.rotation_rad.z) >
        CAMERA3D_TEST_TOLERANCE)
    {
        return false;
    }

    return true;
}

static bool test_camera3d_clear_target_manual_mode_is_noop(void)
{
    camera3d_t camera;

    if (!camera3d_init(&camera))
        return false;

    camera.rotation_rad.x = 0.2f;
    camera.rotation_rad.y = -0.3f;
    camera.rotation_rad.z = 0.4f;

    if (!camera3d_clear_target(&camera))
        return false;

    if (camera.target_enabled)
        return false;

    if (camera.rotation_rad.x != 0.2f)
        return false;

    if (camera.rotation_rad.y != -0.3f)
        return false;

    if (camera.rotation_rad.z != 0.4f)
        return false;

    return true;
}

static bool test_camera3d_clear_target_failure_preserves_state(void)
{
    camera3d_t camera;

    if (!camera3d_init(&camera))
        return false;

    camera.position.x = 1.0f;
    camera.position.y = 2.0f;
    camera.position.z = 3.0f;

    camera.target = camera.position;
    camera.target_enabled = true;

    camera.rotation_rad.x = 0.25f;
    camera.rotation_rad.y = -0.5f;
    camera.rotation_rad.z = 0.75f;

    if (camera3d_clear_target(&camera))
        return false;

    /*
     * Nieudana operacja nie powinna częściowo
     * zmodyfikować stanu kamery.
     */
    if (!camera.target_enabled)
        return false;

    if (camera.rotation_rad.x != 0.25f)
        return false;

    if (camera.rotation_rad.y != -0.5f)
        return false;

    if (camera.rotation_rad.z != 0.75f)
        return false;

    return true;
}

static bool test_camera3d_look_at_points_towards_target(void)
{
    camera3d_t camera;

    if (!camera3d_init(&camera))
        return false;

    camera.position.x = 8.0f;
    camera.position.y = 2.0f;
    camera.position.z = -6.0f;

    const geometry3d_vector3_t target =
    {
        .x = 0.0f,
        .y = 2.4f,
        .z = 0.0f
    };

    if (!camera3d_set_target(&camera, &target))
        return false;

    matrix4_t view_matrix;

    if (!camera3d_build_view_matrix(&camera, &view_matrix))
        return false;

    geometry3d_vector3_t target_in_camera_space;

    if (!camera3d_test_transform_point(
            &view_matrix,
            &target,
            &target_in_camera_space))
    {
        return false;
    }

    /*
     * A point directly in front of the camera
     * must have X ~= 0, Y ~= 0 and positive Z.
     */
    if (fabsf(target_in_camera_space.x) > 1.0e-3f)
        return false;

    if (fabsf(target_in_camera_space.y) > 1.0e-3f)
        return false;

    const float dx = target.x - camera.position.x;
    const float dy = target.y - camera.position.y;
    const float dz = target.z - camera.position.z;

    const float expected_distance =
        sqrtf(dx * dx + dy * dy + dz * dz);

    if (!isfinite(expected_distance))
        return false;

    if (fabsf(
            target_in_camera_space.z - expected_distance) >
        1.0e-3f)
    {
        return false;
    }

    return true;
}

bool test_camera3d_suite(void)
{
    if (!test_camera3d_clear_target_preserves_orientation())
        return false;

    if (!test_camera3d_clear_target_manual_mode_is_noop())
        return false;

    if (!test_camera3d_clear_target_failure_preserves_state())
        return false;

    if (!test_camera3d_look_at_points_towards_target())
        return false;

    return true;
}