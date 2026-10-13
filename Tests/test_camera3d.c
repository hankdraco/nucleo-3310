#include "camera3d.h"

#include <stdbool.h>
#include <math.h>

#define CAMERA3D_TEST_TOLERANCE 0.0001f

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

bool test_camera3d_suite(void)
{
    if (!test_camera3d_clear_target_preserves_orientation())
        return false;

    if (!test_camera3d_clear_target_manual_mode_is_noop())
        return false;

    if (!test_camera3d_clear_target_failure_preserves_state())
        return false;

    return true;
}