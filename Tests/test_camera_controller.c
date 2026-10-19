#include "camera_controller.h"

#include <stdbool.h>
#include <math.h>

#define CAMERA_CONTROLLER_TEST_TOLERANCE 0.0001f

static bool initialize_origin_camera(
    camera3d_t *camera,
    camera_controller_t *controller)
{
    if (!camera3d_init(camera))
        return false;

    return camera_controller_init(controller, camera);
}

static bool test_camera_controller_initializes_from_look_at(void)
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

    camera_controller_t controller = {0};

    if (!camera_controller_init(&controller, &camera))
        return false;

    if (!controller.initialized || camera.target_enabled)
        return false;

    const float expected_yaw = atan2f(-8.0f, 6.0f);
    const float expected_pitch = -atan2f(0.4f, 10.0f);

    if (fabsf(controller.yaw_rad - expected_yaw) >
        CAMERA_CONTROLLER_TEST_TOLERANCE)
    {
        return false;
    }

    if (fabsf(controller.pitch_rad - expected_pitch) >
        CAMERA_CONTROLLER_TEST_TOLERANCE)
    {
        return false;
    }

    return true;
}

static bool test_camera_controller_mouse_rotation(void)
{
    camera3d_t camera;
    camera_controller_t controller;

    if (!initialize_origin_camera(&camera, &controller))
        return false;

    const input_snapshot_t input =
    {
        .key_mask = 0U,
        .mouse_dx = 100,
        .mouse_dy = 20
    };

    /*
     * dt == 0 must not suppress mouse rotation.
     */
    if (!camera_controller_update(
            &controller, &camera, &input, 0.0f))
    {
        return false;
    }

    const float expected_yaw =
        100.0f * CAMERA_CONTROLLER_DEFAULT_MOUSE_SENSITIVITY;

    const float expected_pitch =
        20.0f * CAMERA_CONTROLLER_DEFAULT_MOUSE_SENSITIVITY;

    if (fabsf(camera.rotation_rad.y - expected_yaw) >
        CAMERA_CONTROLLER_TEST_TOLERANCE)
    {
        return false;
    }

    if (fabsf(camera.rotation_rad.x - expected_pitch) >
        CAMERA_CONTROLLER_TEST_TOLERANCE)
    {
        return false;
    }

    /* Mouse input alone must not move the camera. */
    if (camera.position.x != 0.0f ||
        camera.position.y != 0.0f ||
        camera.position.z != 0.0f)
    {
        return false;
    }

    return true;
}

static bool test_camera_controller_forward_movement(void)
{
    camera3d_t camera;
    camera_controller_t controller;

    if (!initialize_origin_camera(&camera, &controller))
        return false;

    const input_snapshot_t input =
    {
        .key_mask = INPUT_KEY_FORWARD,
        .mouse_dx = 0,
        .mouse_dy = 0
    };

    if (!camera_controller_update(
            &controller, &camera, &input, 0.5f))
    {
        return false;
    }

    /*
     * Initial view direction is +Z.
     * Default speed is 5 units/s; dt is 0.5 s.
     */
    if (fabsf(camera.position.x) >
        CAMERA_CONTROLLER_TEST_TOLERANCE)
    {
        return false;
    }

    if (fabsf(camera.position.y) >
        CAMERA_CONTROLLER_TEST_TOLERANCE)
    {
        return false;
    }

    if (fabsf(camera.position.z - 4.0f) >
        CAMERA_CONTROLLER_TEST_TOLERANCE)
    {
        return false;
    }

    return true;
}

static bool test_camera_controller_diagonal_speed(void)
{
    camera3d_t camera;
    camera_controller_t controller;

    if (!initialize_origin_camera(&camera, &controller))
        return false;

    const input_snapshot_t input =
    {
        .key_mask =
            INPUT_KEY_FORWARD | INPUT_KEY_STRAFE_RIGHT,
        .mouse_dx = 0,
        .mouse_dy = 0
    };

    if (!camera_controller_update(
            &controller, &camera, &input, 1.0f))
    {
        return false;
    }

    const float distance = sqrtf(
        camera.position.x * camera.position.x +
        camera.position.y * camera.position.y +
        camera.position.z * camera.position.z);

    if (fabsf(
            distance -
            CAMERA_CONTROLLER_DEFAULT_MOVE_SPEED) >
        CAMERA_CONTROLLER_TEST_TOLERANCE)
    {
        return false;
    }

    return true;
}

static bool test_camera_controller_pitch_clamp(void)
{
    camera3d_t camera;
    camera_controller_t controller;

    if (!initialize_origin_camera(&camera, &controller))
        return false;

    const input_snapshot_t input =
    {
        .key_mask = 0U,
        .mouse_dx = 0,
        .mouse_dy = 32767
    };

    if (!camera_controller_update(
            &controller, &camera, &input, 0.0f))
    {
        return false;
    }

    if (controller.pitch_rad !=
        CAMERA_CONTROLLER_MAX_PITCH_RAD)
    {
        return false;
    }

    if (camera.rotation_rad.x !=
        CAMERA_CONTROLLER_MAX_PITCH_RAD)
    {
        return false;
    }

    return true;
}

static bool test_camera_controller_rejects_negative_dt(void)
{
    camera3d_t camera;
    camera_controller_t controller;

    if (!initialize_origin_camera(&camera, &controller))
        return false;

    const input_snapshot_t input =
    {
        .key_mask = INPUT_KEY_FORWARD,
        .mouse_dx = 100,
        .mouse_dy = 20
    };

    if (camera_controller_update(
            &controller, &camera, &input, -1.0f))
    {
        return false;
    }

    /*
     * Rejected update must not partially change
     * either position or orientation.
     */
    if (camera.position.x != 0.0f ||
        camera.position.y != 0.0f ||
        camera.position.z != 0.0f)
    {
        return false;
    }

    if (camera.rotation_rad.x != 0.0f ||
        camera.rotation_rad.y != 0.0f)
    {
        return false;
    }

    if (controller.yaw_rad != 0.0f ||
        controller.pitch_rad != 0.0f)
    {
        return false;
    }

    return true;
}

static bool test_camera_controller_follows_vehicle_pose(void)
{
    camera3d_t camera;
    camera_controller_t controller;
    if (!initialize_origin_camera(&camera, &controller) ||
        !camera_controller_set_follow_mode(&controller, &camera, true))
    {
        return false;
    }

    const geometry3d_vector3_t vehicle_position =
    {
        .x = 10.0f, .y = 2.0f, .z = 20.0f
    };
    if (!camera_controller_follow_vehicle(
            &controller, &camera, &vehicle_position, 0.0f, 0.016f, true))
    {
        return false;
    }

    if (fabsf(camera.position.x - 10.0f) > CAMERA_CONTROLLER_TEST_TOLERANCE ||
        fabsf(camera.position.y - 8.0f) > CAMERA_CONTROLLER_TEST_TOLERANCE ||
        fabsf(camera.position.z - 12.0f) > CAMERA_CONTROLLER_TEST_TOLERANCE ||
        !camera.target_enabled ||
        fabsf(camera.target.x - 10.0f) > CAMERA_CONTROLLER_TEST_TOLERANCE ||
        fabsf(camera.target.y - 3.0f) > CAMERA_CONTROLLER_TEST_TOLERANCE ||
        fabsf(camera.target.z - 30.0f) > CAMERA_CONTROLLER_TEST_TOLERANCE)
    {
        return false;
    }

    const geometry3d_vector3_t next_position =
    {
        .x = 12.0f, .y = 2.0f, .z = 20.0f
    };
    if (!camera_controller_follow_vehicle(
            &controller, &camera, &next_position, 1.57079632679f,
            0.016f, false))
    {
        return false;
    }

    if (camera.position.x >= 10.0f || camera.position.z <= 12.0f ||
        camera.target.x <= 12.0f)
    {
        return false;
    }

    input_snapshot_t input = {0};
    input.key_mask = INPUT_KEY_FORWARD;
    if (!camera_controller_update(&controller, &camera, &input, 0.5f))
    {
        return false;
    }
    return true;
}

static bool test_camera_controller_returns_to_free_fly(void)
{
    camera3d_t camera;
    camera_controller_t controller;
    if (!initialize_origin_camera(&camera, &controller) ||
        !camera_controller_set_follow_mode(&controller, &camera, true))
    {
        return false;
    }

    const geometry3d_vector3_t position = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
    if (!camera_controller_follow_vehicle(
            &controller, &camera, &position, 0.0f, 0.016f, true) ||
        !camera_controller_set_follow_mode(&controller, &camera, false) ||
        controller.follow_mode || camera.target_enabled)
    {
        return false;
    }

    input_snapshot_t input = { .key_mask = INPUT_KEY_FORWARD };
    return camera_controller_update(&controller, &camera, &input, 0.5f) &&
           camera.position.z != 0.0f;
}

bool test_camera_controller_suite(void)
{
    if (!test_camera_controller_initializes_from_look_at())
        return false;

    if (!test_camera_controller_mouse_rotation())
        return false;

    if (!test_camera_controller_forward_movement())
        return false;

    if (!test_camera_controller_diagonal_speed())
        return false;

    if (!test_camera_controller_pitch_clamp())
        return false;

    if (!test_camera_controller_rejects_negative_dt())
        return false;

    if (!test_camera_controller_follows_vehicle_pose())
        return false;

    if (!test_camera_controller_returns_to_free_fly())
        return false;

    return true;
}
