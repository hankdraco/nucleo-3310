#include "car_modifier.h"

#include <math.h>
#include <stddef.h>

#include "modifier3d.h"
#include "test_common.h"

#define TEST_TOLERANCE 0.0001f
#define TEST_HALF_PI_RAD 1.57079632679489661923f

static bool test_float_equal(float actual, float expected)
{
    return fabsf(actual - expected) <= TEST_TOLERANCE;
}

static bool test_car_modifier_maps_straight_pose(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 1.0f, .y = 0.5f, .z = -2.0f },
        .yaw_rad = 0.0f
    };
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_context_init(&context, &initial_pose, 3.0f));

    const modifier3d_config_t config =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = car_modifier_update,
        .context = &context
    };

    CHECK(modifier3d_add(&instance, &config));
    CHECK(modifier3d_update_instance(&instance, 0.5f));

    CHECK(test_float_equal(instance.transform.position.x, 1.0f));
    CHECK(test_float_equal(instance.transform.position.y, 0.5f));
    CHECK(test_float_equal(instance.transform.position.z, -0.5f));
    CHECK(test_float_equal(instance.transform.rotation_rad.y, 0.0f));
    CHECK(test_float_equal(context.trajectory.pose.position.z, -0.5f));

    return true;
}

static bool test_car_modifier_maps_turn_yaw(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = -2.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_vec3_t pivot =
    {
        .x = 0.0f, .y = 0.0f, .z = 0.0f
    };
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_context_init(&context, &initial_pose, 2.0f));

    const modifier3d_config_t config =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = car_modifier_update,
        .context = &context
    };

    CHECK(modifier3d_add(&instance, &config));
    CHECK(car_trajectory_begin_turn(
        &context.trajectory, &pivot, CAR_TURN_DIRECTION_RIGHT));
    CHECK(modifier3d_update_instance(&instance, TEST_HALF_PI_RAD));

    CHECK(test_float_equal(instance.transform.position.x, 0.0f));
    CHECK(test_float_equal(instance.transform.position.y, 0.0f));
    CHECK(test_float_equal(instance.transform.position.z, 2.0f));
    CHECK(test_float_equal(
        instance.transform.rotation_rad.y, TEST_HALF_PI_RAD));

    return true;
}

static bool test_car_modifier_propagates_trajectory_failure_atomically(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 1.0f, .y = 2.0f, .z = 3.0f },
        .yaw_rad = 0.25f
    };
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_context_init(&context, &initial_pose, 2.0f));

    const modifier3d_config_t config =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = car_modifier_update,
        .context = &context
    };

    CHECK(modifier3d_add(&instance, &config));

    instance.transform.position.x = 10.0f;
    instance.transform.position.y = 11.0f;
    instance.transform.position.z = 12.0f;
    instance.transform.rotation_rad.y = 0.75f;

    const car_trajectory_t trajectory_before = context.trajectory;
    context.trajectory.speed_units_per_second = NAN;

    CHECK(!modifier3d_update_instance(&instance, 0.1f));
    CHECK(test_float_equal(instance.transform.position.x, 10.0f));
    CHECK(test_float_equal(instance.transform.position.y, 11.0f));
    CHECK(test_float_equal(instance.transform.position.z, 12.0f));
    CHECK(test_float_equal(instance.transform.rotation_rad.y, 0.75f));
    CHECK(test_float_equal(
        context.trajectory.pose.position.x,
        trajectory_before.pose.position.x));

    return true;
}

static bool test_car_modifier_rejects_invalid_initialization(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    car_modifier_context_t context;

    CHECK(!car_modifier_context_init(NULL, &initial_pose, 1.0f));
    CHECK(!car_modifier_context_init(&context, &initial_pose, 0.0f));

    return true;
}

bool test_car_modifier_suite(void)
{
    CHECK(test_car_modifier_maps_straight_pose());
    CHECK(test_car_modifier_maps_turn_yaw());
    CHECK(test_car_modifier_propagates_trajectory_failure_atomically());
    CHECK(test_car_modifier_rejects_invalid_initialization());

    return true;
}
