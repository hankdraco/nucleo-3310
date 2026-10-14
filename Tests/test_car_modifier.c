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


static bool test_car_route_consumes_dt_across_approach_turn_and_departure(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 0.0f, .y = 1.0f, .z = -6.0f },
        .yaw_rad = 0.0f
    };
    const float dt_seconds = 3.0f;
    const float turn_time_seconds = TEST_HALF_PI_RAD; /* speed / radius = 1 */
    const float remaining_straight_seconds =
        dt_seconds - 1.0f - turn_time_seconds;

    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_route_init(
        &context,
        &initial_pose,
        3.0f,
        -3.0f,
        3.0f,
        CAR_TURN_DIRECTION_RIGHT));
    CHECK(context.state == CAR_MODIFIER_STATE_APPROACH_TURN);

    CHECK(car_modifier_update(&instance, dt_seconds, &context));

    /*
     * One second reaches the turn, PI/2 seconds completes it, and the
     * remaining time advances the car along +X during the same update.
     */
    CHECK(context.state == CAR_MODIFIER_STATE_DEPARTING);
    CHECK(context.trajectory.phase == CAR_TRAJECTORY_STRAIGHT);
    CHECK(test_float_equal(
        instance.transform.position.x,
        3.0f + 3.0f * remaining_straight_seconds));
    CHECK(test_float_equal(instance.transform.position.y, 1.0f));
    CHECK(test_float_equal(instance.transform.position.z, 0.0f));
    CHECK(test_float_equal(
        instance.transform.rotation_rad.y, TEST_HALF_PI_RAD));

    return true;
}

static bool test_car_route_captures_pivot_relative_to_vehicle(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 2.8f, .y = 1.0f, .z = -40.0f },
        .yaw_rad = 0.0f
    };
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_route_init(
        &context,
        &initial_pose,
        3.0f,
        -12.8f,
        10.0f,
        CAR_TURN_DIRECTION_RIGHT));

    /* Reach the intersection edge and advance a little way into the turn. */
    CHECK(car_modifier_update(&instance, 9.1f, &context));

    CHECK(context.state == CAR_MODIFIER_STATE_TURNING);
    CHECK(test_float_equal(context.trajectory.turn_pivot.x, 12.8f));
    CHECK(test_float_equal(context.trajectory.turn_pivot.y, 1.0f));
    CHECK(test_float_equal(context.trajectory.turn_pivot.z, -12.8f));

    return true;
}

static bool test_car_route_stays_on_approach_before_turn(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 0.0f, .y = 1.0f, .z = -6.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_vec3_t pivot =
    {
        .x = 3.0f, .y = 0.0f, .z = -3.0f
    };
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_route_init(
        &context,
        &initial_pose,
        3.0f,
        &pivot,
        CAR_TURN_DIRECTION_RIGHT));
    CHECK(car_modifier_update(&instance, 0.5f, &context));

    CHECK(context.state == CAR_MODIFIER_STATE_APPROACH_TURN);
    CHECK(test_float_equal(instance.transform.position.z, -4.5f));
    CHECK(test_float_equal(instance.transform.position.x, 0.0f));

    return true;
}

bool test_car_modifier_suite(void)
{
    CHECK(test_car_modifier_maps_straight_pose());
    CHECK(test_car_modifier_maps_turn_yaw());
    CHECK(test_car_modifier_propagates_trajectory_failure_atomically());
    CHECK(test_car_modifier_rejects_invalid_initialization());
    CHECK(test_car_route_consumes_dt_across_approach_turn_and_departure());
    CHECK(test_car_route_captures_pivot_relative_to_vehicle());
    CHECK(test_car_route_stays_on_approach_before_turn());

    return true;
}
