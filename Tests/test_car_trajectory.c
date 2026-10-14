#include "car_trajectory.h"
#include "car_trajectory_instance.h"
#include "geometry3d.h"

#include <math.h>
#include <stddef.h>

#include "test_common.h"

#define TEST_PI_RAD              3.14159265358979323846f
#define TEST_HALF_PI_RAD         1.57079632679489661923f
#define TEST_QUARTER_PI_RAD      0.78539816339744830962f
#define TEST_TOLERANCE           0.0001f

static bool test_float_equal(float actual, float expected)
{
    return fabsf(actual - expected) <= TEST_TOLERANCE;
}

static bool test_pose_equal(
    const car_trajectory_pose_t *actual,
    const car_trajectory_pose_t *expected)
{
    return test_float_equal(actual->position.x, expected->position.x) &&
           test_float_equal(actual->position.y, expected->position.y) &&
           test_float_equal(actual->position.z, expected->position.z) &&
           test_float_equal(actual->yaw_rad, expected->yaw_rad);
}

static bool test_initialization_and_straight_motion(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 1.0f, .y = 0.25f, .z = -2.0f },
        .yaw_rad = 0.0f
    };

    car_trajectory_t trajectory;

    CHECK(car_trajectory_init(&trajectory, &initial_pose, 3.0f));
    CHECK(trajectory.phase == CAR_TRAJECTORY_STRAIGHT);
    CHECK(test_pose_equal(&trajectory.pose, &initial_pose));

    CHECK(car_trajectory_update(&trajectory, 2.0f));
    CHECK(test_float_equal(trajectory.pose.position.x, 1.0f));
    CHECK(test_float_equal(trajectory.pose.position.y, 0.25f));
    CHECK(test_float_equal(trajectory.pose.position.z, 4.0f));
    CHECK(test_float_equal(trajectory.pose.yaw_rad, 0.0f));

    return true;
}

static bool test_straight_motion_uses_yaw(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = TEST_HALF_PI_RAD
    };

    car_trajectory_t trajectory;

    CHECK(car_trajectory_init(&trajectory, &initial_pose, 2.0f));
    CHECK(car_trajectory_update(&trajectory, 0.5f));
    CHECK(test_float_equal(trajectory.pose.position.x, 1.0f));
    CHECK(test_float_equal(trajectory.pose.position.z, 0.0f));

    return true;
}

static bool test_turn_preserves_radius_and_linear_speed(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = -2.0f, .y = 0.5f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_vec3_t pivot =
    {
        .x = 0.0f, .y = 0.0f, .z = 0.0f
    };

    car_trajectory_t trajectory;

    CHECK(car_trajectory_init(&trajectory, &initial_pose, 2.0f));
    CHECK(car_trajectory_begin_turn(
        &trajectory, &pivot, CAR_TURN_DIRECTION_RIGHT));

    CHECK(test_float_equal(trajectory.turn_radius, 2.0f));
    CHECK(test_float_equal(
        trajectory.turn_angular_speed_rad_per_second, 1.0f));

    /* At v=2 and r=2, PI/4 radians takes PI/4 seconds. */
    CHECK(car_trajectory_update(&trajectory, TEST_QUARTER_PI_RAD));
    CHECK(trajectory.phase == CAR_TRAJECTORY_TURNING);
    CHECK(test_float_equal(trajectory.turn_progress_rad, TEST_QUARTER_PI_RAD));
    CHECK(test_float_equal(trajectory.pose.yaw_rad, TEST_QUARTER_PI_RAD));
    CHECK(test_float_equal(
        trajectory.pose.position.x, -1.41421356237f));
    CHECK(test_float_equal(
        trajectory.pose.position.z, 1.41421356237f));

    {
        const float dx = trajectory.pose.position.x - pivot.x;
        const float dz = trajectory.pose.position.z - pivot.z;
        CHECK(test_float_equal(sqrtf(dx * dx + dz * dz), 2.0f));
    }

    return true;
}

static bool test_right_turn_finishes_at_exact_quarter_turn(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = -2.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_vec3_t pivot = { .x = 0.0f, .y = 0.0f, .z = 0.0f };

    car_trajectory_t trajectory;

    CHECK(car_trajectory_init(&trajectory, &initial_pose, 2.0f));
    CHECK(car_trajectory_begin_turn(
        &trajectory, &pivot, CAR_TURN_DIRECTION_RIGHT));
    CHECK(car_trajectory_update(&trajectory, TEST_HALF_PI_RAD));

    CHECK(trajectory.phase == CAR_TRAJECTORY_STRAIGHT);
    CHECK(test_float_equal(trajectory.turn_progress_rad, TEST_HALF_PI_RAD));
    CHECK(test_float_equal(trajectory.pose.position.x, 0.0f));
    CHECK(test_float_equal(trajectory.pose.position.z, 2.0f));
    CHECK(test_float_equal(trajectory.pose.yaw_rad, TEST_HALF_PI_RAD));

    return true;
}

static bool test_left_turn_has_the_opposite_yaw_sign(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 2.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_vec3_t pivot = { .x = 0.0f, .y = 0.0f, .z = 0.0f };

    car_trajectory_t trajectory;

    CHECK(car_trajectory_init(&trajectory, &initial_pose, 2.0f));
    CHECK(car_trajectory_begin_turn(
        &trajectory, &pivot, CAR_TURN_DIRECTION_LEFT));
    CHECK(car_trajectory_update(&trajectory, TEST_HALF_PI_RAD));

    CHECK(trajectory.phase == CAR_TRAJECTORY_STRAIGHT);
    CHECK(test_float_equal(trajectory.pose.position.x, 0.0f));
    CHECK(test_float_equal(trajectory.pose.position.z, 2.0f));
    CHECK(test_float_equal(trajectory.pose.yaw_rad, -TEST_HALF_PI_RAD));

    return true;
}

static bool test_large_dt_preserves_time_after_turn(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = -2.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_vec3_t pivot = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
    const float dt_seconds = 2.0f;
    const float turn_time_seconds = TEST_HALF_PI_RAD; /* omega = 1 rad/s */
    const float remaining_seconds = dt_seconds - turn_time_seconds;

    car_trajectory_t trajectory;

    CHECK(car_trajectory_init(&trajectory, &initial_pose, 2.0f));
    CHECK(car_trajectory_begin_turn(
        &trajectory, &pivot, CAR_TURN_DIRECTION_RIGHT));
    CHECK(car_trajectory_update(&trajectory, dt_seconds));

    CHECK(trajectory.phase == CAR_TRAJECTORY_STRAIGHT);
    CHECK(test_float_equal(
        trajectory.pose.position.x, 2.0f * remaining_seconds));
    CHECK(test_float_equal(trajectory.pose.position.z, 2.0f));
    CHECK(test_float_equal(trajectory.pose.yaw_rad, TEST_HALF_PI_RAD));

    return true;
}

static bool test_turn_result_does_not_depend_on_update_partitioning(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = -2.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_vec3_t pivot = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
    car_trajectory_t one_step;
    car_trajectory_t two_steps;

    CHECK(car_trajectory_init(&one_step, &initial_pose, 2.0f));
    CHECK(car_trajectory_init(&two_steps, &initial_pose, 2.0f));
    CHECK(car_trajectory_begin_turn(
        &one_step, &pivot, CAR_TURN_DIRECTION_RIGHT));
    CHECK(car_trajectory_begin_turn(
        &two_steps, &pivot, CAR_TURN_DIRECTION_RIGHT));

    CHECK(car_trajectory_update(&one_step, 1.0f));
    CHECK(car_trajectory_update(&two_steps, 0.3f));
    CHECK(car_trajectory_update(&two_steps, 0.7f));

    CHECK(test_pose_equal(&one_step.pose, &two_steps.pose));
    CHECK(test_float_equal(
        one_step.turn_progress_rad, two_steps.turn_progress_rad));
    CHECK(one_step.phase == two_steps.phase);

    return true;
}

static bool test_invalid_operations_do_not_change_state(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = -2.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_vec3_t coincident_pivot =
    {
        .x = -2.0f, .y = 9.0f, .z = 0.0f
    };
    car_trajectory_t trajectory;
    car_trajectory_pose_t pose_before;
    car_trajectory_phase_t phase_before;

    CHECK(!car_trajectory_init(&trajectory, &initial_pose, 0.0f));
    CHECK(car_trajectory_init(&trajectory, &initial_pose, 2.0f));
    CHECK(!car_trajectory_begin_turn(
        &trajectory, &coincident_pivot, CAR_TURN_DIRECTION_RIGHT));

    pose_before = trajectory.pose;
    phase_before = trajectory.phase;
    CHECK(!car_trajectory_update(&trajectory, -0.1f));
    CHECK(test_pose_equal(&trajectory.pose, &pose_before));
    CHECK(trajectory.phase == phase_before);

    CHECK(!car_trajectory_begin_turn(
        &trajectory, &coincident_pivot, (car_turn_direction_t)0));
    CHECK(test_pose_equal(&trajectory.pose, &pose_before));
    CHECK(trajectory.phase == phase_before);

    return true;
}


static bool test_trajectory_pose_is_applied_to_geometry_instance(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 1.0f, .y = 0.5f, .z = -2.0f },
        .yaw_rad = TEST_QUARTER_PI_RAD
    };
    const car_trajectory_pose_t turn_start =
    {
        .position = { .x = -2.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_vec3_t turn_pivot =
    {
        .x = 0.0f, .y = 0.0f, .z = 0.0f
    };
    car_trajectory_t trajectory;
    geometry3d_instance_t instance = {0};

    instance.transform.position.x = -99.0f;
    instance.transform.position.y = -99.0f;
    instance.transform.position.z = -99.0f;
    instance.transform.rotation_rad.y = -99.0f;

    CHECK(car_trajectory_init(&trajectory, &initial_pose, 2.0f));
    CHECK(car_trajectory_update(&trajectory, 0.5f));
    CHECK(car_trajectory_apply_pose(&trajectory, &instance));

    CHECK(test_float_equal(
        instance.transform.position.x, trajectory.pose.position.x));
    CHECK(test_float_equal(
        instance.transform.position.y, trajectory.pose.position.y));
    CHECK(test_float_equal(
        instance.transform.position.z, trajectory.pose.position.z));
    CHECK(test_float_equal(
        instance.transform.rotation_rad.y, trajectory.pose.yaw_rad));

    /* The same adapter must preserve both position and yaw during a turn. */
    CHECK(car_trajectory_init(&trajectory, &turn_start, 2.0f));
    CHECK(car_trajectory_begin_turn(
        &trajectory, &turn_pivot, CAR_TURN_DIRECTION_RIGHT));
    CHECK(car_trajectory_update(&trajectory, TEST_QUARTER_PI_RAD));
    CHECK(car_trajectory_apply_pose(&trajectory, &instance));

    CHECK(test_float_equal(
        instance.transform.position.x, trajectory.pose.position.x));
    CHECK(test_float_equal(
        instance.transform.position.y, trajectory.pose.position.y));
    CHECK(test_float_equal(
        instance.transform.position.z, trajectory.pose.position.z));
    CHECK(test_float_equal(
        instance.transform.rotation_rad.y, TEST_QUARTER_PI_RAD));

    /* Invalid pose data must not partially overwrite the render transform. */
    {
        const geometry3d_transform_t before = instance.transform;
        trajectory.pose.yaw_rad = INFINITY;
        CHECK(!car_trajectory_apply_pose(&trajectory, &instance));
        CHECK(test_float_equal(
            instance.transform.position.x, before.position.x));
        CHECK(test_float_equal(
            instance.transform.position.y, before.position.y));
        CHECK(test_float_equal(
            instance.transform.position.z, before.position.z));
        CHECK(test_float_equal(
            instance.transform.rotation_rad.y, before.rotation_rad.y));
    }

    return true;
}

bool test_car_trajectory_suite(void)
{
    CHECK(test_initialization_and_straight_motion());
    CHECK(test_straight_motion_uses_yaw());
    CHECK(test_turn_preserves_radius_and_linear_speed());
    CHECK(test_right_turn_finishes_at_exact_quarter_turn());
    CHECK(test_left_turn_has_the_opposite_yaw_sign());
    CHECK(test_large_dt_preserves_time_after_turn());
    CHECK(test_turn_result_does_not_depend_on_update_partitioning());
    CHECK(test_invalid_operations_do_not_change_state());
    CHECK(test_trajectory_pose_is_applied_to_geometry_instance());

    return true;
}
bool test_car_trajectory_suite(void)
{
    CHECK(test_initialization_and_straight_motion());
    CHECK(test_straight_motion_uses_yaw());
    CHECK(test_turn_preserves_radius_and_linear_speed());
    CHECK(test_right_turn_finishes_at_exact_quarter_turn());
    CHECK(test_left_turn_has_the_opposite_yaw_sign());
    CHECK(test_large_dt_preserves_time_after_turn());
    CHECK(test_turn_result_does_not_depend_on_update_partitioning());
    CHECK(test_invalid_operations_do_not_change_state());
    CHECK(test_trajectory_pose_is_applied_to_geometry_instance());

    return true;
}
