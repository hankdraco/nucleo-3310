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

static bool test_safety_fsm_starts_clear_and_overrides_route_motion(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 1.0f, .y = 1.0f, .z = 2.0f },
        .yaw_rad = 0.25f
    };
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_route_init(
        &context, &initial_pose, 2.0f, 5.0f,
        CAR_MANEUVER_STRAIGHT, 0.0f));
    CHECK(context.safety_state == CAR_MODIFIER_SAFETY_STATE_CLEAR);

    context.safety_state = CAR_MODIFIER_SAFETY_STATE_STOPPED;
    instance.transform.position.x = initial_pose.position.x;
    instance.transform.position.y = initial_pose.position.y;
    instance.transform.position.z = initial_pose.position.z;
    instance.transform.rotation_rad.y = initial_pose.yaw_rad;

    CHECK(car_modifier_safety_update(&instance, 0.5f, &context));
    CHECK(context.safety_state == CAR_MODIFIER_SAFETY_STATE_STOPPED);
    CHECK(car_modifier_update(&instance, 0.5f, &context));
    CHECK(test_float_equal(instance.transform.position.x, initial_pose.position.x));
    CHECK(test_float_equal(instance.transform.position.z, initial_pose.position.z));
    CHECK(test_float_equal(context.approach_distance_remaining, 5.0f));

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
        3.0f,
        CAR_MANEUVER_RIGHT,
        3.0f));
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
        27.2f,
        CAR_MANEUVER_RIGHT,
        10.0f));

    /* Reach the intersection edge and advance a little way into the turn. */
    CHECK(car_modifier_update(&instance, 9.1f, &context));

    CHECK(context.state == CAR_MODIFIER_STATE_CROSSING);
    CHECK(test_float_equal(context.trajectory.turn_pivot.x, 12.8f));
    CHECK(test_float_equal(context.trajectory.turn_pivot.y, 1.0f));
    CHECK(test_float_equal(context.trajectory.turn_pivot.z, -12.8f));

    return true;
}

static bool test_car_route_captures_left_pivot_relative_to_vehicle(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 5.8f, .y = 1.0f, .z = -40.0f },
        .yaw_rad = 0.0f
    };
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_route_init(
        &context,
        &initial_pose,
        3.0f,
        27.2f,
        CAR_MANEUVER_LEFT,
        18.6f));

    /* The pivot is captured at the left intersection corner. */
    CHECK(car_modifier_update(&instance, 9.1f, &context));

    CHECK(context.state == CAR_MODIFIER_STATE_CROSSING);
    CHECK(test_float_equal(context.trajectory.turn_pivot.x, -12.8f));
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
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_route_init(
        &context,
        &initial_pose,
        3.0f,
        3.0f,
        CAR_MANEUVER_RIGHT,
        3.0f));
    CHECK(car_modifier_update(&instance, 0.5f, &context));

    CHECK(context.state == CAR_MODIFIER_STATE_APPROACH_TURN);
    CHECK(test_float_equal(instance.transform.position.z, -4.5f));
    CHECK(test_float_equal(instance.transform.position.x, 0.0f));
    CHECK(test_float_equal(context.approach_distance_remaining, 1.5f));

    return true;
}


static bool test_car_route_supports_a_different_cardinal_heading(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = -6.0f, .y = 1.0f, .z = -6.0f },
        .yaw_rad = TEST_HALF_PI_RAD
    };
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_route_init(
        &context,
        &initial_pose,
        3.0f,
        3.0f,
        CAR_MANEUVER_RIGHT,
        3.0f));

    /* The car approaches along +X, then captures a pivot on its right (-Z). */
    CHECK(car_modifier_update(&instance, 1.0f, &context));

    CHECK(context.state == CAR_MODIFIER_STATE_CROSSING);
    CHECK(test_float_equal(context.trajectory.pose.position.x, -3.0f));
    CHECK(test_float_equal(context.trajectory.pose.position.z, -6.0f));
    CHECK(test_float_equal(context.trajectory.turn_pivot.x, -3.0f));
    CHECK(test_float_equal(context.trajectory.turn_pivot.y, 1.0f));
    CHECK(test_float_equal(context.trajectory.turn_pivot.z, -9.0f));

    return true;
}


static bool test_car_route_supports_straight_through_maneuver(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 2.0f, .y = 1.0f, .z = -6.0f },
        .yaw_rad = 0.0f
    };
    car_modifier_context_t context;
    geometry3d_instance_t instance = {0};
    CHECK(car_modifier_route_init(
        &context, &initial_pose, 3.0f, 3.0f,
        CAR_MANEUVER_STRAIGHT, 0.0f));
    CHECK(car_modifier_update(&instance, 2.0f, &context));
    CHECK(context.state == CAR_MODIFIER_STATE_DEPARTING);
    CHECK(context.trajectory.phase == CAR_TRAJECTORY_STRAIGHT);
    CHECK(test_float_equal(instance.transform.position.x, 2.0f));
    CHECK(test_float_equal(instance.transform.position.y, 1.0f));
    CHECK(test_float_equal(instance.transform.position.z, 0.0f));
    CHECK(test_float_equal(instance.transform.rotation_rad.y, 0.0f));
    return true;
}

static bool test_car_route_rejects_invalid_maneuver(void)
{
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    car_modifier_context_t context;
    CHECK(!car_modifier_route_init(
        &context, &initial_pose, 3.0f, 1.0f,
        (car_maneuver_t)99, 2.0f));
    return true;
}

static bool test_traffic_check_waits_for_turning_agent_to_clear(void)
{
    geometry3d_instance_t agents[2] = {0};
    car_modifier_context_t contexts[2] = {0};
    size_t agent_count = 2U;
    car_modifier_environment_t environment =
    {
        .agent_instances = agents,
        .agent_contexts = contexts,
        .agent_count = &agent_count
    };
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 0.0f, .y = 1.0f, .z = -3.0f },
        .yaw_rad = 0.0f
    };
    CHECK(car_modifier_route_init(
        &contexts[0], &initial_pose, 3.0f, 3.0f,
        CAR_MANEUVER_RIGHT, 3.0f));
    contexts[0].environment = &environment;
    contexts[1].state = CAR_MODIFIER_STATE_CROSSING;
    contexts[1].maneuver = CAR_MANEUVER_STRAIGHT;
    contexts[1].approach_yaw_rad = TEST_HALF_PI_RAD;

    CHECK(car_modifier_update(&agents[0], 1.0f, &contexts[0]));
    CHECK(contexts[0].state == CAR_MODIFIER_STATE_TRAFFIC_CHECK);
    CHECK(test_float_equal(agents[0].transform.position.z, 0.0f));

    const float waiting_x = agents[0].transform.position.x;
    const float waiting_z = agents[0].transform.position.z;
    CHECK(car_modifier_update(&agents[0], 0.5f, &contexts[0]));
    CHECK(contexts[0].state == CAR_MODIFIER_STATE_TRAFFIC_CHECK);
    CHECK(test_float_equal(agents[0].transform.position.x, waiting_x));
    CHECK(test_float_equal(agents[0].transform.position.z, waiting_z));

    contexts[1].state = CAR_MODIFIER_STATE_DEPARTING;
    CHECK(car_modifier_update(&agents[0], 0.0f, &contexts[0]));
    CHECK(contexts[0].state == CAR_MODIFIER_STATE_CROSSING);
    CHECK(contexts[0].trajectory.phase == CAR_TRAJECTORY_TURNING);
    CHECK(test_float_equal(agents[0].transform.position.z, waiting_z));
    return true;
}

static bool test_traffic_check_enters_turn_without_extra_wait_frame(void)
{
    geometry3d_instance_t agents[2] = {0};
    car_modifier_context_t contexts[2] = {0};
    size_t agent_count = 2U;
    car_modifier_environment_t environment =
    {
        .agent_instances = agents,
        .agent_contexts = contexts,
        .agent_count = &agent_count
    };
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 0.0f, .y = 1.0f, .z = -3.0f },
        .yaw_rad = 0.0f
    };
    CHECK(car_modifier_route_init(
        &contexts[0], &initial_pose, 3.0f, 3.0f,
        CAR_MANEUVER_RIGHT, 3.0f));
    contexts[0].environment = &environment;
    contexts[1].state = CAR_MODIFIER_STATE_DEPARTING;

    CHECK(car_modifier_update(&agents[0], 1.5f, &contexts[0]));
    CHECK(contexts[0].state == CAR_MODIFIER_STATE_CROSSING);
    CHECK(contexts[0].trajectory.phase == CAR_TRAJECTORY_TURNING);
    CHECK(agents[0].transform.position.z > -0.1f);
    CHECK(agents[0].transform.rotation_rad.y > 0.0f);
    return true;
}

static bool test_traffic_check_also_blocks_straight_through_route(void)
{
    geometry3d_instance_t agents[2] = {0};
    car_modifier_context_t contexts[2] = {0};
    size_t agent_count = 2U;
    car_modifier_environment_t environment =
    {
        .agent_instances = agents,
        .agent_contexts = contexts,
        .agent_count = &agent_count
    };
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 0.0f, .y = 1.0f, .z = -3.0f },
        .yaw_rad = 0.0f
    };
    CHECK(car_modifier_route_init(
        &contexts[0], &initial_pose, 3.0f, 3.0f,
        CAR_MANEUVER_STRAIGHT, 0.0f));
    contexts[0].environment = &environment;
    contexts[1].state = CAR_MODIFIER_STATE_CROSSING;
    contexts[1].maneuver = CAR_MANEUVER_STRAIGHT;
    contexts[1].approach_yaw_rad = TEST_HALF_PI_RAD;

    CHECK(car_modifier_update(&agents[0], 1.0f, &contexts[0]));
    CHECK(contexts[0].state == CAR_MODIFIER_STATE_TRAFFIC_CHECK);
    const float stop_line_z = agents[0].transform.position.z;
    CHECK(car_modifier_update(&agents[0], 0.5f, &contexts[0]));
    CHECK(test_float_equal(agents[0].transform.position.z, stop_line_z));

    contexts[1].state = CAR_MODIFIER_STATE_DEPARTING;
    CHECK(car_modifier_update(&agents[0], 0.0f, &contexts[0]));
    CHECK(contexts[0].state == CAR_MODIFIER_STATE_DEPARTING);
    CHECK(test_float_equal(agents[0].transform.position.z, stop_line_z));
    return true;
}

static bool test_straight_route_crosses_intersection_before_departing(void)
{
    geometry3d_instance_t agents[2] = {0};
    car_modifier_context_t contexts[2] = {0};
    size_t agent_count = 2U;
    car_modifier_environment_t environment =
    {
        .street_half_width = 5.0f,
        .agent_instances = agents,
        .agent_contexts = contexts,
        .agent_count = &agent_count
    };
    const car_trajectory_pose_t initial_pose =
    {
        .position = { .x = 0.0f, .y = 1.0f, .z = -3.0f },
        .yaw_rad = 0.0f
    };
    CHECK(car_modifier_route_init(
        &contexts[0], &initial_pose, 2.0f, 3.0f,
        CAR_MANEUVER_STRAIGHT, 0.0f));
    contexts[0].environment = &environment;
    contexts[1].state = CAR_MODIFIER_STATE_DEPARTING;

    CHECK(car_modifier_update(&agents[0], 1.5f, &contexts[0]));
    CHECK(contexts[0].state == CAR_MODIFIER_STATE_CROSSING);
    CHECK(test_float_equal(contexts[0].crossing_distance_remaining, 10.0f));
    CHECK(test_float_equal(agents[0].transform.position.z, 0.0f));

    CHECK(car_modifier_update(&agents[0], 2.0f, &contexts[0]));
    CHECK(contexts[0].state == CAR_MODIFIER_STATE_CROSSING);
    CHECK(test_float_equal(contexts[0].crossing_distance_remaining, 6.0f));
    CHECK(test_float_equal(agents[0].transform.position.z, 4.0f));

    CHECK(car_modifier_update(&agents[0], 3.0f, &contexts[0]));
    CHECK(contexts[0].state == CAR_MODIFIER_STATE_DEPARTING);
    CHECK(test_float_equal(contexts[0].crossing_distance_remaining, 0.0f));
    CHECK(test_float_equal(agents[0].transform.position.z, 10.0f));
    return true;
}

static bool test_traffic_conflict_matrix(void)
{
    const car_maneuver_t maneuvers[] =
    {
        CAR_MANEUVER_STRAIGHT,
        CAR_MANEUVER_LEFT,
        CAR_MANEUVER_RIGHT
    };
    const float relative_yaws[] =
    {
        0.0f,
        TEST_HALF_PI_RAD,
        2.0f * TEST_HALF_PI_RAD,
        -TEST_HALF_PI_RAD
    };
    /* Rows: own maneuver; columns: same/left/opposite/right, then B straight/left/right. */
    const bool expected[3][4][3] =
    {
        {
            { false, false, false },
            { true,  true,  false },
            { false, true,  false },
            { true,  true,  true  }
        },
        {
            { false, false, false },
            { true,  true,  false },
            { true,  true,  true  },
            { true,  true,  false }
        },
        {
            { false, false, false },
            { true,  false, false },
            { false, true,  false },
            { false, false, false }
        }
    };

    for (size_t own_index = 0U; own_index < 3U; ++own_index)
    {
        for (size_t relation = 0U; relation < 4U; ++relation)
        {
            for (size_t other_index = 0U; other_index < 3U; ++other_index)
            {
                geometry3d_instance_t agents[2] = {0};
                car_modifier_context_t contexts[2] = {0};
                size_t agent_count = 2U;
                car_modifier_environment_t environment =
                {
                    .street_half_width = 5.0f,
                    .agent_instances = agents,
                    .agent_contexts = contexts,
                    .agent_count = &agent_count
                };
                const car_trajectory_pose_t pose =
                {
                    .position = { .x = 0.0f, .y = 1.0f, .z = 0.0f },
                    .yaw_rad = 0.0f
                };
                const float radius = maneuvers[own_index] == CAR_MANEUVER_STRAIGHT
                    ? 0.0f : 2.0f;
                CHECK(car_modifier_route_init(
                    &contexts[0], &pose, 1.0f, 0.0f,
                    maneuvers[own_index], radius));
                contexts[0].state = CAR_MODIFIER_STATE_TRAFFIC_CHECK;
                contexts[0].environment = &environment;
                contexts[1].state = CAR_MODIFIER_STATE_CROSSING;
                contexts[1].maneuver = maneuvers[other_index];
                contexts[1].approach_yaw_rad = relative_yaws[relation];

                CHECK(car_modifier_update(&agents[0], 0.0f, &contexts[0]));
                CHECK((contexts[0].state == CAR_MODIFIER_STATE_TRAFFIC_CHECK) ==
                      expected[own_index][relation][other_index]);
            }
        }
    }
    return true;
}

bool test_car_modifier_suite(void)
{
    CHECK(test_car_modifier_maps_straight_pose());
    CHECK(test_car_modifier_maps_turn_yaw());
    CHECK(test_car_modifier_propagates_trajectory_failure_atomically());
    CHECK(test_car_modifier_rejects_invalid_initialization());
    CHECK(test_safety_fsm_starts_clear_and_overrides_route_motion());
    CHECK(test_car_route_consumes_dt_across_approach_turn_and_departure());
    CHECK(test_car_route_captures_pivot_relative_to_vehicle());
    CHECK(test_car_route_captures_left_pivot_relative_to_vehicle());
    CHECK(test_car_route_stays_on_approach_before_turn());
    CHECK(test_car_route_supports_a_different_cardinal_heading());
    CHECK(test_car_route_supports_straight_through_maneuver());
    CHECK(test_car_route_rejects_invalid_maneuver());
    CHECK(test_traffic_check_waits_for_turning_agent_to_clear());
    CHECK(test_traffic_check_enters_turn_without_extra_wait_frame());
    CHECK(test_traffic_check_also_blocks_straight_through_route());
    CHECK(test_straight_route_crosses_intersection_before_departing());
    CHECK(test_traffic_conflict_matrix());

    return true;
}
