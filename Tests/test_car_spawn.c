#include "car_modifier.h"

#include <math.h>
#include <stdint.h>

#include "test_common.h"

#define TEST_TOLERANCE (0.0001f)

static bool test_float_equal(float actual, float expected)
{
    return fabsf(actual - expected) <= TEST_TOLERANCE;
}

static car_modifier_environment_t make_test_environment(uint32_t seed)
{
    return (car_modifier_environment_t)
    {
        .city_half_extent_x = 160.0f,
        .city_half_extent_z = 160.0f,
        .street_half_width = 12.8f,
        .lane_edge_offset = 7.0f,
        .minimum_speed_units_per_second = 12.0f,
        .maximum_speed_units_per_second = 18.0f,
        .random_state = seed
    };
}

static bool test_car_spawn_state_is_processed_with_zero_dt(void)
{
    car_modifier_environment_t environment = make_test_environment(0x12345678U);
    car_modifier_context_t context = {0};
    geometry3d_instance_t instance = {0};

    CHECK(car_modifier_context_init_spawn(&context, &environment));
    CHECK(context.state == CAR_MODIFIER_STATE_SPAWN);
    CHECK(context.environment == &environment);

    /* SPAWN is a one-shot state transition, not elapsed-time simulation. */
    CHECK(car_modifier_update(&instance, 0.0f, &context));

    CHECK(context.state == CAR_MODIFIER_STATE_APPROACH_TURN);
    CHECK(context.environment == &environment);
    CHECK(context.trajectory.speed_units_per_second >=
          environment.minimum_speed_units_per_second);
    CHECK(context.trajectory.speed_units_per_second <=
          environment.maximum_speed_units_per_second);
    if (context.maneuver == CAR_MANEUVER_RIGHT)
    {
        CHECK(test_float_equal(
            context.turn_radius, environment.lane_edge_offset));
    }
    else if (context.maneuver == CAR_MANEUVER_LEFT)
    {
        CHECK(test_float_equal(
            context.turn_radius,
            2.0f * environment.street_half_width -
                environment.lane_edge_offset));
    }
    else
    {
        CHECK(context.maneuver == CAR_MANEUVER_STRAIGHT);
        CHECK(test_float_equal(context.turn_radius, 0.0f));
    }
    CHECK(context.trajectory.pose.position.x >=
          -environment.city_half_extent_x);
    CHECK(context.trajectory.pose.position.x <=
          environment.city_half_extent_x);
    CHECK(context.trajectory.pose.position.z >=
          -environment.city_half_extent_z);
    CHECK(context.trajectory.pose.position.z <=
          environment.city_half_extent_z);

    CHECK(test_float_equal(
        instance.transform.position.x,
        context.trajectory.pose.position.x));
    CHECK(test_float_equal(
        instance.transform.position.y,
        context.trajectory.pose.position.y));
    CHECK(test_float_equal(
        instance.transform.position.z,
        context.trajectory.pose.position.z));
    CHECK(test_float_equal(
        instance.transform.rotation_rad.y,
        context.trajectory.pose.yaw_rad));

    return true;
}

static bool test_car_respawns_immediately_without_consuming_leftover_dt(void)
{
    car_modifier_environment_t environment_a = make_test_environment(0xA5A5A5A5U);
    car_modifier_environment_t environment_b = make_test_environment(0xA5A5A5A5U);
    car_modifier_context_t context_a = {0};
    car_modifier_context_t context_b = {0};
    geometry3d_instance_t instance_a = {0};
    geometry3d_instance_t instance_b = {0};

    CHECK(car_modifier_context_init_spawn(&context_a, &environment_a));
    CHECK(car_modifier_context_init_spawn(&context_b, &environment_b));

    /* Bring both independently seeded cars through their first spawn. */
    CHECK(car_modifier_update(&instance_a, 0.0f, &context_a));
    CHECK(car_modifier_update(&instance_b, 0.0f, &context_b));

    /*
     * Force car A to have left the city. Its update must respawn it in this
     * same call and discard the remaining dt instead of moving the new car.
     * Car B explicitly performs the equivalent SPAWN transition with dt=0.
     */
    context_a.state = CAR_MODIFIER_STATE_DEPARTING;
    context_a.trajectory.pose.position.x =
        environment_a.city_half_extent_x + 1.0f;
    context_b.state = CAR_MODIFIER_STATE_SPAWN;

    CHECK(car_modifier_update(&instance_a, 10.0f, &context_a));
    CHECK(car_modifier_update(&instance_b, 0.0f, &context_b));

    CHECK(context_a.state == CAR_MODIFIER_STATE_APPROACH_TURN);
    CHECK(context_b.state == CAR_MODIFIER_STATE_APPROACH_TURN);

    CHECK(test_float_equal(
        context_a.trajectory.pose.position.x,
        context_b.trajectory.pose.position.x));
    CHECK(test_float_equal(
        context_a.trajectory.pose.position.y,
        context_b.trajectory.pose.position.y));
    CHECK(test_float_equal(
        context_a.trajectory.pose.position.z,
        context_b.trajectory.pose.position.z));
    CHECK(test_float_equal(
        context_a.trajectory.pose.yaw_rad,
        context_b.trajectory.pose.yaw_rad));
    CHECK(test_float_equal(
        instance_a.transform.position.x,
        instance_b.transform.position.x));
    CHECK(test_float_equal(
        instance_a.transform.position.z,
        instance_b.transform.position.z));

    return true;
}

bool test_car_spawn_suite(void)
{
    CHECK(test_car_spawn_state_is_processed_with_zero_dt());
    CHECK(test_car_respawns_immediately_without_consuming_leftover_dt());

    return true;
}
