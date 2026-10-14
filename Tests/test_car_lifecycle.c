#include <math.h>

#include "car_lifecycle.h"
#include "test_common.h"

static bool test_car_lifecycle_accepts_positions_inside_city_plan(void)
{
    const car_trajectory_pose_t pose =
    {
        .position = { .x = 12.0f, .y = 4.0f, .z = -30.0f },
        .yaw_rad = 1.0f
    };

    CHECK(car_lifecycle_is_inside_city_plan(&pose, 160.0f, 160.0f));

    return true;
}

static bool test_car_lifecycle_includes_city_plan_boundary(void)
{
    const car_trajectory_pose_t positive_x_boundary =
    {
        .position = { .x = 160.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_pose_t negative_z_boundary =
    {
        .position = { .x = 0.0f, .y = 0.0f, .z = -160.0f },
        .yaw_rad = 0.0f
    };

    CHECK(car_lifecycle_is_inside_city_plan(
        &positive_x_boundary, 160.0f, 160.0f));
    CHECK(car_lifecycle_is_inside_city_plan(
        &negative_z_boundary, 160.0f, 160.0f));

    return true;
}

static bool test_car_lifecycle_rejects_positions_outside_city_plan(void)
{
    const car_trajectory_pose_t outside_positive_x =
    {
        .position = { .x = 160.01f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_pose_t outside_negative_x =
    {
        .position = { .x = -160.01f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_pose_t outside_positive_z =
    {
        .position = { .x = 0.0f, .y = 0.0f, .z = 160.01f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_pose_t outside_negative_z =
    {
        .position = { .x = 0.0f, .y = 0.0f, .z = -160.01f },
        .yaw_rad = 0.0f
    };

    CHECK(!car_lifecycle_is_inside_city_plan(
        &outside_positive_x, 160.0f, 160.0f));
    CHECK(!car_lifecycle_is_inside_city_plan(
        &outside_negative_x, 160.0f, 160.0f));
    CHECK(!car_lifecycle_is_inside_city_plan(
        &outside_positive_z, 160.0f, 160.0f));
    CHECK(!car_lifecycle_is_inside_city_plan(
        &outside_negative_z, 160.0f, 160.0f));

    return true;
}

static bool test_car_lifecycle_rejects_invalid_inputs(void)
{
    const car_trajectory_pose_t invalid_position =
    {
        .position = { .x = NAN, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };
    const car_trajectory_pose_t valid_position =
    {
        .position = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
        .yaw_rad = 0.0f
    };

    CHECK(!car_lifecycle_is_inside_city_plan(NULL, 160.0f, 160.0f));
    CHECK(!car_lifecycle_is_inside_city_plan(
        &invalid_position, 160.0f, 160.0f));
    CHECK(!car_lifecycle_is_inside_city_plan(
        &valid_position, 0.0f, 160.0f));
    CHECK(!car_lifecycle_is_inside_city_plan(
        &valid_position, 160.0f, INFINITY));

    return true;
}

bool test_car_lifecycle_suite(void)
{
    CHECK(test_car_lifecycle_accepts_positions_inside_city_plan());
    CHECK(test_car_lifecycle_includes_city_plan_boundary());
    CHECK(test_car_lifecycle_rejects_positions_outside_city_plan());
    CHECK(test_car_lifecycle_rejects_invalid_inputs());

    return true;
}
