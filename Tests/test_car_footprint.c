#include "car_footprint.h"

#include "test_common.h"

bool test_car_footprint_suite(void)
{
    const car_footprint_t a = { .center_x = 0.0f, .center_z = 0.0f,
        .yaw_rad = 0.0f, .half_width = 1.0f, .half_length = 2.0f };
    car_footprint_t b = { .center_x = 0.0f, .center_z = 3.0f,
        .yaw_rad = 0.0f, .half_width = 1.0f, .half_length = 2.0f };

    CHECK(car_footprints_overlap(&a, &b, 0.0f));
    b.center_z = 4.01f;
    CHECK(!car_footprints_overlap(&a, &b, 0.0f));
    CHECK(car_footprints_overlap(&a, &b, 0.02f));
    b.center_x = 1.0f;
    b.center_z = 0.0f;
    b.yaw_rad = 0.785398163f;
    CHECK(car_footprints_overlap(&a, &b, 0.0f));
    b.center_x = 10.0f;
    CHECK(!car_footprints_overlap(&a, &b, 0.0f));
    CHECK(!car_footprints_overlap(&a, &b, -1.0f));
    return true;
}
