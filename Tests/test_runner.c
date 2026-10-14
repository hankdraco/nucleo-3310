#include <stdbool.h>
#include <stdio.h>

bool test_pcd8544_suite(void);
bool test_display_suite(void);
bool test_input_protocol_suite(void);
bool test_input_state_suite(void);
bool test_camera3d_suite(void);
bool test_camera_controller_suite(void);
bool test_car_trajectory_suite(void);
bool test_car_modifier_suite(void);
bool test_car_spawn_suite(void);
bool test_car_lifecycle_suite(void);
bool test_modifier3d_suite(void);

int main(void)
{
    printf("Tests\n");
    printf("=====\n");

    if (!test_pcd8544_suite())
    {
        printf("PCD8544 TESTS FAILED\n");
        return 1;
    }
    printf("PCD8544 TESTS PASSED\n");

    if (!test_display_suite())
    {
        printf("DISPLAY TESTS FAILED\n");
        return 1;
    }
    printf("DISPLAY TESTS PASSED\n");

    if (!test_input_protocol_suite())
    {
        printf("INPUT PROTOCOL TESTS FAILED\n");
        return 1;
    }
    printf("INPUT PROTOCOL TESTS PASSED\n");

    if (!test_input_state_suite())
    {
        printf("INPUT STATE TESTS FAILED\n");
        return 1;
    }
    printf("INPUT STATE TESTS PASSED\n");

    if (!test_camera3d_suite())
    {
        printf("CAMERA3D TESTS FAILED\n");
        return 1;
    }
    printf("CAMERA3D TESTS PASSED\n");

    if (!test_camera_controller_suite())
    {
        printf("CAMERA CONTROLLER TESTS FAILED\n");
        return 1;
    }
    printf("CAMERA CONTROLLER TESTS PASSED\n");

    if (!test_car_trajectory_suite())
    {
        printf("CAR TRAJECTORY TESTS FAILED\n");
        return 1;
    }
    printf("CAR TRAJECTORY TESTS PASSED\n");

    if (!test_car_modifier_suite())
    {
        printf("CAR MODIFIER TESTS FAILED\n");
        return 1;
    }
    printf("CAR MODIFIER TESTS PASSED\n");

    if (!test_car_spawn_suite())
    {
        printf("CAR SPAWN TESTS FAILED\n");
        return 1;
    }
    printf("CAR SPAWN TESTS PASSED\n");

    if (!test_car_lifecycle_suite())
    {
        printf("CAR LIFECYCLE TESTS FAILED\n");
        return 1;
    }
    printf("CAR LIFECYCLE TESTS PASSED\n");

    if (!test_modifier3d_suite())
    {
        printf("MODIFIER3D TESTS FAILED\n");
        return 1;
    }
    printf("MODIFIER3D TESTS PASSED\n");

    printf("ALL TESTS PASSED\n");

    return 0;
}
