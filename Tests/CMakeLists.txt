cmake_minimum_required(VERSION 3.20)

project(nucleo_3310_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(nucleo_3310_tests
    test_runner.c
    test_pcd8544.c
    test_display.c
    test_fake_transport.c
    test_input_protocol.c
    test_input_state.c
    test_camera3d.c
    test_camera_controller.c
    test_car_trajectory.c
    test_car_modifier.c
    test_car_spawn.c
    test_car_footprint.c
    test_modifier3d.c
    test_scene3d_agents.c
    ../Core/Src/scene3d.c
    ../Core/Src/cube_model.c
    ../Core/Src/modifier3d.c
    ../Core/Src/pcd8544.c
    ../Core/Src/display.c
    ../Core/Src/display_pcd8544.c
    ../Core/Src/input_protocol.c
    ../Core/Src/input_state.c
    ../Core/Src/camera3d.c
    ../Core/Src/camera_controller.c
    ../Core/Src/geometry3d.c
    ../Core/Src/matrix4.c
    ../Core/Src/car_trajectory.c
    ../Core/Src/car_modifier.c
    ../Core/Src/car_footprint.c
)

target_include_directories(nucleo_3310_tests PRIVATE
    ../Core/Inc
)
