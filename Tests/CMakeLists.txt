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
    test_city_models.c
    ../Modules/engine3d/scene3d.c
    ../Modules/engine3d/cube_model.c
    ../Modules/engine3d/modifier3d.c
    ../Drivers/PCD8544/pcd8544.c
    ../Modules/engine3d/display.c
    ../Drivers/PCD8544/display_pcd8544.c
    ../Protocols/input/input_protocol.c
    ../App/input_state.c
    ../Modules/engine3d/camera3d.c
    ../App/camera_controller.c
    ../Modules/engine3d/geometry3d.c
    ../App/city_models.c
    ../Modules/engine3d/matrix4.c
    ../Modules/traffic/car_trajectory.c
    ../Modules/traffic/car_modifier.c
    ../Modules/traffic/car_footprint.c
)

target_include_directories(nucleo_3310_tests PRIVATE
    ..
)
