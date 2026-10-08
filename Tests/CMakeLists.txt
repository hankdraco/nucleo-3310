cmake_minimum_required(VERSION 3.20)

project(nucleo_3310_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(nucleo_3310_tests
    test_runner.c
    test_pcd8544.c
    test_display.c
    ../Core/Src/pcd8544.c
    ../Core/Src/display.c
)

target_include_directories(nucleo_3310_tests PRIVATE
    ../Core/Inc
)