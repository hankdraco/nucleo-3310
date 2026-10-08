cmake_minimum_required(VERSION 3.20)

project(pcd8544_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(pcd8544_tests
    test_pcd8544.c
    ../Core/Src/pcd8544.c
)

target_include_directories(pcd8544_tests PRIVATE
    ../Core/Inc
)