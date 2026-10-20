cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME nucleo-3310)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Drivers/PCD8544/pcd8544.c
    Drivers/PCD8544/pcd8544_transport_stm32.c
    Modules/engine3d/display.c
    Drivers/PCD8544/display_pcd8544.c
    App/app.c
    Modules/engine3d/projection.c
    Modules/engine3d/geometry3d.c
    Modules/engine3d/matrix4.c
    Modules/engine3d/cube_model.c
    Modules/engine3d/renderer3d.c
    Modules/engine3d/scene3d.c
    Modules/engine3d/camera3d.c
    Modules/engine3d/depth_buffer.c
    Modules/engine3d/rasterizer3d.c
    Modules/engine3d/modifier3d.c
    App/scene_loader.c
    App/city_models.c
    App/input_state.c
    Protocols/input/input_protocol.c
    Drivers/STM32/transmission_controller.c
    Protocols/telemetry/telemetry_protocol.c
    Drivers/STM32/telemetry_transport.c
    App/camera_controller.c
    App/camera_demo.c
    Modules/traffic/car_trajectory.c
    Modules/traffic/car_footprint.c
    Modules/traffic/car_modifier.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Project headers are included with their directory prefix.
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
