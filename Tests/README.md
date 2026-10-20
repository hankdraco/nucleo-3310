# Host Tests

The `Tests` directory contains native host-side tests for the portable parts of the project. The tests are built as a separate Windows executable and do not require the STM32 board, STM32 HAL, ST-Link, or LCD hardware.

The host build currently covers:

- the PCD8544 driver, using a fake transport;
- the display abstraction;
- the input-protocol parser;
- the input-state logic;
- camera orientation and view-matrix behavior;
- camera-controller input and movement logic;
- car trajectory, route-FSM and safety-FSM scaffold behavior;
- generic modifier behavior;
- agent pre-update and movement phase ordering in the scene update loop.

These tests verify portable logic. They do not replace on-target verification of UART interrupts, BSP integration, timing, or synchronization between an ISR and the main loop.

## Requirements

The tests require:

- CMake
- Ninja
- GCC from W64DevKit

Make sure `gcc` and `ninja` are available in `PATH`.

## Configure the test build

From the project root:

```powershell
cmake -S Tests -B Tests/build -G Ninja -DCMAKE_C_COMPILER=gcc
```

This creates a separate native CMake build directory under `Tests/build`.

## Build the tests

```powershell
cmake --build Tests/build
```

The test executable is created as:

```text
Tests/build/nucleo_3310_tests.exe
```

## Run the tests

```powershell
.\Tests\build\nucleo_3310_tests.exe
```

A successful run should look like this:

```text
Tests
=====
PCD8544 TESTS PASSED
DISPLAY TESTS PASSED
INPUT PROTOCOL TESTS PASSED
INPUT STATE TESTS PASSED
CAMERA3D TESTS PASSED
CAMERA CONTROLLER TESTS PASSED
CAR TRAJECTORY TESTS PASSED
CAR MODIFIER TESTS PASSED
CAR SPAWN TESTS PASSED
CAR FOOTPRINT TESTS PASSED
MODIFIER3D TESTS PASSED
SCENE3D AGENT UPDATE TESTS PASSED
ALL TESTS PASSED
```

The runner stops at the first failing test suite. Individual tests return `bool`; checks use the project's `CHECK(condition)` macro from `test_common.h`, which reports the source file, line number, and failed condition.

## Test architecture

`test_runner.c` is the only test executable entry point. It calls each suite in turn:

```text
Tests/test_runner.c
  ├── test_pcd8544.c
  │     └── Drivers/PCD8544/pcd8544.c
  │           └── fake transport
  ├── test_display.c
  │     └── Modules/engine3d/display.c
  │           └── PCD8544 interface / fake transport
  ├── input_protocol_tests.c
  │     └── Protocols/input/input_protocol.c
  ├── input_state_tests.c
  │     └── App/input_state.c
  ├── camera3d_tests.c
  │     └── Modules/engine3d/camera3d.c
  │           └── geometry3d.c / matrix4.c
  ├── camera_controller_tests.c
  │     └── App/camera_controller.c
  │           └── camera3d.c / geometry3d.c / matrix4.c
  ├── test_car_trajectory.c / test_car_modifier.c / test_car_spawn.c / test_car_footprint.c
  │     └── Modules/traffic/car_trajectory.c / car_modifier.c
  ├── test_modifier3d.c
  │     └── Modules/engine3d/modifier3d.c
  └── test_scene3d_agents.c
        └── Modules/engine3d/scene3d.c / modifier3d.c
```

The exact source list and dependencies are defined by `Tests/CMakeLists.txt`. Shared implementation files should only be added once to the host target.

The STM32-specific display transport and UART/BSP integration are intentionally excluded from the native test build. This allows portable code to be exercised on the PC without hardware dependencies. For the same reason, the host build's `input_state` tests check state semantics, timeout behavior, mouse-delta accumulation and consumption; they do not prove the correctness of target-side interrupt masking or ISR concurrency.
