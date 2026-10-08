# Host Tests

The `Tests` directory contains host-side tests for the portable parts of the project.

These tests are built as a separate native executable for Windows. They do not require the STM32 board, the STM32 HAL, or the LCD hardware.

## Requirements

The tests require:

* CMake
* Ninja
* GCC from W64DevKit

Make sure `gcc` and `ninja` are available in `PATH`.

## Configure the test build

From the project root:

```powershell
cmake -S Tests -B Tests/build -G Ninja -DCMAKE_C_COMPILER=gcc
```

This creates a separate CMake build directory under `Tests/build`.

## Build the tests

```powershell
cmake --build Tests/build
```

The test executable is created as:

```text
Tests/build/pcd8544_tests.exe
```

## Run the tests

```powershell
.\Tests\build\pcd8544_tests.exe
```

A successful run should look like:

```text
PCD8544 tests
=============
PASS: test_pcd8544_init
ALL TESTS PASSED
```

## Test architecture

The tests compile the portable PCD8544 driver directly:

```text
Tests/test_pcd8544.c
        │
        ▼
Core/Src/pcd8544.c
        │
        ▼
fake transport
```

The STM32-specific transport implementation is intentionally not part of the host test build.

This allows the same PCD8544 driver code to be tested on the PC without requiring the target hardware.
