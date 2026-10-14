# NUCLEO-3310 — An Educational 3D Graphics and Simulation Playground

A small monochrome display. A microcontroller. A hand-written 3D engine. And, eventually, an entire city.

**NUCLEO-3310 is an educational embedded C project built around an STM32 microcontroller and a Nokia 3310/5110-compatible 84 × 48 monochrome LCD.** What started as an exercise in drawing lines on a tiny display has evolved into a small 3D graphics engine with a vector-city scene, a depth buffer, instance-based animation, and interactive camera control from a Windows PC.

The project is deliberately developed as a learning playground rather than a production product. Its purpose is to explore embedded software engineering in practice: low-level hardware access, modular architecture, graphics algorithms, memory management, communication protocols, testing, and increasingly complex simulation logic.

The tiny display is part of the fun. The engineering behind it is the point.

## Project Goals

The project explores several areas of embedded and systems programming:

- Writing hardware drivers and separating hardware-specific code from portable application logic.
- Building a monochrome graphics pipeline suitable for a resource-constrained microcontroller.
- Implementing the mathematical foundations of a 3D engine in C.
- Separating scene geometry, object instances, camera control, and rendering.
- Designing an asynchronous PC-to-microcontroller communication path.
- Testing portable modules natively on a PC.
- Exploring how independent objects can eventually develop context-aware behaviours and interact within a simulated environment.

There is no attempt to hide complexity behind a large graphics framework. The emphasis is on understanding how the individual pieces work and how they fit together.

## Hardware and Development Environment

### Target Hardware

- **Microcontroller:** STM32G474RE on the NUCLEO-G474RE development board.
- **Display:** Nokia 3310/5110-compatible LCD module using the PCD8544 controller.
- **Display resolution:** 84 × 48 monochrome pixels.
- **Display connection:** SPI, with separate control signals.
- **PC connection:** ST-Link Virtual COM Port (VCP), using LPUART1.
- **UART configuration:** 115200 baud, 8 data bits, no parity, 1 stop bit (8N1).

### Software

- C for firmware and the portable engine.
- STM32CubeMX-generated project configuration and STM32 HAL/BSP.
- CMake for build configuration.
- Python for the PC-side controller.
- pySerial for serial communication.
- pygame-ce for keyboard and relative mouse input.
- CMake, Ninja, and GCC from W64DevKit for native host tests.

## Implemented Features

### 1. PCD8544 Display Driver

The project contains a hand-written driver for the PCD8544 display controller.

The driver is responsible for communicating with the LCD, while the hardware-specific STM32 transport is kept separate from the driver's logic.

Implemented functionality includes:

- PCD8544 initialization.
- Command and data transmission.
- SPI-based transport on STM32.
- Separation between the display driver and the underlying transport implementation.
- A dedicated STM32 transport adapter.

The intention is to keep display-controller logic separate from the details of the particular microcontroller and board.

### 2. Monochrome Display and Framebuffer Abstraction

The display layer provides a higher-level interface for drawing without requiring application code to manipulate PCD8544 commands directly.

The framebuffer represents an 84 × 48 monochrome image:

- 84 pixels wide.
- 48 pixels high.
- 1 bit per pixel.
- 504 bytes of image data.

The display subsystem is separated from the PCD8544-specific flushing implementation. Display configuration also supports horizontal and vertical orientation flips.

This abstraction provides a natural boundary between drawing operations and the physical LCD.

### 3. Miniature 3D Graphics Engine

The project includes a custom 3D engine written in C.

Its mathematical foundations include:

- 3D vectors.
- 4 × 4 transformation matrices.
- Matrix multiplication.
- Translation, rotation, and scaling transformations.
- Camera transformations and world-to-camera view matrices.
- Perspective and orthographic projection support.

The engine transforms 3D geometry into screen-space coordinates and prepares it for rendering on the monochrome display.

#### Rendering Pipeline

The graphics pipeline includes:

- Transformation of model geometry through the scene and camera coordinate systems.
- Projection into screen space.
- Near-plane clipping.
- Clipping of lines against the display boundaries.
- A depth buffer.
- Culling to avoid processing or drawing elements that need not contribute to the image.
- Monochrome rendering of the resulting geometry.

Line clipping uses the Liang–Barsky algorithm for clipping against the canvas boundaries.

These algorithms are implemented as part of the project rather than delegated to an external 3D engine.

### 4. Scene and Instance Model

The city scene demonstrates the distinction between shared geometry and individual object instances.

Multiple instances can reference the same mesh while maintaining their own transformations and behaviour.

The current city contains approximately 80 instances that reuse a shared mesh. Their appearance in the scene is determined by their transformations, including position, orientation, and scale.

This makes it possible to construct a larger scene without duplicating identical geometry for every object.

The scene-loading logic is kept separate from the rendering loop, and the application does not need to know the individual models used to construct the scene.

### 5. Instance Modifiers and Animation

The project contains an instance-modifier mechanism for changing instance parameters over time.

Modifiers are updated through the application loop using elapsed time (`dt`), independently of the rendering frequency.

This separates the continuous evolution of scene state from the act of rendering a particular snapshot of that state.

The modifier mechanism is also a starting point for more complex behaviours. Future extensions may allow an instance to react to other participants and to selected information about its environment rather than relying exclusively on time-based transformations.

Context-aware traffic and pedestrian behaviours are not yet implemented.

### 6. Camera Model and Interactive Control

The camera implementation supports:

- Position in world space.
- Look-at targeting.
- Manual orientation.
- Construction of the world-to-camera view matrix.
- A free-fly camera controller.

The free-fly controller provides first-person-style movement:

| Input | Action |
| --- | --- |
| W / S | Move forward / backward |
| A / D | Strafe left / right |
| Space / Ctrl | Move vertically |
| Mouse X | Rotate around the world-space vertical axis |
| Mouse Y | Change pitch |

The camera uses yaw and pitch, without roll. Horizontal rotation is performed around the world's vertical axis, while pitch changes the camera's inclination.

Continuous keyboard movement is scaled by elapsed time. Mouse input is interpreted as relative movement, allowing the camera to rotate independently of the rendering interval.

### 7. PC-to-STM32 UART Communication

The project includes a working communication path from a Windows PC application to the STM32 over the ST-Link Virtual COM Port.

The PC-side application is written in Python and uses pySerial and pygame-ce.

Its responsibilities include:

- Reading the current keyboard state.
- Reading relative mouse movement.
- Accumulating mouse deltas between transmissions.
- Encoding input data into protocol frames.
- Calculating CRC-8.
- Sending frames through the virtual serial port.

On the STM32 side, reception uses the existing BSP-initialized LPUART1 and the STM32 HAL interrupt-driven receive mechanism.

The communication controller, frame parser, and input-state module are separate components.

#### Input-State Semantics

The firmware deliberately treats keyboard and mouse input differently:

- The keyboard mask represents the latest known state and replaces the previous mask.
- Mouse deltas are accumulated until consumed by the application.
- Consuming an input snapshot clears the pending mouse deltas, so they are applied only once.
- If no valid input frame is received for 250 ms, the application treats the input as stale and clears the effective input state.

This avoids maintaining a queue of historical keyboard states while preserving accumulated mouse movement.

#### Serial Protocol

Input frames use a fixed-width hexadecimal representation:

```text
@KKKKKKKK,XXXX,YYYY*CC\n
```

Where:

- `KKKKKKKK` is a 32-bit keyboard mask represented by eight hexadecimal digits.
- `XXXX` is the horizontal mouse delta encoded as a 16-bit two's-complement value.
- `YYYY` is the vertical mouse delta encoded in the same way.
- `CC` is a two-digit hexadecimal CRC-8 checksum.
- `\n` terminates the frame; CRLF line endings are also accepted.

The CRC-8 algorithm uses polynomial `0x07`, initial value `0x00`, and final XOR value `0x00`. It covers the frame bytes from `@` through the last hexadecimal digit of the vertical mouse delta, excluding `*`, the checksum, and the line terminator.

For example:

```text
@00000000,000A,0000*B3
```

This frame represents no pressed keys, a horizontal mouse delta of 10, and a vertical delta of zero.

See [`docs/input-protocol.md`](docs/input-protocol.md) for the detailed protocol definition.

## Architecture

The project is organised around separating responsibilities rather than treating the firmware as one monolithic application.

The main conceptual boundaries are:

- **Hardware and transport:** STM32-specific interfaces, SPI transport, UART reception, and BSP/HAL integration.
- **Display subsystem:** framebuffer operations and the PCD8544 implementation.
- **3D engine:** geometry, matrices, camera mathematics, projection, and rendering.
- **Scene and application:** scene construction, instance state, update scheduling, and rendering orchestration.
- **Input transport and protocol:** byte reception, frame assembly, validation, and publication of input data.
- **Camera controller:** interpretation of keyboard and mouse input independently of the communication transport.
- **PC controller:** platform-side input collection and serial transmission.

The renderer consumes the current scene state and produces an image. It does not need to understand how an instance's state evolved or why an object is moving.

Similarly, the 3D engine does not need to know whether its input originated from a physical keyboard, a mouse, or a serial connection.

## Host-Side Tests

Portable modules are tested in a separate native Windows executable. The test build does not require the STM32 board or LCD hardware.

The existing test suites cover:

- PCD8544 driver logic, using a fake transport.
- Display abstraction.
- Input protocol parsing and validation.
- Input-state accumulation, consumption, and timeout behaviour.
- Camera mathematics.
- Camera-controller behaviour.

Tests use the project's lightweight `bool`-returning test-suite convention, with a shared `CHECK(condition)` macro for reporting failed conditions, source files, and line numbers.

The host build uses CMake, Ninja, and GCC from W64DevKit.

See [`Tests/README.md`](Tests/README.md) for configuration, build, and execution instructions.

Host tests verify portable logic, but they do not replace testing the actual firmware on the STM32 hardware.

## Running the PC Camera Controller

The PC controller lives in [`host/pc_controller/`](host/pc_controller/).

Create and activate a virtual environment from PowerShell:

```powershell
cd host/pc_controller
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Set the appropriate ST-Link COM port in the controller script, flash and start the firmware, and run:

```powershell
python .\run_camera_control.py
```

The controller captures relative mouse movement and keyboard state, then transmits input frames to the STM32.

A separate `send_test_frame.py` script is retained as a diagnostic tool for transmitting manually selected test frames.

## Current Scope and Future Directions

The current implementation focuses on graphics, instance transformations, camera control, communication, and testable module boundaries.

Planned directions include:

- Moving vehicles along predefined road paths.
- A logical road graph representing permitted connections and manoeuvres.
- Vehicle spawning and despawning at the city boundaries.
- Independent vehicle behaviour modifiers that observe nearby traffic.
- Right-hand-priority interactions at an intersection.
- Pedestrian movement and pedestrian-crossing behaviour.
- Decentralised behaviour decisions based on read-only observations of other participants.
- Potential reuse of the same portable 3D engine in a native PC renderer.

These are future development ideas, not features currently claimed as implemented.

Performance optimisation is also an ongoing concern. The city already contains many instances on a resource-constrained target, providing a practical setting for exploring rendering cost and the trade-offs involved in visibility processing.

## Project Philosophy

This project intentionally favours small, understandable steps over premature generalisation.

The goal is to learn by implementing and testing real components, from low-level communication to 3D mathematics and higher-level behaviour.

A 3D city on a tiny monochrome Nokia display may not solve a practical industrial problem. Actually, this project solves absolutely no practical problem known to mankind. It renders a city on a Nokia 3310 display for no reason whatsoever. It does, however, create a surprisingly rich engineering playground in which hardware constraints, software architecture, graphics, communication, and simulation all meet.
