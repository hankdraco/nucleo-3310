# NUCLEO-3310

NUCLEO-3310 is an educational STM32 project that renders and simulates a small 3D city on an 84 × 48 Nokia 3310/5110-compatible LCD.

## Features

- Custom C 3D engine with perspective projection, near-plane clipping, depth buffering, and monochrome rendering.
- City scene built from reusable meshes, with hierarchical transforms and animated objects.
- Multiple independently moving cars with route, spawn, and intersection behavior.
- Vehicle safety logic for following traffic and maintaining separation.
- Intersection entry coordination with right-hand priority and deadlock recovery.
- Three camera modes: car-follow, free-fly, and a closed demo path.
- PC keyboard and relative-mouse control over the ST-Link Virtual COM Port.
- Batched STM32-to-PC timing telemetry over the same UART.
- Native host tests for portable display, input, camera, scene, modifier, and traffic code.

## Hardware

- **Board:** NUCLEO-G474RE with STM32G474RE
- **Display:** PCD8544-based 84 × 48 monochrome LCD
- **Display interface:** SPI
- **PC interface:** ST-Link Virtual COM Port on LPUART1, 115200 baud, 8N1

## Run the PC controller

The controller is in [`Host/pc_controller`](Host/pc_controller). Set `PORT` in `run_controller.py` to the ST-Link COM port, then from PowerShell run:

```powershell
cd Host/pc_controller
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python .\run_controller.py
```

Start the firmware before running the controller. Use **W/A/S/D** to move, **Space/Ctrl** to move vertically, and the mouse to look around in free-fly mode. Press **C** to cycle camera modes.

## Host tests

The native test executable covers PCD8544 and display logic, input parsing and state, camera control, scene and modifier behavior, city geometry, and vehicle trajectory, spawning, footprints, and safety logic. Configure, build, and run the tests using the instructions in [`Tests/README.md`](Tests/README.md).

## Documentation

- [3D engine architecture](Docs/3d-engine.md)
- [PC-to-STM32 input protocol](Docs/input-protocol.md)
- [STM32-to-PC telemetry protocol](Docs/telemetry-protocol.md)
- [Host test instructions and suite details](Tests/README.md)
