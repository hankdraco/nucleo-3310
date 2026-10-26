"""Interactive PC controller for the Nucleo-3310 project."""

from __future__ import annotations

import sys

import pygame
import serial

from protocol import build_frame
from telemetry_protocol import TelemetryDecoder, cycle_delta


PORT = "COM7"  # Change to the ST-Link VCP port.
BAUD_RATE = 115200
SEND_RATE_HZ = 60

WINDOW_SIZE = (1080, 212)

MIN_MOUSE_DELTA = -32768
MAX_MOUSE_DELTA = 32767

# Must match the semantic key bits in input_types.h.
KEY_FORWARD         = 0x00000001
KEY_BACKWARD        = 0x00000002
KEY_STRAFE_LEFT     = 0x00000004
KEY_STRAFE_RIGHT    = 0x00000008
KEY_MOVE_UP         = 0x00000010
KEY_MOVE_DOWN       = 0x00000020
KEY_CAMERA_MODE     = 0x00000040


def set_mouse_capture(captured: bool) -> None:
    """Capture relative mouse input while this window is focused."""
    pygame.event.set_grab(captured)
    pygame.mouse.set_relative_mode(captured)
    pygame.mouse.set_visible(not captured)


def get_key_mask() -> int:
    """Translate physical keyboard state into protocol key bits."""
    keys = pygame.key.get_pressed()
    key_mask = 0

    if keys[pygame.K_w]:
        key_mask |= KEY_FORWARD

    if keys[pygame.K_s]:
        key_mask |= KEY_BACKWARD

    if keys[pygame.K_a]:
        key_mask |= KEY_STRAFE_LEFT

    if keys[pygame.K_d]:
        key_mask |= KEY_STRAFE_RIGHT

    if keys[pygame.K_SPACE]:
        key_mask |= KEY_MOVE_UP

    if keys[pygame.K_LCTRL] or keys[pygame.K_RCTRL]:
        key_mask |= KEY_MOVE_DOWN

    if keys[pygame.K_c]:
        key_mask |= KEY_CAMERA_MODE

    return key_mask


def clamp_mouse_delta(delta: int) -> int:
    """Fit one mouse delta into a signed 16-bit protocol field."""
    return max(MIN_MOUSE_DELTA, min(MAX_MOUSE_DELTA, delta))


def update_min_max(
    minimums: dict[str, float | None],
    maximums: dict[str, float | None],
    metric: str,
    value: float,
) -> None:
    """Include one telemetry reading in its since-start range."""
    current_minimum = minimums[metric]
    current_maximum = maximums[metric]
    minimums[metric] = value if current_minimum is None else min(current_minimum, value)
    maximums[metric] = value if current_maximum is None else max(current_maximum, value)


def format_min_max(
    minimums: dict[str, float | None],
    maximums: dict[str, float | None],
    metric: str,
    precision: int,
) -> str:
    """Format a metric's min-max range, or a placeholder before its first sample."""
    minimum = minimums[metric]
    maximum = maximums[metric]
    if minimum is None or maximum is None:
        return "n/a"
    return f"{minimum:.{precision}f}–{maximum:.{precision}f}"


def draw_status(
    screen: pygame.Surface,
    font: pygame.font.Font,
    focused: bool,
    key_mask: int,
    pending_dx: int,
    pending_dy: int,
    telemetry_status: str,
    telemetry_range_status: str,
) -> None:
    screen.fill((22, 26, 34))

    lines = [
        f"Nucleo-3310 PC Controller - {PORT} @ {BAUD_RATE}",
        "W/S: forward/back | A/D: strafe | SPACE/CTRL: vertical | C: cycle camera mode",
        "Mouse: look | ESC: quit | Switch focus to release mouse",
        (
            f"Focus: {'captured' if focused else 'released'}"
            f" | Keys: 0x{key_mask:08X}"
            f" | Pending mouse: {pending_dx}, {pending_dy}"
        ),
        telemetry_status,
        telemetry_range_status,
    ]

    for index, line in enumerate(lines):
        color = (230, 235, 245) if index < 3 else (130, 210, 170)
        text = font.render(line, True, color)
        screen.blit(text, (14, 12 + index * 32))

    pygame.display.flip()


def run_controller(port: serial.Serial) -> None:
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Nucleo-3310 PC Controller")

    font = pygame.font.SysFont("consolas", 16)
    clock = pygame.time.Clock()

    running = True
    focused = pygame.key.get_focused()

    pending_dx = 0
    pending_dy = 0
    telemetry_decoder = TelemetryDecoder()
    previous_sequence: int | None = None
    previous_render_cycles: int | None = None
    telemetry_fps = 0.0
    telemetry_dropped = 0
    update_ms = 0.0
    render_ms = 0.0
    lcd_ms = 0.0
    telemetry_status = "Telemetry: waiting for STM32"
    telemetry_range_status = "Since start: waiting for STM32 telemetry"
    telemetry_minimums: dict[str, float | None] = {
        "fps": None,
        "update_ms": None,
        "render_ms": None,
        "lcd_ms": None,
        "sequence_gaps": None,
    }
    telemetry_maximums: dict[str, float | None] = {
        "fps": None,
        "update_ms": None,
        "render_ms": None,
        "lcd_ms": None,
        "sequence_gaps": None,
    }

    set_mouse_capture(focused)

    # Discard any mouse movement accumulated during startup.
    pygame.mouse.get_rel()

    try:
        while running:
            incoming = port.read(port.in_waiting)
            for batch in telemetry_decoder.feed(incoming):
                clock_hz = batch.core_clock_hz
                if clock_hz == 0:
                    continue

                for sample in batch.samples:
                    update_cycles = cycle_delta(
                        sample.update_start_cycles,
                        sample.update_end_cycles,
                    )
                    render_cycles = cycle_delta(
                        sample.render_start_cycles,
                        sample.render_end_cycles,
                    )
                    lcd_cycles = cycle_delta(
                        sample.render_end_cycles,
                        sample.lcd_end_cycles,
                    )
                    update_ms = update_cycles * 1000.0 / clock_hz
                    render_ms = render_cycles * 1000.0 / clock_hz
                    lcd_ms = lcd_cycles * 1000.0 / clock_hz

                    if previous_sequence is not None:
                        sequence_delta = (
                            sample.sequence - previous_sequence
                        ) & 0xFFFF
                        elapsed_cycles = cycle_delta(
                            previous_render_cycles or 0,
                            sample.render_start_cycles,
                        )
                        if sequence_delta > 0 and elapsed_cycles > 0:
                            telemetry_fps = (
                                sequence_delta * clock_hz / elapsed_cycles
                            )
                            update_min_max(
                                telemetry_minimums,
                                telemetry_maximums,
                                "fps",
                                telemetry_fps,
                            )
                            if sequence_delta > 1:
                                telemetry_dropped += sequence_delta - 1

                    previous_sequence = sample.sequence
                    previous_render_cycles = sample.render_start_cycles

                    update_min_max(
                        telemetry_minimums,
                        telemetry_maximums,
                        "update_ms",
                        update_ms,
                    )
                    update_min_max(
                        telemetry_minimums,
                        telemetry_maximums,
                        "render_ms",
                        render_ms,
                    )
                    update_min_max(
                        telemetry_minimums,
                        telemetry_maximums,
                        "lcd_ms",
                        lcd_ms,
                    )
                    update_min_max(
                        telemetry_minimums,
                        telemetry_maximums,
                        "sequence_gaps",
                        float(telemetry_dropped),
                    )

                telemetry_status = (
                    f"STM32: {telemetry_fps:5.1f} FPS"
                    f" | update {update_ms:5.2f} ms"
                    f" | render {render_ms:5.2f} ms"
                    f" | LCD {lcd_ms:5.2f} ms"
                    f" | sequence gaps {telemetry_dropped}"
                )
                telemetry_range_status = (
                    "Since start: "
                    f"FPS {format_min_max(telemetry_minimums, telemetry_maximums, 'fps', 1)}"
                    f" | upd {format_min_max(telemetry_minimums, telemetry_maximums, 'update_ms', 2)} ms"
                    f" | render {format_min_max(telemetry_minimums, telemetry_maximums, 'render_ms', 2)} ms"
                    f" | LCD {format_min_max(telemetry_minimums, telemetry_maximums, 'lcd_ms', 2)} ms"
                    f" | gaps {format_min_max(telemetry_minimums, telemetry_maximums, 'sequence_gaps', 0)}"
                )

            # The event queue must be serviced every iteration.
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif (
                    event.type == pygame.KEYDOWN
                    and event.key == pygame.K_ESCAPE
                ):
                    running = False

                elif event.type == pygame.WINDOWFOCUSLOST:
                    focused = False
                    set_mouse_capture(False)

                    # Do not apply old mouse movement after focus loss.
                    pending_dx = 0
                    pending_dy = 0
                    pygame.mouse.get_rel()

                elif event.type == pygame.WINDOWFOCUSGAINED:
                    focused = True
                    set_mouse_capture(True)

                    # Start with a clean relative-mouse baseline.
                    pygame.mouse.get_rel()

            if focused:
                key_mask = get_key_mask()

                relative_x, relative_y = pygame.mouse.get_rel()

                pending_dx += relative_x
                pending_dy += relative_y
            else:
                # Releasing focus must not leave movement active.
                key_mask = 0
                pending_dx = 0
                pending_dy = 0

            # A protocol frame can carry only one signed int16
            # delta per axis. Preserve any unsent remainder.
            send_dx = clamp_mouse_delta(pending_dx)
            send_dy = clamp_mouse_delta(pending_dy)

            frame = build_frame(key_mask, send_dx, send_dy)
            written = port.write(frame)

            if written != len(frame):
                raise RuntimeError(
                    f"Incomplete serial write: {written}/{len(frame)} bytes"
                )

            # Consume only the movement included in this frame.
            pending_dx -= send_dx
            pending_dy -= send_dy

            draw_status(
                screen,
                font,
                focused,
                key_mask,
                pending_dx,
                pending_dy,
                telemetry_status,
                telemetry_range_status,
            )

            # Nominally limit the input/transmission loop to 60 Hz.
            clock.tick(SEND_RATE_HZ)

    finally:
        # Release the mouse even if the loop exits with an error.
        set_mouse_capture(False)

        # On a clean shutdown, explicitly release all camera inputs.
        # The firmware timeout remains the fallback for unexpected exits.
        if port.is_open:
            try:
                port.write(build_frame(0, 0, 0))
                port.flush()
            except (serial.SerialException, OSError):
                pass


def main() -> int:
    pygame.init()

    try:
        with serial.Serial(
            PORT,
            BAUD_RATE,
            timeout=0,
            write_timeout=0.2,
        ) as port:
            print(f"Connected to {PORT} at {BAUD_RATE} baud.")
            print("Focus the PC Controller window to control the camera.")
            print("Press ESC to exit.")

            run_controller(port)

        return 0

    except (serial.SerialException, OSError, RuntimeError, ValueError) as exc:
        print(f"PC Controller failed: {exc}", file=sys.stderr)
        return 1

    finally:
        pygame.quit()


if __name__ == "__main__":
    raise SystemExit(main())
