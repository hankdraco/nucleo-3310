"""Interactive PC controller for the STM32 3D camera."""

from __future__ import annotations

import sys

import pygame
import serial

from protocol import build_frame


PORT = "COM7"  # Change to the ST-Link VCP port.
BAUD_RATE = 115200
SEND_RATE_HZ = 60

WINDOW_SIZE = (760, 150)

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


def draw_status(
    screen: pygame.Surface,
    font: pygame.font.Font,
    focused: bool,
    key_mask: int,
    pending_dx: int,
    pending_dy: int,
) -> None:
    screen.fill((22, 26, 34))

    lines = [
        f"STM32 Camera Controller - {PORT} @ {BAUD_RATE}",
        "W/S: forward/back | A/D: strafe | SPACE/CTRL: vertical | C: camera mode",
        "Mouse: look | ESC: quit | Switch focus to release mouse",
        (
            f"Focus: {'captured' if focused else 'released'}"
            f" | Keys: 0x{key_mask:08X}"
            f" | Pending mouse: {pending_dx}, {pending_dy}"
        ),
    ]

    for index, line in enumerate(lines):
        color = (230, 235, 245) if index < 3 else (130, 210, 170)
        text = font.render(line, True, color)
        screen.blit(text, (14, 12 + index * 32))

    pygame.display.flip()


def run_controller(port: serial.Serial) -> None:
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("STM32 Camera Controller")

    font = pygame.font.SysFont("consolas", 16)
    clock = pygame.time.Clock()

    running = True
    focused = pygame.key.get_focused()

    pending_dx = 0
    pending_dy = 0

    set_mouse_capture(focused)

    # Discard any mouse movement accumulated during startup.
    pygame.mouse.get_rel()

    try:
        while running:
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
            print("Focus the controller window to control the camera.")
            print("Press ESC to exit.")

            run_controller(port)

        return 0

    except (serial.SerialException, OSError, RuntimeError, ValueError) as exc:
        print(f"Camera controller failed: {exc}", file=sys.stderr)
        return 1

    finally:
        pygame.quit()


if __name__ == "__main__":
    raise SystemExit(main())
