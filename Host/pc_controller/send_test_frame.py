"""Send one manually configured frame to the STM32."""

import serial

from protocol import build_frame


PORT = "COM7"  # Change to the ST-Link VCP port.
BAUD_RATE = 115200

# Edit these values to test specific input states.
KEY_MASK = 0
MOUSE_DX = 10
MOUSE_DY = 0


def main() -> None:
    frame = build_frame(KEY_MASK, MOUSE_DX, MOUSE_DY)

    print(f"Frame: {frame.decode('ascii').rstrip()}")

    with serial.Serial(
        PORT,
        BAUD_RATE,
        timeout=0,
        write_timeout=0.2,
    ) as port:
        written = port.write(frame)

        if written != len(frame):
            raise RuntimeError(
                f"Incomplete write: {written}/{len(frame)} bytes"
            )

        print(f"Sent {written} bytes to {PORT}.")


if __name__ == "__main__":
    main()