import serial


PORT = "COM7"  # Zmień na port swojego ST-Link VCP.
BAUD_RATE = 115200

# Edytujesz wyłącznie tę linię podczas testowania.
FRAME_PREFIX = "@00000000,000A,0000*"


def crc8(data: bytes) -> int:
    """CRC-8: polynomial 0x07, init 0x00, xorout 0x00."""
    crc = 0x00

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


def build_frame(frame_prefix: str) -> bytes:
    """Append CRC-8 and newline to a frame prefix ending with '*'."""
    if not frame_prefix.endswith("*"):
        raise ValueError("Frame prefix must end with '*'")

    if "\r" in frame_prefix or "\n" in frame_prefix:
        raise ValueError("Frame prefix must not contain line endings")

    body = frame_prefix[:-1]

    if not body.startswith("@") or "*" in body:
        raise ValueError("Invalid frame prefix")

    checksum = crc8(body.encode("ascii"))

    return f"{body}*{checksum:02X}\n".encode("ascii")


frame = build_frame(FRAME_PREFIX)

print(f"Frame: {frame.decode('ascii').rstrip()}")

with serial.Serial(PORT, BAUD_RATE, timeout=1) as port:
    written = port.write(frame)
    port.flush()

    if written != len(frame):
        raise RuntimeError(
            f"Incomplete write: {written}/{len(frame)} bytes"
        )

    print(f"Sent {written} bytes to {PORT}.")