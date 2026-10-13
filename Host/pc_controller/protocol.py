"""Encoding for the STM32 camera-control protocol."""

from __future__ import annotations


MIN_MOUSE_DELTA = -32768
MAX_MOUSE_DELTA = 32767
MAX_KEY_MASK = 0xFFFFFFFF


def crc8(data: bytes) -> int:
    """CRC-8: polynomial 0x07, init 0x00, xorout 0x00."""
    crc = 0

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


def build_frame(key_mask: int, mouse_dx: int, mouse_dy: int) -> bytes:
    """
    Encode one input frame.

    Mouse deltas use signed 16-bit two's-complement encoding,
    represented as four hexadecimal characters.

    Returns the complete ASCII frame, including CRC and LF.
    """
    if not 0 <= key_mask <= MAX_KEY_MASK:
        raise ValueError("key_mask must fit in uint32")

    if not MIN_MOUSE_DELTA <= mouse_dx <= MAX_MOUSE_DELTA:
        raise ValueError("mouse_dx must fit in int16")

    if not MIN_MOUSE_DELTA <= mouse_dy <= MAX_MOUSE_DELTA:
        raise ValueError("mouse_dy must fit in int16")

    body = (
        f"@{key_mask:08X},"
        f"{mouse_dx & 0xFFFF:04X},"
        f"{mouse_dy & 0xFFFF:04X}"
    )

    checksum = crc8(body.encode("ascii"))

    return f"{body}*{checksum:02X}\n".encode("ascii")