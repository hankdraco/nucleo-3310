"""Decoder for the STM32 raw timing telemetry protocol."""

from __future__ import annotations

from dataclasses import dataclass

SYNC = b"TM"
VERSION = 1
HEADER_SIZE = 8
SAMPLE_SIZE = 22
MAX_SAMPLES = 30


@dataclass(frozen=True)
class TelemetrySample:
    sequence: int
    update_start_cycles: int
    update_end_cycles: int
    render_start_cycles: int
    render_end_cycles: int
    lcd_end_cycles: int


@dataclass(frozen=True)
class TelemetryBatch:
    core_clock_hz: int
    samples: tuple[TelemetrySample, ...]


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class TelemetryDecoder:
    """Incrementally decode batches from a non-blocking serial stream."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[TelemetryBatch]:
        self._buffer.extend(data)
        batches: list[TelemetryBatch] = []

        while True:
            sync_at = self._buffer.find(SYNC)
            if sync_at < 0:
                if self._buffer.endswith(SYNC[:1]):
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break

            if sync_at:
                del self._buffer[:sync_at]
            if len(self._buffer) < HEADER_SIZE:
                break

            version = self._buffer[2]
            count = self._buffer[3]
            if version != VERSION or not 1 <= count <= MAX_SAMPLES:
                del self._buffer[0]
                continue

            frame_size = HEADER_SIZE + count * SAMPLE_SIZE + 2
            if len(self._buffer) < frame_size:
                break

            frame = self._buffer[:frame_size]
            expected_crc = int.from_bytes(frame[-2:], "little")
            if crc16_ccitt(frame[:-2]) != expected_crc:
                del self._buffer[0]
                continue

            samples = []
            offset = HEADER_SIZE
            for _ in range(count):
                record = frame[offset:offset + SAMPLE_SIZE]
                samples.append(TelemetrySample(
                    sequence=int.from_bytes(record[0:2], "little"),
                    update_start_cycles=int.from_bytes(record[2:6], "little"),
                    update_end_cycles=int.from_bytes(record[6:10], "little"),
                    render_start_cycles=int.from_bytes(record[10:14], "little"),
                    render_end_cycles=int.from_bytes(record[14:18], "little"),
                    lcd_end_cycles=int.from_bytes(record[18:22], "little"),
                ))
                offset += SAMPLE_SIZE

            batches.append(TelemetryBatch(
                core_clock_hz=int.from_bytes(frame[4:8], "little"),
                samples=tuple(samples),
            ))
            del self._buffer[:frame_size]

        return batches


def cycle_delta(start: int, end: int) -> int:
    """Return an unsigned 32-bit counter delta, including counter wrap."""
    return (end - start) & 0xFFFFFFFF
