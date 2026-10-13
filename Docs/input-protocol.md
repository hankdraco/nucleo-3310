# PC-to-STM32 Input Protocol

This document specifies the current one-way input protocol used by the Windows PC controller to send keyboard state and relative mouse movement to the STM32 over the ST-Link Virtual COM Port (VCP).

The STM32 receives bytes through LPUART1. The transport controller passes bytes to the protocol parser; a complete and valid frame is then published to the input-state module. The 3D engine consumes an input snapshot separately.

## Serial settings

- Baud rate: `115200`
- Data format: `8N1` (8 data bits, no parity, 1 stop bit)
- Flow control: none
- Text encoding: ASCII
- Line ending: LF (`\n`); the parser also accepts CRLF (`\r\n`)

The current protocol is sent from PC to STM32. No acknowledgement message or sequence number is defined.

## Frame format

```text
@KKKKKKKK,XXXX,YYYY*CC\n
```

The body before the `*` consists of exactly 19 ASCII bytes. The full frame is 23 bytes with LF, or 24 bytes with CRLF.

| Field | Width | Description |
|---|---:|---|
| `@` | 1 character | Start-of-frame marker |
| `KKKKKKKK` | 8 hexadecimal characters | 32-bit keyboard/action mask |
| `,` | 1 character | Field separator |
| `XXXX` | 4 hexadecimal characters | X mouse delta encoded as a 16-bit two's-complement bit pattern |
| `,` | 1 character | Field separator |
| `YYYY` | 4 hexadecimal characters | Y mouse delta encoded as a 16-bit two's-complement bit pattern |
| `*` | 1 character | CRC separator |
| `CC` | 2 hexadecimal characters | CRC-8 of the body |
| `\n` | 1 byte | End of frame; optional preceding `\r` is accepted |

The example below encodes no pressed keys, an X delta of `+10`, and a Y delta of `0`:

```text
@00000000,000A,0000*B3\n
```

Hexadecimal characters are accepted in either case by the parser. The Python encoder emits uppercase hexadecimal.

## Keyboard/action mask

The mask is an unsigned 32-bit value represented by eight hexadecimal digits. The currently defined bits in `input_types.h` are:

| Bit mask | Symbol | Meaning |
|---:|---|---|
| `0x00000001` | `INPUT_KEY_FORWARD` | Move forward (`W` in the PC controller) |
| `0x00000002` | `INPUT_KEY_BACKWARD` | Move backward (`S`) |
| `0x00000004` | `INPUT_KEY_STRAFE_LEFT` | Strafe left (`A`) |
| `0x00000008` | `INPUT_KEY_STRAFE_RIGHT` | Strafe right (`D`) |
| `0x00000010` | `INPUT_KEY_MOVE_UP` | Move up (`Space`) |
| `0x00000020` | `INPUT_KEY_MOVE_DOWN` | Move down (`Ctrl`) |

Bits not listed above are reserved for future use and should be sent as zero. The current controller ignores bits for which it has no action defined.

The mask represents the current state, not a sequence of key-down/key-up events. Each valid frame replaces the previously published mask.

## Mouse deltas

`XXXX` and `YYYY` each encode one signed 16-bit mouse displacement using a two's-complement bit pattern, displayed as four hexadecimal digits. Their accepted numeric range per frame is `-32768` through `32767`.

Examples:

| Signed value | 16-bit hexadecimal representation |
|---:|---|
| `0` | `0000` |
| `10` | `000A` |
| `-1` | `FFFF` |
| `32767` | `7FFF` |
| `-32768` | `8000` |

The receiver interprets the 16-bit pattern as signed data explicitly; the protocol does not depend on an implementation-defined cast from an out-of-range unsigned integer to a signed 16-bit type.

The Python application accumulates relative mouse events between transmissions, sends the accumulated delta in a frame, then subtracts the transmitted amount from its pending total. On STM32, the input-state module accumulates deltas across valid frames until the application consumes a snapshot; consumption clears the pending deltas so they are applied once.

## CRC-8

The CRC is calculated over the 19 bytes from `@` through the last hexadecimal digit of `YYYY`. It excludes the `*`, the two CRC characters, and the line ending.

Parameters:

- Width: 8 bits
- Polynomial: `0x07`
- Initial value: `0x00`
- Input reflection: false
- Output reflection: false
- Final XOR (`xorout`): `0x00`
- Bit processing: most-significant bit first

For example:

```text
Body: @00000000,000A,0000
CRC:  B3
Frame: @00000000,000A,0000*B3\n
```

CRC detects many accidental transmission errors; it is not a cryptographic integrity mechanism.

## Parser and recovery behavior

- Bytes are accumulated in a fixed-size parser buffer; no dynamic allocation is used.
- The parser starts collecting a frame when it receives `@`.
- A subsequent `@` restarts frame collection, allowing resynchronization after damaged input.
- A frame is accepted only after its line ending is received and its length, separators, hexadecimal fields, and CRC have been validated.
- Invalid frames are rejected and do not produce a new input frame.
- If the parser buffer overflows, the current line is discarded until a line ending or a new start marker is encountered.
- UART hardware errors (for example, overrun) belong to the transport/HAL error path, not to CRC validation.

## Timeout and update semantics

The input-state module records the time of the last valid frame. The application treats the input as stale after `250 ms` without a valid update and clears the effective key state and pending mouse deltas. This timeout is an application behavior, not an extra field in the wire format.

The PC controller currently targets a nominal send rate of `60 Hz`. Keyboard state is sent every frame, including an all-zero mask when no supported key is pressed. Mouse deltas are zero when no unreported movement is pending.
