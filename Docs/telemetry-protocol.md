# STM32 → PC telemetry protocol

The STM32 sends binary timing batches over LPUART1 at 115200 baud. This is a
separate, one-way protocol; the existing ASCII input protocol remains
PC → STM32. UART receive callbacks remain owned by `transmission_controller`,
which also dispatches TX completion and errors to `telemetry_transport`.

The protocol encoder has no HAL or serial dependency. The transport sends its
encoded buffer with `HAL_UART_Transmit_IT` and does not wait for completion.
The application keeps that buffer unchanged until the TX-complete callback.

## Frame layout

All multi-byte fields are little-endian. CRC-16/CCITT-FALSE (polynomial
`0x1021`, initial value `0xFFFF`) covers the frame from the sync bytes through
the final sample.

| Field | Size | Meaning |
| --- | ---: | --- |
| Sync | 2 | ASCII `TM` (`54 4D`) |
| Version | 1 | `1` |
| Sample count | 1 | Number of records, 1–30 |
| Core clock | 4 | HCLK in cycles per second |
| Samples | 22 each | Raw cycle counter readings |
| CRC | 2 | CRC-16/CCITT-FALSE |

Each sample contains `sequence` (uint16), followed by uint32 readings for
simulation update start/end, render start/end, and LCD flush end. The sequence
advances once per rendered frame and wraps naturally. The host detects missing
samples from sequence gaps. The update duration covers the `scene3d_update`
call in the application iteration that triggers that render; it is a raw
per-call sample, not a sum of all update calls since the previous render.

The DWT cycle counter is 32-bit and wraps. At the current 170 MHz core clock it
wraps in about 25 seconds. The PC uses unsigned modular subtraction to calculate
elapsed cycles, then converts to time using the clock value in the batch. FPS
and durations are calculated on the PC; the MCU only records and transmits
counter readings. Samples are batched and sent about once per second.
