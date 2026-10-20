#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_PROTOCOL_MAX_SAMPLES 30U
#define TELEMETRY_PROTOCOL_SAMPLE_SIZE 22U
#define TELEMETRY_PROTOCOL_HEADER_SIZE 8U
#define TELEMETRY_PROTOCOL_CRC_SIZE 2U
#define TELEMETRY_PROTOCOL_MAX_FRAME_SIZE \
    (TELEMETRY_PROTOCOL_HEADER_SIZE + \
     TELEMETRY_PROTOCOL_MAX_SAMPLES * TELEMETRY_PROTOCOL_SAMPLE_SIZE + \
     TELEMETRY_PROTOCOL_CRC_SIZE)

/* Raw DWT cycle counter readings from one rendered frame. */
typedef struct
{
    uint16_t sequence;
    uint32_t update_start_cycles;
    uint32_t update_end_cycles;
    uint32_t render_start_cycles;
    uint32_t render_end_cycles;
    uint32_t lcd_end_cycles;
} telemetry_sample_t;

/* Encodes a batch. The caller owns the output buffer. */
bool telemetry_protocol_encode(
    uint32_t core_clock_hz,
    const telemetry_sample_t *samples,
    size_t sample_count,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_size);

#endif /* TELEMETRY_PROTOCOL_H */
