#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_PROTOCOL_MAX_SAMPLES 30U /**< Maximum samples encoded in one frame. */
#define TELEMETRY_PROTOCOL_SAMPLE_SIZE 22U /**< Encoded bytes per timing sample. */
#define TELEMETRY_PROTOCOL_HEADER_SIZE 8U /**< Fixed header size, including sequence/count metadata. */
#define TELEMETRY_PROTOCOL_CRC_SIZE 2U /**< CRC trailer size in bytes. */
#define TELEMETRY_PROTOCOL_MAX_FRAME_SIZE \
    (TELEMETRY_PROTOCOL_HEADER_SIZE + \
     TELEMETRY_PROTOCOL_MAX_SAMPLES * TELEMETRY_PROTOCOL_SAMPLE_SIZE + \
     TELEMETRY_PROTOCOL_CRC_SIZE) /**< Largest encoded telemetry frame in bytes. */

/** Raw DWT cycle counter readings captured for one rendered frame. */
typedef struct
{
    uint16_t sequence; /**< Sequence number used to detect missing samples. */
    uint32_t update_start_cycles; /**< DWT counter at engine-update start. */
    uint32_t update_end_cycles; /**< DWT counter at engine-update completion. */
    uint32_t render_start_cycles; /**< DWT counter at render start. */
    uint32_t render_end_cycles; /**< DWT counter at render completion. */
    uint32_t lcd_end_cycles; /**< DWT counter after LCD transfer completion. */
} telemetry_sample_t;

/** Encode a batch of raw timing samples into the STM32-to-host frame format.
 * The caller owns and retains the output buffer.
 * @param core_clock_hz MCU core clock frequency used by the host to interpret cycles.
 * @param samples Samples to encode.
 * @param sample_count Number of samples; must not exceed TELEMETRY_PROTOCOL_MAX_SAMPLES.
 * @param output Destination frame buffer.
 * @param output_capacity Destination capacity in bytes.
 * @param output_size Receives the encoded frame length.
 * @return true on success; false for invalid arguments or insufficient capacity.
 */
bool telemetry_protocol_encode(
    uint32_t core_clock_hz,
    const telemetry_sample_t *samples,
    size_t sample_count,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_size);

#endif /* TELEMETRY_PROTOCOL_H */
