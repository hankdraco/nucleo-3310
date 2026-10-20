#include "Protocols/telemetry/telemetry_protocol.h"

static void write_u16_le(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8U);
}

static void write_u32_le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8U);
    out[2] = (uint8_t)(value >> 16U);
    out[3] = (uint8_t)(value >> 24U);
}

static uint16_t crc16_ccitt(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0U; i < size; ++i)
    {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
        }
    }

    return crc;
}

bool telemetry_protocol_encode(
    uint32_t core_clock_hz,
    const telemetry_sample_t *samples,
    size_t sample_count,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_size)
{
    if (samples == NULL || output == NULL || output_size == NULL ||
        sample_count == 0U ||
        sample_count > TELEMETRY_PROTOCOL_MAX_SAMPLES)
    {
        return false;
    }

    const size_t frame_size = TELEMETRY_PROTOCOL_HEADER_SIZE +
        sample_count * TELEMETRY_PROTOCOL_SAMPLE_SIZE +
        TELEMETRY_PROTOCOL_CRC_SIZE;
    if (output_capacity < frame_size)
    {
        return false;
    }

    output[0] = 0x54U; /* 'T' */
    output[1] = 0x4DU; /* 'M' */
    output[2] = 1U;    /* Protocol version */
    output[3] = (uint8_t)sample_count;
    write_u32_le(&output[4], core_clock_hz);

    size_t offset = TELEMETRY_PROTOCOL_HEADER_SIZE;
    for (size_t i = 0U; i < sample_count; ++i)
    {
        write_u16_le(&output[offset], samples[i].sequence);
        write_u32_le(&output[offset + 2U], samples[i].update_start_cycles);
        write_u32_le(&output[offset + 6U], samples[i].update_end_cycles);
        write_u32_le(&output[offset + 10U], samples[i].render_start_cycles);
        write_u32_le(&output[offset + 14U], samples[i].render_end_cycles);
        write_u32_le(&output[offset + 18U], samples[i].lcd_end_cycles);
        offset += TELEMETRY_PROTOCOL_SAMPLE_SIZE;
    }

    const uint16_t crc = crc16_ccitt(output, offset);
    write_u16_le(&output[offset], crc);
    *output_size = frame_size;
    return true;
}
