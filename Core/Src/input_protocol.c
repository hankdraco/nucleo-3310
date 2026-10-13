#include "input_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---------- Pomocnicze funkcje walidacji ---------- */

static int hex_digit_value(uint8_t ch)
{
    if (ch >= (uint8_t)'0' && ch <= (uint8_t)'9')
    {
        return (int)(ch - (uint8_t)'0');
    }

    if (ch >= (uint8_t)'A' && ch <= (uint8_t)'F')
    {
        return (int)(ch - (uint8_t)'A') + 10;
    }

    if (ch >= (uint8_t)'a' && ch <= (uint8_t)'f')
    {
        return (int)(ch - (uint8_t)'a') + 10;
    }

    return -1;
}

static bool parse_hex_u32(
    const uint8_t *data,
    size_t length,
    uint32_t *result)
{
    if (data == NULL || result == NULL || length != 8U)
    {
        return false;
    }

    uint32_t value = 0U;

    for (size_t i = 0U; i < length; ++i)
    {
        int digit = hex_digit_value(data[i]);

        if (digit < 0)
        {
            return false;
        }

        value = (value << 4U) | (uint32_t)digit;
    }

    *result = value;

    return true;
}

static bool parse_hex_u16(
    const uint8_t *data,
    size_t length,
    uint16_t *result)
{
    if (data == NULL || result == NULL || length != 4U)
    {
        return false;
    }

    uint16_t value = 0U;

    for (size_t i = 0U; i < length; ++i)
    {
        int digit = hex_digit_value(data[i]);

        if (digit < 0)
        {
            return false;
        }

        value = (uint16_t)(
            ((uint32_t)value << 4U) | (uint32_t)digit);
    }

    *result = value;
    return true;
}

/*
 * Interpretuje 16-bitową wartość zapisaną
 * w kodzie U2 jako liczbę ze znakiem.
 *
 * Wynik int32_t pozwala uniknąć konwersji
 * wartości spoza zakresu int16_t.
 */
static int32_t decode_i16_twos_complement(uint16_t raw)
{
    if (raw <= 0x7FFFU)
    {
        return (int32_t)raw;
    }

    return (int32_t)raw - 65536;
}

/*
 * CRC-8:
 * polynomial = 0x07
 * init       = 0x00
 * xorout     = 0x00
 *
 * Obliczamy od najbardziej znaczącego bitu.
 */
static uint8_t crc8(
    const uint8_t *data,
    size_t length)
{
    uint8_t crc = 0U;

    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];

        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return crc;
}

static bool decode_frame(
    const uint8_t *data,
    size_t length,
    input_frame_t *frame_out)
{
    /*
     * Dokładnie 22 znaki, bez LF i CR.
     */
    if (data == NULL ||
        frame_out == NULL ||
        length != 22U)
    {
        return false;
    }

    if (data[0]  != (uint8_t)'@' ||
        data[9]  != (uint8_t)',' ||
        data[14] != (uint8_t)',' ||
        data[19] != (uint8_t)'*')
    {
        return false;
    }

    /* Odczyt CRC: dokładnie dwie cyfry HEX. */
    int crc_hi = hex_digit_value(data[20]);
    int crc_lo = hex_digit_value(data[21]);

    if (crc_hi < 0 || crc_lo < 0)
    {
        return false;
    }

    uint8_t received_crc =
        (uint8_t)(((uint32_t)crc_hi << 4U) |
                  (uint32_t)crc_lo);

    /*
     * CRC obejmuje bajty od '@' do końca pola DY,
     * czyli 19 bajtów. Nie obejmuje '*' ani CRC.
     */
    if (crc8(data, 19U) != received_crc)
    {
        return false;
    }

    uint32_t key_mask;
    uint16_t raw_dx;
    uint16_t raw_dy;

    if (!parse_hex_u32(data + 1U, 8U, &key_mask) ||
        !parse_hex_u16(data + 10U, 4U, &raw_dx) ||
        !parse_hex_u16(data + 15U, 4U, &raw_dy))
    {
        return false;
    }

    /*
     * Wynik publikujemy dopiero po pełnej walidacji.
     */
    frame_out->key_mask = key_mask;
    frame_out->mouse_dx = decode_i16_twos_complement(raw_dx);
    frame_out->mouse_dy = decode_i16_twos_complement(raw_dy);

    return true;
}

/* ---------- Publiczny interfejs parsera ---------- */

void input_protocol_init(input_protocol_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    parser->length = 0U;
    parser->state = INPUT_PROTOCOL_WAITING;
}

input_protocol_result_t input_protocol_feed(
    input_protocol_t *parser,
    uint8_t byte,
    input_frame_t *frame_out)
{
    if (parser == NULL)
    {
        return INPUT_PROTOCOL_NO_FRAME;
    }

    /*
     * Marker początku pozwala odzyskać synchronizację
     * także wtedy, gdy poprzednia ramka była uszkodzona.
     */
    if (byte == (uint8_t)'@')
    {
        parser->buffer[0] = byte;
        parser->length = 1U;
        parser->state = INPUT_PROTOCOL_COLLECTING;

        return INPUT_PROTOCOL_NO_FRAME;
    }

    if (parser->state == INPUT_PROTOCOL_WAITING)
    {
        return INPUT_PROTOCOL_NO_FRAME;
    }

    /*
     * Po przepełnieniu odrzucamy resztę linii
     * aż do końca lub do następnego markera '@'.
     */
    if (parser->state == INPUT_PROTOCOL_DISCARDING)
    {
        if (byte == (uint8_t)'\n')
        {
            parser->length = 0U;
            parser->state = INPUT_PROTOCOL_WAITING;

            return INPUT_PROTOCOL_FRAME_REJECTED;
        }

        return INPUT_PROTOCOL_NO_FRAME;
    }

    if (byte == (uint8_t)'\n')
    {
        size_t length = parser->length;

        /* Obsługujemy zakończenia LF oraz CRLF. */
        if (length > 0U &&
            parser->buffer[length - 1U] == (uint8_t)'\r')
        {
            --length;
        }

        input_frame_t decoded_frame;

        bool valid = decode_frame(
            parser->buffer,
            length,
            &decoded_frame);

        parser->length = 0U;
        parser->state = INPUT_PROTOCOL_WAITING;

        if (valid && frame_out != NULL)
        {
            *frame_out = decoded_frame;

            return INPUT_PROTOCOL_FRAME_READY;
        }

        return INPUT_PROTOCOL_FRAME_REJECTED;
    }

    /*
     * Bufor zawiera bajty bieżącej ramki,
     * ale nie znak końca linii.
     */
    if (parser->length >= sizeof(parser->buffer))
    {
        parser->length = 0U;
        parser->state = INPUT_PROTOCOL_DISCARDING;

        return INPUT_PROTOCOL_NO_FRAME;
    }

    parser->buffer[parser->length] = byte;
    ++parser->length;

    return INPUT_PROTOCOL_NO_FRAME;
}