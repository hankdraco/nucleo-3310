#include "input_protocol.h"

#include <stdbool.h>
#include <stdint.h>

static input_protocol_result_t feed_text(
    input_protocol_t *parser,
    const char *text,
    input_frame_t *output)
{
    input_protocol_result_t result =
        INPUT_PROTOCOL_NO_FRAME;

    while (*text != '\0')
    {
        input_protocol_result_t current =
            input_protocol_feed(
                parser,
                (uint8_t)(unsigned char)*text,
                output);

        if (current != INPUT_PROTOCOL_NO_FRAME)
        {
            result = current;
        }

        ++text;
    }

    return result;
}

static bool test_input_protocol_zero_values(void)
{
    input_protocol_t parser;
    input_frame_t output = {0};

    input_protocol_init(&parser);

    if (feed_text(
            &parser,
            "@00000000,0000,0000*49\n",
            &output) != INPUT_PROTOCOL_FRAME_READY)
    {
        return false;
    }

    if (output.key_mask != 0U)
        return false;

    if (output.mouse_dx != 0)
        return false;

    if (output.mouse_dy != 0)
        return false;

    return true;
}

static bool test_input_protocol_twos_complement(void)
{
    input_protocol_t parser;
    input_frame_t output = {0};

    input_protocol_init(&parser);

    /* -1 oraz -32768 */
    if (feed_text(
            &parser,
            "@00000001,FFFF,8000*54\n",
            &output) != INPUT_PROTOCOL_FRAME_READY)
    {
        return false;
    }

    if (output.key_mask != 1U)
        return false;

    if (output.mouse_dx != -1)
        return false;

    if (output.mouse_dy != -32768)
        return false;

    /* +32767 oraz -32767 */
    if (feed_text(
            &parser,
            "@00000002,7FFF,8001*99\n",
            &output) != INPUT_PROTOCOL_FRAME_READY)
    {
        return false;
    }

    if (output.key_mask != 2U)
        return false;

    if (output.mouse_dx != 32767)
        return false;

    if (output.mouse_dy != -32767)
        return false;

    return true;
}

static bool test_input_protocol_crlf(void)
{
    input_protocol_t parser;
    input_frame_t output = {0};

    input_protocol_init(&parser);

    if (feed_text(
            &parser,
            "@00000001,FFFF,8000*54\r\n",
            &output) != INPUT_PROTOCOL_FRAME_READY)
    {
        return false;
    }

    if (output.mouse_dx != -1)
        return false;

    if (output.mouse_dy != -32768)
        return false;

    return true;
}

static bool test_input_protocol_bad_crc(void)
{
    input_protocol_t parser;

    input_frame_t output = {
        .key_mask = 123U,
        .mouse_dx = 456,
        .mouse_dy = 789
    };

    input_protocol_init(&parser);

    if (feed_text(
            &parser,
            "@00000000,0000,0000*48\n",
            &output) != INPUT_PROTOCOL_FRAME_REJECTED)
    {
        return false;
    }

    /*
     * Odrzucona ramka nie może zmienić
     * danych wyjściowych parsera.
     */
    if (output.key_mask != 123U)
        return false;

    if (output.mouse_dx != 456)
        return false;

    if (output.mouse_dy != 789)
        return false;

    return true;
}

static bool test_input_protocol_invalid_hex(void)
{
    input_protocol_t parser;
    input_frame_t output = {0};

    input_protocol_init(&parser);

    /*
     * CRC jest poprawne, ale G nie jest
     * prawidłową cyfrą szesnastkową.
     */
    if (feed_text(
            &parser,
            "@00000001,GFFF,8000*2D\n",
            &output) != INPUT_PROTOCOL_FRAME_REJECTED)
    {
        return false;
    }

    return true;
}

static bool test_input_protocol_incomplete_frame(void)
{
    input_protocol_t parser;
    input_frame_t output = {0};

    input_protocol_init(&parser);

    if (feed_text(
            &parser,
            "@00000000,0000,0000*49",
            &output) != INPUT_PROTOCOL_NO_FRAME)
    {
        return false;
    }

    /*
     * Parser powinien rozpoznać ramkę
     * dopiero po otrzymaniu LF.
     */
    if (input_protocol_feed(
            &parser,
            (uint8_t)'\n',
            &output) != INPUT_PROTOCOL_FRAME_READY)
    {
        return false;
    }

    if (output.key_mask != 0U ||
        output.mouse_dx != 0 ||
        output.mouse_dy != 0)
    {
        return false;
    }

    return true;
}

static bool test_input_protocol_overflow_recovery(void)
{
    input_protocol_t parser;
    input_frame_t output = {0};

    input_protocol_init(&parser);

    /* Rozpoczynamy ramkę. */
    if (input_protocol_feed(
            &parser,
            (uint8_t)'@',
            &output) != INPUT_PROTOCOL_NO_FRAME)
    {
        return false;
    }

    /* Celowo przekraczamy pojemność bufora. */
    for (unsigned i = 0U; i < 40U; ++i)
    {
        input_protocol_feed(
            &parser,
            (uint8_t)'A',
            &output);
    }

    if (input_protocol_feed(
            &parser,
            (uint8_t)'\n',
            &output) != INPUT_PROTOCOL_FRAME_REJECTED)
    {
        return false;
    }

    /*
     * Po odrzuceniu nadmiernie długiej linii
     * parser powinien przyjąć nową ramkę.
     */
    if (feed_text(
            &parser,
            "@00000000,0000,0000*49\n",
            &output) != INPUT_PROTOCOL_FRAME_READY)
    {
        return false;
    }

    if (output.key_mask != 0U)
        return false;

    return true;
}

bool test_input_protocol_suite(void)
{
    if (!test_input_protocol_zero_values())
        return false;

    if (!test_input_protocol_twos_complement())
        return false;

    if (!test_input_protocol_crlf())
        return false;

    if (!test_input_protocol_bad_crc())
        return false;

    if (!test_input_protocol_invalid_hex())
        return false;

    if (!test_input_protocol_incomplete_frame())
        return false;

    if (!test_input_protocol_overflow_recovery())
        return false;

    return true;
}