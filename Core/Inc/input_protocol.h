#ifndef INPUT_PROTOCOL_H
#define INPUT_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "input_types.h"

#define INPUT_PROTOCOL_BUFFER_SIZE 32U

typedef enum
{
    INPUT_PROTOCOL_NO_FRAME = 0,
    INPUT_PROTOCOL_FRAME_READY,
    INPUT_PROTOCOL_FRAME_REJECTED

} input_protocol_result_t;

typedef enum
{
    INPUT_PROTOCOL_WAITING = 0,
    INPUT_PROTOCOL_COLLECTING,
    INPUT_PROTOCOL_DISCARDING

} input_protocol_state_t;

/*
 * Stan parsera. Jedna instancja parsera obsługuje
 * jeden strumień bajtów.
 *
 * Struktura jest publiczna, aby można było
 * zaalokować ją statycznie, bez malloc().
 */
typedef struct
{
    uint8_t buffer[INPUT_PROTOCOL_BUFFER_SIZE];
    size_t length;
    input_protocol_state_t state;

} input_protocol_t;

void input_protocol_init(input_protocol_t *parser);

/*
 * Przekazuje parserowi jeden bajt.
 *
 * FRAME_READY:
 *     frame_out zawiera kompletną, poprawną ramkę.
 *
 * FRAME_REJECTED:
 *     Odrzucono kompletną ramkę lub przepełnioną linię.
 *
 * NO_FRAME:
 *     Parser czeka na dalsze bajty.
 *
 * frame_out musi wskazywać prawidłowy obiekt.
 */
input_protocol_result_t input_protocol_feed(
    input_protocol_t *parser,
    uint8_t byte,
    input_frame_t *frame_out);

#endif /* INPUT_PROTOCOL_H */