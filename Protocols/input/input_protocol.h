#ifndef INPUT_PROTOCOL_H
#define INPUT_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "Protocols/input/input_types.h"

#define INPUT_PROTOCOL_BUFFER_SIZE 32U /**< Maximum bytes retained for one protocol line. */

/** Result of feeding one byte to the streaming parser. */
typedef enum
{
    INPUT_PROTOCOL_NO_FRAME = 0,
    INPUT_PROTOCOL_FRAME_READY,
    INPUT_PROTOCOL_FRAME_REJECTED

} input_protocol_result_t;

/** Parser's current line-collection state. */
typedef enum
{
    INPUT_PROTOCOL_WAITING = 0,
    INPUT_PROTOCOL_COLLECTING,
    INPUT_PROTOCOL_DISCARDING

} input_protocol_state_t;

/** State for parsing one byte stream; public for static allocation without malloc. */
typedef struct
{
    uint8_t buffer[INPUT_PROTOCOL_BUFFER_SIZE]; /**< Current line bytes. */
    size_t length; /**< Number of bytes currently stored. */
    input_protocol_state_t state; /**< Waiting, collecting, or discarding an oversized line. */

} input_protocol_t;

/** Reset a parser to the initial waiting state.
 * @param parser Parser instance to initialize.
 */
void input_protocol_init(input_protocol_t *parser);

/** Feed one byte into the streaming parser.
 * On FRAME_READY, frame_out contains a complete decoded frame. On FRAME_REJECTED,
 * a malformed or oversized line was discarded. NO_FRAME means more bytes are needed.
 * @param parser Parser instance for this byte stream.
 * @param byte Next byte received from the stream.
 * @param frame_out Destination for a decoded frame when one is ready.
 * @return Parsing result for this byte.
 */
input_protocol_result_t input_protocol_feed(
    input_protocol_t *parser,
    uint8_t byte,
    input_frame_t *frame_out);

#endif /* INPUT_PROTOCOL_H */
