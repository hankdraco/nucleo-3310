#ifndef INPUT_STATE_H
#define INPUT_STATE_H

#include <stdint.h>

#include "Protocols/input/input_types.h"

#define INPUT_STATE_TIMEOUT_MS 250U /**< Time without valid input after which keys are released. */

/** Initialize shared input state. Call before starting UART reception. */
void input_state_init(void);

/** Publish a decoded frame from the UART receive-interrupt context.
 * @param frame Decoded input frame to publish.
 * @param received_at_ms System tick when the valid frame was received.
 */
void input_state_publish_from_isr(
    const input_frame_t *frame,
    uint32_t received_at_ms);

/** Read the current input snapshot from the main loop.
 * Applies the input timeout and consumes accumulated mouse deltas.
 * @param now_ms Current system tick.
 * @param snapshot Destination for the resulting input state.
 */
void input_state_consume(
    uint32_t now_ms,
    input_snapshot_t *snapshot);

#endif /* INPUT_STATE_H */
