#include "App/input_state.h"

#include <limits.h>
#include <stddef.h>

/* On STM32G474RE, CMSIS briefly disables interrupts while the input state is
 * consumed. In native PC builds the critical section is empty, which is
 * suitable for tests without concurrent access.
 */
#if defined(STM32G474xx)

#include "stm32g4xx.h"

static uint32_t critical_section_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    return primask;
}

static void critical_section_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

#else

static uint32_t critical_section_enter(void)
{
    return 0U;
}

static void critical_section_exit(uint32_t primask)
{
    (void)primask;
}

#endif

/* Shared state is written by the UART receive ISR and read/consumed by the
 * main loop. Assumption: input_state_publish_from_isr() has one writer, the
 * UART receiver.
 */
typedef struct
{
    uint32_t key_mask;
    int32_t mouse_dx;
    int32_t mouse_dy;
    uint32_t last_update_ms;

} input_state_storage_t;

static volatile input_state_storage_t state;

/* Add a delta with saturation, avoiding signed overflow, which is undefined
 * behavior in C.
 */
static int32_t saturating_add_i32(
    int32_t current,
    int32_t delta)
{
    if (delta > 0 && current > INT32_MAX - delta)
    {
        return INT32_MAX;
    }

    if (delta < 0 && current < INT32_MIN - delta)
    {
        return INT32_MIN;
    }

    return current + delta;
}

void input_state_init(void)
{
    /* Initialize this state before starting UART reception. */
    state.key_mask = 0U;
    state.mouse_dx = 0;
    state.mouse_dy = 0;
    state.last_update_ms = 0U;
}

void input_state_publish_from_isr(
    const input_frame_t *frame,
    uint32_t received_at_ms)
{
    if (frame == NULL)
    {
        return;
    }

    /* This function runs in the ISR context. On a single core, the main loop
     * cannot execute concurrently with it, so interrupts are not disabled again.
     */
    state.key_mask = frame->key_mask;

    state.mouse_dx = saturating_add_i32(
        state.mouse_dx,
        frame->mouse_dx);

    state.mouse_dy = saturating_add_i32(
        state.mouse_dy,
        frame->mouse_dy);

    state.last_update_ms = received_at_ms;
}

void input_state_consume(
    uint32_t now_ms,
    input_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    /* Reading the state and clearing mouse deltas must be atomic with respect
     * to the UART interrupt.
     */
    uint32_t primask = critical_section_enter();

    /* Unsigned subtraction handles the tick counter wrapping through
     * UINT32_MAX, assuming the elapsed interval is bounded.
     */
    uint32_t elapsed_ms =
        now_ms - state.last_update_ms;

    if (elapsed_ms >= INPUT_STATE_TIMEOUT_MS)
    {
        state.key_mask = 0U;
        state.mouse_dx = 0;
        state.mouse_dy = 0;
    }

    snapshot->key_mask = state.key_mask;
    snapshot->mouse_dx = state.mouse_dx;
    snapshot->mouse_dy = state.mouse_dy;

    /* Mouse deltas are consumed exactly once. Key state remains held until
     * the next input update or timeout.
     */
    state.mouse_dx = 0;
    state.mouse_dy = 0;

    critical_section_exit(primask);
}
