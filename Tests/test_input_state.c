#include "input_state.h"

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

/*
 * Najnowsza ramka nadpisuje klawisze,
 * ale delty myszy są kumulowane.
 *
 * Snapshot konsumuje delty dokładnie raz.
 */
static bool test_input_state_accumulation_and_consumption(void)
{
    input_state_init();

    input_frame_t first = {
        .key_mask = 0x00000001U,
        .mouse_dx = 5,
        .mouse_dy = -3
    };

    input_frame_t second = {
        .key_mask = 0x00000002U,
        .mouse_dx = -2,
        .mouse_dy = 7
    };

    input_state_publish_from_isr(&first, 1000U);
    input_state_publish_from_isr(&second, 1016U);

    input_snapshot_t snapshot;

    input_state_consume(1017U, &snapshot);

    if (snapshot.key_mask != 0x00000002U)
        return false;

    if (snapshot.mouse_dx != 3)
        return false;

    if (snapshot.mouse_dy != 4)
        return false;

    /*
     * Druga konsumpcja nie może zwrócić
     * ponownie tych samych przyrostów.
     */
    input_state_consume(1018U, &snapshot);

    if (snapshot.key_mask != 0x00000002U)
        return false;

    if (snapshot.mouse_dx != 0)
        return false;

    if (snapshot.mouse_dy != 0)
        return false;

    return true;
}

/*
 * Timeout ma granicę 250 ms:
 * przy 249 ms stan nadal jest aktualny,
 * przy 250 ms zostaje wyzerowany.
 */
static bool test_input_state_timeout_boundary(void)
{
    input_state_init();

    input_frame_t frame = {
        .key_mask = 0x00000055U,
        .mouse_dx = 12,
        .mouse_dy = -8
    };

    input_state_publish_from_isr(&frame, 1000U);

    input_snapshot_t snapshot;

    input_state_consume(1249U, &snapshot);

    if (snapshot.key_mask != 0x00000055U)
        return false;

    if (snapshot.mouse_dx != 12)
        return false;

    if (snapshot.mouse_dy != -8)
        return false;

    input_state_consume(1250U, &snapshot);

    if (snapshot.key_mask != 0U)
        return false;

    if (snapshot.mouse_dx != 0)
        return false;

    if (snapshot.mouse_dy != 0)
        return false;

    return true;
}

/*
 * Stare, nieskonsumowane delty myszy
 * nie mogą zostać wykonane po timeoutcie.
 */
static bool test_input_state_timeout_discards_mouse(void)
{
    input_state_init();

    input_frame_t frame = {
        .key_mask = 0x00000004U,
        .mouse_dx = 100,
        .mouse_dy = -50
    };

    input_state_publish_from_isr(&frame, 2000U);

    input_snapshot_t snapshot;

    input_state_consume(2250U, &snapshot);

    if (snapshot.key_mask != 0U)
        return false;

    if (snapshot.mouse_dx != 0)
        return false;

    if (snapshot.mouse_dy != 0)
        return false;

    return true;
}

/*
 * Po timeoutcie nowa poprawna ramka
 * ponownie aktywuje sterowanie.
 */
static bool test_input_state_recovery_after_timeout(void)
{
    input_state_init();

    input_frame_t old_frame = {
        .key_mask = 0x00000001U,
        .mouse_dx = 20,
        .mouse_dy = 10
    };

    input_state_publish_from_isr(&old_frame, 3000U);

    input_snapshot_t snapshot;

    input_state_consume(3250U, &snapshot);

    if (snapshot.key_mask != 0U ||
        snapshot.mouse_dx != 0 ||
        snapshot.mouse_dy != 0)
    {
        return false;
    }

    input_frame_t new_frame = {
        .key_mask = 0x00000008U,
        .mouse_dx = -3,
        .mouse_dy = 2
    };

    input_state_publish_from_isr(&new_frame, 3251U);

    input_state_consume(3251U, &snapshot);

    if (snapshot.key_mask != 0x00000008U)
        return false;

    if (snapshot.mouse_dx != -3)
        return false;

    if (snapshot.mouse_dy != 2)
        return false;

    return true;
}

/*
 * HAL_GetTick() korzysta z licznika uint32_t,
 * który po przepełnieniu wraca do zera.
 *
 * Odejmowanie bez znaku powinno nadal poprawnie
 * wykrywać timeout na krótkich interwałach.
 */
static bool test_input_state_tick_wraparound(void)
{
    input_state_init();

    uint32_t received_at = UINT32_MAX - 100U;

    input_frame_t frame = {
        .key_mask = 0x00000010U,
        .mouse_dx = 4,
        .mouse_dy = -2
    };

    input_state_publish_from_isr(&frame, received_at);

    input_snapshot_t snapshot;

    /* 249 ms od odbioru, już po przepełnieniu licznika. */
    input_state_consume(received_at + 249U, &snapshot);

    if (snapshot.key_mask != 0x00000010U)
        return false;

    if (snapshot.mouse_dx != 4)
        return false;

    if (snapshot.mouse_dy != -2)
        return false;

    /* 250 ms od odbioru. */
    input_state_consume(received_at + 250U, &snapshot);

    if (snapshot.key_mask != 0U)
        return false;

    if (snapshot.mouse_dx != 0 ||
        snapshot.mouse_dy != 0)
    {
        return false;
    }

    return true;
}

/*
 * Akumulacja myszy nie może powodować
 * przepełnienia ze znakiem w języku C.
 */
static bool test_input_state_mouse_saturation(void)
{
    input_state_init();

    input_frame_t first = {
        .key_mask = 0U,
        .mouse_dx = INT32_MAX,
        .mouse_dy = INT32_MIN
    };

    input_frame_t second = {
        .key_mask = 0U,
        .mouse_dx = 1,
        .mouse_dy = -1
    };

    input_state_publish_from_isr(&first, 4000U);
    input_state_publish_from_isr(&second, 4001U);

    input_snapshot_t snapshot;

    input_state_consume(4002U, &snapshot);

    if (snapshot.mouse_dx != INT32_MAX)
        return false;

    if (snapshot.mouse_dy != INT32_MIN)
        return false;

    return true;
}

bool test_input_state_suite(void)
{
    if (!test_input_state_accumulation_and_consumption())
        return false;

    if (!test_input_state_timeout_boundary())
        return false;

    if (!test_input_state_timeout_discards_mouse())
        return false;

    if (!test_input_state_recovery_after_timeout())
        return false;

    if (!test_input_state_tick_wraparound())
        return false;

    if (!test_input_state_mouse_saturation())
        return false;

    return true;
}