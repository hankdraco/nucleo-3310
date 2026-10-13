#include "input_state.h"

#include <limits.h>
#include <stddef.h>

/*
 * Na STM32G474RE używamy CMSIS do krótkiego
 * wyłączenia przerwań podczas konsumpcji stanu.
 *
 * W kompilacji natywnej na PC sekcja krytyczna
 * jest pusta. Taki wariant nadaje się do testów
 * bez współbieżnego dostępu.
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

/*
 * Współdzielony stan:
 *
 * - zapis z kontekstu przerwania UART,
 * - odczyt i konsumpcja z głównej pętli.
 *
 * Założenie: input_state_publish_from_isr()
 * ma jednego autora - odbiornik UART.
 */
typedef struct
{
    uint32_t key_mask;
    int32_t mouse_dx;
    int32_t mouse_dy;
    uint32_t last_update_ms;

} input_state_storage_t;

static volatile input_state_storage_t state;

/*
 * Dodaje przyrost bez ryzyka przepełnienia
 * typu ze znakiem, które w C byłoby UB.
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
    /*
     * Inicjalizacja musi nastąpić przed
     * uruchomieniem odbioru UART.
     */
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

    /*
     * Funkcja działa w kontekście ISR.
     * Główna pętla nie może wykonywać się
     * równolegle z tym kodem na jednym rdzeniu.
     *
     * Nie wyłączamy tu ponownie przerwań.
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

    /*
     * Pobranie stanu i wyzerowanie myszy
     * muszą stanowić jedną spójną operację
     * względem przerwania UART.
     */
    uint32_t primask = critical_section_enter();

    /*
     * Odejmowanie bez znaku poprawnie obsługuje
     * przejście licznika czasu przez UINT32_MAX,
     * przy założeniu kontrolowanego interwału.
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

    /*
     * Przyrosty myszy są konsumowane dokładnie raz.
     * Stan klawiszy pozostaje utrzymany do kolejnej
     * aktualizacji lub wystąpienia timeoutu.
     */
    state.mouse_dx = 0;
    state.mouse_dy = 0;

    critical_section_exit(primask);
}