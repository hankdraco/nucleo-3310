#ifndef INPUT_STATE_H
#define INPUT_STATE_H

#include <stdint.h>

#include "input_types.h"

#define INPUT_STATE_TIMEOUT_MS 250U

/*
 * Inicjalizuje stan wejścia.
 *
 * Wywołać przed uruchomieniem odbioru UART.
 */
void input_state_init(void);

/*
 * Publikuje dane z poprawnej ramki.
 *
 * Funkcję wywołuje TransmissionController
 * w kontekście przerwania UART.
 *
 * received_at_ms - czas odebrania poprawnej ramki.
 */
void input_state_publish_from_isr(
    const input_frame_t *frame,
    uint32_t received_at_ms);

/*
 * Pobiera aktualny stan wejścia.
 *
 * Wywoływać z głównej pętli aplikacji.
 * Funkcja sprawdza timeout, kopiuje stan
 * i konsumuje oczekujące przyrosty myszy.
 */
void input_state_consume(
    uint32_t now_ms,
    input_snapshot_t *snapshot);

#endif /* INPUT_STATE_H */