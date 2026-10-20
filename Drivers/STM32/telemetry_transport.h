#ifndef TELEMETRY_TRANSPORT_H
#define TELEMETRY_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

/** Initialize the non-blocking UART telemetry transmitter.
 * @param uart HAL UART handle used for transmission.
 * @return true on success; false if uart is invalid or already unavailable.
 */
bool telemetry_transport_init(UART_HandleTypeDef *uart);

/** Start transmitting a caller-owned buffer without waiting.
 * The buffer must remain valid and unchanged until the completion callback.
 * @param data Encoded bytes to transmit.
 * @param size Number of bytes to transmit.
 * @return true if transmission was started; false if busy or HAL rejected it.
 */
bool telemetry_transport_send(const uint8_t *data, size_t size);

/** Check whether an asynchronous UART transmission is in progress.
 * @return true while the transport owns an active transmission.
 */
bool telemetry_transport_is_busy(void);

/** Notify the transport that the UART transmit completed.
 * Called by the module that owns HAL UART callbacks.
 * @param uart UART handle reported by HAL.
 */
void telemetry_transport_on_tx_complete(UART_HandleTypeDef *uart);
/** Notify the transport that a UART error occurred.
 * Called by the module that owns HAL UART callbacks.
 * @param uart UART handle reported by HAL.
 */
void telemetry_transport_on_error(UART_HandleTypeDef *uart);

#endif /* TELEMETRY_TRANSPORT_H */
