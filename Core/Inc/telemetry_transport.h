#ifndef TELEMETRY_TRANSPORT_H
#define TELEMETRY_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

/* Initializes the non-blocking UART transmitter. */
bool telemetry_transport_init(UART_HandleTypeDef *uart);

/* Starts transmitting a caller-owned buffer without waiting. */
bool telemetry_transport_send(const uint8_t *data, size_t size);

bool telemetry_transport_is_busy(void);

/* Called by the single HAL callback owner. */
void telemetry_transport_on_tx_complete(UART_HandleTypeDef *uart);
void telemetry_transport_on_error(UART_HandleTypeDef *uart);

#endif /* TELEMETRY_TRANSPORT_H */
