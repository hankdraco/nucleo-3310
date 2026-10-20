#include "Drivers/STM32/telemetry_transport.h"

#include <limits.h>

static UART_HandleTypeDef *telemetry_uart;
static volatile uint8_t telemetry_tx_busy;

bool telemetry_transport_init(UART_HandleTypeDef *uart)
{
    if (uart == NULL)
    {
        return false;
    }

    telemetry_uart = uart;
    telemetry_tx_busy = 0U;
    return true;
}

bool telemetry_transport_send(const uint8_t *data, size_t size)
{
    if (telemetry_uart == NULL || data == NULL || size == 0U ||
        size > UINT16_MAX || telemetry_tx_busy != 0U)
    {
        return false;
    }

    telemetry_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(
            telemetry_uart,
            (uint8_t *)data,
            (uint16_t)size) != HAL_OK)
    {
        telemetry_tx_busy = 0U;
        return false;
    }

    return true;
}

bool telemetry_transport_is_busy(void)
{
    return telemetry_tx_busy != 0U;
}

void telemetry_transport_on_tx_complete(UART_HandleTypeDef *uart)
{
    if (uart != NULL && uart == telemetry_uart)
    {
        telemetry_tx_busy = 0U;
    }
}

void telemetry_transport_on_error(UART_HandleTypeDef *uart)
{
    if (uart != NULL && uart == telemetry_uart &&
        uart->gState != HAL_UART_STATE_BUSY_TX &&
        uart->gState != HAL_UART_STATE_BUSY_TX_RX)
    {
        telemetry_tx_busy = 0U;
    }
}
