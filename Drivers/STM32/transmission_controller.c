#include "Drivers/STM32/transmission_controller.h"

#include "Protocols/input/input_protocol.h"
#include "App/input_state.h"
#include "stm32g4xx_nucleo.h"
#include "Drivers/STM32/telemetry_transport.h"

#include <stdint.h>

/* One byte passed to HAL. Static storage is required because HAL continues
 * using this buffer after the start function returns.
 */
static uint8_t rx_byte;

/* The transmission controller is the sole owner of this parser. */
static input_protocol_t parser;

/* Set in the ISR if reception cannot be re-armed; read by main(). */
static volatile uint32_t receive_fault;

/* Re-arm reception for one byte. */
static void arm_receive(UART_HandleTypeDef *huart)
{
    if (HAL_UART_Receive_IT(
            huart,
            &rx_byte,
            1U) != HAL_OK)
    {
        receive_fault = 1U;
    }
}

bool transmission_controller_start(void)
{
    input_protocol_init(&parser);
    receive_fault = 0U;

    /* CubeMX did not configure the LPUART1 NVIC line, so configure it here. */
    HAL_NVIC_SetPriority(LPUART1_IRQn, 5U, 0U);
    HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);

    /* Arm reception before enabling the NVIC interrupt. */
    if (HAL_UART_Receive_IT(
            &hcom_uart[COM1],
            &rx_byte,
            1U) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_EnableIRQ(LPUART1_IRQn);

    return true;
}

bool transmission_controller_has_fault(void)
{
    return receive_fault != 0U;
}

/* HAL callback invoked after one byte has been received. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != LPUART1)
    {
        return;
    }

    input_frame_t frame;

    input_protocol_result_t result =
        input_protocol_feed(
            &parser,
            rx_byte,
            &frame);

    if (result == INPUT_PROTOCOL_FRAME_READY)
    {
        input_state_publish_from_isr(
            &frame,
            HAL_GetTick());
    }

    /* NO_FRAME means more bytes are needed. FRAME_REJECTED means the parser
     * discarded a frame; reception can continue with the next one.
     */
    arm_receive(huart);
}

/* HAL callback invoked after a UART error. Discard the incomplete frame and
 * attempt to restart reception from the beginning.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != LPUART1)
    {
        return;
    }

    telemetry_transport_on_error(huart);

    input_protocol_init(&parser);
    arm_receive(huart);
}

/* The transmission controller remains the sole HAL TX callback owner. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    telemetry_transport_on_tx_complete(huart);
}
