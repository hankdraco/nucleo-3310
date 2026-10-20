#include "Drivers/STM32/transmission_controller.h"

#include "Protocols/input/input_protocol.h"
#include "App/input_state.h"
#include "stm32g4xx_nucleo.h"
#include "Drivers/STM32/telemetry_transport.h"

#include <stdint.h>

/*
 * Jeden bajt przekazywany przez HAL.
 *
 * Zmienna statyczna, ponieważ HAL korzysta z niej
 * również po powrocie z funkcji start().
 */
static uint8_t rx_byte;

/* Parser należy wyłącznie do TransmissionController. */
static input_protocol_t parser;

/*
 * Ustawiany w ISR, jeśli nie uda się ponownie
 * uruchomić odbioru. Odczytywany przez main().
 */
static volatile uint32_t receive_fault;

/*
 * Ponownie uzbraja odbiór jednego bajtu.
 */
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

    /*
     * CubeMX nie skonfigurował NVIC dla LPUART1,
     * dlatego konfigurujemy tę linię ręcznie.
     */
    HAL_NVIC_SetPriority(LPUART1_IRQn, 5U, 0U);
    HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);

    /*
     * Najpierw uzbrajamy odbiór, a dopiero potem
     * włączamy przerwanie w NVIC.
     */
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

/*
 * Callback HAL wywoływany po odebraniu jednego bajtu.
 */
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

    /*
     * NO_FRAME: zbieramy dalsze bajty.
     *
     * FRAME_REJECTED: parser odrzucił ramkę;
     * kolejna ramka może być odbierana normalnie.
     */
    arm_receive(huart);
}

/*
 * Callback HAL wywoływany po błędzie UART.
 *
 * Odrzucamy niedokończoną ramkę i próbujemy
 * wznowić odbiór od nowa.
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
