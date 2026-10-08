#include "pcd8544_transport.h"

#include "gpio.h"
#include "spi.h"

static void stm32_set_sce(void *context, bool active)
{
    (void)context;

    HAL_GPIO_WritePin(
        LCD_SCE_GPIO_Port,
        LCD_SCE_Pin,
        active ? GPIO_PIN_RESET : GPIO_PIN_SET
    );
}

static void stm32_set_dc(void *context, bool data)
{
    (void)context;

    HAL_GPIO_WritePin(
        LCD_DC_GPIO_Port,
        LCD_DC_Pin,
        data ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

static void stm32_set_reset(void *context, bool asserted)
{
    (void)context;

    HAL_GPIO_WritePin(
        LCD_RST_GPIO_Port,
        LCD_RST_Pin,
        asserted ? GPIO_PIN_RESET : GPIO_PIN_SET
    );
}

static bool stm32_write(void *context, const uint8_t *data, size_t size)
{
    (void)context;

    if (size > UINT16_MAX)
    {
        return false;
    }

    return HAL_SPI_Transmit(
               &hspi1,
               (uint8_t *)data,
               (uint16_t)size,
               HAL_MAX_DELAY
           ) == HAL_OK;
}

static void stm32_delay_ms(void *context, uint32_t milliseconds)
{
    (void)context;

    HAL_Delay(milliseconds);
}

void pcd8544_transport_stm32_init(pcd8544_transport_t *transport)
{
    transport->context = NULL;

    transport->set_sce = stm32_set_sce;
    transport->set_dc = stm32_set_dc;
    transport->set_reset = stm32_set_reset;

    transport->write = stm32_write;

    transport->delay_ms = stm32_delay_ms;
}