#include "pcd8544.h"

#define PCD8544_CMD_FUNCTION_SET_EXTENDED   0x21U
#define PCD8544_CMD_SET_VOP                 0xBFU
#define PCD8544_CMD_SET_TEMP_COEFF          0x04U
#define PCD8544_CMD_SET_BIAS                0x14U
#define PCD8544_CMD_FUNCTION_SET_BASIC      0x20U
#define PCD8544_CMD_DISPLAY_CONTROL_NORMAL  0x0CU

static bool pcd8544_write_command(
    pcd8544_t *lcd,
    uint8_t command
)
{
    const pcd8544_transport_t *transport = lcd->transport;

    transport->set_dc(transport->context, false);
    transport->set_sce(transport->context, true);

    bool success = transport->write(
        transport->context,
        &command,
        1U
    );

    transport->set_sce(transport->context, false);

    return success;
}

static bool pcd8544_write_data(
    pcd8544_t *lcd,
    uint8_t data
)
{
    const pcd8544_transport_t *transport = lcd->transport;

    transport->set_dc(transport->context, true);
    transport->set_sce(transport->context, true);

    bool success = transport->write(
        transport->context,
        &data,
        1U
    );

    transport->set_sce(transport->context, false);

    return success;
}

static void pcd8544_reset(pcd8544_t *lcd)
{
    const pcd8544_transport_t *transport = lcd->transport;

    transport->set_sce(transport->context, false);
    transport->set_dc(transport->context, false);

    transport->set_reset(transport->context, true);
    transport->delay_ms(transport->context, 1U);

    transport->set_reset(transport->context, false);
    transport->delay_ms(transport->context, 1U);
}

static bool pcd8544_set_position(
    pcd8544_t *lcd,
    uint8_t x,
    uint8_t bank
)
{
    if (x >= PCD8544_WIDTH || bank >= PCD8544_BANKS)
    {
        return false;
    }

    if (!pcd8544_write_command(lcd, 0x40U | bank))
    {
        return false;
    }

    if (!pcd8544_write_command(lcd, 0x80U | x))
    {
        return false;
    }

    return true;
}

bool pcd8544_init(
    pcd8544_t *lcd,
    const pcd8544_transport_t *transport
)
{
    lcd->transport = transport;

    pcd8544_reset(lcd);

    if (!pcd8544_write_command(
            lcd,
            PCD8544_CMD_FUNCTION_SET_EXTENDED))
    {
        return false;
    }

    if (!pcd8544_write_command(
            lcd,
            PCD8544_CMD_SET_VOP))
    {
        return false;
    }

    if (!pcd8544_write_command(
            lcd,
            PCD8544_CMD_SET_TEMP_COEFF))
    {
        return false;
    }

    if (!pcd8544_write_command(
            lcd,
            PCD8544_CMD_SET_BIAS))
    {
        return false;
    }

    if (!pcd8544_write_command(
            lcd,
            PCD8544_CMD_FUNCTION_SET_BASIC))
    {
        return false;
    }

    if (!pcd8544_write_command(
            lcd,
            PCD8544_CMD_DISPLAY_CONTROL_NORMAL))
    {
        return false;
    }

    return true;
}

void pcd8544_test_pattern(pcd8544_t *lcd)
{
    for (uint8_t bank = 0; bank < PCD8544_BANKS; ++bank)
    {
        if (!pcd8544_set_position(lcd, 0U, bank))
        {
            return;
        }

        for (uint8_t x = 0; x < PCD8544_BYTES_PER_BANK; ++x)
        {
            uint8_t pattern = (bank % 2U == 0U)
                ? 0xAAU
                : 0x55U;

            if (!pcd8544_write_data(lcd, pattern))
            {
                return;
            }
        }
    }
}