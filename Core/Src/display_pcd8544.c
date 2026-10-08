#include "display_pcd8544.h"

#include <stdint.h>

bool display_pcd8544_flush(
    const display_t *display,
    pcd8544_t *lcd
)
{
    uint8_t bank_buffer[PCD8544_WIDTH];

    for (uint8_t bank = 0; bank < PCD8544_BANKS; ++bank)
    {
        for (uint8_t x = 0; x < PCD8544_WIDTH; ++x)
        {
            uint8_t value = 0U;

            for (uint8_t bit = 0U; bit < 8U; ++bit)
            {
                uint8_t physical_y = (uint8_t)(bank * 8U + bit);

                uint8_t logical_y = display->flip_y
                    ? (uint8_t)(DISPLAY_HEIGHT - 1U - physical_y)
                    : physical_y;

                bool on = false;

                if (!display_get_pixel(
                        display,
                        x,
                        logical_y,
                        &on))
                {
                    return false;
                }

                if (on)
                {
                    value |= (uint8_t)(1U << bit);
                }
            }

            bank_buffer[x] = value;
        }

        if (!pcd8544_set_position(lcd, 0U, bank))
        {
            return false;
        }

        if (!pcd8544_write_data(
                lcd,
                bank_buffer,
                sizeof(bank_buffer)))
        {
            return false;
        }
    }

    return true;
}