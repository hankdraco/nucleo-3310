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
                uint8_t logical_x = x;
                uint8_t logical_y = (uint8_t)(bank * 8U + bit);

                if (display->config.orientation & DISPLAY_ORIENTATION_FLIP_X)
                {
                    logical_x = DISPLAY_WIDTH - 1U - logical_x;
                }

                if (display->config.orientation & DISPLAY_ORIENTATION_FLIP_Y)
                {
                    logical_y = DISPLAY_HEIGHT - 1U - logical_y;
                }

                bool on = false;

                if (!display_get_pixel(
                        display,
                        logical_x,
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