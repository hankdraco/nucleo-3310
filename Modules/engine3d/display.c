#include "Modules/engine3d/display.h"

#include <stddef.h>
#include <string.h>

static bool display_coordinates_valid(uint8_t x, uint8_t y)
{
    return x < DISPLAY_WIDTH && y < DISPLAY_HEIGHT;
}

void display_init(
    display_t *display,
    const display_config_t *config)
{
    display->config = *config;
    display_clear(display);
}

void display_clear(display_t *display)
{
    memset(
        display->framebuffer,
        0,
        sizeof(display->framebuffer)
    );
}

bool display_set_pixel(
    display_t *display,
    uint8_t x,
    uint8_t y,
    bool on)
{
    if (!display_coordinates_valid(x, y))
    {
        return false;
    }

    size_t bit_index = (size_t)y * DISPLAY_WIDTH + x;
    size_t byte_index = bit_index / 8U;
    uint8_t bit_mask = (uint8_t)(1U << (bit_index % 8U));

    if (on)
    {
        display->framebuffer[byte_index] |= bit_mask;
    }
    else
    {
        display->framebuffer[byte_index] &= (uint8_t)~bit_mask;
    }

    return true;
}

bool display_get_pixel(
    const display_t *display,
    uint8_t x,
    uint8_t y,
    bool *on)
{
    if (!display_coordinates_valid(x, y))
    {
        return false;
    }

    size_t bit_index = (size_t)y * DISPLAY_WIDTH + x;
    size_t byte_index = bit_index / 8U;
    uint8_t bit_mask = (uint8_t)(1U << (bit_index % 8U));

    *on = (display->framebuffer[byte_index] & bit_mask) != 0U;

    return true;
}

bool display_draw_line(
    display_t *display,
    int16_t x0,
    int16_t y0,
    int16_t x1,
    int16_t y1)
{
    if (x0 < 0 || x0 >= DISPLAY_WIDTH ||
        y0 < 0 || y0 >= DISPLAY_HEIGHT ||
        x1 < 0 || x1 >= DISPLAY_WIDTH ||
        y1 < 0 || y1 >= DISPLAY_HEIGHT)
    {
        return false;
    }

    int16_t dx = x1 > x0 ? x1 - x0 : x0 - x1;
    int16_t sx = x0 < x1 ? 1 : -1;

    int16_t dy = y1 > y0 ? -(y1 - y0) : -(y0 - y1);
    int16_t sy = y0 < y1 ? 1 : -1;

    int16_t error = dx + dy;

    while (true)
    {
        display_set_pixel(
            display,
            (uint8_t)x0,
            (uint8_t)y0,
            true
        );

        if (x0 == x1 && y0 == y1)
        {
            break;
        }

        int16_t error2 = 2 * error;

        if (error2 >= dy)
        {
            error += dy;
            x0 += sx;
        }

        if (error2 <= dx)
        {
            error += dx;
            y0 += sy;
        }
    }

    return true;
}