#include "display.h"

#include <stddef.h>
#include <string.h>

static bool display_coordinates_valid(uint8_t x, uint8_t y)
{
    return x < DISPLAY_WIDTH && y < DISPLAY_HEIGHT;
}

void display_init(display_t *display, bool flip_y)
{
    display->flip_y = flip_y;
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