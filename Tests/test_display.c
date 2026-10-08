#include "../Core/Inc/display.h"
#include "test_common.h"

static bool test_display_clear(void)
{
    display_t display;

    display_init(&display);

    display_set_pixel(&display, 10, 20, true);
    display_set_pixel(&display, 30, 40, true);

    display_clear(&display);

    for (uint8_t y = 0; y < DISPLAY_HEIGHT; ++y)
    {
        for (uint8_t x = 0; x < DISPLAY_WIDTH; ++x)
        {
            bool on = true;

            CHECK(display_get_pixel(
                &display,
                x,
                y,
                &on));

            CHECK(on == false);
        }
    }

    return true;
}

static bool test_display_set_pixel(void)
{
    display_t display;

    display_init(&display);

    CHECK(display_set_pixel(
        &display,
        0,
        0,
        true));

    bool on = false;

    CHECK(display_get_pixel(
        &display,
        0,
        0,
        &on));

    CHECK(on == true);

    /*
     * Neighboring pixels must remain unchanged.
     */
    CHECK(display_get_pixel(
        &display,
        1,
        0,
        &on));

    CHECK(on == false);

    CHECK(display_get_pixel(
        &display,
        0,
        1,
        &on));

    CHECK(on == false);

    /*
     * Clearing the same pixel must turn it off again.
     */
    CHECK(display_set_pixel(
        &display,
        0,
        0,
        false));

    CHECK(display_get_pixel(
        &display,
        0,
        0,
        &on));

    CHECK(on == false);

    return true;
}

static bool test_display_pixel_boundaries(void)
{
    display_t display;

    display_init(&display);

    bool on = false;

    CHECK(display_set_pixel(
        &display,
        DISPLAY_WIDTH - 1U,
        DISPLAY_HEIGHT - 1U,
        true));

    CHECK(display_get_pixel(
        &display,
        DISPLAY_WIDTH - 1U,
        DISPLAY_HEIGHT - 1U,
        &on));

    CHECK(on == true);

    /*
     * Out-of-range coordinates must be rejected.
     */
    CHECK(display_set_pixel(
        &display,
        DISPLAY_WIDTH,
        0,
        true) == false);

    CHECK(display_set_pixel(
        &display,
        0,
        DISPLAY_HEIGHT,
        true) == false);

    return true;
}

bool test_display_suite(void)
{
    if (!test_display_clear())
        return false;

    if (!test_display_set_pixel())
        return false;

    if (!test_display_pixel_boundaries())
        return false;

    return true;
}