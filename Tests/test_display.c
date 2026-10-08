#include "../Core/Inc/display.h"
#include "test_common.h"
#include "test_fake_transport.h"
#include "../Core/Inc/display_pcd8544.h"

static bool test_display_clear(void)
{
    display_t display;
    display_config_t config =
    {
        .orientation = DISPLAY_ORIENTATION_NORMAL
    };
    display_init(&display, &config);

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
    display_config_t config =
    {
        .orientation = DISPLAY_ORIENTATION_NORMAL
    };
    display_init(&display, &config);

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
    display_config_t config =
    {
        .orientation = DISPLAY_ORIENTATION_NORMAL
    };
    display_init(&display, &config);

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

static bool test_display_pcd8544_flush(void)
{
    display_t display;
    display_config_t config =
    {
        .orientation = DISPLAY_ORIENTATION_NORMAL
    };
    display_init(&display, &config);

    /*
     * Selected pixels exercise:
     *
     * - bit 0
     * - bit 7
     * - crossing an X byte boundary
     * - crossing a Y bank boundary
     * - bottom-right corner
     */
    CHECK(display_set_pixel(&display, 0U, 0U, true));
    CHECK(display_set_pixel(&display, 1U, 0U, true));
    CHECK(display_set_pixel(&display, 7U, 0U, true));
    CHECK(display_set_pixel(&display, 8U, 0U, true));

    CHECK(display_set_pixel(&display, 0U, 7U, true));
    CHECK(display_set_pixel(&display, 0U, 8U, true));

    CHECK(display_set_pixel(
        &display,
        DISPLAY_WIDTH - 1U,
        DISPLAY_HEIGHT - 1U,
        true));

    fake_transport_t fake;
    fake_transport_init(&fake);

    pcd8544_t lcd =
    {
        .transport = &fake.transport
    };

    CHECK(display_pcd8544_flush(&display, &lcd));
    CHECK(fake.overflowed == false);

    /*
     * Each bank requires:
     *
     *   1 write for Y address
     *   1 write for X address
     *   1 write for 84 bytes of display data
     *
     * 6 banks => 18 write calls.
     */
    CHECK(fake.write_call_count == PCD8544_BANKS * 3U);

    /*
     * Build the expected PCD8544 representation.
     */
    uint8_t expected[
        PCD8544_BANKS
    ][
        PCD8544_WIDTH
    ] = {0};

    expected[0U][0U] = 0x81U;
    expected[0U][1U] = 0x01U;
    expected[0U][7U] = 0x01U;
    expected[0U][8U] = 0x01U;

    expected[1U][0U] = 0x01U;

    expected[5U][83U] = 0x80U;

    /*
     * Verify every bank.
     */
    for (uint8_t bank = 0U; bank < PCD8544_BANKS; ++bank)
    {
        size_t data_call = (size_t)bank * 3U + 2U;

        CHECK(fake.write_sizes[data_call] == PCD8544_WIDTH);

        for (uint8_t x = 0U; x < PCD8544_WIDTH; ++x)
        {
            CHECK(
                fake.write_calls[data_call][x] ==
                expected[bank][x]
            );
        }
    }

    return true;
}

static bool test_display_pcd8544_flush_flip_y(void)
{
    display_t display;
    display_config_t config =
    {
        .orientation = DISPLAY_ORIENTATION_FLIP_Y
    };
    display_init(&display, &config);

    CHECK(display_set_pixel(
        &display,
        0U,
        0U,
        true));

    fake_transport_t fake;
    fake_transport_init(&fake);

    pcd8544_t lcd =
    {
        .transport = &fake.transport
    };

    CHECK(display_pcd8544_flush(&display, &lcd));
    CHECK(fake.overflowed == false);

    /*
     * Logical (0,0) must appear at physical (0,47).
     *
     * Physical Y=47 belongs to bank 5,
     * bit 7 of the first byte.
     */
    size_t data_call = 5U * 3U + 2U;

    CHECK(fake.write_sizes[data_call] == PCD8544_WIDTH);

    CHECK(fake.write_calls[data_call][0U] == 0x80U);

    /*
     * No other pixel should be set in that bank.
     */
    for (uint8_t x = 1U; x < PCD8544_WIDTH; ++x)
    {
        CHECK(fake.write_calls[data_call][x] == 0U);
    }

    /*
     * All other banks must remain empty.
     */
    for (uint8_t bank = 0U; bank < PCD8544_BANKS - 1U; ++bank)
    {
        data_call = (size_t)bank * 3U + 2U;

        for (uint8_t x = 0U; x < PCD8544_WIDTH; ++x)
        {
            CHECK(fake.write_calls[data_call][x] == 0U);
        }
    }

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

    if (!test_display_pcd8544_flush())
        return false;

    if (!test_display_pcd8544_flush_flip_y())
        return false;

    return true;
}