#include "Drivers/PCD8544/pcd8544.h"
#include "test_common.h"
#include "test_fake_transport.h"

static bool test_pcd8544_init(void)
{
    fake_transport_t fake;

    fake_transport_init(&fake);

    pcd8544_t lcd;

    bool result = pcd8544_init(&lcd, &fake.transport);

    CHECK(result);

    /*
     * Hardware reset:
     *
     * RST asserted
     * 1 ms delay
     * RST deasserted
     * 1 ms delay
     */
    CHECK(fake.reset_count == 2);
    CHECK(fake.reset_history[0] == true);
    CHECK(fake.reset_history[1] == false);

    CHECK(fake.delay_count == 2);
    CHECK(fake.delays_ms[0] == 1);
    CHECK(fake.delays_ms[1] == 1);

    /*
     * PCD8544 initialization commands.
     */
    const uint8_t expected_commands[] =
    {
        0x21,
        0xBF,
        0x04,
        0x14,
        0x20,
        0x0C
    };

    CHECK(fake.written_count == sizeof(expected_commands));

    for (size_t i = 0; i < sizeof(expected_commands); ++i)
    {
        CHECK(fake.written_bytes[i] == expected_commands[i]);
    }

    /*
     * All initialization writes are commands,
     * therefore D/C must always be LOW.
     */
    CHECK(fake.dc_count == sizeof(expected_commands) + 1);

    for (size_t i = 0; i < fake.dc_count; ++i)
    {
        CHECK(fake.dc_history[i] == false);
    }

    /*
     * Every command is transmitted while SCE is active,
     * and released afterwards.
     */
    CHECK(fake.sce_count == sizeof(expected_commands) * 2 + 1);

    for (size_t i = 1; i < fake.sce_count; i += 2)
    {
        CHECK(fake.sce_history[i] == true);
        CHECK(fake.sce_history[i + 1] == false);
    }

    return true;
}

static bool test_pcd8544_write_data(void)
{
    fake_transport_t fake;

    fake_transport_init(&fake);

    pcd8544_t lcd =
    {
        .transport = &fake.transport
    };

    const uint8_t data[] =
    {
        0x12,
        0x34,
        0x56,
        0x78
    };

    CHECK(pcd8544_write_data(
        &lcd,
        data,
        sizeof(data)));

    /*
     * The data must be sent as one transport operation.
     */
    CHECK(fake.write_call_count == 1);
    CHECK(fake.write_sizes[0] == sizeof(data));

    /*
     * Verify the transmitted bytes.
     */
    CHECK(fake.written_count == sizeof(data));

    for (size_t i = 0; i < sizeof(data); ++i)
    {
        CHECK(fake.written_bytes[i] == data[i]);
    }

    /*
     * Data mode.
     */
    CHECK(fake.dc_count == 1);
    CHECK(fake.dc_history[0] == true);

    /*
     * SCE is active for the whole packet.
     */
    CHECK(fake.sce_count == 2);
    CHECK(fake.sce_history[0] == true);
    CHECK(fake.sce_history[1] == false);

    /*
     * No reset or delay should occur.
     */
    CHECK(fake.reset_count == 0);
    CHECK(fake.delay_count == 0);

    return true;
}

bool test_pcd8544_suite(void)
{
    if (!test_pcd8544_init())
        return false;

    if (!test_pcd8544_write_data())
        return false;

    return true;
}