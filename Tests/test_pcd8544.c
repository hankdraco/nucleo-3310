#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "../Core/Inc/pcd8544.h"

#define CHECK(condition)                                                   \
    do                                                                      \
    {                                                                       \
        if (!(condition))                                                   \
        {                                                                   \
            printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #condition);   \
            return false;                                                   \
        }                                                                   \
    } while (0)

typedef struct
{
    uint8_t written_bytes[16];
    size_t written_count;

    bool dc_history[16];
    size_t dc_count;

    bool sce_history[16];
    size_t sce_count;

    bool reset_history[16];
    size_t reset_count;

    uint32_t delays_ms[16];
    size_t delay_count;

} fake_transport_t;

static void fake_set_sce(void *context, bool active)
{
    fake_transport_t *fake = context;

    if (fake->sce_count < sizeof(fake->sce_history))
    {
        fake->sce_history[fake->sce_count++] = active;
    }
}

static void fake_set_dc(void *context, bool data)
{
    fake_transport_t *fake = context;

    if (fake->dc_count < sizeof(fake->dc_history))
    {
        fake->dc_history[fake->dc_count++] = data;
    }
}

static void fake_set_reset(void *context, bool asserted)
{
    fake_transport_t *fake = context;

    if (fake->reset_count < sizeof(fake->reset_history))
    {
        fake->reset_history[fake->reset_count++] = asserted;
    }
}

static bool fake_write(
    void *context,
    const uint8_t *data,
    size_t size)
{
    fake_transport_t *fake = context;

    for (size_t i = 0; i < size; ++i)
    {
        CHECK(fake->written_count < sizeof(fake->written_bytes));

        fake->written_bytes[fake->written_count++] = data[i];
    }

    return true;
}

static void fake_delay_ms(void *context, uint32_t milliseconds)
{
    fake_transport_t *fake = context;

    if (fake->delay_count < sizeof(fake->delays_ms))
    {
        fake->delays_ms[fake->delay_count++] = milliseconds;
    }
}

static bool test_pcd8544_init(void)
{
    fake_transport_t fake = {0};

    const pcd8544_transport_t transport =
    {
        .context = &fake,
        .set_sce = fake_set_sce,
        .set_dc = fake_set_dc,
        .set_reset = fake_set_reset,
        .write = fake_write,
        .delay_ms = fake_delay_ms
    };

    pcd8544_t lcd;

    bool result = pcd8544_init(&lcd, &transport);

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

int main(void)
{
    printf("PCD8544 tests\n");
    printf("=============\n");

    if (!test_pcd8544_init())
    {
        printf("TESTS FAILED\n");
        return 1;
    }

    printf("PASS: test_pcd8544_init\n");
    printf("ALL TESTS PASSED\n");

    return 0;
}