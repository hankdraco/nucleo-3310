#include "test_fake_transport.h"

#include <string.h>

static void fake_set_sce(void *context, bool active)
{
    fake_transport_t *fake = context;

    if (fake->sce_count >= FAKE_MAX_HISTORY)
    {
        fake->overflowed = true;
        return;
    }

    fake->sce_history[fake->sce_count++] = active;
}

static void fake_set_dc(void *context, bool data)
{
    fake_transport_t *fake = context;

    if (fake->dc_count >= FAKE_MAX_HISTORY)
    {
        fake->overflowed = true;
        return;
    }

    fake->dc_history[fake->dc_count++] = data;
}

static void fake_set_reset(void *context, bool asserted)
{
    fake_transport_t *fake = context;

    if (fake->reset_count >= FAKE_MAX_HISTORY)
    {
        fake->overflowed = true;
        return;
    }

    fake->reset_history[fake->reset_count++] = asserted;
}

static bool fake_write(
    void *context,
    const uint8_t *data,
    size_t size)
{
    fake_transport_t *fake = context;

    if (fake->write_call_count >= FAKE_MAX_WRITE_CALLS ||
        size > FAKE_MAX_WRITE_SIZE ||
        fake->written_count + size > FAKE_MAX_WRITTEN_BYTES)
    {
        fake->overflowed = true;
        return false;
    }

    fake->write_sizes[fake->write_call_count] = size;

    for (size_t i = 0; i < size; ++i)
    {
        fake->write_calls[fake->write_call_count][i] = data[i];
        fake->written_bytes[fake->written_count++] = data[i];
    }

    ++fake->write_call_count;

    return true;
}

static void fake_delay_ms(void *context, uint32_t milliseconds)
{
    fake_transport_t *fake = context;

    if (fake->delay_count >= FAKE_MAX_HISTORY)
    {
        fake->overflowed = true;
        return;
    }

    fake->delays_ms[fake->delay_count++] = milliseconds;
}

void fake_transport_init(fake_transport_t *fake)
{
    memset(fake, 0, sizeof(*fake));

    fake->transport.context = fake;
    fake->transport.set_sce = fake_set_sce;
    fake->transport.set_dc = fake_set_dc;
    fake->transport.set_reset = fake_set_reset;
    fake->transport.write = fake_write;
    fake->transport.delay_ms = fake_delay_ms;
}