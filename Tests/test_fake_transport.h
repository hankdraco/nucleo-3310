#ifndef TEST_FAKE_TRANSPORT_H
#define TEST_FAKE_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../Core/Inc/pcd8544_transport.h"

#define FAKE_MAX_WRITE_CALLS   32U
#define FAKE_MAX_WRITE_SIZE    84U
#define FAKE_MAX_WRITTEN_BYTES 1024U
#define FAKE_MAX_HISTORY       64U

typedef struct
{
    pcd8544_transport_t transport;

    uint8_t written_bytes[FAKE_MAX_WRITTEN_BYTES];
    size_t written_count;

    uint8_t write_calls[FAKE_MAX_WRITE_CALLS][FAKE_MAX_WRITE_SIZE];
    size_t write_sizes[FAKE_MAX_WRITE_CALLS];
    size_t write_call_count;

    bool dc_history[FAKE_MAX_HISTORY];
    size_t dc_count;

    bool sce_history[FAKE_MAX_HISTORY];
    size_t sce_count;

    bool reset_history[FAKE_MAX_HISTORY];
    size_t reset_count;

    uint32_t delays_ms[FAKE_MAX_HISTORY];
    size_t delay_count;

    bool overflowed;

} fake_transport_t;

void fake_transport_init(fake_transport_t *fake);

#endif /* TEST_FAKE_TRANSPORT_H */