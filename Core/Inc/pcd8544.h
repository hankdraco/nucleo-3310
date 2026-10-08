#ifndef PCD8544_H
#define PCD8544_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "pcd8544_transport.h"

#define PCD8544_WIDTH        84U
#define PCD8544_HEIGHT       48U
#define PCD8544_BANKS        6U
#define PCD8544_BYTES_PER_BANK 84U

typedef struct
{
    const pcd8544_transport_t *transport;

} pcd8544_t;

bool pcd8544_init(
    pcd8544_t *lcd,
    const pcd8544_transport_t *transport
);

bool pcd8544_write_data(
    pcd8544_t *lcd,
    const uint8_t *data,
    size_t size
);

void pcd8544_test_pattern(pcd8544_t *lcd);

#endif /* PCD8544_H */