#ifndef PCD8544_TRANSPORT_H
#define PCD8544_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct
{
    void *context;

    void (*set_sce)(void *context, bool active);
    void (*set_dc)(void *context, bool data);
    void (*set_reset)(void *context, bool asserted);

    bool (*write)(void *context, const uint8_t *data, size_t size);

    void (*delay_ms)(void *context, uint32_t milliseconds);

} pcd8544_transport_t;

void pcd8544_transport_stm32_init(pcd8544_transport_t *transport);

#endif /* PCD8544_TRANSPORT_H */