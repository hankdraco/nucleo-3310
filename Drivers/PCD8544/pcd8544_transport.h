#ifndef PCD8544_TRANSPORT_H
#define PCD8544_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Hardware-independent callbacks used by the PCD8544 driver.
 * Each callback receives the caller-provided context pointer.
 */
typedef struct
{
    void *context; /**< Opaque state passed to each transport callback. */

    void (*set_sce)(void *context, bool active); /**< Control chip enable; active is true while selected. */
    void (*set_dc)(void *context, bool data); /**< Select data mode when true, command mode otherwise. */
    void (*set_reset)(void *context, bool asserted); /**< Assert or release the hardware reset line. */

    bool (*write)(void *context, const uint8_t *data, size_t size); /**< Write bytes over the display bus. */

    void (*delay_ms)(void *context, uint32_t milliseconds); /**< Wait for a duration in milliseconds. */

} pcd8544_transport_t;

/** Initialize the STM32 implementation of the display transport.
 * @param transport Transport structure to fill with STM32 callbacks.
 */
void pcd8544_transport_stm32_init(pcd8544_transport_t *transport);

#endif /* PCD8544_TRANSPORT_H */
