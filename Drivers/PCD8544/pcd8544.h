#ifndef PCD8544_H
#define PCD8544_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "Drivers/PCD8544/pcd8544_transport.h"

#define PCD8544_WIDTH        84U /**< LCD width in pixels. */
#define PCD8544_HEIGHT       48U /**< LCD height in pixels. */
#define PCD8544_BANKS        6U /**< Number of 8-pixel-high controller banks. */
#define PCD8544_BYTES_PER_BANK 84U /**< Bytes in one horizontal bank. */

/** Driver state for a PCD8544 LCD and its transport. */
typedef struct
{
    const pcd8544_transport_t *transport; /**< Hardware transport callbacks. */

} pcd8544_t;

/** Initialize the LCD and send its startup command sequence.
 * @param lcd Driver state to initialize.
 * @param transport Configured hardware transport.
 * @return true on success; false for invalid callbacks or transport failure.
 */
bool pcd8544_init(
    pcd8544_t *lcd,
    const pcd8544_transport_t *transport
);

/** Write a sequence of display data bytes.
 * @param lcd Initialized driver.
 * @param data Data bytes to send.
 * @param size Number of bytes.
 * @return true on success; false if arguments are invalid or transport fails.
 */
bool pcd8544_write_data(
    pcd8544_t *lcd,
    const uint8_t *data,
    size_t size
);

/** Set the LCD RAM cursor to a column and 8-row bank.
 * @param lcd Initialized driver.
 * @param x Horizontal pixel column.
 * @param bank Vertical controller bank index.
 * @return true on success; false if coordinates are out of range or transport fails.
 */
bool pcd8544_set_position(
    pcd8544_t *lcd,
    uint8_t x,
    uint8_t bank
);

/** Write a built-in diagnostic pattern to the display.
 * @param lcd Initialized driver.
 */
void pcd8544_test_pattern(pcd8544_t *lcd);

#endif /* PCD8544_H */
