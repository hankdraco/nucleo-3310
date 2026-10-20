#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#define DISPLAY_WIDTH              84U /**< Display width in pixels. */
#define DISPLAY_HEIGHT             48U /**< Display height in pixels. */
#define DISPLAY_FRAMEBUFFER_SIZE   ((DISPLAY_WIDTH * DISPLAY_HEIGHT) / 8U) /**< Packed monochrome framebuffer bytes. */

#define DISPLAY_ORIENTATION_NORMAL  0x00U /**< No framebuffer mirroring. */
#define DISPLAY_ORIENTATION_FLIP_X  0x01U /**< Mirror framebuffer horizontally. */
#define DISPLAY_ORIENTATION_FLIP_Y  0x02U /**< Mirror framebuffer vertically. */

/** Configuration for a logical display canvas. */
typedef struct
{
    uint8_t orientation; /**< Combination of DISPLAY_ORIENTATION_* flags. */

} display_config_t;

/** Monochrome framebuffer and its coordinate orientation. */
typedef struct
{
    uint8_t framebuffer[DISPLAY_FRAMEBUFFER_SIZE]; /**< One bit per pixel, packed by rows. */
    display_config_t config; /**< Orientation applied to pixel coordinates. */

} display_t;

/** Initialize a display canvas and clear its framebuffer.
 * @param display Canvas storage to initialize.
 * @param config Optional orientation configuration; NULL selects normal orientation.
 */
void display_init(
    display_t *display,
    const display_config_t *config
);

/** Clear every framebuffer pixel.
 * @param display Canvas to clear.
 */
void display_clear(display_t *display);

/** Set one pixel, applying the configured orientation.
 * @param display Canvas to modify.
 * @param x Pixel column.
 * @param y Pixel row.
 * @param on True to set the pixel; false to clear it.
 * @return true on success; false for invalid arguments or coordinates.
 */
bool display_set_pixel(
    display_t *display,
    uint8_t x,
    uint8_t y,
    bool on
);

/** Read one pixel, applying the configured orientation.
 * @param display Canvas to read.
 * @param x Pixel column.
 * @param y Pixel row.
 * @param on Receives true when the pixel is set.
 * @return true on success; false for invalid arguments or coordinates.
 */
bool display_get_pixel(
    const display_t *display,
    uint8_t x,
    uint8_t y,
    bool *on
);

/** Draw an integer line clipped to the display bounds.
 * @param display Canvas to modify.
 * @param x0 Starting column.
 * @param y0 Starting row.
 * @param x1 Ending column.
 * @param y1 Ending row.
 * @return true on success; false if display is NULL.
 */
bool display_draw_line(
    display_t *display,
    int16_t x0,
    int16_t y0,
    int16_t x1,
    int16_t y1
);

#endif /* DISPLAY_H */
