#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#define DISPLAY_WIDTH              84U
#define DISPLAY_HEIGHT             48U
#define DISPLAY_FRAMEBUFFER_SIZE   ((DISPLAY_WIDTH * DISPLAY_HEIGHT) / 8U)

#define DISPLAY_ORIENTATION_NORMAL  0x00U
#define DISPLAY_ORIENTATION_FLIP_X  0x01U
#define DISPLAY_ORIENTATION_FLIP_Y  0x02U

typedef struct
{
    uint8_t orientation;

} display_config_t;

typedef struct
{
    uint8_t framebuffer[DISPLAY_FRAMEBUFFER_SIZE];
    display_config_t config;

} display_t;

void display_init(
    display_t *display,
    const display_config_t *config
);

void display_clear(display_t *display);

bool display_set_pixel(
    display_t *display,
    uint8_t x,
    uint8_t y,
    bool on
);

bool display_get_pixel(
    const display_t *display,
    uint8_t x,
    uint8_t y,
    bool *on
);

#endif /* DISPLAY_H */