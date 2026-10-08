#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#define DISPLAY_WIDTH              84U
#define DISPLAY_HEIGHT             48U
#define DISPLAY_FRAMEBUFFER_SIZE   ((DISPLAY_WIDTH * DISPLAY_HEIGHT) / 8U)

typedef struct
{
    uint8_t framebuffer[DISPLAY_FRAMEBUFFER_SIZE];

} display_t;

void display_init(display_t *display);

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