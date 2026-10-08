#ifndef DISPLAY_PCD8544_H
#define DISPLAY_PCD8544_H

#include <stdbool.h>

#include "display.h"
#include "pcd8544.h"

bool display_pcd8544_flush(
    const display_t *display,
    pcd8544_t *lcd
);

#endif /* DISPLAY_PCD8544_H */