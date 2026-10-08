#ifndef APP_H
#define APP_H

#include <stdbool.h>

typedef struct
{
    /* Application state will go here. */

} app_t;

bool app_init(app_t *app);

void app_run(app_t *app);

#endif /* APP_H */