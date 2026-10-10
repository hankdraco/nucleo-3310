#ifndef APP_H
#define APP_H

#include <stdbool.h>

#include "scene3d.h"
#include "camera3d.h"

typedef struct
{
    float render_accumulator_seconds;
    camera3d_t camera;
    scene3d_t scene;
} app_t;

bool app_init(app_t *app);

bool app_run(app_t *app, float dt_seconds);

#endif /* APP_H */