#ifndef APP_H
#define APP_H

#include <stdbool.h>

#include "camera3d.h"
#include "projection.h"
#include "scene3d.h"

typedef struct
{
    float render_accumulator_seconds;

    scene3d_t scene;
    camera3d_t camera;
    projection_perspective_t projection;
} app_t;

bool app_init(app_t *app);

bool app_run(app_t *app, float dt_seconds);

#endif /* APP_H */