#ifndef APP_H
#define APP_H

#include <stdbool.h>

#include "camera3d.h"
#include "projection.h"
#include "scene3d.h"
#include "camera_controller.h"
#include "car_modifier.h"
#include "input_types.h"

#define APP_CAR_COUNT 7U

typedef struct
{
    float render_accumulator_seconds;

    scene3d_t scene;
    camera3d_t camera;
    camera_controller_t camera_controller;

    /* Shared environment and stable per-car state referenced by scene modifiers. */
    car_modifier_environment_t car_environment;
    car_modifier_context_t car_contexts[APP_CAR_COUNT];
    projection_perspective_t projection;
} app_t;

bool app_init(app_t *app);

bool app_run(
    app_t *app,
    float dt_seconds,
    const input_snapshot_t *input);

#endif /* APP_H */