#ifndef APP_H
#define APP_H

#include <stdbool.h>

#include "scene3d.h"

typedef struct
{
    float angular_velocity_x_rad_s;
    float angular_velocity_y_rad_s;
    float angular_velocity_z_rad_s;
} app_instance_animation_t;

typedef struct
{
    float render_accumulator_seconds;

    scene3d_t scene;

    app_instance_animation_t animations[SCENE3D_MAX_INSTANCES];

} app_t;

bool app_init(app_t *app);

bool app_run(app_t *app, float dt_seconds);

#endif /* APP_H */