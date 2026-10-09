#ifndef APP_H
#define APP_H

#include <stdbool.h>

#include "geometry3d.h"

typedef struct
{
    float angular_velocity_x_rad_s;
    float angular_velocity_y_rad_s;
    float angular_velocity_z_rad_s;
} app_cube_animation_t;

typedef struct
{
    float render_accumulator_seconds;

    geometry3d_instance_t cube;
    app_cube_animation_t cube_animation;

} app_t;

bool app_init(app_t *app);

bool app_run(app_t *app, float dt_seconds);

#endif /* APP_H */