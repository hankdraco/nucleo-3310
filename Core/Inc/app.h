#ifndef APP_H
#define APP_H

#include <stdbool.h>

typedef struct
{
    float angle_x_rad;
    float angular_velocity_x_rad_s;

    float angle_y_rad;
    float angular_velocity_y_rad_s;

} app_cube_animation_t;

typedef struct
{
    float render_accumulator_seconds;

    app_cube_animation_t cube;

} app_t;

bool app_init(app_t *app);

bool  app_run(app_t *app, float dt_seconds);

#endif /* APP_H */