#ifndef APP_H
#define APP_H

#include <stdbool.h>

typedef struct
{
    float angle_rad;
    float angular_velocity_rad_s;

} app_line_animation_t;


typedef struct
{
    float render_accumulator_seconds;

    app_line_animation_t line;

} app_t;

bool app_init(app_t *app);

bool  app_run(app_t *app, float dt_seconds);

#endif /* APP_H */