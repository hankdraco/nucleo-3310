
#ifndef PROJECTION_H
#define PROJECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "geometry3d.h"

typedef struct
{
    float scale;
    float center_x;
    float center_y;
} projection_orthographic_t;

typedef struct
{
    float focal_length;
    float camera_distance;
    float near_plane;
    float center_x;
    float center_y;
} projection_perspective_t;

bool projection_orthographic_project(
    const projection_orthographic_t *projection,
    const geometry3d_vertex_t *vertex,
    int16_t *screen_x,
    int16_t *screen_y);

bool projection_perspective_project(
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *vertex,
    int16_t *screen_x,
    int16_t *screen_y);

bool projection_perspective_project_float(
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *vertex,
    float *screen_x,
    float *screen_y);

#endif