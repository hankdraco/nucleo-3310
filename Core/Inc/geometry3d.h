
#ifndef GEOMETRY3D_H
#define GEOMETRY3D_H

#include <stdint.h>

typedef struct
{
    float x;
    float y;
    float z;
} geometry3d_vertex_t;

#define GEOMETRY3D_CUBE_VERTEX_COUNT 8U
#define GEOMETRY3D_CUBE_EDGE_COUNT 12U

typedef struct
{
    uint8_t start;
    uint8_t end;
} geometry3d_edge_t;

typedef struct
{
    geometry3d_vertex_t vertices[GEOMETRY3D_CUBE_VERTEX_COUNT];
    geometry3d_edge_t edges[GEOMETRY3D_CUBE_EDGE_COUNT];
} geometry3d_cube_t;


geometry3d_vertex_t geometry3d_rotate_y(
    const geometry3d_vertex_t *vertex,
    float angle_rad);

geometry3d_vertex_t geometry3d_rotate_x(
    const geometry3d_vertex_t *vertex,
    float angle_rad);

#endif