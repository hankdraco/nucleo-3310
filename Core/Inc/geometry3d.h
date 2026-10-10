#ifndef GEOMETRY3D_H
#define GEOMETRY3D_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "matrix4.h"
#include "modifier3d_types.h"

typedef struct
{
    float x;
    float y;
    float z;
} geometry3d_vertex_t;

typedef struct
{
    float x;
    float y;
    float z;
} geometry3d_vector3_t;

typedef struct
{
    uint16_t start;
    uint16_t end;
} geometry3d_edge_t;

typedef struct
{
    uint16_t a;
    uint16_t b;
    uint16_t c;
} geometry3d_triangle_t;

/*
 * Immutable geometry description.
 * Does not contain instance-specific transformations.
 */
typedef struct
{
    const geometry3d_vertex_t *vertices;
    size_t vertex_count;

    const geometry3d_edge_t *edges;
    size_t edge_count;

    const geometry3d_triangle_t *triangles;
    size_t triangle_count;
} geometry3d_model_t;

typedef struct
{
    geometry3d_vector3_t position;
    geometry3d_vector3_t rotation_rad;
    geometry3d_vector3_t scale;
} geometry3d_transform_t;

typedef struct
{
    const geometry3d_model_t *model;
    geometry3d_transform_t transform;

    modifier3d_list_t modifiers;
} geometry3d_instance_t;

bool geometry3d_model_is_valid(
    const geometry3d_model_t *model);

bool geometry3d_transform_vertex(
    const matrix4_t *transform,
    const geometry3d_vertex_t *vertex,
    geometry3d_vertex_t *result);

/*
 * Build a local-to-parent transformation matrix from a transform.
 *
 * Matrix order:
 * T * Rz * Rx * Ry * S
 */
bool geometry3d_build_transform_matrix(
    const geometry3d_transform_t *transform,
    matrix4_t *result);

#endif /* GEOMETRY3D_H */