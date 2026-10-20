#ifndef GEOMETRY3D_H
#define GEOMETRY3D_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Modules/engine3d/matrix4.h"
#include "Modules/engine3d/modifier3d_types.h"

/** A point in a model's local coordinate system. */
typedef struct
{
    float x; /**< X coordinate. */
    float y; /**< Y coordinate. */
    float z; /**< Z coordinate. */
} geometry3d_vertex_t;

/** Three-dimensional vector in world or local coordinates. */
typedef struct
{
    float x; /**< X component. */
    float y; /**< Y component. */
    float z; /**< Z component. */
} geometry3d_vector3_t;

/** Index pair describing one wireframe edge in a model. */
typedef struct
{
    uint16_t start; /**< Index of the first vertex. */
    uint16_t end; /**< Index of the second vertex. */
} geometry3d_edge_t;

/** Three vertex indices describing one triangle. */
typedef struct
{
    uint16_t a; /**< First vertex index. */
    uint16_t b; /**< Second vertex index. */
    uint16_t c; /**< Third vertex index. */
} geometry3d_triangle_t;

/**
 * Immutable geometry description.
 * Does not contain instance-specific transformations.
 */
typedef struct
{
    const geometry3d_vertex_t *vertices; /**< Immutable model-space vertices. */
    size_t vertex_count; /**< Number of entries in vertices. */

    const geometry3d_edge_t *edges; /**< Optional wireframe edges. */
    size_t edge_count; /**< Number of entries in edges. */

    const geometry3d_triangle_t *triangles; /**< Optional filled-surface triangles. */
    size_t triangle_count; /**< Number of entries in triangles. */
} geometry3d_model_t;

/** Position, Euler rotation and scale of a model instance. */
typedef struct
{
    geometry3d_vector3_t position; /**< Translation in parent coordinates. */
    geometry3d_vector3_t rotation_rad; /**< Euler angles in radians. */
    geometry3d_vector3_t scale; /**< Per-axis scale factors. */
} geometry3d_transform_t;

/** Renderable model instance with a local transform and modifier list. */
struct geometry3d_instance
{
    const geometry3d_model_t *model; /**< Shared immutable mesh used by this instance. */
    geometry3d_transform_t transform; /**< Mutable local transform. */

    modifier3d_list_t modifiers; /**< Modifiers that update the local transform. */
};

/** Validate model pointers, counts and all referenced vertex indices.
 * @param model Model to validate.
 * @return true when the model data is internally consistent.
 */
bool geometry3d_model_is_valid(
    const geometry3d_model_t *model);

/** Transform one vertex by a 4x4 matrix.
 * @param transform Matrix applied to the input vertex.
 * @param vertex Input vertex.
 * @param result Receives the transformed vertex.
 * @return true on success; false for invalid arguments or non-finite output.
 */
bool geometry3d_transform_vertex(
    const matrix4_t *transform,
    const geometry3d_vertex_t *vertex,
    geometry3d_vertex_t *result);

/** Build a local-to-parent matrix in the order T * Rz * Rx * Ry * S.
 * @param transform Transform parameters to convert.
 * @param result Receives the resulting matrix.
 * @return true on success; false for invalid arguments or non-finite values.
 */
bool geometry3d_build_transform_matrix(
    const geometry3d_transform_t *transform,
    matrix4_t *result);

#endif /* GEOMETRY3D_H */
