#ifndef PROJECTION_H
#define PROJECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "Modules/engine3d/geometry3d.h"

/** Parameters for orthographic projection onto the display plane. */
typedef struct
{
    float scale; /**< Screen units per world-space unit. */
    float center_x; /**< Horizontal screen coordinate of the projection center. */
    float center_y; /**< Vertical screen coordinate of the projection center. */
} projection_orthographic_t;

/** Parameters for perspective projection in camera coordinates. */
typedef struct
{
    float focal_length; /**< Projection focal length in screen units. */
    float near_plane; /**< Minimum positive camera-space depth. */
    float center_x; /**< Horizontal screen coordinate of the projection center. */
    float center_y; /**< Vertical screen coordinate of the projection center. */
} projection_perspective_t;

/** Project a vertex orthographically to integer screen coordinates.
 * @param projection Orthographic parameters.
 * @param vertex Vertex in projection coordinates.
 * @param screen_x Receives horizontal screen coordinate.
 * @param screen_y Receives vertical screen coordinate.
 * @return true on success; false for invalid pointers or non-finite input.
 */
bool projection_orthographic_project(
    const projection_orthographic_t *projection,
    const geometry3d_vertex_t *vertex,
    int16_t *screen_x,
    int16_t *screen_y);

/** Perspective-project a vertex to integer screen coordinates.
 * @param projection Perspective parameters.
 * @param vertex Vertex in camera coordinates; positive Z is in front of camera.
 * @param screen_x Receives horizontal screen coordinate.
 * @param screen_y Receives vertical screen coordinate.
 * @return true if the vertex is projectable and in front of the near plane.
 */
bool projection_perspective_project(
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *vertex,
    int16_t *screen_x,
    int16_t *screen_y);

/** Perspective-project a vertex while preserving fractional screen coordinates.
 * @param projection Perspective parameters.
 * @param vertex Vertex in camera coordinates; positive Z is in front of camera.
 * @param screen_x Receives horizontal screen coordinate.
 * @param screen_y Receives vertical screen coordinate.
 * @return true if the vertex is projectable and in front of the near plane.
 */
bool projection_perspective_project_float(
    const projection_perspective_t *projection,
    const geometry3d_vertex_t *vertex,
    float *screen_x,
    float *screen_y);

#endif /* PROJECTION_H */
