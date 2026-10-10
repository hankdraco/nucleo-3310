#include "city_models.h"

static const geometry3d_vertex_t plane_vertices[] =
{
    { -1.0f, 0.0f, -1.0f },
    {  1.0f, 0.0f, -1.0f },
    {  1.0f, 0.0f,  1.0f },
    { -1.0f, 0.0f,  1.0f }
};

/*
 * Only the perimeter is drawn.
 * The diagonal used by the triangle mesh is not an edge.
 */
static const geometry3d_edge_t plane_edges[] =
{
    { 0U, 1U },
    { 1U, 2U },
    { 2U, 3U },
    { 3U, 0U }
};

/*
 * Two triangles cover the rectangular surface for depth testing.
 */
static const geometry3d_triangle_t plane_triangles[] =
{
    { 0U, 3U, 2U },
    { 0U, 2U, 1U }
};

const geometry3d_model_t geometry3d_plane_model =
{
    .vertices = plane_vertices,
    .vertex_count =
        sizeof(plane_vertices) / sizeof(plane_vertices[0]),

    .edges = plane_edges,
    .edge_count =
        sizeof(plane_edges) / sizeof(plane_edges[0]),

    .triangles = plane_triangles,
    .triangle_count =
        sizeof(plane_triangles) / sizeof(plane_triangles[0])
};