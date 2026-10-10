#include "cube_model.h"

static const geometry3d_vertex_t cube_vertices[] =
{
    { -1.0f, -1.0f, -1.0f },
    {  1.0f, -1.0f, -1.0f },
    {  1.0f,  1.0f, -1.0f },
    { -1.0f,  1.0f, -1.0f },

    { -1.0f, -1.0f,  1.0f },
    {  1.0f, -1.0f,  1.0f },
    {  1.0f,  1.0f,  1.0f },
    { -1.0f,  1.0f,  1.0f }
};

static const geometry3d_edge_t cube_edges[] =
{
    /* Face at z = -1 */
    { 0U, 1U },
    { 1U, 2U },
    { 2U, 3U },
    { 3U, 0U },

    /* Face at z = +1 */
    { 4U, 5U },
    { 5U, 6U },
    { 6U, 7U },
    { 7U, 4U },

    /* Connect both faces */
    { 0U, 4U },
    { 1U, 5U },
    { 2U, 6U },
    { 3U, 7U }
};

static const geometry3d_triangle_t cube_triangles[] =
{
    /* Face at z = -1, outward normal -Z */
    { 0U, 2U, 1U },
    { 0U, 3U, 2U },

    /* Face at z = +1, outward normal +Z */
    { 4U, 5U, 6U },
    { 4U, 6U, 7U },

    /* Face at x = +1, outward normal +X */
    { 1U, 2U, 6U },
    { 1U, 6U, 5U },

    /* Face at x = -1, outward normal -X */
    { 0U, 4U, 7U },
    { 0U, 7U, 3U },

    /* Face at y = +1, outward normal +Y */
    { 3U, 7U, 6U },
    { 3U, 6U, 2U },

    /* Face at y = -1, outward normal -Y */
    { 0U, 1U, 5U },
    { 0U, 5U, 4U }
};

const geometry3d_model_t geometry3d_cube_model =
{
    .vertices = cube_vertices,
    .vertex_count =
        sizeof(cube_vertices) / sizeof(cube_vertices[0]),

    .edges = cube_edges,
    .edge_count =
        sizeof(cube_edges) / sizeof(cube_edges[0]),

    .triangles = cube_triangles,
    .triangle_count =
        sizeof(cube_triangles) / sizeof(cube_triangles[0])
};