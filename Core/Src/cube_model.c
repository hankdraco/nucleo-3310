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

const geometry3d_model_t geometry3d_cube_model =
{
    .vertices = cube_vertices,
    .vertex_count =
        sizeof(cube_vertices) / sizeof(cube_vertices[0]),

    .edges = cube_edges,
    .edge_count =
        sizeof(cube_edges) / sizeof(cube_edges[0])
};