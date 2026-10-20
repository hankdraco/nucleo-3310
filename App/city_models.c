#include "App/city_models.h"

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

/*
 * A unit-footprint plus. The road width is 8% of the half extent, matching
 * the demo city dimensions (street half-width 1.6 / city half-extent 20).
 */
#define CITY_MODELS_PLUS_HALF_WIDTH (0.08f)

static const geometry3d_vertex_t plus_road_vertices[] =
{
    { -CITY_MODELS_PLUS_HALF_WIDTH, 0.0f, -1.0f },
    {  CITY_MODELS_PLUS_HALF_WIDTH, 0.0f, -1.0f },
    {  CITY_MODELS_PLUS_HALF_WIDTH, 0.0f, -CITY_MODELS_PLUS_HALF_WIDTH },
    {  1.0f, 0.0f, -CITY_MODELS_PLUS_HALF_WIDTH },
    {  1.0f, 0.0f,  CITY_MODELS_PLUS_HALF_WIDTH },
    {  CITY_MODELS_PLUS_HALF_WIDTH, 0.0f,  CITY_MODELS_PLUS_HALF_WIDTH },
    {  CITY_MODELS_PLUS_HALF_WIDTH, 0.0f,  1.0f },
    { -CITY_MODELS_PLUS_HALF_WIDTH, 0.0f,  1.0f },
    { -CITY_MODELS_PLUS_HALF_WIDTH, 0.0f,  CITY_MODELS_PLUS_HALF_WIDTH },
    { -1.0f, 0.0f,  CITY_MODELS_PLUS_HALF_WIDTH },
    { -1.0f, 0.0f, -CITY_MODELS_PLUS_HALF_WIDTH },
    { -CITY_MODELS_PLUS_HALF_WIDTH, 0.0f, -CITY_MODELS_PLUS_HALF_WIDTH }
};

/* Only the external plus outline is rendered; mesh diagonals stay invisible. */
static const geometry3d_edge_t plus_road_edges[] =
{
    { 0U, 1U }, { 1U, 2U }, { 2U, 3U }, { 3U, 4U },
    { 4U, 5U }, { 5U, 6U }, { 6U, 7U }, { 7U, 8U },
    { 8U, 9U }, { 9U, 10U }, { 10U, 11U }, { 11U, 0U }
};

/* Five non-overlapping rectangles (four arms and the centre), two triangles each. */
static const geometry3d_triangle_t plus_road_triangles[] =
{
    /* North arm. */
    { 0U, 2U, 11U }, { 0U, 1U, 2U },
    /* Centre. */
    { 11U, 8U, 5U }, { 11U, 5U, 2U },
    /* East arm. */
    { 2U, 5U, 4U }, { 2U, 3U, 5U },
    /* South arm. */
    { 8U, 7U, 6U }, { 8U, 6U, 5U },
    /* West arm. */
    { 10U, 9U, 8U }, { 10U, 8U, 11U }
};

const geometry3d_model_t geometry3d_plus_road_model =
{
    .vertices = plus_road_vertices,
    .vertex_count =
        sizeof(plus_road_vertices) / sizeof(plus_road_vertices[0]),
    .edges = plus_road_edges,
    .edge_count = sizeof(plus_road_edges) / sizeof(plus_road_edges[0]),
    .triangles = plus_road_triangles,
    .triangle_count =
        sizeof(plus_road_triangles) / sizeof(plus_road_triangles[0])
};

#undef CITY_MODELS_PLUS_HALF_WIDTH
