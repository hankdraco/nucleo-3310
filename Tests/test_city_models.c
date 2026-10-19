#include "city_models.h"

#include <math.h>

#include "test_common.h"

static bool test_plus_road_model_is_valid_and_covers_expected_area(void)
{
    const geometry3d_model_t *model = &geometry3d_plus_road_model;
    CHECK(geometry3d_model_is_valid(model));
    CHECK(model->vertex_count == 12U);
    CHECK(model->edge_count == 12U);
    CHECK(model->triangle_count == 10U);

    const float arm_half_width = model->vertices[2U].x;
    CHECK(arm_half_width > 0.0f && arm_half_width < 1.0f);

    float total_triangle_area = 0.0f;
    for (size_t i = 0U; i < model->triangle_count; ++i)
    {
        const geometry3d_triangle_t *triangle = &model->triangles[i];
        const geometry3d_vertex_t *a = &model->vertices[triangle->a];
        const geometry3d_vertex_t *b = &model->vertices[triangle->b];
        const geometry3d_vertex_t *c = &model->vertices[triangle->c];
        const float twice_area =
            (b->x - a->x) * (c->z - a->z) -
            (b->z - a->z) * (c->x - a->x);
        total_triangle_area += 0.5f * fabsf(twice_area);
    }

    /* Union of two full-width strips: 8w minus their 4w² overlap. */
    const float expected_area =
        8.0f * arm_half_width - 4.0f * arm_half_width * arm_half_width;
    CHECK(fabsf(total_triangle_area - expected_area) < 0.0001f);
    return true;
}

bool test_city_models_suite(void)
{
    CHECK(test_plus_road_model_is_valid_and_covers_expected_area());
    return true;
}
