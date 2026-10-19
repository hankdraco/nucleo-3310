#include "scene3d.h"

#include <stddef.h>

#include "cube_model.h"
#include "modifier3d.h"
#include "renderer3d.h"
#include "test_common.h"

typedef struct
{
    int id;
    int *events;
    size_t *event_count;
    const geometry3d_instance_t *other_agent;
    float expected_other_x;
    bool verify_other_position;
    bool move_on_update;
} test_agent_phase_context_t;

static bool test_record_pre_update(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *opaque_context)
{
    test_agent_phase_context_t *context = opaque_context;
    if (instance == NULL || context == NULL || dt_seconds < 0.0f)
    {
        return false;
    }

    if (context->verify_other_position &&
        (context->other_agent == NULL ||
         context->other_agent->transform.position.x != context->expected_other_x))
    {
        return false;
    }

    context->events[(*context->event_count)++] = context->id;
    return true;
}

static bool test_record_route_update(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *opaque_context)
{
    test_agent_phase_context_t *context = opaque_context;
    if (instance == NULL || context == NULL || dt_seconds < 0.0f)
    {
        return false;
    }

    context->events[(*context->event_count)++] = context->id + 10;
    if (context->move_on_update)
    {
        instance->transform.position.x += dt_seconds;
    }
    return true;
}

static bool test_scene_update_runs_agent_phases_separately(void)
{
    int events[8] = {0};
    size_t event_count = 0U;
    test_agent_phase_context_t static_context =
    {
        .id = 20,
        .events = events,
        .event_count = &event_count
    };
    test_agent_phase_context_t first_context =
    {
        .id = 1,
        .events = events,
        .event_count = &event_count,
        .expected_other_x = 2.0f,
        .verify_other_position = true,
        .move_on_update = true
    };
    test_agent_phase_context_t second_context =
    {
        .id = 2,
        .events = events,
        .event_count = &event_count,
        .expected_other_x = 1.0f,
        .verify_other_position = true,
        .move_on_update = true
    };
    geometry3d_instance_t static_instance =
    {
        .model = &geometry3d_cube_model,
        .transform = { .scale = { .x = 1.0f, .y = 1.0f, .z = 1.0f } }
    };
    geometry3d_instance_t first_agent = static_instance;
    geometry3d_instance_t second_agent = static_instance;
    scene3d_t scene;

    static_instance.transform.position.x = 20.0f;
    first_agent.transform.position.x = 1.0f;
    second_agent.transform.position.x = 2.0f;

    const modifier3d_config_t static_modifier =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = test_record_route_update,
        .custom_pre_update_fn = test_record_pre_update,
        .context = &static_context
    };
    const modifier3d_config_t first_modifier =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = test_record_route_update,
        .custom_pre_update_fn = test_record_pre_update,
        .context = &first_context
    };
    const modifier3d_config_t second_modifier =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = test_record_route_update,
        .custom_pre_update_fn = test_record_pre_update,
        .context = &second_context
    };

    CHECK(modifier3d_add(&static_instance, &static_modifier));
    CHECK(modifier3d_add(&first_agent, &first_modifier));
    CHECK(modifier3d_add(&second_agent, &second_modifier));

    scene3d_init(&scene);
    CHECK(scene3d_add_instance(&scene, &static_instance));
    CHECK(scene3d_add_agent_instance(&scene, &first_agent));
    CHECK(scene3d_add_agent_instance(&scene, &second_agent));

    first_context.other_agent = &scene.agent_instances[1];
    second_context.other_agent = &scene.agent_instances[0];

    CHECK(scene3d_update(&scene, 0.5f));
    CHECK(event_count == 5U);
    CHECK(events[0] == 30); /* Static update happens once, without pre-update. */
    CHECK(events[1] == 1);
    CHECK(events[2] == 2);  /* All agent pre-updates precede any movement. */
    CHECK(events[3] == 11);
    CHECK(events[4] == 12);
    CHECK(scene.instances[0].transform.position.x == 20.0f);
    CHECK(scene.agent_instances[0].transform.position.x == 1.5f);
    CHECK(scene.agent_instances[1].transform.position.x == 2.5f);

    return true;
}

/* Rendering is outside this update-pipeline test. */
void depth_buffer_clear(depth_buffer_t *buffer)
{
    (void)buffer;
}

bool renderer3d_rasterize_instance(
    depth_buffer_t *depth_buffer,
    const geometry3d_instance_t *instance,
    const matrix4_t *world_matrix,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection)
{
    (void)depth_buffer;
    (void)instance;
    (void)world_matrix;
    (void)view_matrix;
    (void)projection;
    return true;
}

bool renderer3d_draw_instance(
    display_t *canvas,
    const depth_buffer_t *depth_buffer,
    const geometry3d_instance_t *instance,
    const matrix4_t *world_matrix,
    const matrix4_t *view_matrix,
    const projection_perspective_t *projection)
{
    (void)canvas;
    (void)depth_buffer;
    (void)instance;
    (void)world_matrix;
    (void)view_matrix;
    (void)projection;
    return true;
}

bool test_scene3d_agent_update_suite(void)
{
    CHECK(test_scene_update_runs_agent_phases_separately());
    return true;
}
