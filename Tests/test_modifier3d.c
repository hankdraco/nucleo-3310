#include "modifier3d.h"

#include <math.h>

#include "test_common.h"

#define TEST_TOLERANCE 0.0001f

typedef struct
{
    float distance_per_second;
    unsigned int calls;
} test_custom_context_t;

static bool test_custom_move_instance(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context)
{
    if (instance == NULL || context == NULL ||
        !isfinite(dt_seconds) || dt_seconds < 0.0f)
    {
        return false;
    }

    test_custom_context_t *state = context;
    instance->transform.position.x += state->distance_per_second * dt_seconds;
    instance->transform.position.z += 2.0f * dt_seconds;
    instance->transform.rotation_rad.y += 0.5f * dt_seconds;
    ++state->calls;
    return true;
}

static bool test_custom_callback_fails(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context)
{
    (void)instance;
    (void)dt_seconds;
    (void)context;
    return false;
}

static bool test_custom_modifier_updates_full_transform(void)
{
    test_custom_context_t context =
    {
        .distance_per_second = 3.0f,
        .calls = 0U
    };
    geometry3d_instance_t instance = {0};
    instance.transform.scale.x = 1.0f;
    instance.transform.scale.y = 1.0f;
    instance.transform.scale.z = 1.0f;

    const modifier3d_config_t config =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = test_custom_move_instance,
        .context = &context
    };

    CHECK(modifier3d_add(&instance, &config));
    CHECK(instance.modifiers.count == 1U);
    CHECK(modifier3d_update_instance(&instance, 0.5f));
    CHECK(fabsf(instance.transform.position.x - 1.5f) <= TEST_TOLERANCE);
    CHECK(fabsf(instance.transform.position.z - 1.0f) <= TEST_TOLERANCE);
    CHECK(fabsf(instance.transform.rotation_rad.y - 0.25f) <= TEST_TOLERANCE);
    CHECK(context.calls == 1U);

    return true;
}

static bool test_custom_context_survives_instance_copy(void)
{
    test_custom_context_t context =
    {
        .distance_per_second = 4.0f,
        .calls = 0U
    };
    geometry3d_instance_t original = {0};
    geometry3d_instance_t scene_copy;

    const modifier3d_config_t config =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = test_custom_move_instance,
        .context = &context
    };

    CHECK(modifier3d_add(&original, &config));
    scene_copy = original;

    CHECK(scene_copy.modifiers.items[0].config.context == &context);
    CHECK(modifier3d_update_instance(&scene_copy, 0.25f));
    CHECK(fabsf(scene_copy.transform.position.x - 1.0f) <= TEST_TOLERANCE);
    CHECK(fabsf(original.transform.position.x) <= TEST_TOLERANCE);
    CHECK(context.calls == 1U);

    return true;
}

static bool test_custom_modifier_requires_callback_and_context(void)
{
    test_custom_context_t context = { .distance_per_second = 1.0f };
    geometry3d_instance_t instance = {0};
    modifier3d_config_t config =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = test_custom_move_instance,
        .context = &context
    };

    config.custom_fn = NULL;
    CHECK(!modifier3d_add(&instance, &config));

    config.custom_fn = test_custom_move_instance;
    config.context = NULL;
    CHECK(!modifier3d_add(&instance, &config));

    config.context = &context;
    config.target = MODIFIER3D_TARGET_POSITION_X;
    CHECK(!modifier3d_add(&instance, &config));

    return true;
}

static bool test_custom_callback_failure_is_reported(void)
{
    geometry3d_instance_t instance = {0};
    int context = 1;

    const modifier3d_config_t config =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = test_custom_callback_fails,
        .context = &context
    };

    CHECK(modifier3d_add(&instance, &config));
    CHECK(!modifier3d_update_instance(&instance, 0.1f));

    return true;
}

bool test_modifier3d_suite(void)
{
    CHECK(test_custom_modifier_updates_full_transform());
    CHECK(test_custom_context_survives_instance_copy());
    CHECK(test_custom_modifier_requires_callback_and_context());
    CHECK(test_custom_callback_failure_is_reported());

    return true;
}
