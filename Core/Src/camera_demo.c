#include "camera_demo.h"

#include <math.h>
#include <stddef.h>

typedef struct
{
    geometry3d_vector3_t position;
    geometry3d_vector3_t position_control_in;
    geometry3d_vector3_t position_control_out;
    geometry3d_vector3_t look_target;
    geometry3d_vector3_t look_control_in;
    geometry3d_vector3_t look_control_out;
    float duration_seconds;
} camera_demo_keyframe_t;

/*
 * Camera path through the city, stored in flash:
 * road-level south-to-north, a high city reveal, a dive to the east-west
 * street, a road-level pass, then a high return to the starting point.
 * Each keyframe stores its incoming/outgoing Bezier handles for both the
 * camera position and the look target.
 */
static const camera_demo_keyframe_t camera_demo_path[] =
{
    {
        .position = { .x = 0.0f, .y = 4.0f, .z = 145.0f },
        .position_control_in = { .x = 0.0f, .y = 4.0f, .z = 165.0f },
        .position_control_out = { .x = 0.0f, .y = 4.0f, .z = 125.0f },
        .look_target = { .x = 0.0f, .y = 3.0f, .z = 115.0f },
        .look_control_in = { .x = 0.0f, .y = 3.0f, .z = 135.0f },
        .look_control_out = { .x = 0.0f, .y = 3.0f, .z = 75.0f },
        .duration_seconds = 14.0f
    },
    {
        .position = { .x = 0.0f, .y = 4.0f, .z = -145.0f },
        .position_control_in = { .x = 0.0f, .y = 4.0f, .z = -125.0f },
        .position_control_out = { .x = 0.0f, .y = 4.0f, .z = -165.0f },
        .look_target = { .x = 0.0f, .y = 3.0f, .z = -175.0f },
        .look_control_in = { .x = 0.0f, .y = 3.0f, .z = -135.0f },
        .look_control_out = { .x = 0.0f, .y = 3.0f, .z = -215.0f },
        .duration_seconds = 6.0f
    },
    {
        .position = { .x = 0.0f, .y = 95.0f, .z = -90.0f },
        .position_control_in = { .x = -40.0f, .y = 95.0f, .z = -90.0f },
        .position_control_out = { .x = 40.0f, .y = 95.0f, .z = -90.0f },
        .look_target = { .x = 0.0f, .y = 2.0f, .z = 0.0f },
        .look_control_in = { .x = -40.0f, .y = 4.0f, .z = 0.0f },
        .look_control_out = { .x = 40.0f, .y = 4.0f, .z = 0.0f },
        .duration_seconds = 9.0f
    },
    {
        .position = { .x = 130.0f, .y = 85.0f, .z = 5.0f },
        .position_control_in = { .x = 90.0f, .y = 85.0f, .z = 5.0f },
        .position_control_out = { .x = 170.0f, .y = 85.0f, .z = 5.0f },
        .look_target = { .x = 40.0f, .y = 8.0f, .z = 0.0f },
        .look_control_in = { .x = 0.0f, .y = 8.0f, .z = 0.0f },
        .look_control_out = { .x = 80.0f, .y = 8.0f, .z = 0.0f },
        .duration_seconds = 6.0f
    },
    {
        .position = { .x = 145.0f, .y = 4.0f, .z = 0.0f },
        .position_control_in = { .x = 185.0f, .y = 4.0f, .z = 0.0f },
        .position_control_out = { .x = 105.0f, .y = 4.0f, .z = 0.0f },
        .look_target = { .x = 110.0f, .y = 3.0f, .z = 0.0f },
        .look_control_in = { .x = 150.0f, .y = 3.0f, .z = 0.0f },
        .look_control_out = { .x = 70.0f, .y = 3.0f, .z = 0.0f },
        .duration_seconds = 14.0f
    },
    {
        .position = { .x = -145.0f, .y = 4.0f, .z = 0.0f },
        .position_control_in = { .x = -105.0f, .y = 4.0f, .z = 0.0f },
        .position_control_out = { .x = -185.0f, .y = 4.0f, .z = 0.0f },
        .look_target = { .x = -180.0f, .y = 3.0f, .z = 0.0f },
        .look_control_in = { .x = -140.0f, .y = 3.0f, .z = 0.0f },
        .look_control_out = { .x = -220.0f, .y = 3.0f, .z = 0.0f },
        .duration_seconds = 6.0f
    },
    {
        .position = { .x = -155.0f, .y = 90.0f, .z = 95.0f },
        .position_control_in = { .x = -155.0f, .y = 90.0f, .z = 55.0f },
        .position_control_out = { .x = -155.0f, .y = 90.0f, .z = 135.0f },
        .look_target = { .x = 0.0f, .y = 4.0f, .z = 0.0f },
        .look_control_in = { .x = -40.0f, .y = 4.0f, .z = 0.0f },
        .look_control_out = { .x = 40.0f, .y = 4.0f, .z = 0.0f },
        .duration_seconds = 9.0f
    },
    {
        .position = { .x = 0.0f, .y = 50.0f, .z = 190.0f },
        .position_control_in = { .x = 0.0f, .y = 50.0f, .z = 230.0f },
        .position_control_out = { .x = 0.0f, .y = 50.0f, .z = 150.0f },
        .look_target = { .x = 0.0f, .y = 3.0f, .z = 200.0f },
        .look_control_in = { .x = 0.0f, .y = 3.0f, .z = 240.0f },
        .look_control_out = { .x = 0.0f, .y = 3.0f, .z = 160.0f },
        .duration_seconds = 6.0f
    }
};

#define CAMERA_DEMO_KEYFRAME_COUNT \
    (sizeof(camera_demo_path) / sizeof(camera_demo_path[0]))

static float camera_demo_total_duration(void)
{
    float duration = 0.0f;
    for (size_t i = 0U; i < CAMERA_DEMO_KEYFRAME_COUNT; ++i)
    {
        duration += camera_demo_path[i].duration_seconds;
    }
    return duration;
}

static geometry3d_vector3_t camera_demo_bezier(
    const geometry3d_vector3_t *p0,
    const geometry3d_vector3_t *p1,
    const geometry3d_vector3_t *p2,
    const geometry3d_vector3_t *p3,
    float t)
{
    const float one_minus_t = 1.0f - t;
    const float a = one_minus_t * one_minus_t * one_minus_t;
    const float b = 3.0f * one_minus_t * one_minus_t * t;
    const float c = 3.0f * one_minus_t * t * t;
    const float d = t * t * t;

    return (geometry3d_vector3_t){
        .x = a * p0->x + b * p1->x + c * p2->x + d * p3->x,
        .y = a * p0->y + b * p1->y + c * p2->y + d * p3->y,
        .z = a * p0->z + b * p1->z + c * p2->z + d * p3->z
    };
}

static bool camera_demo_evaluate(
    float elapsed_seconds,
    camera3d_t *camera)
{
    float segment_start_seconds = 0.0f;
    size_t segment_index = CAMERA_DEMO_KEYFRAME_COUNT - 1U;

    for (size_t i = 0U; i < CAMERA_DEMO_KEYFRAME_COUNT; ++i)
    {
        const float segment_end_seconds =
            segment_start_seconds + camera_demo_path[i].duration_seconds;
        if (elapsed_seconds < segment_end_seconds)
        {
            segment_index = i;
            break;
        }
        segment_start_seconds = segment_end_seconds;
    }

    const size_t next_index =
        (segment_index + 1U) % CAMERA_DEMO_KEYFRAME_COUNT;
    const camera_demo_keyframe_t *start = &camera_demo_path[segment_index];
    const camera_demo_keyframe_t *end = &camera_demo_path[next_index];
    const float t = (elapsed_seconds - segment_start_seconds) /
        start->duration_seconds;

    const geometry3d_vector3_t position = camera_demo_bezier(
        &start->position,
        &start->position_control_out,
        &end->position_control_in,
        &end->position,
        t);
    const geometry3d_vector3_t target = camera_demo_bezier(
        &start->look_target,
        &start->look_control_out,
        &end->look_control_in,
        &end->look_target,
        t);

    if (!isfinite(position.x) || !isfinite(position.y) ||
        !isfinite(position.z))
    {
        return false;
    }

    const geometry3d_vector3_t previous_position = camera->position;
    camera->position = position;
    if (!camera3d_set_target(camera, &target))
    {
        camera->position = previous_position;
        return false;
    }

    return true;
}

bool camera_demo_init(camera_demo_t *demo, camera3d_t *camera)
{
    if (demo == NULL || camera == NULL)
    {
        return false;
    }

    demo->elapsed_seconds = 0.0f;
    demo->initialized = true;
    if (!camera_demo_evaluate(0.0f, camera))
    {
        demo->initialized = false;
        return false;
    }

    return true;
}

bool camera_demo_update(
    camera_demo_t *demo,
    camera3d_t *camera,
    float dt_seconds)
{
    if (demo == NULL || camera == NULL || !demo->initialized ||
        !isfinite(dt_seconds) || dt_seconds < 0.0f)
    {
        return false;
    }

    const float total_duration = camera_demo_total_duration();
    const float next_elapsed = fmodf(
        demo->elapsed_seconds + dt_seconds,
        total_duration);
    if (!isfinite(next_elapsed) || next_elapsed < 0.0f ||
        !camera_demo_evaluate(next_elapsed, camera))
    {
        return false;
    }

    demo->elapsed_seconds = next_elapsed;
    return true;
}

#undef CAMERA_DEMO_KEYFRAME_COUNT
