#ifndef MODIFIER3D_TYPES_H
#define MODIFIER3D_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct geometry3d_instance;
typedef struct geometry3d_instance geometry3d_instance_t;

/*
 * One modifier per scalar transform parameter.
 */
#define MODIFIER3D_MAX_PER_INSTANCE 9U

typedef enum
{
    MODIFIER3D_TARGET_POSITION_X = 0,
    MODIFIER3D_TARGET_POSITION_Y,
    MODIFIER3D_TARGET_POSITION_Z,

    MODIFIER3D_TARGET_ROTATION_X,
    MODIFIER3D_TARGET_ROTATION_Y,
    MODIFIER3D_TARGET_ROTATION_Z,

    MODIFIER3D_TARGET_SCALE_X,
    MODIFIER3D_TARGET_SCALE_Y,
    MODIFIER3D_TARGET_SCALE_Z,

    /* Custom behaviors may operate on the complete instance transform. */
    MODIFIER3D_TARGET_ALL,

    MODIFIER3D_TARGET_COUNT
} modifier3d_target_t;

typedef enum
{
    /*
     * Move linearly and wrap at a range boundary.
     * The sign of speed determines the travel direction.
     */
    MODIFIER3D_BEHAVIOR_LINEAR_FORWARD = 0,

    /*
     * Move linearly and reverse direction at range boundaries.
     * The sign of speed determines the initial direction.
     */
    MODIFIER3D_BEHAVIOR_LINEAR_BOUNCE,

    /*
     * Animate between min_value and max_value using:
     *
     * min_value + (max_value - min_value) * abs(sin(phase))
     */
    MODIFIER3D_BEHAVIOR_ABS_SIN,

    /* Invoke the callback supplied when the modifier was created. */
    MODIFIER3D_BEHAVIOR_CUSTOM
} modifier3d_behavior_t;

typedef bool (*modifier3d_custom_fn)(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context);

/*
 * User-configurable part of a modifier.
 */
typedef struct
{
    modifier3d_target_t target;
    modifier3d_behavior_t behavior;

    float min_value;
    float max_value;

    /*
     * LINEAR_FORWARD / LINEAR_BOUNCE:
     * signed parameter units per second.
     *
     * ABS_SIN:
     * signed phase velocity in radians per second.
     */
    float speed;

    /* Required only for CUSTOM behavior; context is caller-owned. */
    modifier3d_custom_fn custom_fn;
    void *context;
} modifier3d_config_t;

/*
 * Runtime modifier.
 *
 * Configuration and mutable internal state are kept together.
 * Internal state is initialized when the modifier is added.
 */
typedef struct
{
    modifier3d_config_t config;

    float phase_rad;
    int8_t direction;
} modifier3d_t;

/*
 * Fixed-capacity modifier list owned by one instance.
 */
typedef struct
{
    modifier3d_t items[MODIFIER3D_MAX_PER_INSTANCE];
    size_t count;
} modifier3d_list_t;

#endif /* MODIFIER3D_TYPES_H */