#ifndef MODIFIER3D_TYPES_H
#define MODIFIER3D_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct geometry3d_instance;
typedef struct geometry3d_instance geometry3d_instance_t;

#define MODIFIER3D_MAX_PER_INSTANCE 9U /**< Maximum number of modifiers attached to one instance. */

/** Transform component affected by a modifier. */
typedef enum
{
    MODIFIER3D_TARGET_POSITION_X = 0, /**< Local translation along X. */
    MODIFIER3D_TARGET_POSITION_Y, /**< Local translation along Y. */
    MODIFIER3D_TARGET_POSITION_Z, /**< Local translation along Z. */

    MODIFIER3D_TARGET_ROTATION_X, /**< Rotation around X. */
    MODIFIER3D_TARGET_ROTATION_Y, /**< Rotation around Y. */
    MODIFIER3D_TARGET_ROTATION_Z, /**< Rotation around Z. */

    MODIFIER3D_TARGET_SCALE_X, /**< Scale along X. */
    MODIFIER3D_TARGET_SCALE_Y, /**< Scale along Y. */
    MODIFIER3D_TARGET_SCALE_Z, /**< Scale along Z. */

    /* Custom behaviors may operate on the complete instance transform. */
    MODIFIER3D_TARGET_ALL, /**< Complete transform, for custom callbacks. */

    MODIFIER3D_TARGET_COUNT /**< Number of target values; not a valid target. */
} modifier3d_target_t;

/** Motion rule used by a modifier. */
typedef enum
{
    /*
     * Move linearly and wrap at a range boundary.
     * The sign of speed determines the travel direction.
     */
    MODIFIER3D_BEHAVIOR_LINEAR_FORWARD = 0, /**< Linear motion that wraps at range boundaries. */

    /*
     * Move linearly and reverse direction at range boundaries.
     * The sign of speed determines the initial direction.
     */
    MODIFIER3D_BEHAVIOR_LINEAR_BOUNCE, /**< Linear motion that reverses at range boundaries. */

    /*
     * Animate between min_value and max_value using:
     *
     * min_value + (max_value - min_value) * abs(sin(phase))
     */
    MODIFIER3D_BEHAVIOR_ABS_SIN, /**< Oscillate between bounds using the absolute sine of phase. */

    /* Invoke the callback supplied when the modifier was created. */
    MODIFIER3D_BEHAVIOR_CUSTOM /**< Delegate updates to a caller-supplied callback. */
} modifier3d_behavior_t;

/** Custom modifier update callback.
 * @param instance Instance whose transform may be modified.
 * @param dt_seconds Elapsed simulation time.
 * @param context Caller-owned callback context.
 * @return true on success; false to report an update failure.
 */
typedef bool (*modifier3d_custom_fn)(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context);

/** Optional first-phase callback, used for sensing before agent movement.
 * @param instance Instance being updated.
 * @param dt_seconds Elapsed simulation time.
 * @param context Caller-owned modifier context.
 * @return true on success; false to report an update error.
 */
typedef bool (*modifier3d_custom_pre_update_fn)(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context);

/** Caller-supplied configuration used to initialize a modifier. */
typedef struct
{
    modifier3d_target_t target; /**< Transform component or complete instance targeted. */
    modifier3d_behavior_t behavior; /**< Motion behavior to apply. */

    float min_value; /**< Lower bound of the behavior's range. */
    float max_value; /**< Upper bound of the behavior's range. */

    /*
     * LINEAR_FORWARD / LINEAR_BOUNCE:
     * signed parameter units per second.
     *
     * ABS_SIN:
     * signed phase velocity in radians per second.
     */
    float speed; /**< Signed parameter or phase velocity, according to behavior. */

    /* Required only for CUSTOM behavior; context is caller-owned. */
    modifier3d_custom_fn custom_fn; /**< Required callback for CUSTOM behavior. */
    modifier3d_custom_pre_update_fn custom_pre_update_fn; /**< Optional sensing callback run before updates. */
    void *context; /**< Caller-owned state passed to both callbacks. */
} modifier3d_config_t;

/** Runtime modifier configuration and mutable animation state. */
typedef struct
{
    modifier3d_config_t config; /**< Immutable behavior configuration. */

    float phase_rad; /**< Runtime phase for periodic behaviors. */
    int8_t direction; /**< Current direction for bouncing behavior. */
} modifier3d_t;

/** Fixed-capacity collection of modifiers owned by one instance. */
typedef struct
{
    modifier3d_t items[MODIFIER3D_MAX_PER_INSTANCE]; /**< Fixed-capacity modifier storage. */
    size_t count; /**< Number of initialized items. */
} modifier3d_list_t;

#endif /* MODIFIER3D_TYPES_H */
