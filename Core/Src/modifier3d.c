#include "modifier3d.h"

#include <math.h>
#include <stddef.h>

#define MODIFIER3D_PI (3.14159265358979323846f)

static float *modifier3d_get_target_pointer(
    geometry3d_transform_t *transform,
    modifier3d_target_t target)
{
    if (transform == NULL)
    {
        return NULL;
    }

    switch (target)
    {
        case MODIFIER3D_TARGET_POSITION_X:
            return &transform->position.x;

        case MODIFIER3D_TARGET_POSITION_Y:
            return &transform->position.y;

        case MODIFIER3D_TARGET_POSITION_Z:
            return &transform->position.z;

        case MODIFIER3D_TARGET_ROTATION_X:
            return &transform->rotation_rad.x;

        case MODIFIER3D_TARGET_ROTATION_Y:
            return &transform->rotation_rad.y;

        case MODIFIER3D_TARGET_ROTATION_Z:
            return &transform->rotation_rad.z;

        case MODIFIER3D_TARGET_SCALE_X:
            return &transform->scale.x;

        case MODIFIER3D_TARGET_SCALE_Y:
            return &transform->scale.y;

        case MODIFIER3D_TARGET_SCALE_Z:
            return &transform->scale.z;

        default:
            return NULL;
    }
}

static bool modifier3d_config_is_valid(
    const modifier3d_config_t *config)
{
    if (config == NULL ||
        (unsigned int)config->target >=
            (unsigned int)MODIFIER3D_TARGET_COUNT ||
        (unsigned int)config->behavior >
            (unsigned int)MODIFIER3D_BEHAVIOR_ABS_SIN)
    {
        return false;
    }

    if (!isfinite(config->min_value) ||
        !isfinite(config->max_value) ||
        !isfinite(config->speed) ||
        config->min_value >= config->max_value ||
        config->speed == 0.0f)
    {
        return false;
    }

    const float range =
        config->max_value - config->min_value;

    /*
     * Ensure the range and the bounce period can be represented
     * as finite floats.
     */
    if (!isfinite(range) ||
        range <= 0.0f ||
        !isfinite(2.0f * range))
    {
        return false;
    }

    return true;
}

bool modifier3d_add(
    geometry3d_instance_t *instance,
    const modifier3d_config_t *config)
{
    if (instance == NULL ||
        !modifier3d_config_is_valid(config))
    {
        return false;
    }

    modifier3d_list_t *list = &instance->modifiers;

    if (list->count > MODIFIER3D_MAX_PER_INSTANCE ||
        list->count >= MODIFIER3D_MAX_PER_INSTANCE)
    {
        return false;
    }

    float *target_value = modifier3d_get_target_pointer(
        &instance->transform,
        config->target);

    if (target_value == NULL || !isfinite(*target_value))
    {
        return false;
    }

    /*
     * Require the current parameter value to be inside the
     * modifier's configured range.
     */
    if (*target_value < config->min_value ||
        *target_value > config->max_value)
    {
        return false;
    }

    /*
     * One modifier owns each scalar parameter.
     */
    for (size_t i = 0U; i < list->count; ++i)
    {
        if (list->items[i].config.target == config->target)
        {
            return false;
        }
    }

    modifier3d_t modifier =
    {
        .config = *config,
        .phase_rad = 0.0f,
        .direction = (config->speed < 0.0f) ? -1 : 1
    };

    /*
     * Initialize the sine phase so that its first value matches
     * the instance's current parameter value.
     *
     * The ascending branch is chosen when the value has multiple
     * possible phases.
     */
    if (config->behavior == MODIFIER3D_BEHAVIOR_ABS_SIN)
    {
        const float range =
            config->max_value - config->min_value;

        float normalized =
            (*target_value - config->min_value) / range;

        if (!isfinite(normalized))
        {
            return false;
        }

        if (normalized < 0.0f) normalized = 0.0f;
        if (normalized > 1.0f) normalized = 1.0f;

        modifier.phase_rad = asinf(normalized);
    }

    list->items[list->count] = modifier;
    ++list->count;

    return true;
}

bool modifier3d_update_instance(
    geometry3d_instance_t *instance,
    float dt_seconds)
{
    if (instance == NULL ||
        !isfinite(dt_seconds) ||
        dt_seconds < 0.0f ||
        instance->modifiers.count > MODIFIER3D_MAX_PER_INSTANCE)
    {
        return false;
    }

    /*
     * A zero delta must not change either the transform or
     * the internal state of a modifier.
     */
    if (dt_seconds == 0.0f)
    {
        return true;
    }

    modifier3d_list_t *list = &instance->modifiers;

    for (size_t i = 0U; i < list->count; ++i)
    {
        modifier3d_t *modifier = &list->items[i];
        const modifier3d_config_t *config = &modifier->config;

        if (!modifier3d_config_is_valid(config))
        {
            return false;
        }

        float *target_value = modifier3d_get_target_pointer(
            &instance->transform,
            config->target);

        if (target_value == NULL || !isfinite(*target_value))
        {
            return false;
        }

        const float range =
            config->max_value - config->min_value;

        float next_value = *target_value;
        float next_phase = modifier->phase_rad;
        int8_t next_direction = modifier->direction;

        switch (config->behavior)
        {
            case MODIFIER3D_BEHAVIOR_LINEAR_FORWARD:
            {
                const float delta = config->speed * dt_seconds;

                if (!isfinite(delta))
                {
                    return false;
                }

                const float candidate =
                    *target_value + delta;

                if (!isfinite(candidate))
                {
                    return false;
                }

                float offset = fmodf(
                    candidate - config->min_value,
                    range);

                if (!isfinite(offset))
                {
                    return false;
                }

                /*
                 * Normalize negative movement into the range.
                 */
                if (offset < 0.0f)
                {
                    offset += range;
                }

                next_value = config->min_value + offset;

                /*
                 * Avoid returning max_value due to rounding:
                 * a completed forward loop wraps to min_value.
                 */
                if (next_value >= config->max_value)
                {
                    next_value = config->min_value;
                }

                break;
            }

            case MODIFIER3D_BEHAVIOR_LINEAR_BOUNCE:
            {
                if (modifier->direction != -1 &&
                    modifier->direction != 1)
                {
                    return false;
                }

                const float period = 2.0f * range;
                const float offset =
                    *target_value - config->min_value;

                /*
                 * Convert the current value and direction into
                 * an unfolded coordinate over a period of 2*range.
                 */
                const float unfolded_start =
                    (modifier->direction > 0)
                        ? offset
                        : period - offset;

                const float travel =
                    fabsf(config->speed) * dt_seconds;

                if (!isfinite(unfolded_start) ||
                    !isfinite(travel))
                {
                    return false;
                }

                const float unfolded =
                    unfolded_start + travel;

                if (!isfinite(unfolded))
                {
                    return false;
                }

                float phase = fmodf(unfolded, period);

                if (!isfinite(phase))
                {
                    return false;
                }

                if (phase < 0.0f)
                {
                    phase += period;
                }

                if (phase < range)
                {
                    next_value = config->min_value + phase;
                    next_direction = 1;
                }
                else if (phase > range)
                {
                    next_value =
                        config->min_value + (period - phase);

                    next_direction = -1;
                }
                else
                {
                    /*
                     * Exactly at the upper boundary:
                     * the next movement points back toward min.
                     */
                    next_value = config->max_value;
                    next_direction = -1;
                }

                break;
            }

            case MODIFIER3D_BEHAVIOR_ABS_SIN:
            {
                const float advanced_phase =
                    modifier->phase_rad +
                    config->speed * dt_seconds;

                if (!isfinite(advanced_phase))
                {
                    return false;
                }

                /*
                 * |sin(phase)| repeats every pi radians.
                 * Keep the internal phase bounded.
                 */
                next_phase = fmodf(
                    advanced_phase,
                    MODIFIER3D_PI);

                if (!isfinite(next_phase))
                {
                    return false;
                }

                if (next_phase < 0.0f)
                {
                    next_phase += MODIFIER3D_PI;
                }

                const float normalized =
                    fabsf(sinf(next_phase));

                next_value =
                    config->min_value + range * normalized;

                break;
            }

            default:
                return false;
        }

        if (!isfinite(next_value))
        {
            return false;
        }

        /*
         * Protect the range against minor floating-point
         * deviations at its boundaries.
         */
        if (next_value < config->min_value)
        {
            next_value = config->min_value;
        }

        if (next_value > config->max_value)
        {
            next_value = config->max_value;
        }

        /*
         * Commit the new parameter value and internal state only
         * after this modifier's calculations have succeeded.
         */
        *target_value = next_value;
        modifier->phase_rad = next_phase;
        modifier->direction = next_direction;
    }

    return true;
}