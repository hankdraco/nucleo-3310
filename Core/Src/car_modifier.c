#include "car_modifier.h"

#include <math.h>
#include <stddef.h>

#define CAR_MODIFIER_TOLERANCE (0.0001f)

bool car_modifier_context_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second)
{
    if (context == NULL)
    {
        return false;
    }

    car_modifier_context_t result = {0};

    if (!car_trajectory_init(
            &result.trajectory,
            initial_pose,
            speed_units_per_second))
    {
        return false;
    }

    result.state = CAR_MODIFIER_STATE_FREE;
    *context = result;
    return true;
}

bool car_modifier_route_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second,
    float distance_to_turn,
    float turn_radius,
    car_turn_direction_t turn_direction)
{
    if (context == NULL ||
        initial_pose == NULL ||
        !isfinite(distance_to_turn) ||
        distance_to_turn < 0.0f ||
        !isfinite(turn_radius) ||
        turn_radius <= 0.0f ||
        (turn_direction != CAR_TURN_DIRECTION_LEFT &&
         turn_direction != CAR_TURN_DIRECTION_RIGHT))
    {
        return false;
    }

    car_modifier_context_t result = {0};

    if (!car_trajectory_init(
            &result.trajectory,
            initial_pose,
            speed_units_per_second))
    {
        return false;
    }

    result.state = CAR_MODIFIER_STATE_APPROACH_TURN;
    result.approach_distance_remaining = distance_to_turn;
    result.turn_radius = turn_radius;
    result.turn_direction = turn_direction;

    *context = result;
    return true;
}

bool car_modifier_update(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context)
{
    if (instance == NULL ||
        context == NULL ||
        !isfinite(dt_seconds) ||
        dt_seconds < 0.0f)
    {
        return false;
    }

    car_modifier_context_t *car_context = context;
    car_modifier_context_t next_context = *car_context;
    float remaining_dt = dt_seconds;

    /*
     * A single update can cross more than one state boundary:
     * approach -> turning -> departing. Each state consumes only the time
     * needed for its event, leaving the rest for the next state.
     */
    while (remaining_dt > 0.0f)
    {
        if (next_context.state == CAR_MODIFIER_STATE_FREE ||
            next_context.state == CAR_MODIFIER_STATE_DEPARTING)
        {
            if (!car_trajectory_update(
                    &next_context.trajectory,
                    remaining_dt))
            {
                return false;
            }

            remaining_dt = 0.0f;
        }
        else if (next_context.state == CAR_MODIFIER_STATE_APPROACH_TURN)
        {
            const float distance_to_turn =
                next_context.approach_distance_remaining;

            if (!isfinite(distance_to_turn) ||
                distance_to_turn < -CAR_MODIFIER_TOLERANCE)
            {
                return false;
            }

            const float clamped_distance =
                (distance_to_turn > 0.0f) ? distance_to_turn : 0.0f;
            const float time_to_turn =
                clamped_distance /
                next_context.trajectory.speed_units_per_second;

            if (!isfinite(time_to_turn))
            {
                return false;
            }

            if (remaining_dt < time_to_turn)
            {
                if (!car_trajectory_update(
                        &next_context.trajectory,
                        remaining_dt))
                {
                    return false;
                }

                next_context.approach_distance_remaining =
                    clamped_distance -
                    next_context.trajectory.speed_units_per_second *
                    remaining_dt;
                remaining_dt = 0.0f;
            }
            else
            {
                /*
                 * Arrive exactly at the turn start, then capture the pivot
                 * relative to the vehicle's current heading. The pivot stays
                 * fixed in the trajectory snapshot for the whole quarter-turn.
                 */
                if (!car_trajectory_update(
                        &next_context.trajectory,
                        time_to_turn))
                {
                    return false;
                }

                next_context.approach_distance_remaining = 0.0f;

                const float turn_sign =
                    (next_context.turn_direction ==
                     CAR_TURN_DIRECTION_RIGHT) ? 1.0f : -1.0f;
                const float yaw = next_context.trajectory.pose.yaw_rad;
                const car_trajectory_vec3_t turn_pivot =
                {
                    .x = next_context.trajectory.pose.position.x +
                         turn_sign * next_context.turn_radius * cosf(yaw),
                    .y = next_context.trajectory.pose.position.y,
                    .z = next_context.trajectory.pose.position.z -
                         turn_sign * next_context.turn_radius * sinf(yaw)
                };

                if (!car_trajectory_begin_turn(
                        &next_context.trajectory,
                        &turn_pivot,
                        next_context.turn_direction))
                {
                    return false;
                }

                remaining_dt -= time_to_turn;
                next_context.state = CAR_MODIFIER_STATE_TURNING;
            }
        }
        else if (next_context.state == CAR_MODIFIER_STATE_TURNING)
        {
            /*
             * car_trajectory_update() clamps the quarter-turn and applies
             * any leftover time to straight travel in the new direction.
             */
            if (!car_trajectory_update(
                    &next_context.trajectory,
                    remaining_dt))
            {
                return false;
            }

            remaining_dt = 0.0f;

            if (next_context.trajectory.phase == CAR_TRAJECTORY_STRAIGHT)
            {
                next_context.state = CAR_MODIFIER_STATE_DEPARTING;
            }
        }
        else
        {
            return false;
        }
    }

    const car_trajectory_pose_t *pose = &next_context.trajectory.pose;

    if (!isfinite(pose->position.x) ||
        !isfinite(pose->position.y) ||
        !isfinite(pose->position.z) ||
        !isfinite(pose->yaw_rad))
    {
        return false;
    }

    instance->transform.position.x = pose->position.x;
    instance->transform.position.y = pose->position.y;
    instance->transform.position.z = pose->position.z;
    instance->transform.rotation_rad.y = pose->yaw_rad;

    *car_context = next_context;
    return true;
}
