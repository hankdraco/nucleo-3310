#include "car_modifier.h"

#include <math.h>
#include <stddef.h>

#define CAR_MODIFIER_TOLERANCE (0.0001f)
#define CAR_MODIFIER_PI_RAD (3.14159265358979323846f)
static bool car_modifier_inside_city(const car_trajectory_pose_t *p,const car_modifier_environment_t *e)
{
 return p&&e&&isfinite(p->position.x)&&isfinite(p->position.z)&&
 isfinite(e->city_half_extent_x)&&isfinite(e->city_half_extent_z)&&
 e->city_half_extent_x>0.0f&&e->city_half_extent_z>0.0f&&
 p->position.x>=-e->city_half_extent_x&&p->position.x<=e->city_half_extent_x&&
 p->position.z>=-e->city_half_extent_z&&p->position.z<=e->city_half_extent_z;
}
static uint32_t car_modifier_random_next(uint32_t *s)
{
 uint32_t x=*s;if(x==0U)x=0x6D2B79F5U;
 x^=x<<13U;x^=x>>17U;x^=x<<5U;*s=x;return x;
}
static float car_modifier_random_range(uint32_t *s,float lo,float hi)
{
 return lo+(hi-lo)*(float)(car_modifier_random_next(s)>>8U)/16777215.0f;
}
static bool car_modifier_spawn(car_modifier_context_t *c,uint32_t *committed)
{
 if(!c||!c->environment||!committed)return false;
 car_modifier_environment_t *e=c->environment;
 if(!isfinite(e->city_half_extent_x)||!isfinite(e->city_half_extent_z)||
 !isfinite(e->street_half_width)||!isfinite(e->lane_edge_offset)||
 !isfinite(e->minimum_speed_units_per_second)||!isfinite(e->maximum_speed_units_per_second)||
 e->city_half_extent_x<=e->street_half_width||e->city_half_extent_z<=e->street_half_width||
 e->street_half_width<=0.0f||e->lane_edge_offset<0.0f||
 e->minimum_speed_units_per_second<=0.0f||
 e->maximum_speed_units_per_second<e->minimum_speed_units_per_second)return false;
 uint32_t rng=e->random_state,entry=car_modifier_random_next(&rng)%4U;
 car_maneuver_t maneuver=(car_maneuver_t)(car_modifier_random_next(&rng)%3U);
 float speed=car_modifier_random_range(&rng,e->minimum_speed_units_per_second,e->maximum_speed_units_per_second);
 float lane=e->street_half_width-e->lane_edge_offset;
 car_trajectory_pose_t pose={.position={.x=lane,.y=1.0f,.z=0.0f},.yaw_rad=0.0f};
 switch(entry){
 case 0U:pose.position.z=-e->city_half_extent_z;break;
 case 1U:pose.position.x=-lane;pose.position.z=e->city_half_extent_z;pose.yaw_rad=CAR_MODIFIER_PI_RAD;break;
 case 2U:pose.position.x=-e->city_half_extent_x;pose.position.z=-lane;pose.yaw_rad=CAR_MODIFIER_PI_RAD*0.5f;break;
 default:pose.position.x=e->city_half_extent_x;pose.position.z=lane;pose.yaw_rad=-CAR_MODIFIER_PI_RAD*0.5f;break;
 }
 float distance=entry<2U?e->city_half_extent_z-e->street_half_width:e->city_half_extent_x-e->street_half_width;
 float radius=2.0f*e->street_half_width-e->lane_edge_offset;
 car_modifier_context_t result={0};
 if(!car_modifier_route_init(&result,&pose,speed,distance,maneuver,maneuver==CAR_MANEUVER_STRAIGHT?0.0f:radius))return false;
 result.environment=e;*c=result;*committed=rng;return true;
}

bool car_modifier_context_init_spawn(
    car_modifier_context_t *context,
    car_modifier_environment_t *environment)
{
    if (context == NULL || environment == NULL)
    {
        return false;
    }

    car_modifier_context_t result = {0};
    result.state = CAR_MODIFIER_STATE_SPAWN;
    result.environment = environment;
    *context = result;

    return true;
}

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
    float distance_to_maneuver,
    car_maneuver_t maneuver,
    float turn_radius)
{
    const bool is_turn =
        maneuver == CAR_MANEUVER_LEFT || maneuver == CAR_MANEUVER_RIGHT;

    if (context == NULL || initial_pose == NULL ||
        !isfinite(distance_to_maneuver) || distance_to_maneuver < 0.0f ||
        (maneuver != CAR_MANEUVER_STRAIGHT && !is_turn) ||
        (is_turn && (!isfinite(turn_radius) || turn_radius <= 0.0f)))
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
    result.approach_distance_remaining = distance_to_maneuver;
    result.turn_radius = is_turn ? turn_radius : 0.0f;
    result.maneuver = maneuver;

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
    bool spawned = false;
    uint32_t next_random_state = 0U;
    /* SPAWN is a one-shot transition, independent of elapsed time. */
    if (next_context.state == CAR_MODIFIER_STATE_SPAWN)
    {
        if (!car_modifier_spawn(&next_context, &next_random_state)) return false;
        spawned = true;
        remaining_dt = 0.0f;
    }

    /*
     * A single update can cross more than one state boundary:
     * approach -> turning -> departing. Each state consumes only the time
     * needed for its event, leaving the rest for the next state.
     */
    while (remaining_dt > 0.0f)
    {
        if (next_context.state == CAR_MODIFIER_STATE_FREE)
        {
            if (!car_trajectory_update(&next_context.trajectory, remaining_dt)) return false;
            remaining_dt = 0.0f;
        }
        else if (next_context.state == CAR_MODIFIER_STATE_DEPARTING)
        {
            if (next_context.environment &&
                !car_modifier_inside_city(&next_context.trajectory.pose,next_context.environment))
            {
                next_context.state=CAR_MODIFIER_STATE_SPAWN;
                if(!car_modifier_spawn(&next_context,&next_random_state))return false;
                spawned=true;remaining_dt=0.0f;break;
            }
            if(!car_trajectory_update(&next_context.trajectory,remaining_dt))return false;
            remaining_dt=0.0f;
            if(next_context.environment &&
                !car_modifier_inside_city(&next_context.trajectory.pose,next_context.environment))
            {
                next_context.state=CAR_MODIFIER_STATE_SPAWN;
                if(!car_modifier_spawn(&next_context,&next_random_state))return false;
                spawned=true;
            }
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

                remaining_dt -= time_to_turn;

                if (next_context.maneuver == CAR_MANEUVER_STRAIGHT)
                {
                    next_context.state = CAR_MODIFIER_STATE_DEPARTING;
                    continue;
                }

                const car_turn_direction_t turn_direction =
                    (next_context.maneuver == CAR_MANEUVER_RIGHT)
                        ? CAR_TURN_DIRECTION_RIGHT
                        : CAR_TURN_DIRECTION_LEFT;
                const float turn_sign =
                    (turn_direction == CAR_TURN_DIRECTION_RIGHT) ? 1.0f : -1.0f;
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
                        &next_context.trajectory, &turn_pivot, turn_direction))
                {
                    return false;
                }

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
        else if (next_context.state == CAR_MODIFIER_STATE_SPAWN)
        {
            if (!car_modifier_spawn(&next_context,&next_random_state)) return false;
            spawned=true;remaining_dt=0.0f;
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

    if(spawned&&next_context.environment)next_context.environment->random_state=next_random_state;
    *car_context = next_context;
    return true;
}
