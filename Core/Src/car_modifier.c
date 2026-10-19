#include "car_modifier.h"
#include "car_footprint.h"
#include "scene3d.h"

#include <math.h>
#include <stddef.h>

#define CAR_MODIFIER_TOLERANCE (0.0001f)
#define CAR_MODIFIER_PI_RAD (3.14159265358979323846f)
#define CAR_SAFETY_SENSOR_RANGE_UNITS (40.0f)
#define CAR_SAFETY_STANDSTILL_GAP_UNITS (1.5f)
#define CAR_SAFETY_TIME_HEADWAY_SECONDS (1.5f)
#define CAR_SAFETY_MAX_ACCELERATION_UNITS_PER_SECOND_SQUARED (4.0f)
#define CAR_SAFETY_MAX_DECELERATION_UNITS_PER_SECOND_SQUARED (14.0f)
#define CAR_SAFETY_LANE_OVERLAP_TOLERANCE_UNITS (0.25f)
#define CAR_TRAFFIC_REACTION_TIME_SECONDS (0.20f)
#define CAR_TRAFFIC_STOP_MARGIN_UNITS (1.0f)
#define CAR_TRAFFIC_STOP_CLEARANCE_UNITS (0.5f)
static bool car_modifier_inside_city(const car_trajectory_pose_t *p,const car_modifier_environment_t *e)
{
 return p&&e&&isfinite(p->position.x)&&isfinite(p->position.z)&&
 isfinite(e->city_half_extent_x)&&isfinite(e->city_half_extent_z)&&
 e->city_half_extent_x>0.0f&&e->city_half_extent_z>0.0f&&
 p->position.x>=-e->city_half_extent_x&&p->position.x<=e->city_half_extent_x&&
 p->position.z>=-e->city_half_extent_z&&p->position.z<=e->city_half_extent_z;
}

typedef enum
{
    CAR_MODIFIER_RELATION_SAME = 0,
    CAR_MODIFIER_RELATION_LEFT,
    CAR_MODIFIER_RELATION_OPPOSITE,
    CAR_MODIFIER_RELATION_RIGHT,
    CAR_MODIFIER_RELATION_INVALID
} car_modifier_relative_approach_t;

static car_modifier_relative_approach_t car_modifier_get_relative_approach(
    float reference_yaw,
    float other_yaw)
{
    if (!isfinite(reference_yaw) || !isfinite(other_yaw))
    {
        return CAR_MODIFIER_RELATION_INVALID;
    }

    const float two_pi = 2.0f * CAR_MODIFIER_PI_RAD;
    float delta = fmodf(other_yaw - reference_yaw, two_pi);
    if (delta < 0.0f)
    {
        delta += two_pi;
    }
    if (delta <= CAR_MODIFIER_TOLERANCE ||
        two_pi - delta <= CAR_MODIFIER_TOLERANCE)
    {
        return CAR_MODIFIER_RELATION_SAME;
    }
    if (fabsf(delta - CAR_MODIFIER_PI_RAD * 0.5f) <= CAR_MODIFIER_TOLERANCE)
    {
        return CAR_MODIFIER_RELATION_LEFT;
    }
    if (fabsf(delta - CAR_MODIFIER_PI_RAD) <= CAR_MODIFIER_TOLERANCE)
    {
        return CAR_MODIFIER_RELATION_OPPOSITE;
    }
    if (fabsf(delta - CAR_MODIFIER_PI_RAD * 1.5f) <= CAR_MODIFIER_TOLERANCE)
    {
        return CAR_MODIFIER_RELATION_RIGHT;
    }
    return CAR_MODIFIER_RELATION_INVALID;
}

static bool car_modifier_routes_conflict(
    car_maneuver_t own_maneuver,
    car_modifier_relative_approach_t other_approach,
    car_maneuver_t other_maneuver)
{
    if (other_approach == CAR_MODIFIER_RELATION_SAME)
    {
        return false;
    }

    if (own_maneuver != CAR_MANEUVER_RIGHT &&
        own_maneuver != CAR_MANEUVER_STRAIGHT &&
        own_maneuver != CAR_MANEUVER_LEFT)
    {
        return true;
    }
    if (other_approach == CAR_MODIFIER_RELATION_INVALID ||
        (other_maneuver != CAR_MANEUVER_RIGHT &&
         other_maneuver != CAR_MANEUVER_STRAIGHT &&
         other_maneuver != CAR_MANEUVER_LEFT))
    {
        return true;
    }

    switch (own_maneuver)
    {
        case CAR_MANEUVER_RIGHT:
            return (other_approach == CAR_MODIFIER_RELATION_LEFT &&
                    other_maneuver == CAR_MANEUVER_STRAIGHT) ||
                   (other_approach == CAR_MODIFIER_RELATION_OPPOSITE &&
                    other_maneuver == CAR_MANEUVER_LEFT);

        case CAR_MANEUVER_LEFT:
            return (other_approach == CAR_MODIFIER_RELATION_LEFT &&
                    (other_maneuver == CAR_MANEUVER_STRAIGHT ||
                     other_maneuver == CAR_MANEUVER_LEFT)) ||
                   (other_approach == CAR_MODIFIER_RELATION_OPPOSITE &&
                    (other_maneuver == CAR_MANEUVER_RIGHT ||
                     other_maneuver == CAR_MANEUVER_STRAIGHT ||
                     other_maneuver == CAR_MANEUVER_LEFT)) ||
                   (other_approach == CAR_MODIFIER_RELATION_RIGHT &&
                    (other_maneuver == CAR_MANEUVER_STRAIGHT ||
                     other_maneuver == CAR_MANEUVER_LEFT));

        case CAR_MANEUVER_STRAIGHT:
            return (other_approach == CAR_MODIFIER_RELATION_LEFT &&
                    (other_maneuver == CAR_MANEUVER_STRAIGHT ||
                     other_maneuver == CAR_MANEUVER_LEFT)) ||
                   (other_approach == CAR_MODIFIER_RELATION_OPPOSITE &&
                    other_maneuver == CAR_MANEUVER_LEFT) ||
                   (other_approach == CAR_MODIFIER_RELATION_RIGHT);

        default:
            return true;
    }
}

static bool car_modifier_is_frontmost_buffer_agent(
    const car_modifier_environment_t *environment,
    size_t agent_index,
    const car_modifier_context_t *agent_override)
{
    if (environment == NULL || environment->agent_contexts == NULL ||
        environment->agent_count == NULL ||
        *environment->agent_count > SCENE3D_MAX_AGENTS ||
        agent_index >= *environment->agent_count)
    {
        return false;
    }

    const car_modifier_context_t *agent = agent_override != NULL
        ? agent_override : &environment->agent_contexts[agent_index];
    if (agent->state != CAR_MODIFIER_STATE_BUFFER_ZONE) return false;

    for (size_t i = 0U; i < *environment->agent_count; ++i)
    {
        if (i == agent_index) continue;
        const car_modifier_context_t *other =
            &environment->agent_contexts[i];
        if (other->state != CAR_MODIFIER_STATE_BUFFER_ZONE ||
            car_modifier_get_relative_approach(
                agent->approach_yaw_rad, other->approach_yaw_rad) !=
                CAR_MODIFIER_RELATION_SAME)
        {
            continue;
        }

        if (other->approach_distance_remaining <
                agent->approach_distance_remaining - CAR_MODIFIER_TOLERANCE ||
            (fabsf(other->approach_distance_remaining -
                   agent->approach_distance_remaining) <=
                 CAR_MODIFIER_TOLERANCE && i < agent_index))
        {
            return false;
        }
    }
    return true;
}

static bool car_modifier_has_active_reservation(
    const car_modifier_environment_t *environment,
    size_t agent_index)
{
    const car_modifier_context_t *context =
        &environment->agent_contexts[agent_index];
    return context->intersection_reserved != 0U &&
           car_modifier_is_frontmost_buffer_agent(
               environment, agent_index, NULL) &&
           context->safety_state != CAR_MODIFIER_SAFETY_STATE_STOPPED;
}

static bool car_modifier_is_deadlock_winner(
    const car_modifier_environment_t *environment,
    size_t self_index,
    uint16_t *deadlock_group_mask)
{
    if (environment == NULL || deadlock_group_mask == NULL ||
        environment->agent_instances == NULL ||
        environment->agent_contexts == NULL ||
        environment->agent_count == NULL ||
        *environment->agent_count > SCENE3D_MAX_AGENTS)
    {
        return false;
    }

    *deadlock_group_mask = 0U;
    const size_t count = *environment->agent_count;
    if (self_index >= count) return false;

    uint16_t wait_edges[SCENE3D_MAX_AGENTS] = {0U};
    for (size_t i = 0U; i < count; ++i)
    {
        const car_modifier_context_t *context =
            &environment->agent_contexts[i];
        if (context->state == CAR_MODIFIER_STATE_CROSSING)
        {
            return false;
        }
        if (context->state != CAR_MODIFIER_STATE_BUFFER_ZONE ||
            !car_modifier_is_frontmost_buffer_agent(environment, i, NULL)) continue;

        for (size_t j = 0U; j < count; ++j)
        {
            if (i == j || environment->agent_contexts[j].state !=
                              CAR_MODIFIER_STATE_BUFFER_ZONE ||
                !car_modifier_is_frontmost_buffer_agent(environment, j, NULL))
            {
                continue;
            }

            const car_modifier_context_t *other_context =
                &environment->agent_contexts[j];
            const car_modifier_relative_approach_t relation =
                car_modifier_get_relative_approach(
                    context->approach_yaw_rad,
                    other_context->approach_yaw_rad);
            if (!car_modifier_routes_conflict(
                    context->maneuver, relation, other_context->maneuver))
            {
                continue;
            }

            if (car_modifier_has_active_reservation(environment, j) ||
                relation == CAR_MODIFIER_RELATION_RIGHT ||
                (relation != CAR_MODIFIER_RELATION_LEFT && j < i))
            {
                wait_edges[i] |= (uint16_t)((uint16_t)1U << j);
            }
        }
    }

    /* Transitive closure identifies which agents can reach each other. */
    for (size_t via = 0U; via < count; ++via)
    {
        const uint16_t via_bit = (uint16_t)((uint16_t)1U << via);
        for (size_t from = 0U; from < count; ++from)
        {
            if ((wait_edges[from] & via_bit) != 0U)
            {
                wait_edges[from] |= wait_edges[via];
            }
        }
    }

    const uint16_t self_bit = (uint16_t)((uint16_t)1U << self_index);
    if ((wait_edges[self_index] & self_bit) == 0U)
    {
        return false;
    }

    size_t winner_index = count;
    for (size_t i = 0U; i < count; ++i)
    {
        const uint16_t i_bit = (uint16_t)((uint16_t)1U << i);
        if ((wait_edges[self_index] & i_bit) != 0U &&
            (wait_edges[i] & self_bit) != 0U)
        {
            *deadlock_group_mask |= i_bit;
            if (i < winner_index) winner_index = i;
        }
    }

    return winner_index == self_index;
}

static bool car_modifier_has_traffic_blocker(
    const geometry3d_instance_t *self,
    const car_modifier_context_t *context,
    bool *blocked)
{
    if (self == NULL || context == NULL || blocked == NULL)
    {
        return false;
    }

    const car_modifier_environment_t *environment = context->environment;
    *blocked = false;
    if (environment == NULL)
    {
        return true;
    }

    const bool has_registry = environment->agent_instances != NULL &&
                              environment->agent_contexts != NULL &&
                              environment->agent_count != NULL;
    const bool has_any_registry = environment->agent_instances != NULL ||
                                  environment->agent_contexts != NULL ||
                                  environment->agent_count != NULL;
    if (!has_any_registry)
    {
        return true;
    }
    if (!has_registry || *environment->agent_count > SCENE3D_MAX_AGENTS)
    {
        return false;
    }

    size_t self_index = SIZE_MAX;
    uint16_t priority_blockers = 0U;
    bool hard_blocked = false;
    for (size_t i = 0U; i < *environment->agent_count; ++i)
    {
        if (&environment->agent_instances[i] == self)
        {
            self_index = i;
            break;
        }
    }
    if (self_index == SIZE_MAX) return false;

    if (context->state == CAR_MODIFIER_STATE_BUFFER_ZONE &&
        !car_modifier_is_frontmost_buffer_agent(
            environment, self_index, context))
    {
        *blocked = true;
        return true;
    }

    for (size_t i = 0U; i < *environment->agent_count; ++i)
    {
        if (i == self_index) continue;
        const car_modifier_context_t *other_context = &environment->agent_contexts[i];
        if (other_context->state != CAR_MODIFIER_STATE_CROSSING &&
            other_context->state != CAR_MODIFIER_STATE_BUFFER_ZONE)
            continue;
        if (other_context->intersection_reserved > 1U) return false;
        if (other_context->state == CAR_MODIFIER_STATE_BUFFER_ZONE &&
            !car_modifier_is_frontmost_buffer_agent(environment, i, NULL))
            continue;

        const car_modifier_relative_approach_t relation =
            car_modifier_get_relative_approach(
                context->approach_yaw_rad,
                other_context->approach_yaw_rad);
        if (!car_modifier_routes_conflict(
                context->maneuver, relation, other_context->maneuver))
            continue;

        if (other_context->state == CAR_MODIFIER_STATE_CROSSING ||
            car_modifier_has_active_reservation(environment, i))
        {
            hard_blocked = true;
            continue;
        }

        if (context->intersection_reserved != 0U)
        {
            continue;
        }

        /* Among unreserved buffer contenders, right-hand priority wins. */
        if (relation == CAR_MODIFIER_RELATION_RIGHT ||
            (relation != CAR_MODIFIER_RELATION_LEFT && i < self_index))
        {
            priority_blockers |= (uint16_t)((uint16_t)1U << i);
        }
    }

    uint16_t deadlock_group_mask = 0U;
    if (priority_blockers != 0U &&
        car_modifier_is_deadlock_winner(
            environment, self_index, &deadlock_group_mask))
    {
        /* A deadlock winner ignores only the blockers in its own cycle. */
        priority_blockers &= (uint16_t)~deadlock_group_mask;
    }
    *blocked = hard_blocked || priority_blockers != 0U;
    return true;
}

static float car_modifier_get_stop_line_offset(void)
{
    return CAR_FOOTPRINT_HALF_LENGTH_UNITS +
           CAR_TRAFFIC_STOP_CLEARANCE_UNITS;
}

static float car_modifier_get_buffer_zone_length(float maximum_speed)
{
    const float stop_line_offset = car_modifier_get_stop_line_offset();
    return stop_line_offset + CAR_TRAFFIC_STOP_MARGIN_UNITS +
        maximum_speed * CAR_TRAFFIC_REACTION_TIME_SECONDS +
        (maximum_speed * maximum_speed) /
            (2.0f * CAR_SAFETY_MAX_DECELERATION_UNITS_PER_SECOND_SQUARED);
}

static float car_modifier_get_safe_approach_speed(float distance_to_stop_line)
{
    if (distance_to_stop_line <= CAR_TRAFFIC_STOP_MARGIN_UNITS) return 0.0f;
    const float deceleration =
        CAR_SAFETY_MAX_DECELERATION_UNITS_PER_SECOND_SQUARED;
    const float reaction_speed =
        deceleration * CAR_TRAFFIC_REACTION_TIME_SECONDS;
    const float available_distance =
        distance_to_stop_line - CAR_TRAFFIC_STOP_MARGIN_UNITS;
    return sqrtf(
        reaction_speed * reaction_speed +
        2.0f * deceleration * available_distance) - reaction_speed;
}

static float car_modifier_get_stopping_distance(float speed_units_per_second)
{
    return CAR_TRAFFIC_STOP_MARGIN_UNITS +
        speed_units_per_second * CAR_TRAFFIC_REACTION_TIME_SECONDS +
        (speed_units_per_second * speed_units_per_second) /
            (2.0f * CAR_SAFETY_MAX_DECELERATION_UNITS_PER_SECOND_SQUARED);
}

static bool car_modifier_safety_find_lead(
    const geometry3d_instance_t *self,
    const car_modifier_context_t *context,
    bool *has_lead,
    float *gap_units,
    float *lead_speed)
{
    const car_modifier_environment_t *environment = context->environment;
    *has_lead = false;
    *gap_units = 0.0f;
    *lead_speed = 0.0f;
    if (environment == NULL || environment->agent_instances == NULL ||
        environment->agent_contexts == NULL || environment->agent_count == NULL)
    {
        return environment == NULL ||
            (environment->agent_instances == NULL &&
             environment->agent_contexts == NULL && environment->agent_count == NULL);
    }
    if (*environment->agent_count > SCENE3D_MAX_AGENTS) return false;

    const float yaw = self->transform.rotation_rad.y;
    const float fx = sinf(yaw), fz = cosf(yaw);
    const float sx = cosf(yaw), sz = -sinf(yaw);
    const float self_hx = fabsf(self->transform.scale.x);
    const float self_hz = fabsf(self->transform.scale.z);
    const float self_forward = self_hz;
    const float self_side = self_hx;
    const float self_x = self->transform.position.x;
    const float self_z = self->transform.position.z;
    if (!isfinite(yaw) || !isfinite(self_hx) || !isfinite(self_hz) ||
        self_hx <= 0.0f || self_hz <= 0.0f ||
        !isfinite(self_x) || !isfinite(self_z)) return false;

    float nearest_gap = CAR_SAFETY_SENSOR_RANGE_UNITS;
    for (size_t i = 0U; i < *environment->agent_count; ++i)
    {
        const geometry3d_instance_t *other = &environment->agent_instances[i];
        const car_modifier_context_t *other_context = &environment->agent_contexts[i];
        if (other == self || other_context->state == CAR_MODIFIER_STATE_SPAWN)
        {
            continue;
        }
        const float other_yaw = other->transform.rotation_rad.y;
        const float ofx = sinf(other_yaw), ofz = cosf(other_yaw);
        /* Ignore crossing traffic: this controller only handles longitudinal following. */
        if (!isfinite(other_yaw) || ofx * fx + ofz * fz < 0.95f) continue;
        const float ohx = fabsf(other->transform.scale.x);
        const float ohz = fabsf(other->transform.scale.z);
        const float dx = other->transform.position.x - self_x;
        const float dz = other->transform.position.z - self_z;
        if (!isfinite(dx) || !isfinite(dz) || !isfinite(ohx) || !isfinite(ohz) ||
            ohx <= 0.0f || ohz <= 0.0f) return false;
        const float longitudinal = dx * fx + dz * fz;
        const float lateral = dx * sx + dz * sz;
        const float osx = cosf(other_yaw), osz = -sinf(other_yaw);
        const float other_forward = ohz * fabsf(ofx * fx + ofz * fz) +
                                    ohx * fabsf(osx * fx + osz * fz);
        const float other_side = ohz * fabsf(ofx * sx + ofz * sz) +
                                 ohx * fabsf(osx * sx + osz * sz);
        const float gap = longitudinal - self_forward - other_forward;
        if (longitudinal <= 0.0f ||
            fabsf(lateral) > self_side + other_side + CAR_SAFETY_LANE_OVERLAP_TOLERANCE_UNITS ||
            gap >= nearest_gap) continue;

        nearest_gap = gap;
        *has_lead = true;
        *gap_units = gap;
        *lead_speed = (other_context->state == CAR_MODIFIER_STATE_BUFFER_ZONE &&
                       other_context->approach_distance_remaining <=
                           car_modifier_get_stop_line_offset() + CAR_MODIFIER_TOLERANCE) ||
                      other_context->safety_state == CAR_MODIFIER_SAFETY_STATE_STOPPED
            ? 0.0f : other_context->trajectory.speed_units_per_second;
        if (!isfinite(*lead_speed) || *lead_speed < 0.0f) return false;
    }
    return true;
}

static bool car_modifier_begin_maneuver(car_modifier_context_t *context)
{
    if (context->maneuver == CAR_MANEUVER_STRAIGHT)
    {
        context->crossing_distance_remaining =
            context->environment != NULL &&
            isfinite(context->environment->street_half_width) &&
            context->environment->street_half_width > 0.0f
                ? 2.0f * context->environment->street_half_width
                : 0.0f;
        context->state = context->crossing_distance_remaining > 0.0f
            ? CAR_MODIFIER_STATE_CROSSING
            : CAR_MODIFIER_STATE_DEPARTING;
        return true;
    }

    const car_turn_direction_t direction =
        (context->maneuver == CAR_MANEUVER_RIGHT)
            ? CAR_TURN_DIRECTION_RIGHT
            : CAR_TURN_DIRECTION_LEFT;
    const float turn_sign =
        (direction == CAR_TURN_DIRECTION_RIGHT) ? 1.0f : -1.0f;
    const float yaw = context->trajectory.pose.yaw_rad;
    const car_trajectory_vec3_t pivot =
    {
        .x = context->trajectory.pose.position.x +
             turn_sign * context->turn_radius * cosf(yaw),
        .y = context->trajectory.pose.position.y,
        .z = context->trajectory.pose.position.z -
             turn_sign * context->turn_radius * sinf(yaw)
    };
    if (!car_trajectory_begin_turn(&context->trajectory, &pivot, direction))
    {
        return false;
    }
    context->crossing_distance_remaining = 0.0f;
    context->state = CAR_MODIFIER_STATE_CROSSING;
    return true;
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
typedef enum
{
    CAR_MODIFIER_SPAWN_ERROR = 0,
    CAR_MODIFIER_SPAWN_BLOCKED,
    CAR_MODIFIER_SPAWN_SUCCESS
} car_modifier_spawn_result_t;

static bool car_modifier_spawn_location_is_clear(
    const geometry3d_instance_t *self,
    const car_modifier_context_t *context,
    const car_trajectory_pose_t *candidate)
{
    const car_modifier_environment_t *environment = context->environment;
    if (environment->agent_instances == NULL || environment->agent_contexts == NULL ||
        environment->agent_count == NULL)
    {
        return environment->agent_instances == NULL &&
               environment->agent_contexts == NULL &&
               environment->agent_count == NULL;
    }

    if (*environment->agent_count > SCENE3D_MAX_AGENTS)
    {
        return false;
    }

    const car_footprint_t candidate_footprint =
    {
        .center_x = candidate->position.x,
        .center_z = candidate->position.z,
        .yaw_rad = candidate->yaw_rad,
        .half_width = CAR_FOOTPRINT_HALF_WIDTH_UNITS,
        .half_length = CAR_FOOTPRINT_HALF_LENGTH_UNITS
    };

    for (size_t i = 0U; i < *environment->agent_count; ++i)
    {
        const geometry3d_instance_t *other = &environment->agent_instances[i];
        if (other == self || environment->agent_contexts[i].state == CAR_MODIFIER_STATE_SPAWN)
        {
            continue;
        }

        const car_footprint_t other_footprint =
        {
            .center_x = other->transform.position.x,
            .center_z = other->transform.position.z,
            .yaw_rad = other->transform.rotation_rad.y,
            .half_width = fabsf(other->transform.scale.x),
            .half_length = fabsf(other->transform.scale.z)
        };
        if (!isfinite(other_footprint.center_x) || !isfinite(other_footprint.center_z) ||
            !isfinite(other_footprint.yaw_rad) || other_footprint.half_width <= 0.0f ||
            other_footprint.half_length <= 0.0f)
        {
            return false;
        }
        if (car_footprints_overlap(
                &candidate_footprint,
                &other_footprint,
                CAR_FOOTPRINT_SPAWN_CLEARANCE_UNITS))
        {
            return false;
        }
    }

    return true;
}

static car_modifier_spawn_result_t car_modifier_spawn(
    geometry3d_instance_t *instance,
    car_modifier_context_t *c,
    uint32_t *committed)
{
 if(!instance||!c||!c->environment||!committed)return CAR_MODIFIER_SPAWN_ERROR;
 car_modifier_environment_t *e=c->environment;
 if(!isfinite(e->city_half_extent_x)||!isfinite(e->city_half_extent_z)||
 !isfinite(e->street_half_width)||!isfinite(e->lane_edge_offset)||
 !isfinite(e->minimum_speed_units_per_second)||!isfinite(e->maximum_speed_units_per_second)||
 e->city_half_extent_x<=e->street_half_width||e->city_half_extent_z<=e->street_half_width||
 e->street_half_width<=0.0f||e->lane_edge_offset<0.0f||
 e->minimum_speed_units_per_second<=0.0f||
 e->maximum_speed_units_per_second<e->minimum_speed_units_per_second)return CAR_MODIFIER_SPAWN_ERROR;
 const bool has_agent_registry = e->agent_instances != NULL &&
                                 e->agent_contexts != NULL &&
                                 e->agent_count != NULL;
 const bool has_any_agent_registry = e->agent_instances != NULL ||
                                     e->agent_contexts != NULL ||
                                     e->agent_count != NULL;
 if (has_any_agent_registry && !has_agent_registry)
 {
     return CAR_MODIFIER_SPAWN_ERROR;
 }
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
 /*
  * Every car spawns in its right-hand lane. A right turn follows the
  * inner corner (lane_edge_offset); a left turn follows the outer corner
  * (two street half-widths minus the lane offset).
  */
 float radius = 0.0f;
 if (maneuver == CAR_MANEUVER_RIGHT)
 {
     radius = e->lane_edge_offset;
 }
 else if (maneuver == CAR_MANEUVER_LEFT)
 {
     radius = 2.0f * e->street_half_width - e->lane_edge_offset;
 }
 car_modifier_context_t result={0};
 result.safety_state=CAR_MODIFIER_SAFETY_STATE_CLEAR;
 if(!car_modifier_route_init(&result,&pose,speed,distance,maneuver,radius))return CAR_MODIFIER_SPAWN_ERROR;
 result.environment=e;
 result.buffer_zone_length_units=car_modifier_get_buffer_zone_length(e->maximum_speed_units_per_second);
 *committed = rng;
 if (!car_modifier_spawn_location_is_clear(instance, c, &pose))
 {
     return CAR_MODIFIER_SPAWN_BLOCKED;
 }
 result.safety_state=c->safety_state;
 *c=result;return CAR_MODIFIER_SPAWN_SUCCESS;
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
    result.safety_state = CAR_MODIFIER_SAFETY_STATE_CLEAR;
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
    result.approach_yaw_rad = initial_pose->yaw_rad;
    result.safety_state = CAR_MODIFIER_SAFETY_STATE_CLEAR;
    result.cruise_speed_units_per_second = speed_units_per_second;
    result.safety_target_speed_units_per_second = speed_units_per_second;
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
    result.safety_state = CAR_MODIFIER_SAFETY_STATE_CLEAR;
    result.cruise_speed_units_per_second = speed_units_per_second;
    result.safety_target_speed_units_per_second = speed_units_per_second;
    result.approach_distance_remaining = distance_to_maneuver;
    result.turn_radius = is_turn ? turn_radius : 0.0f;
    result.maneuver = maneuver;
    result.approach_yaw_rad = initial_pose->yaw_rad;

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
    if (next_context.safety_state != CAR_MODIFIER_SAFETY_STATE_CLEAR &&
        next_context.safety_state != CAR_MODIFIER_SAFETY_STATE_YIELDING &&
        next_context.safety_state != CAR_MODIFIER_SAFETY_STATE_FOLLOWING &&
        next_context.safety_state != CAR_MODIFIER_SAFETY_STATE_STOPPED)
    {
        return false;
    }

    bool release_reservation =
        next_context.safety_state == CAR_MODIFIER_SAFETY_STATE_STOPPED;
    if (!release_reservation && next_context.environment != NULL &&
        next_context.environment->agent_instances != NULL &&
        next_context.environment->agent_count != NULL &&
        *next_context.environment->agent_count <= SCENE3D_MAX_AGENTS)
    {
        for (size_t i = 0U; i < *next_context.environment->agent_count; ++i)
        {
            if (&next_context.environment->agent_instances[i] == instance)
            {
                release_reservation =
                    !car_modifier_is_frontmost_buffer_agent(
                        next_context.environment, i, &next_context);
                break;
            }
        }
    }
    if (next_context.state == CAR_MODIFIER_STATE_BUFFER_ZONE &&
        next_context.intersection_reserved != 0U && release_reservation)
    {
        next_context.intersection_reserved = 0U;
    }

    float remaining_dt = dt_seconds;
    bool spawned = false;
    uint32_t next_random_state = 0U;
    /* A blocked SPAWN consumes its random attempt, then retries next update. */
    if (next_context.state == CAR_MODIFIER_STATE_SPAWN)
    {
        const car_modifier_spawn_result_t spawn_result =
            car_modifier_spawn(instance, &next_context, &next_random_state);
        if (spawn_result == CAR_MODIFIER_SPAWN_ERROR) return false;
        if (spawn_result == CAR_MODIFIER_SPAWN_BLOCKED)
        {
            if (next_context.environment != NULL)
            {
                next_context.environment->random_state = next_random_state;
            }
            return true;
        }
        spawned = true;
        remaining_dt = 0.0f;
    }

    if (!spawned && next_context.state == CAR_MODIFIER_STATE_APPROACH_TURN &&
        next_context.buffer_zone_length_units > 0.0f &&
        next_context.approach_distance_remaining <=
            next_context.buffer_zone_length_units)
    {
        next_context.state = CAR_MODIFIER_STATE_BUFFER_ZONE;
    }

    float target_speed = next_context.safety_target_speed_units_per_second;
    bool traffic_blocked = false;
    if (!spawned && next_context.state == CAR_MODIFIER_STATE_BUFFER_ZONE)
    {
        const float current_speed = next_context.trajectory.speed_units_per_second;
        if (!isfinite(current_speed) || current_speed < 0.0f ||
            !isfinite(target_speed) || target_speed < 0.0f) return false;
        if (!car_modifier_has_traffic_blocker(
                instance, &next_context, &traffic_blocked)) return false;
        const float distance_to_stop_line = fmaxf(
            0.0f,
            next_context.approach_distance_remaining -
                car_modifier_get_stop_line_offset());
        if (traffic_blocked)
        {
            const float traffic_speed =
                car_modifier_get_safe_approach_speed(distance_to_stop_line);
            if (traffic_speed < target_speed) target_speed = traffic_speed;
        }
        else if (next_context.cruise_speed_units_per_second < target_speed)
        {
            target_speed = next_context.cruise_speed_units_per_second;
        }

        if (!traffic_blocked &&
            next_context.safety_state != CAR_MODIFIER_SAFETY_STATE_STOPPED &&
            next_context.intersection_reserved == 0U &&
            distance_to_stop_line <= car_modifier_get_stopping_distance(current_speed))
            next_context.intersection_reserved = 1U;
    }

    if (!spawned && dt_seconds > 0.0f &&
        next_context.safety_state != CAR_MODIFIER_SAFETY_STATE_STOPPED)
    {
        const float current_speed = next_context.trajectory.speed_units_per_second;
        if (!isfinite(current_speed) || current_speed < 0.0f ||
            !isfinite(target_speed) || target_speed < 0.0f) return false;
        if (current_speed == 0.0f && target_speed == 0.0f)
        {
            remaining_dt = 0.0f;
        }
        else
        {
            const float rate = target_speed < current_speed
                ? CAR_SAFETY_MAX_DECELERATION_UNITS_PER_SECOND_SQUARED
                : CAR_SAFETY_MAX_ACCELERATION_UNITS_PER_SECOND_SQUARED;
            float updated_speed = current_speed;
            const float max_delta = rate * dt_seconds;
            if (target_speed < current_speed)
                updated_speed = fmaxf(target_speed, current_speed - max_delta);
            else
                updated_speed = fminf(target_speed, current_speed + max_delta);
            if (updated_speed <= 0.001f)
            {
                updated_speed = 0.0f;
                remaining_dt = 0.0f;
            }
            next_context.trajectory.speed_units_per_second = updated_speed;
        }
        if (next_context.trajectory.phase == CAR_TRAJECTORY_TURNING)
        {
            if (next_context.trajectory.speed_units_per_second > 0.0f)
                next_context.trajectory.turn_angular_speed_rad_per_second =
                    next_context.trajectory.speed_units_per_second /
                    next_context.trajectory.turn_radius;
        }
    }

    /* A fully stopped vehicle holds position; FOLLOWING brakes through the speed controller. */
    if (next_context.safety_state == CAR_MODIFIER_SAFETY_STATE_STOPPED)
    {
        remaining_dt = 0.0f;
    }
    else if (!spawned && next_context.state == CAR_MODIFIER_STATE_BUFFER_ZONE &&
             dt_seconds == 0.0f && !traffic_blocked &&
             (next_context.maneuver == CAR_MANEUVER_STRAIGHT ||
              next_context.trajectory.speed_units_per_second >
                  CAR_MODIFIER_TOLERANCE) &&
             next_context.approach_distance_remaining <=
                 car_modifier_get_stop_line_offset() + CAR_MODIFIER_TOLERANCE)
    {
        next_context.intersection_reserved = 1U;
        next_context.approach_distance_remaining = 0.0f;
        if (!car_modifier_begin_maneuver(&next_context)) return false;
        next_context.intersection_reserved = 0U;
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
                remaining_dt=0.0f;break;
            }
            if(!car_trajectory_update(&next_context.trajectory,remaining_dt))return false;
            remaining_dt=0.0f;
            if(next_context.environment &&
                !car_modifier_inside_city(&next_context.trajectory.pose,next_context.environment))
            {
                next_context.state=CAR_MODIFIER_STATE_SPAWN;
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

            const float buffer_length = next_context.buffer_zone_length_units;
            const float distance_to_buffer = fmaxf(
                0.0f, distance_to_turn - buffer_length);
            const float time_to_buffer = distance_to_buffer /
                next_context.trajectory.speed_units_per_second;

            if (!isfinite(time_to_buffer))
            {
                return false;
            }

            if (remaining_dt < time_to_buffer)
            {
                if (!car_trajectory_update(
                        &next_context.trajectory,
                        remaining_dt))
                {
                    return false;
                }

                next_context.approach_distance_remaining =
                    distance_to_turn -
                    next_context.trajectory.speed_units_per_second *
                    remaining_dt;
                remaining_dt = 0.0f;
            }
            else
            {
                /*
                 * Enter the buffer zone before the maneuver point. When no
                 * buffer is configured, this is the legacy stop-line point.
                 */
                if (!car_trajectory_update(
                        &next_context.trajectory,
                        time_to_buffer))
                {
                    return false;
                }

                next_context.approach_distance_remaining = buffer_length;
                remaining_dt -= time_to_buffer;
                next_context.state = CAR_MODIFIER_STATE_BUFFER_ZONE;
                if (buffer_length <= 0.0f)
                {
                    bool traffic_blocked = false;
                    if (!car_modifier_has_traffic_blocker(
                            instance, &next_context, &traffic_blocked))
                        return false;
                    if (traffic_blocked)
                    {
                        remaining_dt = 0.0f;
                        break;
                    }
                    next_context.intersection_reserved = 1U;
                    next_context.approach_distance_remaining = 0.0f;
                    if (!car_modifier_begin_maneuver(&next_context)) return false;
                    next_context.intersection_reserved = 0U;
                    continue;
                }

                continue;
            }
        }
        else if (next_context.state == CAR_MODIFIER_STATE_BUFFER_ZONE)
        {
            bool traffic_blocked = false;
            if (!car_modifier_has_traffic_blocker(
                    instance, &next_context, &traffic_blocked)) return false;

            const float stop_line_offset = car_modifier_get_stop_line_offset();
            const float distance_to_stop_line = fmaxf(
                0.0f,
                next_context.approach_distance_remaining - stop_line_offset);
            const float speed = next_context.trajectory.speed_units_per_second;

            if (traffic_blocked)
            {
                if (distance_to_stop_line <= CAR_MODIFIER_TOLERANCE ||
                    speed <= CAR_MODIFIER_TOLERANCE)
                {
                    remaining_dt = 0.0f;
                    break;
                }

                const float time_to_stop_line = distance_to_stop_line / speed;
                const float move_time = (remaining_dt < time_to_stop_line)
                    ? remaining_dt : time_to_stop_line;
                if (!car_trajectory_update(&next_context.trajectory, move_time))
                    return false;
                next_context.approach_distance_remaining -= speed * move_time;
                if (move_time >= time_to_stop_line)
                {
                    next_context.approach_distance_remaining = stop_line_offset;
                    remaining_dt = 0.0f;
                }
                else
                {
                    remaining_dt = 0.0f;
                }
                break;
            }

            if (!next_context.intersection_reserved &&
                next_context.safety_state != CAR_MODIFIER_SAFETY_STATE_STOPPED &&
                distance_to_stop_line <= car_modifier_get_stopping_distance(speed))
            {
                next_context.intersection_reserved = 1U;
            }

            if (speed <= CAR_MODIFIER_TOLERANCE)
            {
                remaining_dt = 0.0f;
                break;
            }

            const float distance_to_maneuver =
                next_context.approach_distance_remaining;
            const float time_to_maneuver = distance_to_maneuver / speed;
            if (!isfinite(time_to_maneuver)) return false;
            if (remaining_dt < time_to_maneuver)
            {
                if (!car_trajectory_update(
                        &next_context.trajectory, remaining_dt)) return false;
                next_context.approach_distance_remaining -= speed * remaining_dt;
                const float remaining_to_stop_line = fmaxf(
                    0.0f,
                    next_context.approach_distance_remaining - stop_line_offset);
                if (!next_context.intersection_reserved &&
                    remaining_to_stop_line <= car_modifier_get_stopping_distance(speed))
                    next_context.intersection_reserved = 1U;
                remaining_dt = 0.0f;
            }
            else
            {
                if (!car_trajectory_update(
                        &next_context.trajectory, time_to_maneuver)) return false;
                next_context.approach_distance_remaining = 0.0f;
                remaining_dt -= time_to_maneuver;
                if (!next_context.intersection_reserved)
                    next_context.intersection_reserved = 1U;
                if (!car_modifier_begin_maneuver(&next_context)) return false;
                next_context.intersection_reserved = 0U;
                continue;
            }
        }
        else if (next_context.state == CAR_MODIFIER_STATE_CROSSING)
        {
            if (next_context.trajectory.phase == CAR_TRAJECTORY_TURNING)
            {
                /* The arc's quarter-turn completion clears the crossing state. */
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
                const float crossing_distance =
                    next_context.crossing_distance_remaining;
                if (!isfinite(crossing_distance) || crossing_distance < 0.0f)
                {
                    return false;
                }

                if (crossing_distance <= CAR_MODIFIER_TOLERANCE)
                {
                    next_context.crossing_distance_remaining = 0.0f;
                    next_context.state = CAR_MODIFIER_STATE_DEPARTING;
                    continue;
                }

                const float time_to_clear = crossing_distance /
                    next_context.trajectory.speed_units_per_second;
                if (!isfinite(time_to_clear))
                {
                    return false;
                }

                if (remaining_dt < time_to_clear)
                {
                    if (!car_trajectory_update(
                            &next_context.trajectory, remaining_dt))
                    {
                        return false;
                    }
                    next_context.crossing_distance_remaining =
                        crossing_distance -
                        next_context.trajectory.speed_units_per_second * remaining_dt;
                    remaining_dt = 0.0f;
                }
                else
                {
                    if (!car_trajectory_update(
                            &next_context.trajectory, time_to_clear))
                    {
                        return false;
                    }
                    next_context.crossing_distance_remaining = 0.0f;
                    next_context.state = CAR_MODIFIER_STATE_DEPARTING;
                    remaining_dt -= time_to_clear;
                }
            }
        }
        else if (next_context.state == CAR_MODIFIER_STATE_SPAWN)
        {
            const car_modifier_spawn_result_t spawn_result =
                car_modifier_spawn(instance, &next_context, &next_random_state);
            if (spawn_result == CAR_MODIFIER_SPAWN_ERROR) return false;
            if (spawn_result == CAR_MODIFIER_SPAWN_SUCCESS) spawned = true;
            else if (next_context.environment != NULL)
                next_context.environment->random_state = next_random_state;
            remaining_dt=0.0f;
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

bool car_modifier_safety_update(
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
    if (car_context->safety_state != CAR_MODIFIER_SAFETY_STATE_CLEAR &&
        car_context->safety_state != CAR_MODIFIER_SAFETY_STATE_YIELDING &&
        car_context->safety_state != CAR_MODIFIER_SAFETY_STATE_FOLLOWING &&
        car_context->safety_state != CAR_MODIFIER_SAFETY_STATE_STOPPED)
        return false;
    if (car_context->state == CAR_MODIFIER_STATE_SPAWN)
    {
        car_context->safety_state = CAR_MODIFIER_SAFETY_STATE_CLEAR;
        car_context->safety_target_speed_units_per_second = 0.0f;
        return true;
    }
    const float current_speed = car_context->trajectory.speed_units_per_second;
    const float cruise_speed = car_context->cruise_speed_units_per_second;
    if (!isfinite(current_speed) || current_speed < 0.0f ||
        !isfinite(cruise_speed) || cruise_speed <= 0.0f) return false;

    bool has_lead = false;
    float gap = 0.0f;
    float lead_speed = 0.0f;
    if (!car_modifier_safety_find_lead(
            instance, car_context, &has_lead, &gap, &lead_speed)) return false;
    float target_speed = cruise_speed;
    if (has_lead)
    {
        const float deceleration =
            CAR_SAFETY_MAX_DECELERATION_UNITS_PER_SECOND_SQUARED;
        const float time_buffer =
            deceleration * CAR_SAFETY_TIME_HEADWAY_SECONDS;
        float available_gap = gap - CAR_SAFETY_STANDSTILL_GAP_UNITS;
        if (available_gap < 0.0f) available_gap = 0.0f;
        const float radicand = time_buffer * time_buffer +
            lead_speed * lead_speed + 2.0f * deceleration * available_gap;
        target_speed = sqrtf(radicand) - time_buffer;
        if (gap <= CAR_SAFETY_STANDSTILL_GAP_UNITS) target_speed = 0.0f;
        if (target_speed < 0.0f) target_speed = 0.0f;
        if (target_speed > cruise_speed) target_speed = cruise_speed;
    }
    car_context->safety_target_speed_units_per_second = target_speed;
    if (has_lead && target_speed <= 0.001f && current_speed <= 0.05f)
        car_context->safety_state = CAR_MODIFIER_SAFETY_STATE_STOPPED;
    else if (has_lead && target_speed + 0.001f < current_speed)
        car_context->safety_state = CAR_MODIFIER_SAFETY_STATE_FOLLOWING;
    else
        car_context->safety_state = CAR_MODIFIER_SAFETY_STATE_CLEAR;
    return true;
}
