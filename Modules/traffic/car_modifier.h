#ifndef CAR_MODIFIER_H
#define CAR_MODIFIER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "Modules/traffic/car_trajectory.h"
#include "Modules/engine3d/geometry3d.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Route-lifecycle state of a car agent. */
typedef enum
{
    CAR_MODIFIER_STATE_FREE = 0, /**< Unconstrained trajectory-only motion. */
    CAR_MODIFIER_STATE_APPROACH_TURN, /**< Driving toward the approach buffer. */
    CAR_MODIFIER_STATE_BUFFER_ZONE, /**< Repeatedly checking entry conditions and adjusting speed. */
    CAR_MODIFIER_STATE_CROSSING, /**< Traversing the intersection. */
    CAR_MODIFIER_STATE_DEPARTING, /**< Leaving the city after the route. */
    CAR_MODIFIER_STATE_SPAWN /**< Searching for a valid spawn location. */
} car_modifier_state_t;

/** Current action selected by the independent collision-safety controller. */
typedef enum
{
    CAR_MODIFIER_SAFETY_STATE_CLEAR = 0, /**< No vehicle ahead requires a speed reduction. */
    CAR_MODIFIER_SAFETY_STATE_YIELDING, /**< Slowing or stopped to give way to a vehicle. */
    CAR_MODIFIER_SAFETY_STATE_FOLLOWING, /**< Matching the speed of a lead vehicle. */
    CAR_MODIFIER_SAFETY_STATE_STOPPED /**< Fully stopped to maintain the safety gap. */
} car_modifier_safety_state_t;

/** Forward declaration of the caller-owned context for one car agent. */
typedef struct car_modifier_context car_modifier_context_t;

/** Shared world data referenced by every car context.
 * The random generator state is shared by all cars to produce deterministic choices.
 */
typedef struct
{
    float city_half_extent_x; /**< Half-size of the city along X. */
    float city_half_extent_z; /**< Half-size of the city along Z. */
    float street_half_width; /**< Half-width of the road around the intersection. */
    float lane_edge_offset; /**< Vehicle-center offset from the road centerline. */
    float minimum_speed_units_per_second; /**< Lower bound for randomized cruise speed. */
    float maximum_speed_units_per_second; /**< Upper bound for randomized cruise speed. */
    uint32_t random_state; /**< Shared deterministic pseudo-random generator state. */
    geometry3d_instance_t *agent_instances; /**< Scene-owned agent instance array. */
    car_modifier_context_t *agent_contexts; /**< Stable context array paired with agent_instances. */
    size_t *agent_count; /**< Pointer to the number of active agents. */
} car_modifier_environment_t;

/** Maneuver selected for a car at the intersection. */
typedef enum
{
    CAR_MANEUVER_STRAIGHT = 0, /**< Continue straight through the intersection. */
    CAR_MANEUVER_LEFT, /**< Turn left at the intersection. */
    CAR_MANEUVER_RIGHT /**< Turn right at the intersection. */
} car_maneuver_t;

/** Caller-owned runtime state for one car modifier.
 * Keeps route and safety FSMs separate from the trajectory motion model.
 */
struct car_modifier_context
{
    car_trajectory_t trajectory; /**< Portable kinematic model. */
    car_modifier_state_t state; /**< Current route-lifecycle FSM state. */
    car_modifier_safety_state_t safety_state; /**< Current collision-safety response. */
    /* A nonzero value reserves a conflict-free intersection entry. */
    uint8_t intersection_reserved; /**< Nonzero while this agent holds intersection entry reservation. */
    float buffer_zone_length_units; /**< Length of the approach buffer before the intersection. */
    float cruise_speed_units_per_second; /**< Target speed outside safety restrictions. */
    float safety_target_speed_units_per_second; /**< Speed target set by the safety pre-update. */
    float approach_distance_remaining; /**< Distance remaining to the stop/turn point. */
    /* Remaining straight-line distance through the intersection. */
    float crossing_distance_remaining; /**< Straight-line distance remaining through the intersection. */
    float turn_radius; /**< Radius used for left/right intersection turns. */
    car_maneuver_t maneuver; /**< Selected straight, left or right route. */
    /* Heading on the incoming road, preserved while the trajectory turns. */
    float approach_yaw_rad; /**< Heading on the incoming road before a turn starts. */
    car_modifier_environment_t *environment; /**< Shared world state used for sensing and spawn checks. */
};

/** Prepare a context to begin retryable spawn processing.
 * @param context Per-car context to initialize.
 * @param environment Shared environment used by the car.
 * @return true on success; false for invalid pointers or configuration.
 */
bool car_modifier_context_init_spawn(
    car_modifier_context_t *context,
    car_modifier_environment_t *environment);

/** Initialize a trajectory-only car without the route FSM.
 * @param context Per-car context to initialize.
 * @param initial_pose Starting pose.
 * @param speed_units_per_second Initial positive speed.
 * @return true on success; false for invalid arguments.
 */
bool car_modifier_context_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second);

/** Initialize a route approaching an intersection maneuver from any heading.
 * Turns capture a fixed pivot at the maneuver point; straight travel ignores turn_radius.
 * @param context Per-car context to initialize.
 * @param initial_pose Starting pose.
 * @param speed_units_per_second Initial speed.
 * @param distance_to_maneuver Straight approach distance.
 * @param maneuver Selected intersection maneuver.
 * @param turn_radius Radius used for a left or right turn.
 * @return true on success; false for invalid arguments or route geometry.
 */
bool car_modifier_route_init(
    car_modifier_context_t *context,
    const car_trajectory_pose_t *initial_pose,
    float speed_units_per_second,
    float distance_to_maneuver,
    car_maneuver_t maneuver,
    float turn_radius);

/** Custom-modifier callback that advances the car route FSM and instance pose.
 * On failure, neither context nor instance transform is changed.
 * @param instance Renderable car instance to synchronize.
 * @param dt_seconds Elapsed simulation time.
 * @param context Pointer to this car's car_modifier_context_t.
 * @return true on success; false for invalid state or update failure.
 */
bool car_modifier_update(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context);

/** First agent-update phase: sense traffic and calculate a safe target speed.
 * @param instance Renderable car instance being evaluated.
 * @param dt_seconds Elapsed simulation time.
 * @param context Pointer to this car's car_modifier_context_t.
 * @return true on success; false for invalid context or sensing failure.
 */
bool car_modifier_safety_update(
    geometry3d_instance_t *instance,
    float dt_seconds,
    void *context);

#ifdef __cplusplus
}
#endif

#endif /* CAR_MODIFIER_H */
