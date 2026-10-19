# 3D Engine Architecture

This document describes the 3D engine as it is implemented in the `car-agents-tdd` branch of **NUCLEO-3310**. It focuses on the current design, the reasons behind its main boundaries, and the extension points intended for future simulation features.

The engine is a compact, hand-written C implementation designed for a resource-constrained STM32 and an 84 × 48 monochrome display. It deliberately uses fixed-capacity data structures and avoids dynamic allocation in the scene and modifier paths.

> **Implementation status:** this document distinguishes existing behaviour from planned directions. Static scene instances and dynamic agents are stored in separate fixed-capacity arrays. The agent list currently contains the cars.

## 1. Design goals

The engine is intended to make the individual parts of a small 3D pipeline understandable and testable:

- Keep immutable mesh geometry separate from per-object state.
- Keep scene management, transforms, camera, projection, rasterization, and hardware display output in separate modules.
- Support multiple instances of the same model without copying its vertex and edge arrays.
- Animate ordinary transform parameters with reusable modifiers, while allowing complex behaviours through custom callbacks.
- Support parent-child transforms for articulated or assembled objects.
- Keep vehicle motion mathematics independent from scene rendering and the STM32 hardware.
- Prefer bounded arrays and explicit limits over dynamic allocation.
- Validate inputs and reject invalid state rather than silently propagating non-finite values.

The current scene is a demonstration environment: a ground plane, crossing streets, buildings, an animated central assembly with six attached cubes, and several moving cars.

## 2. Main modules and responsibilities

| Module | Responsibility |
| --- | --- |
| `geometry3d` | Mesh and instance data types, validation, and local transform-matrix construction |
| `matrix4` | 4 × 4 matrix operations and translation, scale, and rotation matrices |
| `scene3d` | Fixed-capacity instance storage, parent relationships, update traversal, and render traversal |
| `scene_loader` | Construction of the demo scene, its models, transforms, modifiers, cars, camera, and projection |
| `modifier3d` | Modifier configuration, per-instance modifier lists, and modifier updates |
| `camera3d` | Camera state, look-at mode, and world-to-camera view matrix |
| `camera_controller` | Applying the current input snapshot to the camera |
| `projection` | Orthographic and perspective projection from camera-space coordinates to screen coordinates |
| `renderer3d` | Transforming mesh vertices, near-plane clipping, depth rasterization, and visible edge drawing |
| `rasterizer3d` / `depth_buffer` | Low-level pixel/line/triangle rasterization and depth storage |
| `car_trajectory` | Portable constant-speed straight motion and circular quarter-turn mathematics |
| `car_modifier` | Vehicle route FSM with a traffic-check gate, safety-supervision FSM scaffold, spawn/respawn policy, and synchronization with a renderable instance |
| `app` | Top-level update/render scheduling and the display hardware boundary |

The application initializes the scene through `scene_loader_load()`; it does not need to know which models or demo objects are created. The loader owns scene composition, while `app` owns the long-lived runtime state required by the scene.

## 3. Meshes and instances

### 3.1 Mesh: immutable geometry

A `geometry3d_model_t` describes geometry, not an object's location or animation. It contains pointers and counts for:

- vertices (`geometry3d_vertex_t`);
- edges (`geometry3d_edge_t`), which identify pairs of vertex indices;
- triangles (`geometry3d_triangle_t`), which identify triples of vertex indices.

The arrays are referenced through `const` pointers. A model is therefore suitable for sharing between many instances and for storing in static/constant memory.

The engine validates model definitions before accepting them. Vertex indices in edges and triangles must be within the model's vertex count. The vertex count is limited by the 16-bit indices and, for rendering, by `RENDERER3D_MAX_VERTICES` (currently 64).

### 3.2 Instance: one use of a mesh

A `geometry3d_instance_t` references a model and owns its own:

- position;
- rotation in radians around X, Y, and Z;
- scale;
- fixed-capacity list of modifiers.

This separation is important. Five cars can all reference the same cube model while having different transforms and independent modifier contexts. The engine does not need five copies of the cube's vertex, edge, or triangle arrays.

The current car graphic is intentionally a scaled cube used as a temporary visual proxy. Vehicle motion is not coupled to the cube model: the trajectory and route state can later be paired with a more detailed mesh without redesigning the motion system.

### 3.3 Fixed capacities

The current limits are explicit:

- `SCENE3D_MAX_INSTANCES`: 64 instances in one scene;
- `RENDERER3D_MAX_VERTICES`: 64 vertices in a rendered model;
- `MODIFIER3D_MAX_PER_INSTANCE`: 9 modifiers per instance, corresponding to the nine scalar transform parameters.

These bounds make memory use predictable and avoid allocation/free failure modes on the microcontroller. Adding an instance or modifier beyond its capacity fails rather than writing beyond the allocated arrays.

## 4. Scene storage and hierarchy

### 4.1 The scene owns instances by value

A `scene3d_t` contains a fixed array of static scene instances (`instances[]`) with a parallel array of parent indices, plus a separate fixed array of dynamic agents (`agent_instances[]`). `scene3d_add_instance()` copies a regular instance into the first array; `scene3d_add_agent_instance()` copies an agent into the second. The mesh itself is not copied; each instance retains its pointer to the shared model. The combined number of static instances and agents is limited to `SCENE3D_MAX_INSTANCES`; the agent array has its own smaller `SCENE3D_MAX_AGENTS` bound to keep memory use predictable.

A parent index of `SCENE3D_NO_PARENT_INDEX` marks a root instance. The scene API allows a child to be attached to a parent or detached again. It rejects invalid indices, self-parenting, and relationships that would introduce a hierarchy cycle.

### 4.2 World transforms

Each instance stores a **local transform**. During rendering, `scene3d_build_world_transform()` walks from the instance to its ancestors, collects the chain in a fixed-size local array, and then combines transforms in root-to-child order.

The transform matrix built by `geometry3d_build_transform_matrix()` uses the order:

```text
M_local = T * Rz * Rx * Ry * S
```

With the project's column-vector convention, a child's world matrix is composed as:

```text
M_world(child) = M_world(parent) * M_local(child)
```

This allows an attached object to move and rotate with its parent while retaining its own local offset and animation. The central animated centerpiece and its six children demonstrate this relationship.

The hierarchy is checked during transform construction as well, so a malformed parent chain does not result in an unbounded traversal.

## 5. Modifiers: reusable transform animation and custom behaviour

A modifier belongs to one instance. The instance's `modifier3d_list_t` stores a fixed array of modifier records. Each record contains its configuration and its mutable runtime state, such as phase and direction.

### 5.1 Targets

Scalar modifiers can target any one of the nine transform parameters:

- position X, Y, or Z;
- rotation X, Y, or Z;
- scale X, Y, or Z.

Only one scalar modifier may own a given target on a particular instance. This avoids ambiguous updates in which two modifiers compete to write the same parameter.

Custom modifiers use `MODIFIER3D_TARGET_ALL`, because a custom callback may update several transform components together.

### 5.2 Built-in behaviours

The current built-in behaviours are:

- **Linear forward:** advances a parameter at a signed rate and wraps at a configured boundary.
- **Linear bounce:** advances a parameter and reverses direction at the configured boundaries.
- **Absolute sine:** animates between the configured minimum and maximum using `min + (max - min) * abs(sin(phase))`.
- **Custom:** calls a supplied function with the instance, elapsed time, and an opaque context pointer.

The `speed` field is measured in parameter units per second for the linear behaviours and in radians per second for the sine phase.

A zero-delta update is explicitly a no-op for the generic modifier system. This matters when the application needs to initialize a scene or inspect state without advancing ordinary animations.

### 5.3 Custom modifier ownership and lifetime

A custom modifier stores a callback and a `void *context`. The context is **caller-owned**: copying an instance copies the pointer, not the memory it points to. The context must remain valid for as long as any scene instance can invoke the callback.

This is why the vehicle contexts live in the long-lived `app_t` object rather than as local variables in `scene_loader_load()`. The loader creates instances that retain pointers to those contexts; the scene can use those pointers on every later update.

This is a deliberate, allocation-free ownership model. It is also a contract that must be respected whenever instances or custom modifiers are added.

## 6. Vehicle simulation

The vehicle system separates route decisions from the mathematics used to move along a path.

### 6.1 Trajectory mathematics

`car_trajectory` is a portable kinematic module. It does not know about scenes, rendering, STM32 peripherals, or random numbers.

Vehicles move in the world XZ plane. Y is preserved, yaw is rotation around world +Y, and yaw zero means forward along +Z. Positive yaw turns from +Z toward +X.

The trajectory supports:

- straight motion at a constant positive linear speed;
- a quarter-turn around a captured pivot;
- continuing straight after the turn completes, including applying leftover time from the same update.

During a turn, the trajectory stores the turn's start pose and pivot. It evaluates the current pose from that snapshot and the angular progress instead of repeatedly accumulating small position deltas. This avoids accumulated drift along the circular arc. Angular speed is derived from linear speed and turn radius, so the intended linear speed is preserved.

### 6.2 Route modifier and finite-state machine

`car_modifier` owns the vehicle's route behaviour and its connection to the scene. Each vehicle has a separate `car_modifier_context_t`, which includes its trajectory, route-FSM state, safety-FSM state, cruise and target speeds, approach distance, selected manoeuvre, original approach heading, turn radius, and a pointer to the shared environment.

The states are:

| State | Purpose |
| --- | --- |
| `SPAWN` | Retry a candidate route and pose until the spawn footprint is clear |
| `FREE` | Straight trajectory without route-manoeuvre handling |
| `APPROACH_TURN` | Travels straight until the selected manoeuvre point |
| `BUFFER_ZONE` | Approaches the intersection entry, checks traffic repeatedly, and waits when required |
| `CROSSING` | Occupies the intersection while turning or travelling straight through |
| `DEPARTING` | Continues along the outgoing street and checks whether the car has left the city |

For the demo's routed cars, the typical progression is:

```text
SPAWN -> APPROACH_TURN -> BUFFER_ZONE -> CROSSING -> DEPARTING
                                  |                       |
                                  +-- straight route -----+
                                                          |
                                              leaves city bounds
                                                          |
                                                        SPAWN
```

A vehicle enters `BUFFER_ZONE` before the manoeuvre point and checks traffic on every update while it approaches the stop line. The zone is sized from the configured maximum speed, a reaction interval, maximum braking deceleration, vehicle half-length, and a clearance margin. This gives a car enough room to brake smoothly if it must yield. Vehicles in the same approach are non-conflicting; for other approaches, the route-conflict table determines whether paths cross.

The relative approach is derived from the original incoming heading, stored in `approach_yaw_rad` so a vehicle's changing yaw during a turn cannot change its identity. “From the left/right” is relative to the candidate vehicle A. The conflict table below lists the other vehicle B's maneuver; `C` means the paths conflict and `—` means they do not. Vehicles from the same approach are always considered non-conflicting.

| A's maneuver | B approaches from | B right | B straight | B left |
| --- | --- | --- | --- | --- |
| Right | Left | — | C | — |
| Right | Opposite | — | — | C |
| Right | Right | — | — | — |
| Left | Left | — | C | C |
| Left | Opposite | C | C | C |
| Left | Right | — | C | C |
| Straight | Left | — | C | C |
| Straight | Opposite | — | — | C |
| Straight | Right | C | C | C |
| Any | Same approach | — | — | — |

Within `BUFFER_ZONE`, a conflicting vehicle already in `CROSSING` always has priority. For conflicting vehicles also waiting in buffer zones, the right-hand rule decides: a car yields to a conflicting car approaching from its right. If that rule does not resolve the pair, the lower index in the sequentially processed agent array proceeds first. Each car can build a small wait-for graph from the live agent contexts: an edge A → B means A must yield to B. If the intersection has neither a crossing vehicle nor an active reservation and a strongly connected group forms a wait cycle, the lowest-index member of that group may ignore blockers inside that group. It still yields to blockers outside the group, then reserves its entry as usual. Sequential updates make the other cars observe its reservation or `CROSSING` state before they can also enter.

The first check that grants entry inside the car's stopping distance sets the `uint8_t intersection_reserved` flag in its context. This reservation is not a separate FSM state and cannot be revoked by a later arrival with formal priority; other conflicting buffer-zone cars yield until the reserved car enters and clears the intersection. A car that must yield follows a braking speed profile and stops before the line; if traffic permits, it keeps or accelerates toward cruise speed. A stopped turning car waits for a positive simulation time step before beginning its turn, so turn angular speed is initialized from a positive linear speed. Turning vehicles remain in `CROSSING` until the trajectory completes its quarter-turn. Straight-through vehicles remain in `CROSSING` for a distance of `2 * street_half_width`, then enter `DEPARTING`. A car that leaves the configured city bounds receives a newly selected spawn pose and route.

A separate safety-supervision FSM (`CLEAR`, `FOLLOWING`, and `STOPPED`, with `YIELDING` reserved) checks for the nearest vehicle ahead within 40 units that is travelling in nearly the same direction and overlaps the current lane. It uses the cars' live transforms and projected rectangular footprints; agents in `SPAWN` and crossing traffic are ignored. A vehicle waiting in `BUFFER_ZONE` counts as stopped for following traffic. The controller aims to preserve a 1.5-unit standstill gap and a 1.5-second time headway, limits acceleration to 4 units/s² and braking to 14 units/s², and caps its speed at its route's cruise speed. Its target speed accounts for both the following gap and the braking distance; `FOLLOWING` brakes or accelerates while route progress continues, and `STOPPED` is entered once the vehicle has halted behind a lead. When the lane ahead clears, the car accelerates back toward cruise speed. The pre-update phase only computes the target; movement and speed changes happen later in the normal agent update phase.

The spawn state is processed even when `dt_seconds == 0`, allowing the loader to initialize the car's transform before the first rendered frame. When a candidate is blocked, the route state and transform remain unchanged and the shared random state advances so the next frame can try another location. When a car leaves the city and respawns during an update, the newly selected route does not consume the leftover time from that update. This avoids immediately advancing the new route by a time interval that belonged to the previous one.

### 6.3 Shared environment, independent car contexts

The `car_modifier_environment_t` holds shared world parameters:

- city half-extents on X and Z;
- street half-width and lane offset;
- minimum and maximum vehicle speeds;
- the shared pseudorandom-generator state.

All vehicles point to the same environment. Each vehicle nevertheless has its own context and trajectory, so its position, route, FSM state, and turning progress are independent.

The random generator is shared so that each spawn advances one common deterministic sequence rather than maintaining a separate random stream per vehicle. The environment's lifetime must outlast every vehicle context that references it.

The application configures the car count through `APP_CAR_COUNT`. The loader loops over the requested count and initializes a distinct context and agent instance for each car. The count must fit the application's context array and the scene's overall instance capacity.

### 6.4 Transactional update intent

The route modifier works on a copy of the car context while calculating an update, validates the resulting pose, and only then commits the context and instance transform. Spawn-related random state is also committed when the update succeeds. This is intended to prevent an invalid calculation from leaving half-updated vehicle state behind.

## 7. Static scenery and dynamic vehicles

### 7.1 Current implementation

The demo scene contains several logical categories:

- ground and crossing street planes;
- city-building instances;
- a central animated centerpiece and its attached children;
- moving vehicle instances.

Static buildings have no modifiers. The centerpiece and its children use scalar rotation modifiers. Each car uses a custom modifier linked to its own route context.

Static scenery and non-car animations occupy `scene3d_t.instances[]`. Cars occupy `scene3d_t.agent_instances[]`, making the dynamic-agent collection directly available for simulation queries. Both collections are updated by `scene3d_update()` and included in both render passes. Parent-child transforms currently apply to regular scene instances; agents are root instances with their own world transforms.

### Car spawn and footprint checks

The route FSM's `SPAWN` state can persist across multiple updates. Each update samples a new spawn candidate. If its oriented XZ footprint overlaps an active car, the attempt consumes its random values, leaves the car's route state and transform unchanged, and retries on a later update. A successful attempt initializes the route and places the car immediately, but does not consume remaining frame time. Cars are processed sequentially, so a car spawned earlier in the agent list is visible to later spawn attempts in the same scene update.

The shared car environment points at the live agent instance array, its parallel car-context array, and the live agent count. Spawn checks skip the current car and agents whose route state is `SPAWN`; a waiting car's previous transform therefore does not reserve a spawn location. `car_footprint` provides the geometry-only oriented rectangle intersection test (SAT) in the XZ plane. Footprint dimensions match the current cube proxy, with a small spawn clearance. This separates world geometry from route lifecycle decisions, which remain in `car_modifier`.

### 7.2 Why separate static and interactive lists may be useful later

The separate agent collection supports simulation queries such as:

- **static world objects**, such as buildings and road geometry;
- **interactive agents**, such as cars that can sense and react to other cars.

Vehicle awareness should not need to scan every object that happens to be rendered. For example, when a car searches for nearby traffic, buildings and ground planes are generally not candidate *vehicles*. A dedicated agent collection would make that query straightforward and communicate the intended interaction domain in the data model.

Vehicle awareness can scan the agent list without considering ground, roads, or buildings, while rendering still sees every renderable instance in the scene. The current five-car demonstration can use a direct scan; a spatial index may become useful if the number of agents grows significantly.

Separate lists alone will not solve every scaling problem. If the number of agents grows significantly, a spatial index (for example, a uniform grid) or another broad-phase query structure may be needed to avoid checking every vehicle against every other vehicle. That optimization is not required for the current five-car demonstration.

## 8. Update and render pipeline

The application separates simulation updates from the slower display refresh.

### 8.1 Application update

`app_run()` validates its arguments, updates camera control from the current input snapshot, and advances the scene when `dt_seconds > 0`. It accumulates elapsed time for rendering and schedules a render when the accumulator reaches the configured interval, currently `1 / 30` second.

Thus, simulation and input handling are not tied to every LCD refresh. The display is small and comparatively slow, while the scene's state can continue to advance between rendered frames.

### 8.2 Scene update

`scene3d_update()` updates regular scene instances once through `modifier3d_update_instance()`. Agents then run in two phases, both restricted to `agent_instances[]`:

1. The pre-update phase invokes optional custom callbacks such as the car safety controller. All agents complete this phase before any agent's route callback runs, so sensing reads a consistent set of pre-movement positions without copying agent state.
2. The normal update phase advances each agent's modifiers and route FSM.

Instances without modifiers remain unchanged. Ordinary scalar modifiers update their target transform parameters; custom modifiers invoke their callback, such as `car_modifier_update()` for a vehicle. Static instances do not participate in the agent pre-update phase.

This generic traversal means the scene does not need to understand the internals of a car FSM or the mathematical details of a rotation modifier.

### 8.3 Render passes

`scene3d_render()` clears the depth buffer and renders in two passes:

1. **Depth/rasterization pass:** for each regular instance and agent, build its world transform and rasterize its triangles into the depth buffer.
2. **Edge pass:** for each regular instance and agent, build the same world transform and draw its edges, using the depth buffer to suppress edges hidden behind nearer surfaces.

For every instance, the renderer combines the world transform with the camera view matrix, transforms the model vertices into camera space, clips line segments crossing the near plane, projects visible points, and clips projected lines to the 84 × 48 canvas. Canvas clipping uses the Liang–Barsky line-clipping algorithm.

The plane meshes use triangles to contribute depth even though their visible representation is primarily their perimeter edges. The depth pass is therefore useful even for this wireframe-style display.

### 8.4 Camera and perspective projection

The camera can aim at a world-space target (look-at mode) or use a manually controlled orientation. The camera module builds a world-to-camera view matrix. The controller is a separate module that applies the application's input snapshot to camera state.

The current demo uses perspective projection. In camera space, projected screen coordinates follow the form:

```text
screen_x = center_x + focal_length * x / z
screen_y = center_y - focal_length * y / z
```

The near plane prevents points too close to or behind the camera from being projected. The projection module also provides orthographic projection for other uses.

## 9. Scene composition

`scene_loader_load()` constructs the current demonstration scene in a defined sequence:

1. Initialize the scene and camera.
2. Add the ground plane and the two crossing street planes.
3. Add the static buildings around the central streets.
4. Add the central cube assembly and its three-axis rotation modifiers.
5. Add six child cubes, attach them to the centerpiece, and configure their own rotation modifiers.
6. Add the configured number of cars to the agent list, each with its own persistent context and custom modifier.
7. Configure the initial camera view and perspective projection.

The loader fails if any part of the scene cannot be configured. On failure it resets the scene and camera and clears the projection structure, rather than returning a partially loaded scene as a success.

## 10. Current constraints and extension points

The current design intentionally keeps the simulation small and explicit. Important constraints to keep in mind:

- Scene instances, agents, modifiers, and model vertex scratch storage have fixed limits.
- The scene has separate static-instance and agent arrays, with a shared total capacity. Agent instances are roots and do not currently participate in parent-child hierarchies.
- Cars are independent agents with basic same-lane following and intersection conflict handling; full footprint-based collision avoidance across arbitrary trajectories is not implemented.
- There is no broad-phase spatial index for agent queries.
- Vehicle visuals are placeholder cubes, independent of the trajectory implementation.
- The depth/edge pipeline is designed for a tiny monochrome display, not a general-purpose GPU.

Natural next steps include a richer vehicle mesh, sensors and awareness queries, interaction rules, collision handling, and possibly spatial partitioning if the number of agents grows. These should build on the existing boundaries: trajectory mathematics stays portable, the route FSM owns vehicle decisions, custom modifiers bridge behaviour to scene instances, and the renderer remains concerned with visibility and pixels.

## 11. Source map

The principal implementation files are:

- `Core/Inc/geometry3d.h`, `Core/Src/geometry3d.c`
- `Core/Inc/scene3d.h`, `Core/Src/scene3d.c`
- `Core/Inc/scene_loader.h`, `Core/Src/scene_loader.c`
- `Core/Inc/modifier3d_types.h`, `Core/Inc/modifier3d.h`, `Core/Src/modifier3d.c`
- `Core/Inc/camera3d.h`, `Core/Src/camera3d.c`
- `Core/Inc/projection.h`, `Core/Src/projection.c`
- `Core/Inc/renderer3d.h`, `Core/Src/renderer3d.c`
- `Core/Inc/car_trajectory.h`, `Core/Src/car_trajectory.c`
- `Core/Inc/car_modifier.h`, `Core/Src/car_modifier.c`
- `Core/Inc/app.h`, `Core/Src/app.c`

When implementation details change, this document should be updated alongside the relevant code. It is intended to explain the architecture at the current milestone, not to prescribe behaviour that has not yet been implemented.
