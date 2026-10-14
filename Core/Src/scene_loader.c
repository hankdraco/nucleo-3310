#include <stddef.h>

#include "scene_loader.h"
#include "cube_model.h"
#include "car_modifier.h"
#include "display.h"
#include "modifier3d.h"
#include "city_models.h"

#define SCENE_LOADER_PI_RAD (3.14159265358979323846f)

#define SCENE_LOADER_CITY_SCALE         (8.0f)
#define SCENE_LOADER_CITY_HALF_EXTENT 20.0f

#define SCENE_LOADER_BUILDING_COLUMNS 4U
#define SCENE_LOADER_BUILDING_ROWS    3U
#define SCENE_LOADER_CITY_QUADRANTS   4U

#define SCENE_LOADER_ATTACHMENT_COUNT   6U

#define SCENE_LOADER_CAR_SPEED_UNITS_PER_SECOND (3.0f)
#define SCENE_LOADER_CAR_TURN_RADIUS (3.0f)
#define SCENE_LOADER_CAR_SPAWN_Z (-40.0f)

static const geometry3d_vector3_t
    scene_loader_attachment_positions[
        SCENE_LOADER_ATTACHMENT_COUNT] =
{
    /* Child on the +X face. */
    { .x =  1.15f, .y =  0.0f, .z =  0.0f },

    /* Child on the -X face. */
    { .x = -1.15f, .y =  0.0f, .z =  0.0f },

    /* Child on the +Y face. */
    { .x =  0.0f, .y =  1.15f, .z =  0.0f },

    /* Child on the -Y face. */
    { .x =  0.0f, .y = -1.15f, .z =  0.0f },

    /* Child on the +Z face. */
    { .x =  0.0f, .y =  0.0f, .z =  1.15f },

    /* Child on the -Z face. */
    { .x =  0.0f, .y =  0.0f, .z = -1.15f }
};

static const modifier3d_target_t
    scene_loader_attachment_rotation_axes[
        SCENE_LOADER_ATTACHMENT_COUNT] =
{
    MODIFIER3D_TARGET_ROTATION_X,
    MODIFIER3D_TARGET_ROTATION_X,
    MODIFIER3D_TARGET_ROTATION_Y,
    MODIFIER3D_TARGET_ROTATION_Y,
    MODIFIER3D_TARGET_ROTATION_Z,
    MODIFIER3D_TARGET_ROTATION_Z
};

static const float
    scene_loader_attachment_rotation_speeds[
        SCENE_LOADER_ATTACHMENT_COUNT] =
{
     0.60f,
    -0.45f,
     0.55f,
    -0.70f,
     0.65f,
    -0.50f
};

static bool scene_loader_add_rotation_modifiers(
    geometry3d_instance_t *instance,
    float speed_x,
    float speed_y,
    float speed_z)
{
    if (instance == NULL)
    {
        return false;
    }

    const modifier3d_config_t rotation_x =
    {
        .target = MODIFIER3D_TARGET_ROTATION_X,
        .behavior = MODIFIER3D_BEHAVIOR_LINEAR_FORWARD,
        .min_value = -SCENE_LOADER_PI_RAD,
        .max_value = SCENE_LOADER_PI_RAD,
        .speed = speed_x
    };

    const modifier3d_config_t rotation_y =
    {
        .target = MODIFIER3D_TARGET_ROTATION_Y,
        .behavior = MODIFIER3D_BEHAVIOR_LINEAR_FORWARD,
        .min_value = -SCENE_LOADER_PI_RAD,
        .max_value = SCENE_LOADER_PI_RAD,
        .speed = speed_y
    };

    const modifier3d_config_t rotation_z =
    {
        .target = MODIFIER3D_TARGET_ROTATION_Z,
        .behavior = MODIFIER3D_BEHAVIOR_LINEAR_FORWARD,
        .min_value = -SCENE_LOADER_PI_RAD,
        .max_value = SCENE_LOADER_PI_RAD,
        .speed = speed_z
    };

    return modifier3d_add(instance, &rotation_x) &&
           modifier3d_add(instance, &rotation_y) &&
           modifier3d_add(instance, &rotation_z);
}

static bool scene_loader_add_rotation_modifier(
    geometry3d_instance_t *instance,
    modifier3d_target_t target,
    float speed)
{
    if (instance == NULL)
    {
        return false;
    }

    const modifier3d_config_t config =
    {
        .target = target,
        .behavior = MODIFIER3D_BEHAVIOR_LINEAR_FORWARD,
        .min_value = -SCENE_LOADER_PI_RAD,
        .max_value = SCENE_LOADER_PI_RAD,
        .speed = speed
    };

    return modifier3d_add(instance, &config);
}

static const float city_building_x_positions[
    SCENE_LOADER_BUILDING_COLUMNS] =
{
    4.4f, 8.4f, 12.4f, 16.4f
};

static const float city_building_z_positions[
    SCENE_LOADER_BUILDING_ROWS] =
{
    4.4f, 10.5f, 16.3f
};

/* Half-widths and half-depths, since cube vertices span [-1, +1]. */
static const float city_building_half_widths[
    SCENE_LOADER_BUILDING_COLUMNS] =
{
    0.75f, 0.95f, 1.15f, 0.85f
};

static const float city_building_half_depths[
    SCENE_LOADER_BUILDING_ROWS] =
{
    0.80f, 1.00f, 0.75f
};

/*
 * Each value is a building's half-height.
 * Quadrants:
 *   0: +X, +Z
 *   1: -X, +Z
 *   2: +X, -Z
 *   3: -X, -Z
 *
 * Entries are row-major: row * 4 + column.
 */
static const float city_building_half_heights[4][12] =
{
    {
        0.85f, 1.40f, 0.95f, 2.20f,
        1.25f, 1.80f, 1.40f, 2.60f,
        2.20f, 0.80f, 1.65f, 2.40f
    },
    {
        1.70f, 0.80f, 2.30f, 1.20f,
        2.60f, 1.10f, 1.40f, 1.90f,
        1.00f, 2.40f, 0.75f, 2.20f
    },
    {
        1.00f, 2.10f, 1.40f, 2.60f,
        0.80f, 2.50f, 1.10f, 1.70f,
        1.90f, 1.20f, 2.90f, 0.90f
    },
    {
        2.50f, 1.00f, 1.70f, 1.30f,
        1.50f, 2.90f, 0.90f, 2.10f,
        1.20f, 1.80f, 2.40f, 0.80f
    }
};

static bool scene_loader_add_city_buildings(scene3d_t *scene)
{
    if (scene == NULL)
    {
        return false;
    }

    for (size_t quadrant = 0U;
         quadrant < SCENE_LOADER_CITY_QUADRANTS;
         ++quadrant)
    {
        const float sign_x =
            ((quadrant & 1U) != 0U) ? -1.0f : 1.0f;

        const float sign_z =
            ((quadrant & 2U) != 0U) ? -1.0f : 1.0f;

        for (size_t row = 0U;
             row < SCENE_LOADER_BUILDING_ROWS;
             ++row)
        {
            for (size_t column = 0U;
                 column < SCENE_LOADER_BUILDING_COLUMNS;
                 ++column)
            {
                /*
                 * Leave two central positions in the middle row
                 * empty in every quadrant, creating wider open areas.
                 */
                if (row == 1U &&
                    (column == 1U || column == 2U))
                {
                    continue;
                }

                const size_t height_index =
                    row * SCENE_LOADER_BUILDING_COLUMNS + column;

                const float half_height =
                    city_building_half_heights[quadrant][height_index];

                const geometry3d_instance_t building =
                {
                    .model = &geometry3d_cube_model,
                    .transform =
                    {
                        .position =
                        {
                            .x = sign_x *
                                city_building_x_positions[column] *
                                SCENE_LOADER_CITY_SCALE,

                            .y = half_height *
                                SCENE_LOADER_CITY_SCALE,

                            .z = sign_z *
                                city_building_z_positions[row] *
                                SCENE_LOADER_CITY_SCALE
                        },

                        .rotation_rad =
                        {
                            .x = 0.0f,
                            .y = 0.0f,
                            .z = 0.0f
                        },

                        .scale =
                        {
                            .x = city_building_half_widths[column] *
                                SCENE_LOADER_CITY_SCALE,

                            .y = half_height *
                                SCENE_LOADER_CITY_SCALE,

                            .z = city_building_half_depths[row] *
                                SCENE_LOADER_CITY_SCALE
                        }
                    }
                };

                if (!scene3d_add_instance(scene, &building))
                {
                    return false;
                }
            }
        }
    }

    return true;
}


/*
 * Add one car instance to the central north-south street.
 *
 * The behavior context belongs to the caller (app_t), not this function's
 * stack. scene3d_add_instance() copies the instance and its context pointer,
 * so the caller-owned context must remain alive while the scene is updated.
 */
static bool scene_loader_add_car(
    scene3d_t *scene,
    car_modifier_context_t *car_context)
{
    if (scene == NULL || car_context == NULL)
    {
        return false;
    }

    const car_trajectory_pose_t initial_pose =
    {
        .position =
        {
            .x = 0.0f,
            .y = 1.0f,
            .z = SCENE_LOADER_CAR_SPAWN_Z
        },
        .yaw_rad = 0.0f
    };

    /*
     * Approach from the -Z outskirts. The pivot is three units to the
     * right of the lane and at the turn-start Z coordinate.
     */
    const car_trajectory_vec3_t turn_pivot =
    {
        .x = SCENE_LOADER_CAR_TURN_RADIUS,
        .y = 0.0f,
        .z = -SCENE_LOADER_CAR_TURN_RADIUS
    };

    if (!car_modifier_route_init(
            car_context,
            &initial_pose,
            SCENE_LOADER_CAR_SPEED_UNITS_PER_SECOND,
            &turn_pivot,
            CAR_TURN_DIRECTION_RIGHT))
    {
        return false;
    }

    /*
     * A small cube is a temporary visual proxy for the car. The trajectory
     * and custom modifier do not depend on this particular model.
     */
    geometry3d_instance_t car =
    {
        .model = &geometry3d_cube_model,
        .transform =
        {
            .position =
            {
                .x = initial_pose.position.x,
                .y = initial_pose.position.y,
                .z = initial_pose.position.z
            },
            .rotation_rad =
            {
                .x = 0.0f,
                .y = initial_pose.yaw_rad,
                .z = 0.0f
            },
            .scale =
            {
                .x = 1.25f,
                .y = 1.00f,
                .z = 2.25f
            }
        }
    };

    const modifier3d_config_t car_behavior =
    {
        .target = MODIFIER3D_TARGET_ALL,
        .behavior = MODIFIER3D_BEHAVIOR_CUSTOM,
        .custom_fn = car_modifier_update,
        .context = car_context
    };

    if (!modifier3d_add(&car, &car_behavior))
    {
        return false;
    }

    return scene3d_add_instance(scene, &car);
}

bool scene_loader_load(
    scene3d_t *scene,
    camera3d_t *camera,
    projection_perspective_t *projection,
    car_modifier_context_t *car_context)
{
    if (scene == NULL ||
        camera == NULL ||
        projection == NULL ||
        car_context == NULL)
    {
        return false;
    }

    scene3d_init(scene);

    if (!camera3d_init(camera))
    {
        goto fail;
    }

    /*
     * 1. Ground: 40 x 40 world units.
     * Its triangles contribute depth; only its outline is drawn.
     */
    const geometry3d_instance_t ground =
    {
        .model = &geometry3d_plane_model,
        .transform =
        {
            .position = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
            .rotation_rad = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
            .scale =
            {
                .x = SCENE_LOADER_CITY_HALF_EXTENT *
                    SCENE_LOADER_CITY_SCALE,
                .y = 1.0f,
                .z = SCENE_LOADER_CITY_HALF_EXTENT *
                    SCENE_LOADER_CITY_SCALE
            }
        }
    };

    /*
     * 2. Two crossing streets.
     * The slightly elevated outlines remain distinct from the ground.
     */
    const geometry3d_instance_t street_x =
    {
        .model = &geometry3d_plane_model,
        .transform =
        {
            .position = { .x = 0.0f, .y = 0.02f, .z = 0.0f },
            .rotation_rad = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
            .scale =
            {
                .x = SCENE_LOADER_CITY_HALF_EXTENT *
                    SCENE_LOADER_CITY_SCALE,
                .y = 1.0f,
                .z = 1.6f * SCENE_LOADER_CITY_SCALE
            }
        }
    };

    const geometry3d_instance_t street_z =
    {
        .model = &geometry3d_plane_model,
        .transform =
        {
            .position = { .x = 0.0f, .y = 0.025f, .z = 0.0f },
            .rotation_rad = { .x = 0.0f, .y = 0.0f, .z = 0.0f },
            .scale =
            {
                .x = 1.6f * SCENE_LOADER_CITY_SCALE,
                .y = 1.0f,
                .z = SCENE_LOADER_CITY_HALF_EXTENT *
                    SCENE_LOADER_CITY_SCALE
            }
        }
    };

    if (!scene3d_add_instance(scene, &ground) ||
        !scene3d_add_instance(scene, &street_x) ||
        !scene3d_add_instance(scene, &street_z))
    {
        goto fail;
    }

    /*
     * 3. Static city buildings.
     * These instances have no modifiers.
     */
    if (!scene_loader_add_city_buildings(scene))
    {
        goto fail;
    }

    /*
     * 4. Central animated centerpiece.
     *
     * It retains the existing three-axis rotation and all six children.
     * The elevated origin keeps the rotating assembly above the ground
     * for every orientation, including children carried underneath it.
     */
    const geometry3d_instance_t centerpiece =
    {
        .model = &geometry3d_cube_model,
        .transform =
        {
            .position =
            {
                .x = 0.0f,
                .y = 3.5f,
                .z = 0.0f
            },
            .rotation_rad =
            {
                .x = 0.25f,
                .y = -0.4f,
                .z = 0.2f
            },
            .scale =
            {
                .x = 1.4f,
                .y = 1.4f,
                .z = 1.4f
            }
        }
    };

    const size_t centerpiece_index = scene->instance_count;

    if (!scene3d_add_instance(scene, &centerpiece))
    {
        goto fail;
    }

    geometry3d_instance_t *centerpiece_instance =
        scene3d_get_instance(scene, centerpiece_index);

    if (centerpiece_instance == NULL)
    {
        goto fail;
    }

    if (!scene_loader_add_rotation_modifiers(
            centerpiece_instance,
            -0.35f,
            0.55f,
            0.65f))
    {
        goto fail;
    }

    /*
     * 5. Six animated children around the centerpiece.
     */
    for (size_t i = 0U;
         i < SCENE_LOADER_ATTACHMENT_COUNT;
         ++i)
    {
        const geometry3d_instance_t child =
        {
            .model = &geometry3d_cube_model,
            .transform =
            {
                .position =
                {
                    .x = scene_loader_attachment_positions[i].x * 1.7f,
                    .y = scene_loader_attachment_positions[i].y * 1.7f,
                    .z = scene_loader_attachment_positions[i].z * 1.7f
                },
                .rotation_rad =
                {
                    .x = 0.0f,
                    .y = 0.0f,
                    .z = 0.0f
                },
                .scale =
                {
                    .x = 0.12f * 2.5f,
                    .y = 0.12f * 2.5f,
                    .z = 0.12f * 2.5f
                }
            }
        };

        const size_t child_index = scene->instance_count;

        if (!scene3d_add_instance(scene, &child))
        {
            goto fail;
        }

        if (!scene3d_set_parent(
                scene,
                child_index,
                centerpiece_index))
        {
            goto fail;
        }

        geometry3d_instance_t *child_instance =
            scene3d_get_instance(scene, child_index);

        if (child_instance == NULL)
        {
            goto fail;
        }

        if (!scene_loader_add_rotation_modifier(
                child_instance,
                scene_loader_attachment_rotation_axes[i],
                scene_loader_attachment_rotation_speeds[i]))
        {
            goto fail;
        }
    }

    /*
     * 6. One moving car on the central north-south street.
     */
    if (!scene_loader_add_car(scene, car_context))
    {
        goto fail;
    }

    /*
     * 7. Elevated oblique view of the city and central intersection.
     */
    // camera->position.x = 36.0f;
    // camera->position.y = 52.0f;
    // camera->position.z = -54.0f;
    camera->position.x = 8.0f;
    camera->position.y = 2.0f;
    camera->position.z = -6.0f;

    const geometry3d_vector3_t target =
    {
        .x = 0.0f,
        .y = 2.4f,
        .z = 0.0f
    };

    if (!camera3d_set_target(camera, &target))
    {
        goto fail;
    }

    const projection_perspective_t loaded_projection =
    {
        .focal_length = 48.0f,
        .near_plane = 0.1f,
        .center_x = (float)(DISPLAY_WIDTH / 2U),
        .center_y = (float)(DISPLAY_HEIGHT / 2U)
    };

    *projection = loaded_projection;

    return true;

fail:
    scene3d_init(scene);
    (void)camera3d_init(camera);
    *projection = (projection_perspective_t){0};

    return false;
}