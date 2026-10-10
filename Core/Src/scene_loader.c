#include <stddef.h>

#include "scene_loader.h"
#include "cube_model.h"
#include "display.h"
#include "modifier3d.h"

#define SCENE_LOADER_PI_RAD (3.14159265358979323846f)

#define SCENE_LOADER_NEAR_CUBE_INDEX 0U
#define SCENE_LOADER_FAR_CUBE_INDEX  1U

#define SCENE_LOADER_ATTACHMENT_COUNT 6U

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

static bool scene_loader_add_pulsating_scale_modifiers(
    geometry3d_instance_t *instance)
{
    if (instance == NULL)
    {
        return false;
    }

    const modifier3d_config_t scale_x =
    {
        .target = MODIFIER3D_TARGET_SCALE_X,
        .behavior = MODIFIER3D_BEHAVIOR_ABS_SIN,
        .min_value = 0.4f,
        .max_value = 0.8f,
        .speed = 1.8f
    };

    const modifier3d_config_t scale_y =
    {
        .target = MODIFIER3D_TARGET_SCALE_Y,
        .behavior = MODIFIER3D_BEHAVIOR_ABS_SIN,
        .min_value = 1.7333333f,
        .max_value = 3.4666667f,
        .speed = 1.8f
    };

    const modifier3d_config_t scale_z =
    {
        .target = MODIFIER3D_TARGET_SCALE_Z,
        .behavior = MODIFIER3D_BEHAVIOR_ABS_SIN,
        .min_value = 0.4f,
        .max_value = 0.8f,
        .speed = 1.8f
    };

    return modifier3d_add(instance, &scale_x) &&
           modifier3d_add(instance, &scale_y) &&
           modifier3d_add(instance, &scale_z);
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

bool scene_loader_load(
    scene3d_t *scene,
    camera3d_t *camera,
    projection_perspective_t *projection)
{
    if (scene == NULL ||
        camera == NULL ||
        projection == NULL)
    {
        return false;
    }

    /*
     * Start with a clean scene and a freshly initialized camera.
     */
    scene3d_init(scene);

    if (!camera3d_init(camera))
    {
        goto fail;
    }

    /*
     * Demonstration instance definitions.
     */
    const geometry3d_instance_t near_cube =
    {
        .model = &geometry3d_cube_model,
        .transform =
        {
            .position = { .x = -1.1f, .y = 0.0f, .z = -1.0f },
            .rotation_rad = { .x = 0.55f, .y = 0.65f, .z = 0.0f },
            .scale = { .x = 0.6f, .y = 2.6f, .z = 0.6f }
        }
    };

    const geometry3d_instance_t far_cube =
    {
        .model = &geometry3d_cube_model,
        .transform =
        {
            .position = { .x = 5.1f, .y = 0.15f, .z = 1.2f },
            .rotation_rad = { .x = 0.25f, .y = -0.4f, .z = 0.2f },
            .scale = { .x = 1.4f, .y = 1.4f, .z = 1.4f }
        }
    };

    if (!scene3d_add_instance(scene, &near_cube) ||
        !scene3d_add_instance(scene, &far_cube))
    {
        goto fail;
    }
    
    /*
     * Add six child cubes around the far cube.
     *
     * Root indices remain:
     *   0: near cube
     *   1: far cube
     *
     * Child indices are assigned as the instances are added.
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
                    .x = scene_loader_attachment_positions[i].x *
                        1.7f,

                    .y = scene_loader_attachment_positions[i].y *
                        1.7f,

                    .z = scene_loader_attachment_positions[i].z *
                        1.7f
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
                SCENE_LOADER_FAR_CUBE_INDEX))
        {
            goto fail;
        }

        /*
         * Configure the instance stored in the scene, not
         * the temporary local copy above.
         */
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
     * Configure modifiers on the instances stored in the scene.
     * scene3d_add_instance() copies its input instance.
     */
    geometry3d_instance_t *near_instance =
        scene3d_get_instance(
            scene,
            SCENE_LOADER_NEAR_CUBE_INDEX);

    geometry3d_instance_t *far_instance =
        scene3d_get_instance(
            scene,
            SCENE_LOADER_FAR_CUBE_INDEX);

    if (near_instance == NULL || far_instance == NULL)
    {
        goto fail;
    }

    if (!scene_loader_add_rotation_modifiers(
            near_instance,
            0.55f,
            0.85f,
            0.15f))
    {
        goto fail;
    }

    if (!scene_loader_add_rotation_modifiers(
            far_instance,
            -0.35f,
            0.55f,
            0.65f))
    {
        goto fail;
    }

    if (!scene_loader_add_pulsating_scale_modifiers(
            near_instance))
    {
        goto fail;
    }

    /*
     * Camera setup: preserve the successful look-at configuration.
     */
    camera->position.x = 9.0f;
    camera->position.y = 0.0f;
    camera->position.z = -5.0f;

    const geometry3d_vector3_t target = { .x = 5.1f, .y = 0.15f, .z = 1.2f };
    // {
    //     .x = -1.1f,
    //     .y = 0.0f,
    //     .z = -1.0f
    // };

    if (!camera3d_set_target(camera, &target))
    {
        goto fail;
    }

    /*
     * Perspective projection for the 84x48 display.
     */
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
    /*
     * Do not leave a partially configured scene behind.
     */
    scene3d_init(scene);
    (void)camera3d_init(camera);
    *projection = (projection_perspective_t){0};

    return false;
}