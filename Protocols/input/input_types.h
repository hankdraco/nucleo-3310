#ifndef INPUT_TYPES_H
#define INPUT_TYPES_H

#include <stdint.h>

/**
 * Semantic input key mask.
 *
 * These bits describe actions, not physical
 * keyboard scan codes or ASCII characters.
 */
#define INPUT_KEY_FORWARD       UINT32_C(0x00000001) /**< Move forward. */
#define INPUT_KEY_BACKWARD      UINT32_C(0x00000002) /**< Move backward. */
#define INPUT_KEY_STRAFE_LEFT   UINT32_C(0x00000004) /**< Move left without changing heading. */
#define INPUT_KEY_STRAFE_RIGHT  UINT32_C(0x00000008) /**< Move right without changing heading. */
#define INPUT_KEY_MOVE_UP       UINT32_C(0x00000010) /**< Move vertically upward. */
#define INPUT_KEY_MOVE_DOWN     UINT32_C(0x00000020) /**< Move vertically downward. */

/** Press edge cycles through car-follow, demo and free-fly camera modes. */
#define INPUT_KEY_CAMERA_MODE UINT32_C(0x00000040)

/** Decoded input from one valid protocol frame; mouse deltas are since the prior frame. */
typedef struct
{
    uint32_t key_mask; /**< Semantic key-action bitmask. */
    int32_t mouse_dx; /**< Horizontal mouse displacement since the previous frame. */
    int32_t mouse_dy; /**< Vertical mouse displacement since the previous frame. */

} input_frame_t;

/** Input state consumed by the main loop; mouse deltas may aggregate multiple frames. */
typedef struct
{
    uint32_t key_mask; /**< Currently held semantic key actions. */
    int32_t mouse_dx; /**< Accumulated horizontal displacement since last consume. */
    int32_t mouse_dy; /**< Accumulated vertical displacement since last consume. */

} input_snapshot_t;

#endif /* INPUT_TYPES_H */
