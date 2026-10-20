#ifndef INPUT_TYPES_H
#define INPUT_TYPES_H

#include <stdint.h>

/*
 * Semantic input key mask.
 *
 * These bits describe actions, not physical
 * keyboard scan codes or ASCII characters.
 */
#define INPUT_KEY_FORWARD       UINT32_C(0x00000001)
#define INPUT_KEY_BACKWARD      UINT32_C(0x00000002)
#define INPUT_KEY_STRAFE_LEFT   UINT32_C(0x00000004)
#define INPUT_KEY_STRAFE_RIGHT  UINT32_C(0x00000008)
#define INPUT_KEY_MOVE_UP       UINT32_C(0x00000010)
#define INPUT_KEY_MOVE_DOWN     UINT32_C(0x00000020)

/* A press-edge cycles through car-follow, demo, and free-fly camera modes. */
#define INPUT_KEY_CAMERA_MODE UINT32_C(0x00000040)

/*
 * Dane z jednej poprawnej ramki protokołu.
 *
 * mouse_dx / mouse_dy oznaczają przyrost ruchu
 * myszy od poprzedniej transmisji.
 */
typedef struct
{
    uint32_t key_mask;
    int32_t mouse_dx;
    int32_t mouse_dy;

} input_frame_t;

/*
 * Snapshot wejścia pobierany przez główną pętlę.
 *
 * Przyrosty myszy mogą być sumą wielu ramek.
 */
typedef struct
{
    uint32_t key_mask;
    int32_t mouse_dx;
    int32_t mouse_dy;

} input_snapshot_t;

#endif /* INPUT_TYPES_H */
