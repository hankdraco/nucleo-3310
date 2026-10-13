#ifndef INPUT_TYPES_H
#define INPUT_TYPES_H

#include <stdint.h>

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