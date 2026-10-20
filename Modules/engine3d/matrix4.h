#ifndef MATRIX4_H
#define MATRIX4_H

#include <stdbool.h>

#define MATRIX4_SIZE 4U /**< Number of rows and columns in a homogeneous transform matrix. */

/** Row-major 4x4 matrix used for affine and camera transformations. */
typedef struct
{
    float m[MATRIX4_SIZE][MATRIX4_SIZE]; /**< Matrix coefficients indexed by row, then column. */
} matrix4_t;

/** Replace a matrix with the identity matrix.
 * @param matrix Matrix to initialize.
 */
void matrix4_set_identity(matrix4_t *matrix);

/**
 * @brief Calculate result = left * right. Result may alias either input.
 * @param result Destination matrix.
 * @param left Left operand.
 * @param right Right operand.
 * @return true on success; false for invalid pointers or non-finite input.
 */
bool matrix4_multiply(
    matrix4_t *result,
    const matrix4_t *left,
    const matrix4_t *right);

/** Set a rotation matrix around the X axis.
 * @param matrix Destination matrix.
 * @param angle_rad Rotation angle in radians.
 */
void matrix4_set_rotation_x(matrix4_t *matrix, float angle_rad);
/** Set a rotation matrix around the Y axis.
 * @param matrix Destination matrix.
 * @param angle_rad Rotation angle in radians.
 */
void matrix4_set_rotation_y(matrix4_t *matrix, float angle_rad);
/** Set a rotation matrix around the Z axis.
 * @param matrix Destination matrix.
 * @param angle_rad Rotation angle in radians.
 */
void matrix4_set_rotation_z(matrix4_t *matrix, float angle_rad);

/** Set a nonuniform scale matrix.
 * @param matrix Destination matrix.
 * @param scale_x Scale on X.
 * @param scale_y Scale on Y.
 * @param scale_z Scale on Z.
 */
void matrix4_set_scale(
    matrix4_t *matrix,
    float scale_x,
    float scale_y,
    float scale_z);

/** Set a translation matrix.
 * @param matrix Destination matrix.
 * @param translate_x Translation on X.
 * @param translate_y Translation on Y.
 * @param translate_z Translation on Z.
 */
void matrix4_set_translation(
    matrix4_t *matrix,
    float translate_x,
    float translate_y,
    float translate_z);

#endif /* MATRIX4_H */
