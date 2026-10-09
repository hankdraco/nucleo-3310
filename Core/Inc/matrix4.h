
#ifndef MATRIX4_H
#define MATRIX4_H

#include <stdbool.h>

#define MATRIX4_SIZE 4U

typedef struct
{
    float m[MATRIX4_SIZE][MATRIX4_SIZE];
} matrix4_t;

/* Set the matrix to the identity matrix. */
void matrix4_set_identity(matrix4_t *matrix);

/*
 * Calculate result = left * right.
 *
 * The function supports result aliasing either input.
 */
bool matrix4_multiply(
    matrix4_t *result,
    const matrix4_t *left,
    const matrix4_t *right);

void matrix4_set_rotation_x(matrix4_t *matrix, float angle_rad);
void matrix4_set_rotation_y(matrix4_t *matrix, float angle_rad);
void matrix4_set_rotation_z(matrix4_t *matrix, float angle_rad);

#endif /* MATRIX4_H */