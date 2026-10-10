#include "matrix4.h"

#include <stddef.h>
#include <math.h>

void matrix4_set_identity(matrix4_t *matrix)
{
    if (matrix == NULL)
    {
        return;
    }

    for (size_t row = 0U; row < MATRIX4_SIZE; ++row)
    {
        for (size_t column = 0U;
             column < MATRIX4_SIZE;
             ++column)
        {
            matrix->m[row][column] =
                (row == column) ? 1.0f : 0.0f;
        }
    }
}

bool matrix4_multiply(
    matrix4_t *result,
    const matrix4_t *left,
    const matrix4_t *right)
{
    if (result == NULL ||
        left == NULL ||
        right == NULL)
    {
        return false;
    }

    matrix4_t temporary = {0};

    for (size_t row = 0U; row < MATRIX4_SIZE; ++row)
    {
        for (size_t column = 0U;
             column < MATRIX4_SIZE;
             ++column)
        {
            for (size_t k = 0U; k < MATRIX4_SIZE; ++k)
            {
                temporary.m[row][column] +=
                    left->m[row][k] *
                    right->m[k][column];
            }
        }
    }

    *result = temporary;

    return true;
}


void matrix4_set_rotation_x(matrix4_t *matrix, float angle_rad)
{
    if (matrix == NULL)
    {
        return;
    }

    const float c = cosf(angle_rad);
    const float s = sinf(angle_rad);

    matrix4_set_identity(matrix);

    matrix->m[1][1] = c;
    matrix->m[1][2] = -s;
    matrix->m[2][1] = s;
    matrix->m[2][2] = c;
}

void matrix4_set_rotation_y(matrix4_t *matrix, float angle_rad)
{
    if (matrix == NULL)
    {
        return;
    }

    const float c = cosf(angle_rad);
    const float s = sinf(angle_rad);

    matrix4_set_identity(matrix);

    matrix->m[0][0] = c;
    matrix->m[0][2] = s;
    matrix->m[2][0] = -s;
    matrix->m[2][2] = c;
}

void matrix4_set_rotation_z(matrix4_t *matrix, float angle_rad)
{
    if (matrix == NULL)
    {
        return;
    }

    const float c = cosf(angle_rad);
    const float s = sinf(angle_rad);

    matrix4_set_identity(matrix);

    matrix->m[0][0] = c;
    matrix->m[0][1] = -s;
    matrix->m[1][0] = s;
    matrix->m[1][1] = c;
}

void matrix4_set_scale(
    matrix4_t *matrix,
    float scale_x,
    float scale_y,
    float scale_z)
{
    if (matrix == NULL)
    {
        return;
    }

    matrix4_set_identity(matrix);

    matrix->m[0][0] = scale_x;
    matrix->m[1][1] = scale_y;
    matrix->m[2][2] = scale_z;
}

void matrix4_set_translation(
    matrix4_t *matrix,
    float translate_x,
    float translate_y,
    float translate_z)
{
    if (matrix == NULL)
    {
        return;
    }

    matrix4_set_identity(matrix);

    matrix->m[0][3] = translate_x;
    matrix->m[1][3] = translate_y;
    matrix->m[2][3] = translate_z;
}