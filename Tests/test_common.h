#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <stdio.h>

#define CHECK(condition)                                                    \
    do                                                                      \
    {                                                                       \
        if (!(condition))                                                   \
        {                                                                   \
            printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #condition);    \
            return false;                                                   \
        }                                                                   \
    } while (0)

#endif /* TEST_COMMON_H */