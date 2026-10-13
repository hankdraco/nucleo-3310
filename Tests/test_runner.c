#include <stdbool.h>
#include <stdio.h>

bool test_pcd8544_suite(void);
bool test_display_suite(void);
bool test_input_protocol_suite(void);

int main(void)
{
    printf("Tests\n");
    printf("=====\n");

    if (!test_pcd8544_suite())
    {
        printf("PCD8544 TESTS FAILED\n");
        return 1;
    }
    printf("PCD8544 TESTS PASSED\n");


    if (!test_display_suite())
    {
        printf("DISPLAY TESTS FAILED\n");
        return 1;
    }
    printf("DISPLAY TESTS PASSED\n");


    if (!test_input_protocol_suite())
    {
        printf("INPUT PROTOCOL TESTS FAILED\n");
        return 1;
    }
    printf("INPUT PROTOCOL TESTS PASSED\n");


    printf("ALL TESTS PASSED\n");

    return 0;
}