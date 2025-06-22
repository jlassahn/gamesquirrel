
#include "unit_tests.h"
#include <stdio.h>

extern void TestCharQueue(void);
extern void TestDiskCache(void);
extern void TestFatFs(void);

static int test_errors = 0;
static int total_errors = 0;

int main(void)
{
    printf("Unit Tests...\n");
    RUN_TEST(TestCharQueue);
    RUN_TEST(TestDiskCache);
    RUN_TEST(TestFatFs);

    if (total_errors > 0)
    {
        printf("UNIT TESTS FAILED, Total errors %d\n", total_errors);
        return 1;
    }
    else
    {
        printf("UNIT TESTS PASSED\n");
        return 0;
    }
}


bool RunTest(TestFn fn, const char *name)
{
    printf("%s\n", name);
    test_errors = 0;
    fn();
    if (test_errors > 0)
    {
        printf("FAILED %s -- %d errors\n", name, test_errors);
        return false;
    }
    return true;
}

void Check(bool val, const char *text, const char *message, const char *file, int line)
{
    if (val)
        return;

    test_errors ++;
    total_errors ++;
    if (message)
        printf("FAILED %s at line %d of %s\n", message, line, file);
    else
        printf("FAILED (%s) at line %d of %s\n", text, line, file);
}


