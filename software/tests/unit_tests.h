
#ifndef INCLUDED_UNIT_TESTS_H
#define INCLUDED_UNIT_TESTS_H

#include <stdlib.h>
#include <stdbool.h>

typedef void (*TestFn)(void);

bool RunTest(TestFn fn, const char *name);
void Check(bool val, const char *text, const char *message, const char *file, int line);

#define RUN_TEST(x)  RunTest((x), #x)

#define CHECK(x)              Check((x), #x, NULL, __FILE__, __LINE__)
#define CHECK_MESSAGE(x, msg) Check((x), #x, (msg), __FILE__, __LINE__)
#define FAIL(msg)             Check(false, "FAIL", (msg), __FILE__, __LINE__)

#endif

