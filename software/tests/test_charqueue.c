
#include "unit_tests.h"
#include "gamesquirrel/charqueue.h"
#include <string.h>

void TestCharQueue(void)
{
	CharQueue queue;
	memset(&queue, 0, sizeof(queue));

	CHECK(CharQueue_Used(&queue) == 0);
	CHECK(CharQueue_Free(&queue) == 63);

	CHECK(CharQueue_Write(&queue, "Hello", 5) == 5);
	CHECK(CharQueue_Used(&queue) == 5);
	CHECK(CharQueue_Free(&queue) == 58);

	CHECK(CharQueue_Write(&queue, "1234567890", 10) == 10);
	CHECK(CharQueue_Used(&queue) == 15);
	CHECK(CharQueue_Free(&queue) == 48);

	CHECK(CharQueue_Write(&queue, "1234567890", 10) == 10);
	CHECK(CharQueue_Used(&queue) == 25);
	CHECK(CharQueue_Free(&queue) == 38);

	CHECK(CharQueue_Write(&queue, "1234567890", 10) == 10);
	CHECK(CharQueue_Used(&queue) == 35);
	CHECK(CharQueue_Free(&queue) == 28);

	CHECK(CharQueue_Write(&queue, "1234567890", 10) == 10);
	CHECK(CharQueue_Used(&queue) == 45);
	CHECK(CharQueue_Free(&queue) == 18);

	CHECK(CharQueue_Write(&queue, "1234567890", 10) == 10);
	CHECK(CharQueue_Used(&queue) == 55);
	CHECK(CharQueue_Free(&queue) == 8);

	CHECK(CharQueue_Write(&queue, "1234567890", 10) == 8);
	CHECK(CharQueue_Used(&queue) == 63);
	CHECK(CharQueue_Free(&queue) == 0);

	char data[16] = {0};

	CHECK(CharQueue_Read(&queue, data, 10) == 10);
	CHECK(memcmp(data, "Hello12345", 10) == 0);
	CHECK(CharQueue_Used(&queue) == 53);
	CHECK(CharQueue_Free(&queue) == 10);

	CHECK(CharQueue_Write(&queue, "xx", 2) == 2);
	CHECK(CharQueue_Used(&queue) == 55);
	CHECK(CharQueue_Free(&queue) == 8);

	CHECK(CharQueue_Read(&queue, data, 5) == 5);
	CHECK(memcmp(data, "67890", 5) == 0);
	CHECK(CharQueue_Used(&queue) == 50);
	CHECK(CharQueue_Free(&queue) == 13);

	CHECK(CharQueue_Read(&queue, data, 10) == 10);
	CHECK(memcmp(data, "1234567890", 10) == 0);
	CHECK(CharQueue_Used(&queue) == 40);
	CHECK(CharQueue_Free(&queue) == 23);

	CHECK(CharQueue_Read(&queue, data, 10) == 10);
	CHECK(memcmp(data, "1234567890", 10) == 0);
	CHECK(CharQueue_Used(&queue) == 30);
	CHECK(CharQueue_Free(&queue) == 33);

	CHECK(CharQueue_Read(&queue, data, 10) == 10);
	CHECK(memcmp(data, "1234567890", 10) == 0);
	CHECK(CharQueue_Used(&queue) == 20);
	CHECK(CharQueue_Free(&queue) == 43);

	CHECK(CharQueue_Read(&queue, data, 10) == 10);
	CHECK(memcmp(data, "1234567890", 10) == 0);
	CHECK(CharQueue_Used(&queue) == 10);
	CHECK(CharQueue_Free(&queue) == 53);

	CHECK(CharQueue_Read(&queue, data, 10) == 10);
	CHECK(memcmp(data, "12345678xx", 10) == 0);
	CHECK(CharQueue_Used(&queue) == 0);
	CHECK(CharQueue_Free(&queue) == 63);

	CHECK(CharQueue_Read(&queue, data, 10) == 0);
	CHECK(CharQueue_Used(&queue) == 0);
	CHECK(CharQueue_Free(&queue) == 63);

	CHECK(CharQueue_Write(&queue, "ABCDEFG", 7) == 7);
	CHECK(CharQueue_Used(&queue) == 7);
	CHECK(CharQueue_Free(&queue) == 56);

	CHECK(CharQueue_Read(&queue, data, 16) == 7);
	CHECK(memcmp(data, "ABCDEFG", 7) == 0);
	CHECK(CharQueue_Used(&queue) == 0);
	CHECK(CharQueue_Free(&queue) == 63);
}

