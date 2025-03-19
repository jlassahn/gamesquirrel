/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn

  System calls into the GameSquirrel system software for use by applications.
*/

#include <stdint.h>
#include <stdbool.h>

typedef enum LibraryID
{
	LIB_INVALID = 0,
	LIB_SYSTEM
}
LibraryID;

typedef struct Version Version;
struct Version
{
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
	uint8_t unused;
};

typedef struct SystemCalls SystemCalls;
struct SystemCalls
{
	Version version;

	void * (*FindLibrary)(LibraryID id, uint8_t major_version);

};

extern SystemCalls *system;

