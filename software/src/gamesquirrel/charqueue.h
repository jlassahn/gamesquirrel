/*
  SPDX-License-Identifier: Apache-2.0
  Copyright 2025 Jeff Lassahn
*/

#ifndef INCLUDED_CHARQUEUE_H
#define INCLUDED_CHARQUEUE_H

#include <stdint.h>

#define CHARQUEUE_SIZE 64
_Static_assert(((CHARQUEUE_SIZE - 1) & CHARQUEUE_SIZE) == 0, "CHARQUEUE_SIZE must be a power of 2");

#define CHARQUEUE_MASK (CHARQUEUE_SIZE - 1)

typedef struct CharQueue CharQueue;
struct CharQueue
{
    uint32_t read;
    uint32_t write;
    uint8_t data[CHARQUEUE_SIZE];
};

static inline int CharQueue_Used(CharQueue *queue)
{
    uint32_t read = __atomic_load_n(&queue->read, __ATOMIC_SEQ_CST);
    uint32_t write = __atomic_load_n(&queue->write, __ATOMIC_SEQ_CST);

    return (write - read) & CHARQUEUE_MASK;
}

static inline int CharQueue_Free(CharQueue *queue)
{
    uint32_t read = __atomic_load_n(&queue->read, __ATOMIC_SEQ_CST);
    uint32_t write = __atomic_load_n(&queue->write, __ATOMIC_SEQ_CST);

    return (read - write - 1) & CHARQUEUE_MASK;
}

static inline int CharQueue_Read(CharQueue *queue, char *data, int max_bytes)
{
    uint32_t read = __atomic_load_n(&queue->read, __ATOMIC_SEQ_CST);
    uint32_t write = __atomic_load_n(&queue->write, __ATOMIC_SEQ_CST);
    int length = (write - read) & CHARQUEUE_MASK;
    if (length > max_bytes)
        length = max_bytes;

    for (int i=0; i<length; i++)
    {
        data[i] = queue->data[read];
        read = (read + 1) & CHARQUEUE_MASK;
    }

    __atomic_store_n(&queue->read, read, __ATOMIC_SEQ_CST);
    return length;
}

static inline int CharQueue_Write(CharQueue *queue, const char *data, int max_bytes)
{
    uint32_t read = __atomic_load_n(&queue->read, __ATOMIC_SEQ_CST);
    uint32_t write = __atomic_load_n(&queue->write, __ATOMIC_SEQ_CST);
    int length = (read - write - 1) & CHARQUEUE_MASK;
    if (length > max_bytes)
        length = max_bytes;

    for (int i=0; i<length; i++)
    {
        queue->data[write] = data[i];
        write = (write + 1) & CHARQUEUE_MASK;
    }

    __atomic_store_n(&queue->write, write, __ATOMIC_SEQ_CST);
    return length;
}

#endif

