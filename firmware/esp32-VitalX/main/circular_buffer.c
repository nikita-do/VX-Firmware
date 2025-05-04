#include "app_main.h"

void buffer_init(CircularBuffer_t *cb)
{
    cb->head = 0;
    cb->tail = 0;
    cb->full = false;
}

bool buffer_is_empty(CircularBuffer_t *cb)
{
    return (!cb->full && (cb->head == cb->tail));
}

bool buffer_is_full(CircularBuffer_t *cb)
{
    return cb->full;
}

size_t buffer_data_count(CircularBuffer_t *cb)
{
    if (cb->full)
        return BUFFER_SIZE;
    if (cb->head >= cb->tail)
        return cb->head - cb->tail;
    return BUFFER_SIZE - cb->tail + cb->head;
}

size_t buffer_distance(size_t from, size_t to)
{
    return (to - from) & (BUFFER_SIZE - 1); // Use bitwise AND for modulus
}

bool buffer_put(CircularBuffer_t *cb, uint32_t data)
{
    // Check if the buffer is full
    if (cb->full)
    {
        printf("⚠️ Buffer is full! Cannot add data.\n");
        return false;
    }

    cb->buffer[cb->head] = data;
    cb->head = (cb->head + 1) & (BUFFER_SIZE - 1); // Use bitwise AND for modulus

    if (cb->head == cb->tail)
    {
        cb->full = true;
    }

    return true;
}

// Returns true and fills temp_buffer with PROCESS_SIZE samples if a full chunk is ready
bool buffer_get_chunk(CircularBuffer_t *cb, uint32_t *temp_buffer)
{
    // Ensure PROCESS_SIZE is valid
    if (PROCESS_SIZE > BUFFER_SIZE)
    {
        printf("⚠️ PROCESS_SIZE exceeds BUFFER_SIZE. Adjust the configuration.\n");
        return false;
    }

    size_t available = buffer_distance(cb->tail, cb->head);
    if (!cb->full && available < PROCESS_SIZE)
    {
        return false; // Not enough data yet
    }

    if (cb->tail + PROCESS_SIZE <= BUFFER_SIZE)
    {
        // No wraparound
        memcpy(temp_buffer, &cb->buffer[cb->tail], PROCESS_SIZE * sizeof(uint32_t));
    }
    else
    {
        // Partial-wrap: copy in two parts
        size_t first_part = BUFFER_SIZE - cb->tail;
        size_t second_part = PROCESS_SIZE - first_part;

        memcpy(temp_buffer, &cb->buffer[cb->tail], first_part * sizeof(uint32_t));
        memcpy(temp_buffer + first_part, &cb->buffer[0], second_part * sizeof(uint32_t));
    }

    cb->tail = (cb->tail + PROCESS_SIZE) & (BUFFER_SIZE - 1); // Use bitwise AND for modulus
    cb->full = false;                                                           // Data has been processed
    return true;
}

void buffer_print(CircularBuffer_t *cb)
{
    if (buffer_is_empty(cb))
    {
        printf("Buffer is empty.\n");
        return;
    }

    printf("Buffer contents: ");
    size_t i = cb->tail;
    do
    {
        printf("%ld ", cb->buffer[i]);
        i = (i + 1) % BUFFER_SIZE;
    } while (i != cb->head || (cb->full && i == cb->tail));
    printf("\n");
}
