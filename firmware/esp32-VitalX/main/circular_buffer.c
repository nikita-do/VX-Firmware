#include "app_main.h"

void buffer_init(CircularBuffer_t *cb, const char *name)
{
    cb->head = 0;
    cb->tail = 0;
    strncpy(cb->name, name, BUFFER_NAME_MAX_LEN - 1);
    cb->name[BUFFER_NAME_MAX_LEN - 1] = '\0'; // Ensure null-termination
}

bool buffer_is_full(CircularBuffer_t *cb)
{
    return ((cb->head + 1) & (BUFFER_SIZE - 1)) == cb->tail;
}

size_t buffer_data_count(CircularBuffer_t *cb)
{
    if (cb->head >= cb->tail)
        return cb->head - cb->tail;
    return BUFFER_SIZE - cb->tail + cb->head;
}

void buffer_put(CircularBuffer_t *cb, uint32_t data)
{
    // Check if the buffer is full
    if (buffer_is_full(cb))
    {
        // printf("⚠️ Buffer '%s' is full! Overwriting oldest data.\n", cb->name);
        cb->tail = (cb->tail + 1) & (BUFFER_SIZE - 1); // Advance tail to overwrite oldest data
    }

    cb->buffer[cb->head] = data;
    cb->head = (cb->head + 1) & (BUFFER_SIZE - 1); // Use bitwise AND for modulus
}

// Returns true and fills temp_buffer with PROCESS_SIZE samples if a full chunk is ready
bool buffer_get_chunk(CircularBuffer_t *cb, uint32_t *temp_buffer)
{
    // Ensure SAMPLE_BATCH is valid
    if (SAMPLE_BATCH > BUFFER_SIZE)
    {
        printf("⚠️ PROCESS_SIZE exceeds BUFFER_SIZE. Adjust the configuration.\n");
        return false;
    }

    size_t available = buffer_data_count(cb);
    if (available < SAMPLE_BATCH)
    {
        printf("⚠️ Not enough data in buffer '%s' to get a chunk. Available: %zu, Required: %d\n", cb->name, available, SAMPLE_BATCH);
        return false; // Not enough data yet
    }

    if (cb->tail + SAMPLE_BATCH <= BUFFER_SIZE)
    {
        // No wraparound
        memcpy(temp_buffer, &cb->buffer[cb->tail], SAMPLE_BATCH * sizeof(uint32_t));
    }
    else
    {
        // Partial-wrap: copy in two parts
        size_t first_part = BUFFER_SIZE - cb->tail;
        size_t second_part = SAMPLE_BATCH - first_part;

        memcpy(temp_buffer, &cb->buffer[cb->tail], first_part * sizeof(uint32_t));
        memcpy(temp_buffer + first_part, &cb->buffer[0], second_part * sizeof(uint32_t));
    }

    cb->tail = (cb->tail + SAMPLE_BATCH) & (BUFFER_SIZE - 1); // Use bitwise AND for modulus
    return true;
}

void buffer_print(CircularBuffer_t *cb)
{
    if (cb->head == cb->tail)
    {
        printf("Buffer '%s' is empty.\n", cb->name);
        return;
    }

    printf("Buffer '%s' contents: ", cb->name);
    size_t i = cb->tail;
    while (i != cb->head)
    {
        printf("%lu ", cb->buffer[i]);
        i = (i + 1) & (BUFFER_SIZE - 1); // Use bitwise AND for modulus
    }
    printf("\n");
}
