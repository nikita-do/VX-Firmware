#include "app_main.h"

void buffer_init(CircularBuffer_t *cb, const char *name)
{
    cb->write = 0;
    cb->read = 0;
        strncpy(cb->name, name, BUFFER_NAME_MAX_LEN - 1);
    cb->name[BUFFER_NAME_MAX_LEN - 1] = '\0'; // Ensure null-termination
}

bool buffer_is_full(CircularBuffer_t *cb)
{
    return ((cb->write + 1) & (BUFFER_SIZE - 1)) == cb->read;
}

size_t buffer_data_count(CircularBuffer_t *cb)
{
    if (cb->write >= cb->read)
        return cb->write - cb->read;
    return BUFFER_SIZE - cb->read + cb->write;
}

void buffer_put(CircularBuffer_t *cb, uint16_t data)
{
    // Check if the buffer is full
    if (buffer_is_full(cb))
    {
        // printf("⚠️ Buffer '%s' is full! Overwriting oldest data.\n", cb->name);
        cb->read = (cb->read + 1) & (BUFFER_SIZE - 1); // Advance read to overwrite oldest data
    }

    cb->buffer[cb->write] = data;
    cb->write = (cb->write + 1) & (BUFFER_SIZE - 1); // Use modulus for wraparound
}

// Returns true and fills temp_buffer with PROCESS_SIZE samples if a full chunk is ready
bool buffer_get_chunk(CircularBuffer_t *cb, uint16_t *temp_buffer)
{
    // Ensure N_SAMPLE is valid
    if (N_SAMPLE > BUFFER_SIZE)
    {
        printf("⚠️ PROCESS_SIZE exceeds buffer size. Adjust the configuration.\n");
        return false;
    }

    size_t available = buffer_data_count(cb);
    if (available < N_SAMPLE)
    {
        printf("⚠️ Not enough data in buffer '%s' to get a chunk. Available: %zu, Required: %d\n", cb->name, available, N_SAMPLE);
        return false; // Not enough data yet
    }

    if (cb->read + N_SAMPLE <= BUFFER_SIZE)
    {
        // No wraparound
        memcpy(temp_buffer, &cb->buffer[cb->read], N_SAMPLE * sizeof(uint16_t));
    }
    else
    {
        // Partial-wrap: copy in two parts
        size_t first_part = BUFFER_SIZE - cb->read;
        size_t second_part = N_SAMPLE - first_part;

        memcpy(temp_buffer, &cb->buffer[cb->read], first_part * sizeof(uint16_t));
        memcpy(temp_buffer + first_part, &cb->buffer[0], second_part * sizeof(uint16_t));
    }

    cb->read = (cb->read + N_SAMPLE) & (BUFFER_SIZE - 1); // Use modulus for wraparound
    return true;
}

void buffer_print(CircularBuffer_t *cb)
{
    if (cb->write == cb->read)
    {
        printf("Buffer '%s' is empty.\n", cb->name);
        return;
    }

    printf("Buffer '%s' contents: ", cb->name);
    size_t i = cb->read;
    while (i != cb->write)
    {
        printf("%d ", cb->buffer[i]);
        i = (i + 1) & (BUFFER_SIZE - 1); // Use bitwise AND for modulus
    }
    printf("\n");
}
