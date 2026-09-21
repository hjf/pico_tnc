#pragma once

#include <stdbool.h>
#include <stdint.h>

#define KISS_FEND_BYTE  0xc0
#define KISS_FESC_BYTE  0xdb
#define KISS_TFEND_BYTE 0xdc
#define KISS_TFESC_BYTE 0xdd

enum kiss_stream_state {
    KISS_OUTSIDE = 0,
    KISS_INSIDE,
    KISS_FESC,
    KISS_ERROR,
};

// Called for each non-empty decoded KISS frame. Returning false leaves the
// decoder outside a frame (used by the serial KISS exit command).
typedef bool (*kiss_stream_frame_fn)(void *ctx, const uint8_t *frame, int len);

void kiss_stream_input(uint8_t *buffer, int capacity, int *length,
                       uint8_t *state, uint8_t byte,
                       kiss_stream_frame_fn frame_fn, void *ctx);
