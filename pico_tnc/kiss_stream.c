#include "kiss_stream.h"

void kiss_stream_input(uint8_t *buffer, int capacity, int *length,
                       uint8_t *state, uint8_t byte,
                       kiss_stream_frame_fn frame_fn, void *ctx)
{
    if (!buffer || capacity <= 0 || !length || !state || !frame_fn) return;

    switch (*state) {
        case KISS_OUTSIDE:
            if (byte == KISS_FEND_BYTE) {
                *length = 0;
                *state = KISS_INSIDE;
            }
            return;

        case KISS_INSIDE:
            if (byte == KISS_FEND_BYTE) {
                bool remain_inside = true;
                if (*length > 0) remain_inside = frame_fn(ctx, buffer, *length);
                *length = 0;
                // A delimiter terminates one frame and synchronizes the next.
                // Repeated delimiters therefore produce no empty callbacks.
                *state = remain_inside ? KISS_INSIDE : KISS_OUTSIDE;
                return;
            }
            if (byte == KISS_FESC_BYTE) {
                *state = KISS_FESC;
                return;
            }
            break;

        case KISS_FESC:
            if (byte == KISS_TFEND_BYTE) byte = KISS_FEND_BYTE;
            else if (byte == KISS_TFESC_BYTE) byte = KISS_FESC_BYTE;
            else {
                *length = 0;
                *state = (byte == KISS_FEND_BYTE) ? KISS_INSIDE : KISS_ERROR;
                return;
            }
            *state = KISS_INSIDE;
            break;

        case KISS_ERROR:
        default:
            if (byte == KISS_FEND_BYTE) {
                *length = 0;
                *state = KISS_INSIDE;
            }
            return;
    }

    if (*length >= capacity) {
        *length = 0;
        *state = KISS_ERROR;
        return;
    }
    buffer[(*length)++] = byte;
}
