#pragma once

#include <stddef.h>
#include <stdint.h>

namespace wallter::logbuf {

// Install a vprintf hook that mirrors all ESP_LOG output into a fixed-size RAM
// ring buffer while still forwarding to the console. Call once at startup.
void init();

// Copy the current buffer contents in chronological order into out (up to max
// bytes). Returns the number of bytes written.
size_t snapshot(uint8_t *out, size_t max);

// Total capacity of the ring buffer in bytes.
size_t capacity();

} // namespace wallter::logbuf
