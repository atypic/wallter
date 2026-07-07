#include "logbuf.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

namespace wallter::logbuf {

static constexpr size_t kCapacity = 8192;

static uint8_t g_buf[kCapacity];
static size_t g_head = 0;   // next write position
static size_t g_count = 0;  // valid bytes, <= kCapacity
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

static vprintf_like_t g_prev_vprintf = nullptr;

static void append(const char *data, int len) {
    portENTER_CRITICAL(&g_mux);
    for (int i = 0; i < len; ++i) {
        g_buf[g_head] = (uint8_t)data[i];
        g_head = (g_head + 1) % kCapacity;
        if (g_count < kCapacity) {
            ++g_count;
        }
    }
    portEXIT_CRITICAL(&g_mux);
}

static int vprintf_hook(const char *fmt, va_list args) {
    char line[256];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);

    if (n > 0) {
        int len = (n < (int)sizeof(line)) ? n : (int)sizeof(line) - 1;
        append(line, len);
    }

    if (g_prev_vprintf) {
        return g_prev_vprintf(fmt, args);
    }
    return n;
}

void init() {
    g_prev_vprintf = esp_log_set_vprintf(&vprintf_hook);
}

size_t snapshot(uint8_t *out, size_t max) {
    portENTER_CRITICAL(&g_mux);
    size_t count = g_count;
    size_t start = (g_head + kCapacity - g_count) % kCapacity;
    size_t n = (count < max) ? count : max;
    for (size_t i = 0; i < n; ++i) {
        out[i] = g_buf[(start + i) % kCapacity];
    }
    portEXIT_CRITICAL(&g_mux);
    return n;
}

size_t capacity() {
    return kCapacity;
}

} // namespace wallter::logbuf
