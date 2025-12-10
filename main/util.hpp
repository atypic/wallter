#pragma once

#include <stdint.h>

// Compute percentage of progress towards a target given
// current position, start baseline, and target position.
// Returns [0,100] with rounding toward nearest and clamps.
uint8_t compute_progress_percent(int32_t current_pos, int32_t start_pos, int32_t target_pos);

// Configure hardware debounce on a pin (Due-specific).
// If usecs==0, disables the filter; otherwise enables with computed divider.
void setDebounce(int pin, int usecs);

#include <Arduino.h>
#include <stdint.h>

// Returns true if 'period_ms' has elapsed since 'last_ms'.
// If true, updates 'last_ms' to current millis().
inline bool throttle(uint32_t &last_ms, uint32_t period_ms) {
    uint32_t now = millis();
    if (last_ms + period_ms > now) {
        return false;
    }
    last_ms = now;
    return true;
}

// Convenience scope guard-style macro:
// Use as: PERIODIC(500, last_foo) { /* body runs at most every 500ms */ }
#define PERIODIC(period_ms, last_var) \
    for (bool _run = throttle(last_var, (period_ms)); _run; _run = false)
