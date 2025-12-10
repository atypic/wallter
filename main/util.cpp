#include "util.hpp"
// Firmware build provides Arduino/Due symbols; tests won't compile this file.
#include <Arduino.h>
#include "boards.h"

uint8_t compute_progress_percent(int32_t current_pos, int32_t start_pos, int32_t target_pos) {
    if (target_pos == start_pos) {
        return 100;
    }
    int64_t num = (int64_t)current_pos - (int64_t)start_pos;
    int64_t den = (int64_t)target_pos - (int64_t)start_pos;
    if (den == 0) {
        return 0;
    }
    if ((den > 0 && current_pos >= target_pos) || (den < 0 && current_pos <= target_pos)) {
        return 100;
    }
    int32_t pct = (int32_t)((num * 100 + (den > 0 ? den / 2 : -den / 2)) / den);
    if (pct < 0) {
        pct = 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    return (uint8_t)pct;
}

void setDebounce(int pin, int usecs) {  // reject spikes shorter than usecs on pin
    if (usecs) {
        g_APinDescription[pin].pPort->PIO_IFER = g_APinDescription[pin].ulPin;
        g_APinDescription[pin].pPort->PIO_DIFSR |= g_APinDescription[pin].ulPin;
    } else {
        g_APinDescription[pin].pPort->PIO_IFDR = g_APinDescription[pin].ulPin;
        g_APinDescription[pin].pPort->PIO_DIFSR &= ~g_APinDescription[pin].ulPin;
        return;
    }

    int div = (usecs / 31) - 1;
    if (div < 0) {
        div = 0;
    }
    if (div > 16383) {
        div = 16383;
    }
    g_APinDescription[pin].pPort->PIO_SCDR = div;
}
