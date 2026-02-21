#pragma once

#include <stdint.h>
#include "esp_err.h"

namespace wallter::calibration {

// Calibration sweep: angles 20..60 in 5 degree steps.
static constexpr int kFirstAngle = 20;
static constexpr int kLastAngle  = 60;
static constexpr int kStepAngle  = 5;
static constexpr int kCount      = ((kLastAngle - kFirstAngle) / kStepAngle) + 1; // 9

// Initializes NVS (safe to call multiple times; ESP-IDF handles internally).
esp_err_t init_nvs();

// Loads target ticks from NVS if present; otherwise uses defaults.
// Always ensures home (lowest angle) is 0 ticks and derives the 15° entry.
bool load_or_default(uint32_t *target_ticks, int target_len);

// Saves the current calibration slice (20..60 step 5) from the provided target table.
esp_err_t save_from_target_ticks(const uint32_t *target_ticks, int target_len);

} // namespace wallter::calibration
