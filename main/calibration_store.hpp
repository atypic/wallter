#pragma once

#include <stdint.h>
#include "esp_err.h"

#include "angle_utils.hpp"

namespace wallter::calibration {

// Calibration sweep: full angle table (LOWEST_ANGLE..HIGHEST_ANGLE) in ANGLE_STEP.
static constexpr int kFirstAngle = LOWEST_ANGLE;
static constexpr int kLastAngle  = HIGHEST_ANGLE;
static constexpr int kStepAngle  = ANGLE_STEP;
static constexpr int kCount      = wallter::kMaxAngles;

// Initializes NVS (safe to call multiple times; ESP-IDF handles internally).
esp_err_t init_nvs();

// Loads target ticks from NVS if present; otherwise uses defaults.
// Always ensures home (lowest angle) is 0 ticks and derives the 15° entry.
bool load_or_default(uint32_t *target_ticks, int target_len);

// Saves the current calibration table (LOWEST_ANGLE..HIGHEST_ANGLE in ANGLE_STEP)
// from the provided target table.
esp_err_t save_from_target_ticks(const uint32_t *target_ticks, int target_len);

// Additional metadata describing the usable angle range.
// min/max are in degrees, aligned to the global ANGLE_STEP.
struct CalMeta {
	uint8_t min_angle_deg;
	uint8_t max_angle_deg;
};

// Loads min/max angles from NVS if present; otherwise returns defaults.
// Returns true if loaded from NVS.
bool load_meta_or_default(CalMeta &out);

// Saves min/max angles to NVS.
esp_err_t save_meta(const CalMeta &meta);

// Deletes stored calibration data from NVS (ticks + meta). Next boot/load will use defaults.
esp_err_t erase_calibration();

} // namespace wallter::calibration
