#pragma once

#include "app_config.hpp"

namespace wallter {

// Maximum possible number of selectable targets: every ANGLE_STEP multiple from
// ANGLE_STEP up to HIGHEST_ANGLE (e.g. 5°,10°,...,70° → 14 entries for ANGLE_STEP=5,HIGHEST_ANGLE=70).
// The actual count after homing may be smaller depending on the measured home angle and user max.
static constexpr int kMaxAngles = HIGHEST_ANGLE / ANGLE_STEP;

inline int angle_to_index(int angle_deg) {
    if (angle_deg < LOWEST_ANGLE || angle_deg > HIGHEST_ANGLE) {
        return -1;
    }
    int delta = angle_deg - LOWEST_ANGLE;
    if ((delta % ANGLE_STEP) != 0) {
        return -1;
    }
    int idx = delta / ANGLE_STEP;
    if (idx < 0 || idx >= kMaxAngles) {
        return -1;
    }
    return idx;
}

} // namespace wallter
