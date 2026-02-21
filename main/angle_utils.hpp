#pragma once

#include "app_config.hpp"

namespace wallter {

static constexpr int kMaxAngles = ((HIGHEST_ANGLE - LOWEST_ANGLE) / ANGLE_STEP) + 1;

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
