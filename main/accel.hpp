#pragma once
#include "esp_err.h"

namespace wallter::accel {

// Initialize BMA400 on I2C_NUM_1 (pins BMA400_SDA_PIN / BMA400_SCL_PIN).
// Must be called once before update() / read_angle_deg().
esp_err_t init();

// Update the filtered angle estimate. Call exactly once per control loop iteration.
//
// direction_hint:
//   +1.0f  — motor is extending  (angle must be non-decreasing)
//   -1.0f  — motor is retracting (angle must be non-increasing)
//    0.0f  — motor stopped       (EMA runs freely)
//
// Two-stage filter:
//   1. EMA smoothing (alpha = kAlpha) removes high-frequency noise.
//   2. Directional gate: during motion the filtered value is clamped to only
//      move in the commanded direction, preventing noise from causing a
//      premature or wrong-direction stop.
void update(float direction_hint);

// Returns the last filtered angle in degrees (cached from the most recent update()).
//
// Mounting assumptions (adjust signs in accel.cpp if needed):
//   X axis = axis of board rotation
//   Z axis = points outward from board surface (into the air)
//   atan2(-accel_y, -accel_z) = tilt angle from horizontal
float read_angle_deg();

// Set a fixed offset (degrees) added to the raw accelerometer angle.
// Typically loaded from calibration at startup.
void set_angle_offset(float offset_deg);

} // namespace wallter::accel
