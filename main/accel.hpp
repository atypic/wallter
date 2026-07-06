#pragma once
#include "esp_err.h"

namespace wallter::accel {

// Initialize BMA400 on I2C_NUM_1 (pins BMA400_SDA_PIN / BMA400_SCL_PIN).
// Must be called once before update() / read_angle_deg().
esp_err_t init();

// Update the filtered angle estimate. Call once per control loop iteration.
//
// Pipeline: 1g-magnitude gate (reject vibration/impact) -> median window
// (reject impulsive spikes) -> EMA smoothing. Combined with the sensor's
// hardware oversampling (OSR3).
void update();

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
