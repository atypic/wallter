#pragma once

#include <stdint.h>

namespace wallter::ble_ota {

struct ButtonEvents {
	bool extend;
	bool retract;
	bool stop;
	bool home;
};

// Initializes BLE (NimBLE) and starts advertising an OTA GATT service.
// Requires Bluetooth + NimBLE enabled in sdkconfig.
void init();

// Publishes the current/target angle (degrees) for the app UI.
// Safe to call from the main loop.
void set_current_angle_deg(float angle_deg);

// Returns (and clears) any button-like commands received over BLE.
// Intended to be called from the main loop thread.
ButtonEvents poll_button_events();

// Calibration settings payload: min_angle_deg, max_angle_deg, angle_offset_tenths.
struct Settings {
    uint8_t min_angle_deg;
    uint8_t max_angle_deg;
    int8_t  angle_offset_tenths;
};

// Callbacks for reading/writing calibration settings over BLE.
typedef Settings (*SettingsReadCb)();
typedef bool (*SettingsWriteCb)(const Settings &s);

// Register callbacks so the BLE settings characteristic can read/write cal meta.
void set_settings_callbacks(SettingsReadCb read_cb, SettingsWriteCb write_cb);

} // namespace wallter::ble_ota
