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

} // namespace wallter::ble_ota
