#pragma once

namespace wallter::ble_ota {

// Initializes BLE (NimBLE) and starts advertising an OTA GATT service.
// Requires Bluetooth + NimBLE enabled in sdkconfig.
void init();

} // namespace wallter::ble_ota
