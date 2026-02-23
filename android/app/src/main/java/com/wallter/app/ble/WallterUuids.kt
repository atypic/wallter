package com.wallter.app.ble

import java.util.UUID

object WallterUuids {
    // NOTE: These UUIDs match what Android reports during GATT discovery.
    // They correspond to the firmware's BLE_UUID128_INIT usage (little-endian byte order).
    val OTA_SERVICE: UUID = UUID.fromString("00010234-7c8d-2c9e-2f4d-2a4d7d5a9b9a")
    val OTA_CTRL: UUID = UUID.fromString("00020234-7c8d-2c9e-2f4d-2a4d7d5a9b9a")
    val OTA_DATA: UUID = UUID.fromString("00030234-7c8d-2c9e-2f4d-2a4d7d5a9b9a")
    val OTA_STATUS: UUID = UUID.fromString("00040234-7c8d-2c9e-2f4d-2a4d7d5a9b9a")

    val CONTROL_SERVICE: UUID = UUID.fromString("00050234-7c8d-2c9e-2f4d-2a4d7d5a9b9a")
    val BUTTONS: UUID = UUID.fromString("00060234-7c8d-2c9e-2f4d-2a4d7d5a9b9a")
    val ANGLE: UUID = UUID.fromString("00070234-7c8d-2c9e-2f4d-2a4d7d5a9b9a")

    const val OP_BEGIN: Byte = 0x01
    const val OP_END: Byte = 0x03
    const val OP_ABORT: Byte = 0x04
    const val OP_REBOOT: Byte = 0x05

    const val FLAG_HAS_SHA256: Byte = 0x01

    const val BTN_EXTEND: Byte = 0x01
    const val BTN_RETRACT: Byte = 0x02
    const val BTN_STOP: Byte = 0x04
    const val BTN_HOME: Byte = 0x08
}
