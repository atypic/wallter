# wallter

ESP-IDF firmware for the Wallter device (ESP32-S3). This repo supports OTA firmware updates delivered over BLE from a mobile app.

## User Manual

### Power-On Behavior

When Wallter powers on, the display shows the firmware version (e.g. `v2.0.0 ARCTIC_CYTRON`) for 2 seconds, then runs through a startup animation. What happens next depends on which buttons are held during power-on:

| Buttons held at power-on | Result |
|---|---|
| None | Normal boot: self-test → homing |
| **EXTEND only** | Enter boot menu |
| **EXTEND + RETRACT** (both) | Skip self-test, go straight to homing |

### Self-Test

On a normal boot (no buttons held), the firmware runs an automatic self-test before homing:

1. **Phase 1** — The motors retract briefly, then extend for 1 second. The display shows `Self test 1 / Extending.`. Each motor must produce at least 80 encoder ticks to pass.
2. **Phase 2** — The motors retract for 1 second. The display shows `Self test 2 / Retracting.`. Again, 80 ticks minimum per motor.

If a motor fails, the display shows `SELF TEST FAILED` with the failing motor number and the device halts. This indicates a hardware problem (motor not connected, encoder fault, etc.). Power-cycle to retry.

If the test passes: `Self test / PASSED` is shown briefly, then homing begins.

### Homing

After self-test (or after skipping it), the device homes by retracting until the motors stall (no encoder activity for 800 ms). The display shows `Homing... / Please wait`. Once homed, the encoder counters reset to zero and the device reads the current angle from the accelerometer.

After homing, the device automatically extends to the configured minimum angle if needed, then enters normal operation.

### Normal Operation

In normal operation the display shows the current angle (e.g. `Current: 45.0°`). Use the two buttons:

- **EXTEND** — Move to the next higher target angle (in 5° steps).
- **RETRACT** — Move to the next lower target angle.

While moving, the display shows the target angle and a progress bar: `Target: 50.0° / [######        ]`.

The motor speed ramps from MINSPEED up to the configured max speed. Extend and retract can have independent max speeds (configured via the app or boot menu).

### Boot Menu

Hold **EXTEND only** at power-on to enter the boot menu. The display shows a list of options. Navigate with:

- **RETRACT** — Cycle to the next menu item.
- **EXTEND** — Select / confirm the highlighted item.

#### Menu Items

| Item | Description |
|---|---|
| **Set max angle** | Set the upper angle limit. RETRACT cycles through angles (5° steps), EXTEND saves. |
| **Set min angle** | Set the lower angle limit. Same navigation as above. |
| **Angle offset** | Adjust the accelerometer angle offset in 0.1° steps (range ±12.7°). RETRACT increments by +0.1°, EXTEND saves. |
| **Run self test** | Manually run the self-test sequence (same as the automatic boot test). |
| **Jog mode** | Directly drive motors for testing. First select which motors (Both / M1 / M2), then hold EXTEND to run forward, RETRACT to run backward. Press both to re-select motors. |
| **HAL test** | Run encoder feedback test. Each motor extends then retracts for 900 ms each. Reports tick counts and pass/fail per motor. Press any button to exit. |
| **Reset cal** | Erase all calibration data from NVS (tick table, angle limits, offset, speeds). Resets to compile-time defaults on next boot. |

After exiting a menu item, the device proceeds to homing and then normal operation.

### Android App

The Wallter companion app connects to the device over BLE. The device advertises as `wallter` (or the custom name if changed).

#### Connecting

1. Open the app and tap **Scan**. Nearby Wallter devices appear in the list.
2. Tap **Connect** to connect to the first found device.
3. The app automatically reads the firmware version, device settings, and device name.

#### Remote Control

Use the **Higher angle** / **Lower angle** buttons to step through target angles, same as the physical buttons.

#### Device Settings

When connected, the Device Settings section shows:

| Setting | Description |
|---|---|
| **Device name** | The BLE advertised name (max 20 characters). Takes effect on next device reboot. |
| **Min angle** | Lower angle limit in degrees. |
| **Max angle** | Upper angle limit in degrees. |
| **Angle offset** | Accelerometer angle correction in degrees (e.g. `1.5` = +1.5°). |
| **Max extend speed** | Maximum motor speed when extending. |
| **Max retract speed** | Maximum motor speed when retracting. |

Tap **Save to device** to write all settings. Tap **Refresh** to re-read from the device.

#### Firmware Update (OTA)

1. The app fetches available firmware releases from GitHub automatically (pull down to refresh the list).
2. Select a firmware version from the list (highlighted in blue).
3. Tap **Start OTA** and confirm the dialog.
4. The progress bar shows transfer progress. Do not disconnect or power off during the update.
5. When complete, tap **Reboot device** to activate the new firmware.

### Troubleshooting

| Symptom | Cause / Fix |
|---|---|
| `SELF TEST FAILED` on boot | Motor or encoder wiring issue. Check connections and power-cycle. Hold both buttons to skip the test temporarily. |
| `PANIC: M0 NO OUT` | Motor 0 did not produce encoder ticks when expected. Check motor/encoder wiring. |
| Device won't connect via BLE | Ensure Bluetooth is enabled and permissions are granted. Try toggling Bluetooth off/on. If the device was OTA'd, the Android BLE cache may be stale — disconnect and reconnect. |
| Settings not showing in app | Wait a few seconds after connecting. Tap **Refresh** if still empty. |
| OTA fails mid-transfer | Reconnect and retry. The device returns to the previous firmware if the update didn't complete. |
| Wrong angle readings | Adjust the **Angle offset** in the boot menu or app. The value is added to the raw accelerometer reading. |

---

## Build & Flash

This is an ESP-IDF (v5.5.1) project.

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Notes:
- Project defaults are in `sdkconfig.defaults`. The generated `sdkconfig` is intentionally not tracked.
- The partition table is custom (`partitions.csv`) and includes two OTA slots (`ota_0` / `ota_1`).

## Release Artifacts (for the App)

CI builds produce:
- `build/wallter.bin`
- `build/wallter.bin.sha256` (a text file containing the SHA-256 of the `.bin`)

On tags matching `v*`, CI also uploads a convenience OTA asset to the GitHub Release:
- `wallter-ota.bin`
- `wallter-ota.bin.sha256`

On tags matching `v*`, CI also produces a flashable bundle:
- `wallter-esp32s3-flashable-<tag>.zip` (bootloader + partition table + app + flasher args)

On tags matching `v*`, these assets are uploaded as GitHub Release assets.

## BLE OTA GATT Protocol

The device advertises as a BLE peripheral and exposes a custom GATT service used to stream a firmware image into the next OTA slot.

### Service & Characteristics

All UUIDs are 128-bit (shown in canonical string form).

- Service UUID: `7d5a9b9a-2a4d-2f4d-9e2c-8d7c34020100`
- Control characteristic (Write): `7d5a9b9a-2a4d-2f4d-9e2c-8d7c34020200`
- Data characteristic (Write): `7d5a9b9a-2a4d-2f4d-9e2c-8d7c34020300`
- Status characteristic (Read + Notify): `7d5a9b9a-2a4d-2f4d-9e2c-8d7c34020400`

Device name (GAP): `wallter`

### Transport Rules

- **Endianness:** all multi-byte integers are **little-endian**.
- **Data chunking:** the firmware currently copies each GATT write into a 512-byte stack buffer. Keep Data writes **<= 512 bytes**.
  - With an ATT MTU of 247, a practical payload per write is typically <= 244 bytes.
- **Progress:** Status notifications are sent on state changes (BEGIN/READY/ERROR/ABORT). For live progress, the app should periodically read Status.

### Control Characteristic

Control is a write-only command channel.

#### BEGIN

Starts a new OTA session.

Payload layout:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | `op` = `0x01` |
| 1 | 1 | `flags` |
| 2 | 2 | `reserved` (write 0) |
| 4 | 4 | `image_size` (bytes) |
| 8 | 32 | `sha256` (optional, present when `flags & 0x01`) |

Flags:
- `0x01` = SHA-256 is present and must match.

Behavior:
- Calls the device-side OTA writer begin.
- Transitions Status `state` to Receiving on success.

#### END

Finishes the OTA session, validates the image, and marks the new partition as the next boot target.

Payload:
- `op` = `0x03` (single byte)

Behavior:
- Calls the device-side OTA writer finish.
- On success, transitions Status `state` to Ready.
- Does **not** reboot automatically.

#### ABORT

Aborts an in-progress update.

Payload:
- `op` = `0x04` (single byte)

Behavior:
- Aborts OTA writer and resets Status to Idle.

#### REBOOT

Reboots the device.

Payload:
- `op` = `0x05` (single byte)

Behavior:
- Immediately calls `esp_restart()`.

### Data Characteristic

Raw firmware bytes. Each write appends bytes to the current OTA image.

Payload:
- `chunk` = 0..512 bytes (recommended: <= MTU-3)

### Status Characteristic

Status is a packed struct returned by Read and also sent by Notify.

Payload layout (`sizeof(Status) = 48` bytes):

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | `state` |
| 1 | 1 | `last_err` |
| 2 | 2 | `reserved` |
| 4 | 4 | `bytes_written` |
| 8 | 4 | `image_size` |
| 12 | 4 | `crc32` |
| 16 | 32 | `sha256` (computed digest of the received image) |

State values:
- `0` = Idle
- `1` = Receiving
- `2` = Ready
- `3` = Error

`last_err`:
- `0` = OK
- non-zero = generic failure (app should ABORT and retry)

### Recommended App Flow

1) Connect and discover the service/characteristics.
2) (Optional but recommended) enable notifications on Status.
3) Write BEGIN to Control with `image_size` and (optionally) the expected SHA-256.
4) Stream the firmware with repeated writes to Data.
   - If the connection drops, reconnect, read Status to get `bytes_written`, and continue streaming from that offset.
5) Write END to Control.
6) Wait for Status `state = Ready`.
7) Write REBOOT to Control (or instruct the user to power-cycle).

### Security

The current OTA service does not require pairing/bonding. Treat the BLE OTA channel as trusted physical-proximity access.

## BLE Control GATT Protocol

In addition to OTA, the firmware exposes a small control service intended to mimic the device's physical buttons (angle up/down).

### Service & Characteristic

- Control Service UUID: `7d5a9b9a-2a4d-2f4d-9e2c-8d7c34020500`
- Buttons characteristic (Write): `7d5a9b9a-2a4d-2f4d-9e2c-8d7c34020600`

### Buttons Characteristic

Write a 1-byte bitmask representing one or more button-like events:

- `0x01` = Extend (angle up)
- `0x02` = Retract (angle down)
- `0x04` = Stop
- `0x08` = Home

Notes:
- Events are latched until the main loop consumes them.
- The app can send repeated Extend/Retract events to step through target angles.
