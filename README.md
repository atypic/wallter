# wallter

ESP-IDF firmware for the Wallter device (ESP32-S3). This repo supports OTA firmware updates delivered over BLE from a mobile app.

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

On tags matching `v*`, these two files are uploaded as GitHub Release assets.

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
