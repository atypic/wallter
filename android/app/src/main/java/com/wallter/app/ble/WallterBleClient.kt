package com.wallter.app.ble

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.util.Log
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withTimeout
import java.io.IOException
import java.util.UUID

class WallterBleClient(private val context: Context) {

    private companion object {
        private const val TAG = "WallterBleClient"
        private const val CCC_DESCRIPTOR_UUID = "00002902-0000-1000-8000-00805f9b34fb"
    }

    data class ScanDevice(val device: BluetoothDevice, val name: String?, val rssi: Int)

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)

    private val bluetoothManager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val adapter: BluetoothAdapter? get() = bluetoothManager.adapter

    private val _scanResults = MutableStateFlow<List<ScanDevice>>(emptyList())
    val scanResults: StateFlow<List<ScanDevice>> = _scanResults

    private val _connectedDeviceName = MutableStateFlow<String?>(null)
    val connectedDeviceName: StateFlow<String?> = _connectedDeviceName

    private val _isConnected = MutableStateFlow(false)
    val isConnected: StateFlow<Boolean> = _isConnected

    private val _status = MutableStateFlow<WallterStatus?>(null)
    val status: StateFlow<WallterStatus?> = _status

    private var gatt: BluetoothGatt? = null

    private var otaCtrl: BluetoothGattCharacteristic? = null
    private var otaData: BluetoothGattCharacteristic? = null
    private var otaStatus: BluetoothGattCharacteristic? = null
    private var buttons: BluetoothGattCharacteristic? = null
    private var angle: BluetoothGattCharacteristic? = null

    private val opMutex = Mutex()

    private var mtuDeferred: CompletableDeferred<Boolean>? = null
    private var servicesDeferred: CompletableDeferred<Boolean>? = null
    private var readDeferred: CompletableDeferred<ByteArray>? = null
    private var readUuid: UUID? = null
    private var writeChrDeferred: CompletableDeferred<Int>? = null
    private var writeChrUuid: UUID? = null
    private var writeDescDeferred: CompletableDeferred<Int>? = null
    private var writeDescUuid: UUID? = null

    private var serviceDiscoveryRetried = false

    private var lastGattTable: String? = null

    @SuppressLint("DiscouragedPrivateApi")
    private fun refreshDeviceCache(gatt: BluetoothGatt): Boolean {
        return try {
            val method = gatt.javaClass.getMethod("refresh")
            (method.invoke(gatt) as? Boolean) ?: false
        } catch (t: Throwable) {
            Log.w(TAG, "gatt.refresh() not available: ${t.message}")
            false
        }
    }

    private val scanCallback = object : ScanCallback() {
        private fun handleResult(result: ScanResult) {
            val dev = result.device
            val name = result.scanRecord?.deviceName ?: dev.name

            // Prefer filtering by service UUID (more reliable than device name).
            val hasWallterService = result.scanRecord?.serviceUuids?.any {
                it.uuid == WallterUuids.OTA_SERVICE || it.uuid == WallterUuids.CONTROL_SERVICE
            } == true

            val isWallterName = name?.equals("wallter", ignoreCase = true) == true

            Log.d(
                TAG,
                "scan: addr=${dev.address} name=${name ?: "(null)"} rssi=${result.rssi} uuids=${result.scanRecord?.serviceUuids}"
            )

            if (!hasWallterService && !isWallterName) return

            val entry = ScanDevice(dev, name, result.rssi)
            _scanResults.value = (_scanResults.value.filterNot { it.device.address == dev.address } + entry)
                .sortedByDescending { it.rssi }
        }

        override fun onScanResult(callbackType: Int, result: ScanResult) {
            handleResult(result)
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>) {
            for (r in results) {
                handleResult(r)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            Log.e(TAG, "scan failed: errorCode=$errorCode")
        }
    }

    @SuppressLint("MissingPermission")
    fun startScan() {
        val scanner = adapter?.bluetoothLeScanner ?: return
        _scanResults.value = emptyList()
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        scanner.startScan(null, settings, scanCallback)
    }

    @SuppressLint("MissingPermission")
    fun stopScan() {
        val scanner = adapter?.bluetoothLeScanner ?: return
        scanner.stopScan(scanCallback)
    }

    @SuppressLint("MissingPermission")
    suspend fun connect(device: BluetoothDevice) {
        disconnect()

        var attemptedReconnect = false
        while (true) {
            serviceDiscoveryRetried = false
            lastGattTable = null

            try {
                servicesDeferred = CompletableDeferred()
                mtuDeferred = CompletableDeferred()

                gatt = device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE)

                val mtuOk = withTimeout(15_000) { mtuDeferred?.await() } ?: false
                if (!mtuOk) {
                    throw IOException("MTU request failed")
                }

                val svcsOk = withTimeout(15_000) { servicesDeferred?.await() } ?: false
                if (!svcsOk) {
                    val table = lastGattTable?.takeIf { it.isNotBlank() } ?: "(no services discovered)"
                    throw IOException("Service discovery failed (missing required characteristics). Discovered: $table")
                }

                _connectedDeviceName.value = device.name
                _isConnected.value = true

                enableStatusNotifications()
                return
            } catch (t: Throwable) {
                disconnect()
                if (!attemptedReconnect && t is IOException && t.message?.contains("Service discovery failed") == true) {
                    attemptedReconnect = true
                    delay(500)
                    continue
                }
                throw t
            }
        }
    }

    @SuppressLint("MissingPermission")
    fun disconnect() {
        try {
            gatt?.disconnect()
        } catch (_: Exception) {
        }
        try {
            gatt?.close()
        } catch (_: Exception) {
        }
        gatt = null

        otaCtrl = null
        otaData = null
        otaStatus = null
        buttons = null
        angle = null

        serviceDiscoveryRetried = false
        lastGattTable = null

        _isConnected.value = false
        _connectedDeviceName.value = null
    }

    suspend fun sendButtons(mask: Byte) {
        val chr = buttons ?: throw IOException("Buttons characteristic not found")
        writeCharacteristic(chr, byteArrayOf(mask))
    }

    suspend fun readAngleDeg(): Float? {
        val chr = angle ?: return null
        val bytes = readCharacteristic(chr)
        if (bytes.size < 4) return null

        // Little-endian IEEE754 float32.
        val bits = (bytes[0].toInt() and 0xff) or
            ((bytes[1].toInt() and 0xff) shl 8) or
            ((bytes[2].toInt() and 0xff) shl 16) or
            ((bytes[3].toInt() and 0xff) shl 24)
        return Float.fromBits(bits)
    }

    suspend fun otaBegin(imageSize: Int, sha256: ByteArray?) {
        val ctrl = otaCtrl ?: throw IOException("OTA control characteristic not found")

        val flags: Byte = if (sha256 != null && sha256.size == 32) WallterUuids.FLAG_HAS_SHA256 else 0
        val payloadLen = if (flags.toInt() != 0) 8 + 32 else 8
        val payload = ByteArray(payloadLen)
        payload[0] = WallterUuids.OP_BEGIN
        payload[1] = flags
        payload[2] = 0
        payload[3] = 0
        // image_size little-endian at offset 4
        payload[4] = (imageSize and 0xFF).toByte()
        payload[5] = ((imageSize ushr 8) and 0xFF).toByte()
        payload[6] = ((imageSize ushr 16) and 0xFF).toByte()
        payload[7] = ((imageSize ushr 24) and 0xFF).toByte()
        if (flags.toInt() != 0) {
            sha256!!.copyInto(payload, destinationOffset = 8, startIndex = 0, endIndex = 32)
        }

        writeCharacteristic(ctrl, payload)
    }

    suspend fun otaWriteChunk(chunk: ByteArray) {
        val data = otaData ?: throw IOException("OTA data characteristic not found")
        writeCharacteristic(data, chunk)
    }

    suspend fun otaEnd() {
        val ctrl = otaCtrl ?: throw IOException("OTA control characteristic not found")
        writeCharacteristic(ctrl, byteArrayOf(WallterUuids.OP_END))
    }

    suspend fun otaAbort() {
        val ctrl = otaCtrl ?: throw IOException("OTA control characteristic not found")
        writeCharacteristic(ctrl, byteArrayOf(WallterUuids.OP_ABORT))
    }

    suspend fun reboot() {
        val ctrl = otaCtrl ?: throw IOException("OTA control characteristic not found")
        writeCharacteristic(ctrl, byteArrayOf(WallterUuids.OP_REBOOT))
    }

    suspend fun readOtaStatus(): WallterStatus? {
        val statusChr = otaStatus ?: throw IOException("OTA status characteristic not found")
        val bytes = readCharacteristic(statusChr)
        return WallterStatus.parse(bytes)
    }

    private suspend fun enableStatusNotifications() {
        val g = gatt ?: return
        val statusChr = otaStatus ?: return

        @SuppressLint("MissingPermission")
        if (!g.setCharacteristicNotification(statusChr, true)) {
            throw IOException("setCharacteristicNotification failed")
        }

        val cccd = statusChr.getDescriptor(UUID.fromString(CCC_DESCRIPTOR_UUID))
            ?: throw IOException("CCCD descriptor missing")

        val value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        writeDescriptor(cccd, value)
    }

    private suspend fun readCharacteristic(chr: BluetoothGattCharacteristic): ByteArray {
        val g = gatt ?: throw IOException("Not connected")

        return opMutex.withLock {
            val deferred = CompletableDeferred<ByteArray>()
            readDeferred = deferred
            readUuid = chr.uuid

            @SuppressLint("MissingPermission")
            val ok = g.readCharacteristic(chr)
            if (!ok) {
                readDeferred = null
                readUuid = null
                throw IOException("readCharacteristic returned false")
            }

            withTimeout(10_000) { deferred.await() }
        }
    }

    private suspend fun writeCharacteristic(chr: BluetoothGattCharacteristic, value: ByteArray) {
        val g = gatt ?: throw IOException("Not connected")

        opMutex.withLock {
            val deferred = CompletableDeferred<Int>()
            writeChrDeferred = deferred
            writeChrUuid = chr.uuid

            chr.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            chr.value = value

            @SuppressLint("MissingPermission")
            val ok = g.writeCharacteristic(chr)
            if (!ok) {
                writeChrDeferred = null
                writeChrUuid = null
                throw IOException("writeCharacteristic returned false")
            }

            val status = withTimeout(15_000) { deferred.await() }
            if (status != BluetoothGatt.GATT_SUCCESS) {
                throw IOException("GATT characteristic write failed: $status")
            }
        }
    }

    private suspend fun writeDescriptor(desc: BluetoothGattDescriptor, value: ByteArray) {
        val g = gatt ?: throw IOException("Not connected")

        opMutex.withLock {
            val deferred = CompletableDeferred<Int>()
            writeDescDeferred = deferred
            writeDescUuid = desc.uuid

            desc.value = value

            @SuppressLint("MissingPermission")
            val ok = g.writeDescriptor(desc)
            if (!ok) {
                writeDescDeferred = null
                writeDescUuid = null
                throw IOException("writeDescriptor returned false")
            }

            val status = withTimeout(10_000) { deferred.await() }
            if (status != BluetoothGatt.GATT_SUCCESS) {
                throw IOException("GATT descriptor write failed: $status")
            }
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED && status == BluetoothGatt.GATT_SUCCESS) {
                gatt.requestMtu(247)
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                scope.launch {
                    disconnect()
                }
            }
        }

        @SuppressLint("MissingPermission")
        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            mtuDeferred?.complete(status == BluetoothGatt.GATT_SUCCESS)
            mtuDeferred = null
            gatt.discoverServices()
        }

        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                servicesDeferred?.complete(false)
                servicesDeferred = null
                return
            }

            // Dump all discovered services + characteristics to help debug UUID mismatches.
            val sb = StringBuilder()
            for (svc in gatt.services) {
                Log.d(TAG, "svc: ${svc.uuid}")
                sb.append("svc:").append(svc.uuid).append(" ")
                for (ch in svc.characteristics) {
                    Log.d(TAG, "  chr: ${ch.uuid} props=0x${ch.properties.toString(16)}")
                    sb.append("chr:").append(ch.uuid).append("(0x").append(ch.properties.toString(16)).append(") ")
                }
            }
            lastGattTable = sb.toString().trim()

            // Resolve characteristics globally (some Android stacks can be finicky about
            // getService(UUID) with custom 128-bit services).
            otaCtrl = findCharacteristicAny(gatt, WallterUuids.OTA_CTRL)
            otaData = findCharacteristicAny(gatt, WallterUuids.OTA_DATA)
            otaStatus = findCharacteristicAny(gatt, WallterUuids.OTA_STATUS)
            buttons = findCharacteristicAny(gatt, WallterUuids.BUTTONS)
            angle = findCharacteristicAny(gatt, WallterUuids.ANGLE)

            if (buttons == null) {
                Log.e(TAG, "Buttons characteristic not found after service discovery")
                if (!serviceDiscoveryRetried) {
                    serviceDiscoveryRetried = true
                    val refreshed = refreshDeviceCache(gatt)
                    Log.w(TAG, "Retrying service discovery after cache refresh (refreshed=$refreshed)")
                    scope.launch {
                        delay(300)
                        gatt.discoverServices()
                    }
                    return
                }

                servicesDeferred?.complete(false)
                servicesDeferred = null
                return
            }

            servicesDeferred?.complete(true)
            servicesDeferred = null
        }

        @Deprecated("Deprecated in API 33")
        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
            val bytes = characteristic.value ?: return
            handleCharacteristicChanged(characteristic, bytes)
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray
        ) {
            handleCharacteristicChanged(characteristic, value)
        }

        @Deprecated("Deprecated in API 33")
        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int
        ) {
            // Android 12 and below call this overload.
            val value = characteristic.value ?: ByteArray(0)
            handleCharacteristicRead(characteristic, value, status)
        }

        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
            status: Int
        ) {
            // Android 13+ calls this overload.
            handleCharacteristicRead(characteristic, value, status)
        }

        override fun onCharacteristicWrite(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int
        ) {
            if (characteristic.uuid == writeChrUuid) {
                writeChrDeferred?.complete(status)
                writeChrDeferred = null
                writeChrUuid = null
            }
        }

        override fun onDescriptorWrite(gatt: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
            if (descriptor.uuid == writeDescUuid) {
                writeDescDeferred?.complete(status)
                writeDescDeferred = null
                writeDescUuid = null
            }
        }
    }

    private fun handleCharacteristicChanged(characteristic: BluetoothGattCharacteristic, value: ByteArray) {
        if (characteristic.uuid == WallterUuids.OTA_STATUS) {
            val parsed = WallterStatus.parse(value) ?: return
            _status.value = parsed
        }
    }

    private fun handleCharacteristicRead(
        characteristic: BluetoothGattCharacteristic,
        value: ByteArray,
        status: Int
    ) {
        if (status == BluetoothGatt.GATT_SUCCESS && characteristic.uuid == readUuid) {
            readDeferred?.complete(value)
        } else if (characteristic.uuid == readUuid) {
            readDeferred?.completeExceptionally(IOException("GATT read failed: $status"))
        }
        readDeferred = null
        readUuid = null
    }

    private fun findCharacteristic(gatt: BluetoothGatt, serviceUuid: UUID, chrUuid: UUID): BluetoothGattCharacteristic? {
        val svc: BluetoothGattService? = gatt.getService(serviceUuid)
        val direct = svc?.getCharacteristic(chrUuid)
        if (direct != null) return direct

        // Fallback: some Android stacks behave oddly with custom services;
        // search across all discovered services for the characteristic UUID.
        for (s in gatt.services) {
            val c = s.getCharacteristic(chrUuid)
            if (c != null) return c
        }
        return null
    }

    private fun findCharacteristicAny(gatt: BluetoothGatt, chrUuid: UUID): BluetoothGattCharacteristic? {
        for (s in gatt.services) {
            val c = s.getCharacteristic(chrUuid)
            if (c != null) return c
        }
        return null
    }

}
