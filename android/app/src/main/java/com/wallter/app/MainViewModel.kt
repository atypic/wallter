package com.wallter.app

import android.content.Context
import android.net.Uri
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.wallter.app.ble.WallterBleClient
import com.wallter.app.ble.WallterStatus
import com.wallter.app.ble.WallterUuids
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.InputStream
import java.security.MessageDigest

data class UiState(
    val permissionsGranted: Boolean = false,
    val isScanning: Boolean = false,
    val scanResults: List<WallterBleClient.ScanDevice> = emptyList(),
    val isConnected: Boolean = false,
    val connectedDeviceName: String? = null,
    val firmwareUri: Uri? = null,
    val firmwareName: String? = null,
    val otaProgress: Float = 0f,
    val otaStatusText: String = "Idle",
    val currentAngleDeg: Float? = null,
    val testAngleDeg: Float = 10f,
    val lastError: String? = null,
)

class MainViewModel : ViewModel() {

    private var client: WallterBleClient? = null

    private val permissionsGranted = MutableStateFlow(false)
    private val isScanning = MutableStateFlow(false)
    private val scanResults = MutableStateFlow<List<WallterBleClient.ScanDevice>>(emptyList())
    private val isConnected = MutableStateFlow(false)
    private val connectedDeviceName = MutableStateFlow<String?>(null)
    private val otaProgress = MutableStateFlow(0f)
    private val otaStatusText = MutableStateFlow("Idle")
    private val currentAngleDeg = MutableStateFlow<Float?>(null)
    private val testAngleDeg = MutableStateFlow(10f)
    private var lastAngleUpdateMs: Long = 0L
    private val lastError = MutableStateFlow<String?>(null)

    private val firmwareUri = MutableStateFlow<Uri?>(null)
    private val firmwareName = MutableStateFlow<String?>(null)

    val uiState: StateFlow<UiState> = combine(
        permissionsGranted,
        isScanning,
        scanResults,
        isConnected,
        connectedDeviceName,
    ) { perms, scanning, results, connected, connectedName ->
        UiState(
            permissionsGranted = perms,
            isScanning = scanning,
            scanResults = results,
            isConnected = connected,
            connectedDeviceName = connectedName,
        )
    }
        .combine(otaProgress) { s, prog -> s.copy(otaProgress = prog) }
        .combine(otaStatusText) { s, text -> s.copy(otaStatusText = text) }
        .combine(currentAngleDeg) { s, a -> s.copy(currentAngleDeg = a) }
        .combine(testAngleDeg) { s, a -> s.copy(testAngleDeg = a) }
        .combine(lastError) { s, err -> s.copy(lastError = err) }
        .combine(firmwareUri) { s, uri -> s.copy(firmwareUri = uri) }
        .combine(firmwareName) { s, name -> s.copy(firmwareName = name) }
        .stateIn(viewModelScope, kotlinx.coroutines.flow.SharingStarted.Eagerly, UiState())

    fun onPermissionsResult(grants: Map<String, Boolean>) {
        permissionsGranted.value = grants.values.all { it }
    }

    fun toggleScan() {
        if (!permissionsGranted.value) {
            lastError.value = "Bluetooth permissions not granted"
            return
        }

        val c = client ?: run {
            lastError.value = "BLE client not initialized"
            return
        }

        if (isScanning.value) {
            isScanning.value = false
            c.stopScan()
        } else {
            isScanning.value = true
            c.startScan()
        }
    }

    fun connectFirstResult() {
        if (!permissionsGranted.value) {
            lastError.value = "Bluetooth permissions not granted"
            return
        }
        val c = client ?: run {
            lastError.value = "BLE client not initialized"
            return
        }
        val first = c.scanResults.value.firstOrNull() ?: run {
            lastError.value = "No devices found"
            return
        }

        viewModelScope.launch {
            try {
                isScanning.value = false
                c.stopScan()
                c.connect(first.device)
                lastError.value = null
            } catch (t: Throwable) {
                lastError.value = t.message ?: t.toString()
            }
        }
    }

    fun disconnect() {
        client?.disconnect()
    }

    fun sendAngleUp() {
        val c = client ?: return
        viewModelScope.launch {
            try {
                // Test mode: always update local setpoint immediately.
                val next = (testAngleDeg.value + 5f).coerceAtMost(90f)
                testAngleDeg.value = next

                // If connected, also send the button event to the device.
                if (isConnected.value) {
                    c.sendButtons(WallterUuids.BTN_EXTEND)
                }
            } catch (t: Throwable) {
                lastError.value = t.message ?: t.toString()
            }
        }
    }

    fun sendAngleDown() {
        val c = client ?: return
        viewModelScope.launch {
            try {
                // Test mode: always update local setpoint immediately.
                val next = (testAngleDeg.value - 5f).coerceAtLeast(0f)
                testAngleDeg.value = next

                // If connected, also send the button event to the device.
                if (isConnected.value) {
                    c.sendButtons(WallterUuids.BTN_RETRACT)
                }
            } catch (t: Throwable) {
                lastError.value = t.message ?: t.toString()
            }
        }
    }

    fun setFirmwareUri(uri: Uri?) {
        firmwareUri.value = uri
        firmwareName.value = uri?.lastPathSegment
    }

    fun reboot() {
        val c = client ?: return
        viewModelScope.launch {
            try {
                c.reboot()
            } catch (t: Throwable) {
                lastError.value = t.message ?: t.toString()
            }
        }
    }

    fun startOta(context: Context) {
        val c = client ?: return
        val uri = firmwareUri.value ?: return

        viewModelScope.launch {
            otaProgress.value = 0f
            otaStatusText.value = "Preparing…"
            lastError.value = null

            try {
                withContext(Dispatchers.IO) {
                    context.contentResolver.openInputStream(uri).use { input ->
                        if (input == null) throw IllegalStateException("Unable to open firmware")
                        val bytes = input.readBytes()
                        val sha256 = MessageDigest.getInstance("SHA-256").digest(bytes)

                        otaStatusText.value = "BEGIN"
                        c.otaBegin(imageSize = bytes.size, sha256 = sha256)

                        otaStatusText.value = "Streaming…"
                        val chunkSize = 240
                        var offset = 0
                        while (offset < bytes.size) {
                            val end = minOf(offset + chunkSize, bytes.size)
                            c.otaWriteChunk(bytes.copyOfRange(offset, end))
                            offset = end
                            otaProgress.value = offset.toFloat() / bytes.size.toFloat()
                        }

                        otaStatusText.value = "END"
                        c.otaEnd()

                        // Wait for Ready (either via notifications or polling reads).
                        val deadline = System.currentTimeMillis() + 30_000
                        while (System.currentTimeMillis() < deadline) {
                            val st = c.readOtaStatus()
                            if (st != null && st.state == 2 && st.lastErr == 0) {
                                otaStatusText.value = "Ready"
                                otaProgress.value = 1f
                                return@withContext
                            }
                            if (st != null && st.state == 3) {
                                throw IllegalStateException("Device reported error")
                            }
                            delay(500)
                        }
                        throw IllegalStateException("Timed out waiting for Ready")
                    }
                }
            } catch (t: Throwable) {
                otaStatusText.value = "Error"
                lastError.value = t.message ?: t.toString()
                try {
                    c.otaAbort()
                } catch (_: Throwable) {
                }
            }
        }
    }

    private fun ensureClient() {
        if (client == null) {
            // ViewModels don’t have a context by default; the UI calls startOta with one.
            // For scanning/connecting we lazily create the client from the app context.
            // This method is intentionally a no-op until the first UI call provides a context.
        }
    }

    fun ensureClientWithContext(context: Context) {
        if (client == null) {
            client = WallterBleClient(context.applicationContext)

            val c = client!!
            viewModelScope.launch {
                c.scanResults.collect { scanResults.value = it }
            }
            viewModelScope.launch {
                c.isConnected.collect { isConnected.value = it }
            }
            viewModelScope.launch {
                c.connectedDeviceName.collect { connectedDeviceName.value = it }
            }

            // Poll current angle while connected (device transmits target angle over a BLE characteristic).
            viewModelScope.launch {
                c.isConnected.collect { connected ->
                    if (!connected) {
                        currentAngleDeg.value = null
                        lastAngleUpdateMs = 0L
                        return@collect
                    }

                    // Seed test angle from the first read if possible.
                    while (c.isConnected.value) {
                        try {
                            val a = c.readAngleDeg()
                            if (a != null) {
                                currentAngleDeg.value = a
                                testAngleDeg.value = a
                                lastAngleUpdateMs = System.currentTimeMillis()
                            } else {
                                val now = System.currentTimeMillis()
                                if (lastAngleUpdateMs != 0L && (now - lastAngleUpdateMs) > 2_000L) {
                                    currentAngleDeg.value = null
                                }
                            }
                        } catch (_: Throwable) {
                            // Keep polling; transient GATT errors happen.
                            val now = System.currentTimeMillis()
                            if (lastAngleUpdateMs != 0L && (now - lastAngleUpdateMs) > 2_000L) {
                                currentAngleDeg.value = null
                            }
                        }
                        delay(500)
                    }
                }
            }
            viewModelScope.launch {
                c.status.collect { st ->
                    if (st != null) {
                        otaStatusText.value = when (st.state) {
                            0 -> "Idle"
                            1 -> "Receiving"
                            2 -> "Ready"
                            3 -> "Error"
                            else -> "State ${st.state}"
                        }
                    }
                }
            }
        }
    }
}
