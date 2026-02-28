package com.wallter.app

import android.content.Context
import android.net.Uri
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.wallter.app.BuildConfig
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
import org.json.JSONArray
import org.json.JSONObject
import java.io.InputStream
import java.net.HttpURLConnection
import java.net.URL
import java.io.IOException
import java.security.MessageDigest

data class UiState(
    val permissionsGranted: Boolean = false,
    val isScanning: Boolean = false,
    val scanResults: List<WallterBleClient.ScanDevice> = emptyList(),
    val isConnected: Boolean = false,
    val connectedDeviceName: String? = null,
    val firmwareUri: Uri? = null,
    val firmwareName: String? = null,
    val firmwareUrl: String? = null,
    val availableFirmwares: List<FirmwareAsset> = emptyList(),
    val isLoadingFirmwares: Boolean = false,
    val otaProgress: Float = 0f,
    val otaStatusText: String = "Idle",
    val currentAngleDeg: Float? = null,
    val testAngleDeg: Float = 10f,
    val lastError: String? = null,
)

data class FirmwareAsset(
    val name: String,
    val downloadUrl: String,
    val sizeBytes: Long,
    val releaseName: String?,
    val publishedAt: String?,
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

    private var moveRequestId: Long = 0L

    private val firmwareUri = MutableStateFlow<Uri?>(null)
    private val firmwareName = MutableStateFlow<String?>(null)
    private val firmwareUrl = MutableStateFlow<String?>(null)

    private val availableFirmwares = MutableStateFlow<List<FirmwareAsset>>(emptyList())
    private val isLoadingFirmwares = MutableStateFlow(false)

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
        .combine(firmwareUrl) { s, url -> s.copy(firmwareUrl = url) }
        .combine(availableFirmwares) { s, list -> s.copy(availableFirmwares = list) }
        .combine(isLoadingFirmwares) { s, loading -> s.copy(isLoadingFirmwares = loading) }
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
                lastError.value = null

                // Not connected: keep the existing “test mode” local update.
                if (!isConnected.value) {
                    val next = (testAngleDeg.value + 5f).coerceAtMost(90f)
                    testAngleDeg.value = next
                    return@launch
                }

                val before = currentAngleDeg.value
                val reqId = ++moveRequestId
                c.sendButtons(WallterUuids.BTN_EXTEND)
                val accepted = waitForAngleChange(reqId, before)
                if (!accepted) {
                    lastError.value = "Move not accepted"
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
                lastError.value = null

                // Not connected: keep the existing “test mode” local update.
                if (!isConnected.value) {
                    val next = (testAngleDeg.value - 5f).coerceAtLeast(0f)
                    testAngleDeg.value = next
                    return@launch
                }

                val before = currentAngleDeg.value
                val reqId = ++moveRequestId
                c.sendButtons(WallterUuids.BTN_RETRACT)
                val accepted = waitForAngleChange(reqId, before)
                if (!accepted) {
                    lastError.value = "Move not accepted"
                }
            } catch (t: Throwable) {
                lastError.value = t.message ?: t.toString()
            }
        }
    }

    private suspend fun waitForAngleChange(requestId: Long, before: Float?): Boolean {
        // If another move request comes in, this one becomes obsolete.
        val startMs = System.currentTimeMillis()
        val timeoutMs = 1_500L
        val epsilon = 0.25f

        while (System.currentTimeMillis() - startMs < timeoutMs) {
            if (requestId != moveRequestId) return true
            if (!isConnected.value) return true

            val now = currentAngleDeg.value
            val changed = when {
                before == null && now != null -> true
                before != null && now == null -> false
                before != null && now != null -> kotlin.math.abs(now - before) >= epsilon
                else -> false
            }
            if (changed) return true
            delay(100)
        }
        return false
    }

    fun setFirmwareUri(uri: Uri?) {
        firmwareUri.value = uri
        firmwareName.value = uri?.lastPathSegment
        firmwareUrl.value = null
    }

    fun setFirmwareFromUrl(name: String, url: String) {
        firmwareUri.value = null
        firmwareName.value = name
        firmwareUrl.value = url
    }

    fun refreshAvailableFirmwares() {
        viewModelScope.launch {
            isLoadingFirmwares.value = true
            try {
                val list = withContext(Dispatchers.IO) {
                    fetchGithubFirmwareAssets()
                }
                availableFirmwares.value = list
                if (list.isEmpty()) {
                    lastError.value = "No firmware assets found in GitHub releases"
                }
            } catch (t: Throwable) {
                lastError.value = t.message ?: t.toString()
            } finally {
                isLoadingFirmwares.value = false
            }
        }
    }

    private fun fetchGithubFirmwareAssets(): List<FirmwareAsset> {
        val owner = BuildConfig.GITHUB_OWNER
        val repo = BuildConfig.GITHUB_REPO
        val apiUrl = "https://api.github.com/repos/${owner}/${repo}/releases"
        val url = URL(apiUrl)
        val conn = (url.openConnection() as HttpURLConnection).apply {
            requestMethod = "GET"
            setRequestProperty("Accept", "application/vnd.github+json")
            setRequestProperty("User-Agent", "WallterApp")
            connectTimeout = 10_000
            readTimeout = 10_000
        }

        val token = BuildConfig.GITHUB_TOKEN.takeIf { it.isNotBlank() }
        if (token != null) {
            conn.setRequestProperty("Authorization", "Bearer $token")
        }

        val code = conn.responseCode
        val stream = if (code in 200..299) conn.inputStream else conn.errorStream
        stream.use { input ->
            val body = input?.bufferedReader()?.readText().orEmpty()
            if (code !in 200..299) {
                val msg = try {
                    JSONObject(body).optString("message").takeIf { it.isNotBlank() }
                } catch (_: Throwable) {
                    null
                }
                val hint = if (code == 404 && token == null) {
                    " Repo may be private; provide a GitHub token at build time (Gradle property github.token or env GITHUB_TOKEN)."
                } else {
                    ""
                }
                throw IOException("GitHub releases fetch failed for $apiUrl: HTTP $code ${conn.responseMessage}${msg?.let { ": $it" } ?: ""}.$hint")
            }

            val arr = JSONArray(body)
            val out = mutableListOf<FirmwareAsset>()

            // Preserve GitHub ordering (newest releases first, then asset order).
            for (i in 0 until arr.length()) {
                val rel = arr.optJSONObject(i) ?: continue
                val releaseName = rel.optString("name").takeIf { it.isNotBlank() }
                    ?: rel.optString("tag_name").takeIf { it.isNotBlank() }
                val publishedAt = rel.optString("published_at").takeIf { it.isNotBlank() }

                val assets = rel.optJSONArray("assets") ?: continue
                for (j in 0 until assets.length()) {
                    val asset = assets.optJSONObject(j) ?: continue
                    val name = asset.optString("name")
                    val dl = asset.optString("browser_download_url")
                    val size = asset.optLong("size", -1L).coerceAtLeast(0L)
                    if (name.isBlank() || dl.isBlank()) continue

                    if (!name.endsWith(".bin", ignoreCase = true)) continue

                    out += FirmwareAsset(
                        name = name,
                        downloadUrl = dl,
                        sizeBytes = size,
                        releaseName = releaseName,
                        publishedAt = publishedAt,
                    )
                }
            }

            return out
        }
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
        val uri = firmwareUri.value
        val url = firmwareUrl.value

        if (uri == null && url == null) return

        viewModelScope.launch {
            otaProgress.value = 0f
            otaStatusText.value = "Preparing…"
            lastError.value = null

            try {
                withContext(Dispatchers.IO) {
                    val bytes = if (uri != null) {
                        context.contentResolver.openInputStream(uri).use { input ->
                            if (input == null) throw IllegalStateException("Unable to open firmware")
                            input.readBytes()
                        }
                    } else {
                        otaStatusText.value = "Downloading…"
                        downloadFirmware(url!!)
                    }

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

    private fun downloadFirmware(downloadUrl: String): ByteArray {
        val url = URL(downloadUrl)
        val conn = (url.openConnection() as HttpURLConnection).apply {
            requestMethod = "GET"
            setRequestProperty("Accept", "application/octet-stream")
            setRequestProperty("User-Agent", "WallterApp")
            connectTimeout = 15_000
            readTimeout = 30_000
            instanceFollowRedirects = true
        }

        val token = BuildConfig.GITHUB_TOKEN.takeIf { it.isNotBlank() }
        if (token != null) {
            conn.setRequestProperty("Authorization", "Bearer $token")
        }

        val code = conn.responseCode
        val stream = if (code in 200..299) conn.inputStream else conn.errorStream
        stream.use { input ->
            val bytesOrText = input?.readBytes() ?: ByteArray(0)
            if (code !in 200..299) {
                val preview = bytesOrText.decodeToString().take(200)
                val hint = if (code == 404 && token == null) {
                    " Repo may be private; set github.token in android/local.properties."
                } else {
                    ""
                }
                throw IOException("Firmware download failed: HTTP $code ${conn.responseMessage}: ${preview}.$hint")
            }
            return bytesOrText
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
