package com.wallter.app.ui

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import com.wallter.app.BuildConfig
import com.wallter.app.MainViewModel
import com.wallter.app.FirmwareAsset

@Composable
fun WallterRoot(viewModel: MainViewModel = viewModel()) {
    val context = LocalContext.current

    // Ensure the BLE client is created and its flows are collected before scanning.
    LaunchedEffect(Unit) {
        viewModel.ensureClientWithContext(context)
    }

    val permissionsLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        viewModel.onPermissionsResult(grants)
    }

    LaunchedEffect(Unit) {
        val perms = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT,
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
            )
        }
        permissionsLauncher.launch(perms)
    }

    WallterApp(viewModel = viewModel)
}

private enum class Mode { Control, Maintenance }

@Composable
fun WallterApp(viewModel: MainViewModel) {
    val context = LocalContext.current

    val ui by viewModel.uiState.collectAsState()
    var mode by remember { mutableStateOf(Mode.Control) }

    LaunchedEffect(mode) {
        if (mode == Mode.Maintenance && ui.availableFirmwares.isEmpty() && !ui.isLoadingFirmwares) {
            viewModel.refreshAvailableFirmwares()
        }
    }

    // Local file picking is intentionally removed in favor of listing public GitHub releases.

    Scaffold { padding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(padding)
                .padding(16.dp)
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            Text("Wallter", style = MaterialTheme.typography.headlineSmall)

            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                if (mode == Mode.Control) {
                    Button(onClick = { mode = Mode.Control }) { Text("Control") }
                    OutlinedButton(onClick = { mode = Mode.Maintenance }) { Text("Maintenance") }
                } else {
                    OutlinedButton(onClick = { mode = Mode.Control }) { Text("Control") }
                    Button(onClick = { mode = Mode.Maintenance }) { Text("Maintenance") }
                }
            }

            ConnectionSection(
                isScanning = ui.isScanning,
                isConnected = ui.isConnected,
                connectedName = ui.connectedDeviceName,
                scanCount = ui.scanResults.size,
                onScanToggle = { viewModel.toggleScan() },
                onDisconnect = { viewModel.disconnect() },
                onConnectFirst = { viewModel.connectFirstResult() },
            )

            if (mode == Mode.Control) {
                ControlSection(
                    enabled = true,
                    angleDeg = ui.currentAngleDeg ?: ui.testAngleDeg,
                    onUp = { viewModel.sendAngleUp() },
                    onDown = { viewModel.sendAngleDown() },
                )
            } else {
                MaintenanceSection(
                    enabled = ui.isConnected,
                    selectedUrl = ui.firmwareUrl,
                    availableFirmwares = ui.availableFirmwares,
                    isLoadingFirmwares = ui.isLoadingFirmwares,
                    progress = ui.otaProgress,
                    statusText = ui.otaStatusText,
                    otaError = ui.lastError,
                    deviceFirmwareVersion = ui.deviceFirmwareVersion,
                    onSelectFirmware = { asset -> viewModel.setFirmwareFromUrl(asset.name, asset.downloadUrl) },
                    onStartOta = { viewModel.startOta(context) },
                    onReboot = { viewModel.reboot() },
                )

                if (ui.isConnected) {
                    Spacer(modifier = Modifier.height(8.dp))

                    DeviceSettingsSection(
                        settings = ui.deviceSettings,
                        onRefresh = { viewModel.refreshDeviceSettings() },
                        onSave = { min, max, offset -> viewModel.saveDeviceSettings(min, max, offset) },
                    )
                }
            }

            if (!ui.permissionsGranted) {
                Text(
                    "Bluetooth permissions not granted. The app can’t scan/connect.",
                    color = MaterialTheme.colorScheme.error,
                )
            }

            if (ui.lastError != null) {
                Text(ui.lastError!!, color = MaterialTheme.colorScheme.error)
            }
        }
    }
}

@Composable
private fun ConnectionSection(
    isScanning: Boolean,
    isConnected: Boolean,
    connectedName: String?,
    scanCount: Int,
    onScanToggle: () -> Unit,
    onDisconnect: () -> Unit,
    onConnectFirst: () -> Unit,
) {
    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Text("Connection", style = MaterialTheme.typography.titleMedium)
        Text(
            when {
                isConnected -> "Connected: ${connectedName ?: "(unknown)"}"
                isScanning -> "Scanning… (${scanCount} found)"
                else -> "Disconnected"
            }
        )

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = onScanToggle) {
                Text(if (isScanning) "Stop scan" else "Scan")
            }
            Button(onClick = onConnectFirst, enabled = !isConnected && scanCount > 0) {
                Text("Connect")
            }
            OutlinedButton(onClick = onDisconnect, enabled = isConnected) {
                Text("Disconnect")
            }
        }
    }
}

@Composable
private fun ControlSection(
    enabled: Boolean,
    angleDeg: Float?,
    onUp: () -> Unit,
    onDown: () -> Unit,
) {
    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Text("Angle", style = MaterialTheme.typography.titleMedium)
        Text("Current: ${angleDeg?.let { "${it.toInt()}°" } ?: "—"}")
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = onUp, enabled = enabled, modifier = Modifier.weight(1f)) {
                Text("Down")
            }
            Button(onClick = onDown, enabled = enabled, modifier = Modifier.weight(1f)) {
                Text("Up")
            }
        }
    }
}

@Composable
private fun MaintenanceSection(
    enabled: Boolean,
    selectedUrl: String?,
    availableFirmwares: List<FirmwareAsset>,
    isLoadingFirmwares: Boolean,
    progress: Float,
    statusText: String,
    otaError: String?,
    deviceFirmwareVersion: String?,
    onSelectFirmware: (FirmwareAsset) -> Unit,
    onStartOta: () -> Unit,
    onReboot: () -> Unit,
) {
    val isOtaRunning = statusText in listOf("Preparing…", "Downloading…", "Sending to device…", "Verifying…")
    var showConfirmDialog by remember { mutableStateOf(false) }

    if (showConfirmDialog) {
        AlertDialog(
            onDismissRequest = { showConfirmDialog = false },
            title = { Text("Start firmware update?") },
            text = {
                Text("The device will be unavailable during the update. " +
                     "Do not disconnect or power off until it completes.")
            },
            confirmButton = {
                Button(onClick = {
                    showConfirmDialog = false
                    onStartOta()
                }) { Text("Update") }
            },
            dismissButton = {
                OutlinedButton(onClick = { showConfirmDialog = false }) {
                    Text("Cancel")
                }
            },
        )
    }

    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Text("Firmware Update", style = MaterialTheme.typography.titleMedium)

        if (deviceFirmwareVersion != null) {
            Text(
                "Device: $deviceFirmwareVersion",
                style = MaterialTheme.typography.bodyMedium,
                fontWeight = FontWeight.SemiBold,
            )
        }

        // --- Firmware list ---
        Text(
            if (availableFirmwares.isEmpty()) {
                if (isLoadingFirmwares) "Loading firmwares…" else "No firmwares loaded"
            } else {
                "Select firmware:"
            },
            style = MaterialTheme.typography.bodyMedium,
        )

        for (asset in availableFirmwares) {
            val isSelected = asset.downloadUrl == selectedUrl
            OutlinedButton(
                onClick = { onSelectFirmware(asset) },
                enabled = !isOtaRunning,
                modifier = Modifier.fillMaxWidth(),
                border = BorderStroke(
                    width = if (isSelected) 2.dp else 1.dp,
                    color = if (isSelected) MaterialTheme.colorScheme.primary
                            else MaterialTheme.colorScheme.outline,
                ),
                colors = if (isSelected) ButtonDefaults.outlinedButtonColors(
                    containerColor = MaterialTheme.colorScheme.primaryContainer,
                ) else ButtonDefaults.outlinedButtonColors(),
            ) {
                val sizeKb = (asset.sizeBytes / 1024L).toInt()
                val label = buildString {
                    if (asset.releaseName != null) append("${asset.releaseName} — ")
                    append("${asset.name} (${sizeKb} KB)")
                }
                Text(label)
            }
        }

        // --- OTA controls ---
        Button(
            onClick = { showConfirmDialog = true },
            enabled = enabled && selectedUrl != null && !isOtaRunning,
            modifier = Modifier.fillMaxWidth(),
        ) { Text(if (isOtaRunning) statusText else "Start OTA") }

        if (isOtaRunning || progress > 0f) {
            LinearProgressIndicator(
                progress = { progress },
                modifier = Modifier.fillMaxWidth().height(6.dp),
            )
            Text(
                "${statusText} — ${(progress * 100).toInt()}%",
                style = MaterialTheme.typography.bodySmall,
            )
        } else if (statusText != "Idle") {
            Text(
                statusText,
                style = MaterialTheme.typography.bodySmall,
                color = if (statusText == "Error") MaterialTheme.colorScheme.error
                        else MaterialTheme.colorScheme.onSurface,
            )
        }

        if (otaError != null) {
            Text(
                otaError,
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.error,
            )
        }

        if (statusText == "Done! Reboot to activate.") {
            Button(
                onClick = onReboot,
                enabled = enabled,
                modifier = Modifier.fillMaxWidth(),
            ) { Text("Reboot device") }
        }

        Spacer(modifier = Modifier.height(8.dp))
        Text("About", style = MaterialTheme.typography.titleMedium)
        Text("Wallter ${BuildConfig.VERSION_NAME} (${BuildConfig.VERSION_CODE})")

        OutlinedButton(onClick = onReboot, enabled = enabled) { Text("Reboot") }
    }
}

@Composable
private fun DeviceSettingsSection(
    settings: com.wallter.app.ble.WallterBleClient.DeviceSettings?,
    onRefresh: () -> Unit,
    onSave: (minAngle: Int, maxAngle: Int, offsetTenths: Int) -> Unit,
) {
    var minAngle by remember(settings) { mutableStateOf(settings?.minAngleDeg?.toString() ?: "") }
    var maxAngle by remember(settings) { mutableStateOf(settings?.maxAngleDeg?.toString() ?: "") }
    var offsetDeg by remember(settings) {
        mutableStateOf(settings?.angleOffsetTenths?.let { "%.1f".format(it / 10f) } ?: "")
    }

    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Text("Device Settings", style = MaterialTheme.typography.titleMedium)

        if (settings == null) {
            Text("Loading…", style = MaterialTheme.typography.bodySmall)
            OutlinedButton(onClick = onRefresh) {
                Text("Read from device")
            }
        } else {
            OutlinedTextField(
                value = minAngle,
                onValueChange = { minAngle = it },
                label = { Text("Min angle (°)") },
                modifier = Modifier.fillMaxWidth(),
            )
            OutlinedTextField(
                value = maxAngle,
                onValueChange = { maxAngle = it },
                label = { Text("Max angle (°)") },
                modifier = Modifier.fillMaxWidth(),
            )
            OutlinedTextField(
                value = offsetDeg,
                onValueChange = { offsetDeg = it },
                label = { Text("Angle offset (°)") },
                modifier = Modifier.fillMaxWidth(),
            )

            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(
                    onClick = {
                        val min = minAngle.toIntOrNull() ?: return@Button
                        val max = maxAngle.toIntOrNull() ?: return@Button
                        val ofs = offsetDeg.toFloatOrNull() ?: return@Button
                        onSave(min, max, (ofs * 10f).toInt())
                    },
                ) {
                    Text("Save to device")
                }
                OutlinedButton(onClick = onRefresh) {
                    Text("Refresh")
                }
            }
        }
    }
}
