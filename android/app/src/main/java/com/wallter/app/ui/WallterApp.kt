package com.wallter.app.ui

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
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
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import com.wallter.app.MainViewModel

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

    val pickBinLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.OpenDocument()
    ) { uri: Uri? ->
        viewModel.setFirmwareUri(uri)
    }

    Scaffold { padding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(padding)
                .padding(16.dp),
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
                    firmwareName = ui.firmwareName,
                    progress = ui.otaProgress,
                    statusText = ui.otaStatusText,
                    onPickBin = { pickBinLauncher.launch(arrayOf("application/octet-stream", "*/*")) },
                    onStartOta = { viewModel.startOta(context) },
                    onReboot = { viewModel.reboot() },
                )
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
    firmwareName: String?,
    progress: Float,
    statusText: String,
    onPickBin: () -> Unit,
    onStartOta: () -> Unit,
    onReboot: () -> Unit,
) {
    Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Text("Firmware Update", style = MaterialTheme.typography.titleMedium)
        Text("Selected: ${firmwareName ?: "(none)"}")
        Text("Status: $statusText")
        Text("Progress: ${(progress * 100).toInt()}%")

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = onPickBin, enabled = enabled) { Text("Choose .bin") }
            Button(onClick = onStartOta, enabled = enabled && firmwareName != null) { Text("Start") }
        }

        OutlinedButton(onClick = onReboot, enabled = enabled) { Text("Reboot") }
    }
}
