package com.wallter.app

import androidx.compose.runtime.Composable
import androidx.compose.ui.platform.LocalContext
import androidx.lifecycle.viewmodel.compose.viewModel
import com.wallter.app.ui.WallterRoot as RootUi

@Composable
fun WallterRoot() {
    val context = LocalContext.current
    val vm: MainViewModel = viewModel()
    vm.ensureClientWithContext(context)
    RootUi(viewModel = vm)
}
