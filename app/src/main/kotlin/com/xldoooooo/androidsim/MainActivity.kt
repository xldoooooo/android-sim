package com.xldoooooo.androidsim

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.lifecycleScope
import com.xldoooooo.androidsim.control.PIDController
import com.xldoooooo.androidsim.logger.CsvLogger
import com.xldoooooo.androidsim.sim.SimLoop
import com.xldoooooo.androidsim.sim.models.QuadrotorModel
import java.io.File

class MainActivity : ComponentActivity() {

    private var simLoop: SimLoop? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            var logText by remember { mutableStateOf("Ready. Press ▶ Start to run simulation.\n") }
            val scrollState = rememberScrollState()

            // 自动滚动到底部
            LaunchedEffect(logText) { scrollState.animateScrollTo(scrollState.maxValue) }

            MaterialTheme {
                Column(
                    modifier = Modifier
                        .fillMaxSize()
                        .padding(12.dp)
                ) {
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Button(onClick = {
                            logText = "Simulation started...\n"
                            startSim { line -> logText += line + "\n" }
                        }) { Text("▶ Start") }

                        Button(onClick = {
                            simLoop?.stop()
                            logText += "\n[Stopped]\n"
                        }) { Text("■ Stop") }
                    }

                    Spacer(Modifier.height(8.dp))
                    HorizontalDivider()
                    Spacer(Modifier.height(4.dp))

                    Box(
                        modifier = Modifier
                            .fillMaxSize()
                            .verticalScroll(scrollState)
                    ) {
                        Text(
                            text = logText,
                            fontSize = 11.sp,
                            fontFamily = FontFamily.Monospace,
                            lineHeight = 16.sp
                        )
                    }
                }
            }
        }
    }

    private fun startSim(onLog: (String) -> Unit) {
        val model = QuadrotorModel()
        val dt    = 0.005  // 控制步长 5ms

        // 四通道 PID：高度(z) + roll + pitch + yaw
        val pidZ     = PIDController(kp = 5.0, ki = 0.5,  kd = 2.0, dt = dt)
        val pidRoll  = PIDController(kp = 3.0, ki = 0.0,  kd = 0.5, dt = dt)
        val pidPitch = PIDController(kp = 3.0, ki = 0.0,  kd = 0.5, dt = dt)
        val pidYaw   = PIDController(kp = 1.0, ki = 0.0,  kd = 0.2, dt = dt)

        // 目标：悬停在 z=2m，姿态角全为0
        val goalZ    = 2.0
        val goalRoll = 0.0; val goalPitch = 0.0; val goalYaw = 0.0

        val outFile = File(getExternalFilesDir(null), "sim_output.csv")
        val logger  = CsvLogger(outFile, model)

        simLoop = SimLoop(
            model      = model,
            controller = { _, x ->
                // x[2]=pz, x[6]=roll, x[7]=pitch, x[8]=yaw
                doubleArrayOf(
                    model.mass * 9.81 + pidZ.compute(goalZ, x[2]),
                    pidRoll.compute(goalRoll, x[6]),
                    pidPitch.compute(goalPitch, x[7]),
                    pidYaw.compute(goalYaw, x[8])
                )
            },
            x0       = DoubleArray(model.stateDim),  // 初始状态全零（地面静止）
            tEnd     = 30.0,
            dt       = dt,
            useRK45  = false,   // 改为 true 则使用自适应步长 RK45
            logger   = logger,
            onLogLine = onLog
        )
        simLoop!!.start(lifecycleScope)
    }

    override fun onDestroy() {
        super.onDestroy()
        simLoop?.stop()
    }
}
