package com.xldoooooo.androidsim

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.background
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.lifecycleScope
import com.xldoooooo.androidsim.control.PIDController
import com.xldoooooo.androidsim.logger.CsvLogger
import com.xldoooooo.androidsim.sim.PythonDynamicsModel
import com.xldoooooo.androidsim.sim.SimLoop
import com.xldoooooo.androidsim.sim.models.QuadrotorModel
import java.io.File

// ── 默认 Python 脚本（从 assets 读取） ────────────────────────
private const val DEFAULT_SCRIPT_ASSET = "default_model.py"
private const val PAGE_SIZE = 50

class MainActivity : ComponentActivity() {

    private var simLoop: SimLoop? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val defaultScript = assets.open(DEFAULT_SCRIPT_ASSET).bufferedReader().readText()

        setContent {
            // ── 配置状态 ──────────────────────────────────────
            var tEndText   by remember { mutableStateOf("30.0") }
            var dtText     by remember { mutableStateOf("0.005") }
            var useRK45    by remember { mutableStateOf(false) }
            var usePython  by remember { mutableStateOf(true) }
            var scriptCode by remember { mutableStateOf(defaultScript) }

            // ── 运行状态 ──────────────────────────────────────
            var logText    by remember { mutableStateOf("Ready. Press ▶ Start.\n") }
            var isRunning  by remember { mutableStateOf(false) }
            val logScroll  = rememberScrollState()
            LaunchedEffect(logText) { logScroll.animateScrollTo(logScroll.maxValue) }

            // ── 数据翻阅状态 ──────────────────────────────────
            var csvRows    by remember { mutableStateOf<List<List<String>>>(emptyList()) }
            var csvHeaders by remember { mutableStateOf<List<String>>(emptyList()) }
            var dataPage   by remember { mutableStateOf(0) }

            // ── Tab 状态 ──────────────────────────────────────
            var selectedTab by remember { mutableStateOf(0) }
            val tabs = listOf("⚙️ 配置", "▶ 运行", "📊 数据")

            MaterialTheme {
                Column(Modifier.fillMaxSize()) {

                    // ── Tab Bar ───────────────────────────────
                    TabRow(selectedTabIndex = selectedTab) {
                        tabs.forEachIndexed { i, title ->
                            Tab(
                                selected = selectedTab == i,
                                onClick  = { selectedTab = i },
                                text     = { Text(title) }
                            )
                        }
                    }

                    when (selectedTab) {

                        // ════════════════════════════════════════
                        // Tab 0 — 配置页
                        // ════════════════════════════════════════
                        0 -> Column(
                            modifier = Modifier
                                .fillMaxSize()
                                .padding(12.dp)
                                .verticalScroll(rememberScrollState()),
                            verticalArrangement = Arrangement.spacedBy(10.dp)
                        ) {
                            Text("仿真参数", style = MaterialTheme.typography.titleMedium)

                            OutlinedTextField(
                                value = tEndText,
                                onValueChange = { tEndText = it },
                                label = { Text("仿真总时长 tEnd (s)") },
                                modifier = Modifier.fillMaxWidth(),
                                singleLine = true
                            )

                            OutlinedTextField(
                                value = dtText,
                                onValueChange = { dtText = it },
                                label = { Text("控制步长 dt (s)") },
                                modifier = Modifier.fillMaxWidth(),
                                singleLine = true
                            )

                            Row(verticalAlignment = Alignment.CenterVertically) {
                                Switch(checked = useRK45, onCheckedChange = { useRK45 = it })
                                Spacer(Modifier.width(8.dp))
                                Text(if (useRK45) "积分器：RK45（自适应步长，精确）" else "积分器：RK4（定步长，快速）")
                            }

                            HorizontalDivider()
                            Text("动力学模型", style = MaterialTheme.typography.titleMedium)

                            Row(verticalAlignment = Alignment.CenterVertically) {
                                Switch(checked = usePython, onCheckedChange = { usePython = it })
                                Spacer(Modifier.width(8.dp))
                                Text(if (usePython) "模式：Python 脚本" else "模式：内置 Kotlin 四旋翼模型")
                            }

                            if (usePython) {
                                Text(
                                    "Python 模型脚本（需定义 STATE_DIM, CTRL_DIM, deriv(t,x,u)）",
                                    style = MaterialTheme.typography.bodySmall,
                                    color = MaterialTheme.colorScheme.onSurfaceVariant
                                )
                                OutlinedTextField(
                                    value = scriptCode,
                                    onValueChange = { scriptCode = it },
                                    modifier = Modifier
                                        .fillMaxWidth()
                                        .heightIn(min = 300.dp),
                                    textStyle = LocalTextStyle.current.copy(
                                        fontFamily = FontFamily.Monospace,
                                        fontSize = 12.sp
                                    ),
                                    label = { Text("model.py") }
                                )
                            }
                        }

                        // ════════════════════════════════════════
                        // Tab 1 — 运行页
                        // ════════════════════════════════════════
                        1 -> Column(
                            modifier = Modifier
                                .fillMaxSize()
                                .padding(12.dp)
                        ) {
                            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                                Button(
                                    onClick = {
                                        if (!isRunning) {
                                            logText = "Simulation started…\n"
                                            isRunning = true
                                            dataPage = 0
                                            startSim(
                                                tEnd       = tEndText.toDoubleOrNull() ?: 30.0,
                                                dt         = dtText.toDoubleOrNull()   ?: 0.005,
                                                useRK45    = useRK45,
                                                usePython  = usePython,
                                                scriptCode = scriptCode,
                                                onLog = { line -> logText += line + "\n" },
                                                onDone = { headers, rows ->
                                                    csvHeaders = headers
                                                    csvRows    = rows
                                                    isRunning  = false
                                                    logText   += "\n✅ 仿真完成，共 ${rows.size} 行数据\n"
                                                }
                                            )
                                        }
                                    },
                                    enabled = !isRunning
                                ) { Text("▶ Start") }

                                Button(onClick = {
                                    simLoop?.stop()
                                    isRunning = false
                                    logText += "\n[Stopped]\n"
                                }) { Text("■ Stop") }
                            }

                            Spacer(Modifier.height(8.dp))
                            HorizontalDivider()
                            Spacer(Modifier.height(4.dp))

                            Box(
                                modifier = Modifier
                                    .fillMaxSize()
                                    .verticalScroll(logScroll)
                            ) {
                                Text(
                                    text       = logText,
                                    fontSize   = 11.sp,
                                    fontFamily = FontFamily.Monospace,
                                    lineHeight = 16.sp
                                )
                            }
                        }

                        // ════════════════════════════════════════
                        // Tab 2 — 数据翻阅页
                        // ════════════════════════════════════════
                        2 -> Column(Modifier.fillMaxSize().padding(8.dp)) {
                            if (csvRows.isEmpty()) {
                                Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
                                    Text("暂无数据，请先在「运行」页执行仿真",
                                         color = MaterialTheme.colorScheme.onSurfaceVariant)
                                }
                            } else {
                                val totalPages = (csvRows.size + PAGE_SIZE - 1) / PAGE_SIZE
                                val pageRows   = csvRows.drop(dataPage * PAGE_SIZE).take(PAGE_SIZE)

                                // 翻页控制栏
                                Row(
                                    Modifier.fillMaxWidth(),
                                    horizontalArrangement = Arrangement.SpaceBetween,
                                    verticalAlignment = Alignment.CenterVertically
                                ) {
                                    IconButton(
                                        onClick  = { if (dataPage > 0) dataPage-- },
                                        enabled  = dataPage > 0
                                    ) { Text("◀") }

                                    Text(
                                        "第 ${dataPage + 1} / $totalPages 页  " +
                                        "（行 ${dataPage * PAGE_SIZE + 1}–${minOf((dataPage + 1) * PAGE_SIZE, csvRows.size)} / ${csvRows.size}）",
                                        style = MaterialTheme.typography.bodySmall
                                    )

                                    IconButton(
                                        onClick  = { if (dataPage < totalPages - 1) dataPage++ },
                                        enabled  = dataPage < totalPages - 1
                                    ) { Text("▶") }
                                }

                                HorizontalDivider()

                                // 表格（横向 + 纵向滚动）
                                val hScroll = rememberScrollState()
                                Box(
                                    Modifier
                                        .fillMaxSize()
                                        .horizontalScroll(hScroll)
                                ) {
                                    LazyColumn {
                                        // 表头
                                        item {
                                            Row {
                                                csvHeaders.forEach { h ->
                                                    TableCell(h, isHeader = true)
                                                }
                                            }
                                        }
                                        // 数据行
                                        items(pageRows) { row ->
                                            Row {
                                                row.forEach { cell ->
                                                    TableCell(cell)
                                                }
                                            }
                                            HorizontalDivider(color = Color.LightGray, thickness = 0.5.dp)
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // ── 仿真启动函数 ──────────────────────────────────────────
    private fun startSim(
        tEnd:       Double,
        dt:         Double,
        useRK45:    Boolean,
        usePython:  Boolean,
        scriptCode: String,
        onLog:      (String) -> Unit,
        onDone:     (headers: List<String>, rows: List<List<String>>) -> Unit
    ) {
        val model = if (usePython) {
            PythonDynamicsModel(scriptCode)
        } else {
            QuadrotorModel()
        }

        val outFile = File(getExternalFilesDir(null), "sim_output.csv")
        val logger  = CsvLogger(outFile, model)

        // 若 Python 模型定义了 controller 则使用，否则回退到内置 PID
        val pythonModel = model as? PythonDynamicsModel

        val pidZ     = PIDController(kp = 5.0,  ki = 0.5, kd = 2.0, dt = dt)
        val pidRoll  = PIDController(kp = 3.0,  ki = 0.0, kd = 0.5, dt = dt)
        val pidPitch = PIDController(kp = 3.0,  ki = 0.0, kd = 0.5, dt = dt)
        val pidYaw   = PIDController(kp = 1.0,  ki = 0.0, kd = 0.2, dt = dt)
        val goalZ    = 2.0

        simLoop = SimLoop(
            model      = model,
            controller = { t, x ->
                pythonModel?.controllerOrNull(t, x)
                    ?: if (model.ctrlDim == 4 && model.stateDim >= 9) {
                        val quadModel = if (model is QuadrotorModel) model
                                        else QuadrotorModel()
                        doubleArrayOf(
                            quadModel.mass * 9.81 + pidZ.compute(goalZ, x[2]),
                            pidRoll.compute(0.0, x[6]),
                            pidPitch.compute(0.0, x[7]),
                            pidYaw.compute(0.0, x[8])
                        )
                    } else {
                        DoubleArray(model.ctrlDim) // 零控制兜底
                    }
            },
            x0        = DoubleArray(model.stateDim),
            tEnd      = tEnd,
            dt        = dt,
            useRK45   = useRK45,
            logger    = logger,
            onLogLine = onLog,
            onDone    = {
                // 读取 CSV 并解析
                val lines   = outFile.readLines()
                val headers = lines.firstOrNull()?.split(",") ?: emptyList()
                val rows    = lines.drop(1).map { it.split(",") }
                onDone(headers, rows)
            }
        )
        simLoop!!.start(lifecycleScope)
    }

    override fun onDestroy() {
        super.onDestroy()
        simLoop?.stop()
    }
}

// ── 表格单元格组件 ──────────────────────────────────────────
@Composable
private fun TableCell(text: String, isHeader: Boolean = false) {
    Text(
        text      = text,
        modifier  = Modifier
            .width(90.dp)
            .background(if (isHeader) MaterialTheme.colorScheme.primaryContainer else Color.Transparent)
            .padding(horizontal = 4.dp, vertical = 3.dp),
        fontSize  = 11.sp,
        fontFamily = FontFamily.Monospace,
        textAlign = TextAlign.End,
        maxLines  = 1,
        color     = if (isHeader) MaterialTheme.colorScheme.onPrimaryContainer
                    else MaterialTheme.colorScheme.onSurface
    )
}
