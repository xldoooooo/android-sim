package com.xldoooooo.androidsim.logger

import android.util.Log
import com.xldoooooo.androidsim.sim.DynamicsModel
import java.io.File
import java.io.FileWriter

class CsvLogger(outputFile: File, model: DynamicsModel) {
    private val writer = FileWriter(outputFile, false)

    init {
        val header = buildString {
            append("time")
            repeat(model.stateDim) { append(",x$it") }
            repeat(model.ctrlDim)  { append(",u$it") }
        }
        writer.appendLine(header)
        writer.flush()
    }

    /** 记录一行，返回格式化字符串（供 UI 显示） */
    fun log(t: Double, x: DoubleArray, u: DoubleArray): String {
        val line = buildString {
            append("%.4f".format(t))
            x.forEach { append(",%.6f".format(it)) }
            u.forEach { append(",%.6f".format(it)) }
        }
        writer.appendLine(line)
        writer.flush()
        Log.d("SIM", line)
        return line
    }

    fun close() = writer.close()
}
