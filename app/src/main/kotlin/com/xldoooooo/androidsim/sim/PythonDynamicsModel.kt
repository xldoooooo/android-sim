package com.xldoooooo.androidsim.sim

import com.chaquo.python.Python

/**
 * 将用户编写的 Python 脚本桥接为 DynamicsModel。
 *
 * Python 脚本必须定义：
 *   STATE_DIM: int
 *   CTRL_DIM:  int
 *   def deriv(t: float, x: list[float], u: list[float]) -> list[float]: ...
 *   def controller(t: float, x: list[float]) -> list[float]: ...  # 可选，若无则用 Kotlin 侧控制器
 */
class PythonDynamicsModel(scriptCode: String) : DynamicsModel {

    private val module by lazy {
        val py = Python.getInstance()
        val builtins = py.getBuiltins()
        // exec 脚本到一个临时 module 命名空间
        val moduleDict = py.getModule("types").callAttr("ModuleType", "user_model").also { mod ->
            builtins.callAttr("exec", scriptCode, mod.callAttr("__dict__"))
        }
        moduleDict
    }

    override val stateDim: Int get() = module.callAttr("__getattribute__", "STATE_DIM").toInt()
    override val ctrlDim:  Int get() = module.callAttr("__getattribute__", "CTRL_DIM").toInt()

    override fun deriv(t: Double, x: DoubleArray, u: DoubleArray): DoubleArray {
        val py = Python.getInstance()
        val xList  = x.map { it }.let { py.builtins.callAttr("list", it.toTypedArray()) }
        val uList  = u.map { it }.let { py.builtins.callAttr("list", it.toTypedArray()) }
        val result = module.callAttr("deriv", t, xList, uList)
        return DoubleArray(result.asList().size) { i -> result.asList()[i].toDouble() }
    }

    /** 若脚本定义了 controller 函数则调用，否则返回 null */
    fun controllerOrNull(t: Double, x: DoubleArray): DoubleArray? {
        return try {
            val py     = Python.getInstance()
            val xList  = x.map { it }.let { py.builtins.callAttr("list", it.toTypedArray()) }
            val result = module.callAttr("controller", t, xList)
            DoubleArray(result.asList().size) { i -> result.asList()[i].toDouble() }
        } catch (e: Exception) {
            null
        }
    }
}
