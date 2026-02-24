package com.xldoooooo.androidsim.sim

import kotlin.math.*

/**
 * Dormand-Prince RK45 自适应步长积分器（与 MATLAB ode45 同算法）
 *
 * @param model    动力学模型
 * @param absTol   绝对误差容限
 * @param relTol   相对误差容限
 * @param dtMin    最小步长
 * @param dtMax    最大步长
 */
class RungeKutta45(
    private val model: DynamicsModel,
    private val absTol: Double = 1e-6,
    private val relTol: Double = 1e-3,
    private val dtMin:  Double = 1e-7,
    private val dtMax:  Double = 0.1
) {
    // Dormand-Prince 系数
    private val c2 = 1.0 / 5;   private val c3 = 3.0 / 10
    private val c4 = 4.0 / 5;   private val c5 = 8.0 / 9

    private val a21 = 1.0 / 5
    private val a31 = 3.0 / 40;    private val a32 = 9.0 / 40
    private val a41 = 44.0 / 45;   private val a42 = -56.0 / 15;   private val a43 = 32.0 / 9
    private val a51 = 19372.0 / 6561; private val a52 = -25360.0 / 2187
    private val a53 = 64448.0 / 6561; private val a54 = -212.0 / 729
    private val a61 = 9017.0 / 3168;  private val a62 = -355.0 / 33
    private val a63 = 46732.0 / 5247; private val a64 = 49.0 / 176
    private val a65 = -5103.0 / 18656

    // 5阶权重
    private val b1 = 35.0 / 384;   private val b3 = 500.0 / 1113
    private val b4 = 125.0 / 192;  private val b5 = -2187.0 / 6784
    private val b6 = 11.0 / 84

    // 误差估计权重
    private val e1 = 71.0 / 57600;   private val e3 = -71.0 / 16695
    private val e4 = 71.0 / 1920;    private val e5 = -17253.0 / 339200
    private val e6 = 22.0 / 525;     private val e7 = -1.0 / 40

    data class StepResult(
        val tNew: Double,
        val xNew: DoubleArray,
        val dtNext: Double,
        val accepted: Boolean
    )

    fun step(t: Double, x: DoubleArray, u: DoubleArray, dt: Double): StepResult {
        val n = x.size

        fun f(tt: Double, xx: DoubleArray) = model.deriv(tt, xx, u)
        fun DoubleArray.addScaled(v: DoubleArray, s: Double) =
            DoubleArray(n) { i -> this[i] + s * v[i] }

        val k1 = f(t, x)
        val k2 = f(t + c2 * dt, x.addScaled(k1, a21 * dt))
        val k3 = f(t + c3 * dt, DoubleArray(n) { i -> x[i] + (a31 * k1[i] + a32 * k2[i]) * dt })
        val k4 = f(t + c4 * dt, DoubleArray(n) { i -> x[i] + (a41 * k1[i] + a42 * k2[i] + a43 * k3[i]) * dt })
        val k5 = f(t + c5 * dt, DoubleArray(n) { i -> x[i] + (a51 * k1[i] + a52 * k2[i] + a53 * k3[i] + a54 * k4[i]) * dt })
        val k6 = f(t + dt,      DoubleArray(n) { i -> x[i] + (a61 * k1[i] + a62 * k2[i] + a63 * k3[i] + a64 * k4[i] + a65 * k5[i]) * dt })

        // 5阶解
        val x5 = DoubleArray(n) { i ->
            x[i] + dt * (b1 * k1[i] + b3 * k3[i] + b4 * k4[i] + b5 * k5[i] + b6 * k6[i])
        }

        val k7 = f(t + dt, x5)

        // 误差估计
        val err = DoubleArray(n) { i ->
            dt * (e1 * k1[i] + e3 * k3[i] + e4 * k4[i] + e5 * k5[i] + e6 * k6[i] + e7 * k7[i])
        }

        // 误差范数（相对+绝对混合）
        val errNorm = sqrt(err.indices.sumOf { i ->
            val sc = absTol + relTol * max(abs(x[i]), abs(x5[i]))
            (err[i] / sc) * (err[i] / sc)
        } / n)

        // 步长调节
        val factor = (0.9 * errNorm.pow(-0.2)).coerceIn(0.1, 5.0)
        val dtNext = (dt * factor).coerceIn(dtMin, dtMax)

        return if (errNorm <= 1.0) {
            StepResult(t + dt, x5, dtNext, accepted = true)
        } else {
            StepResult(t, x, dtNext, accepted = false)
        }
    }
}
