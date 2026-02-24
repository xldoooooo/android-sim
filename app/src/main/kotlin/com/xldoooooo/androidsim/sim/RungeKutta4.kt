package com.xldoooooo.androidsim.sim

/**
 * 经典四阶 Runge-Kutta 定步长积分器
 */
class RungeKutta4(private val model: DynamicsModel) {

    /**
     * 推进一步
     * @param t  当前时间
     * @param x  当前状态
     * @param u  当前控制输入（本步内保持不变）
     * @param dt 步长
     * @return   下一时刻状态
     */
    fun step(t: Double, x: DoubleArray, u: DoubleArray, dt: Double): DoubleArray {
        val k1 = model.deriv(t,          x,                        u)
        val k2 = model.deriv(t + dt / 2, x.addScaled(k1, dt / 2), u)
        val k3 = model.deriv(t + dt / 2, x.addScaled(k2, dt / 2), u)
        val k4 = model.deriv(t + dt,     x.addScaled(k3, dt),      u)

        return DoubleArray(x.size) { i ->
            x[i] + (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
        }
    }

    private fun DoubleArray.addScaled(v: DoubleArray, scale: Double) =
        DoubleArray(size) { i -> this[i] + scale * v[i] }
}
