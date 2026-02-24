package com.xldoooooo.androidsim.control

/**
 * 单通道 PID 控制器
 * 带积分限幅、微分低通滤波
 *
 * @param kp         比例增益
 * @param ki         积分增益
 * @param kd         微分增益
 * @param dt         控制步长（秒）
 * @param intMax     积分限幅
 * @param outMin     输出下限
 * @param outMax     输出上限
 * @param dfiltTau   微分低通滤波时间常数（秒）
 */
class PIDController(
    private val kp:       Double,
    private val ki:       Double,
    private val kd:       Double,
    private val dt:       Double,
    private val intMax:   Double = 10.0,
    private val outMin:   Double = -100.0,
    private val outMax:   Double =  100.0,
    private val dfiltTau: Double = 0.01
) {
    private var integral  = 0.0
    private var prevError = 0.0
    private var filtDeriv = 0.0
    private val alpha = dt / (dfiltTau + dt)

    fun compute(setpoint: Double, measured: Double): Double {
        val error = setpoint - measured
        integral  = (integral + error * dt).coerceIn(-intMax, intMax)
        val rawDeriv = (error - prevError) / dt
        filtDeriv    = alpha * rawDeriv + (1 - alpha) * filtDeriv
        prevError    = error
        return (kp * error + ki * integral + kd * filtDeriv).coerceIn(outMin, outMax)
    }

    fun reset() {
        integral  = 0.0
        prevError = 0.0
        filtDeriv = 0.0
    }
}
