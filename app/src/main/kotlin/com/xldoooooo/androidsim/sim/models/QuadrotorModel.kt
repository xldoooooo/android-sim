package com.xldoooooo.androidsim.sim.models

import com.xldoooooo.androidsim.sim.DynamicsModel
import kotlin.math.*

/**
 * 四旋翼简化动力学模型（示例）
 *
 * 状态 x (12维):
 *   [0-2]  位置        px, py, pz       (m)
 *   [3-5]  速度        vx, vy, vz       (m/s)
 *   [6-8]  欧拉角      roll, pitch, yaw (rad)
 *   [9-11] 角速度      p,   q,   r      (rad/s)
 *
 * 控制 u (4维):
 *   [0]    总推力 F    (N)
 *   [1-3]  力矩 τx, τy, τz (N·m)
 */
class QuadrotorModel(
    val mass: Double = 1.0,
    private val Ixx:  Double = 0.0196,
    private val Iyy:  Double = 0.0196,
    private val Izz:  Double = 0.0264,
    private val g:    Double = 9.81
) : DynamicsModel {

    override val stateDim = 12
    override val ctrlDim  = 4

    override fun deriv(t: Double, x: DoubleArray, u: DoubleArray): DoubleArray {
        val vx = x[3]; val vy = x[4]; val vz = x[5]
        val phi = x[6]; val theta = x[7]
        val p = x[9]; val q = x[10]; val r = x[11]

        val F    = u[0]
        val taux = u[1]; val tauy = u[2]; val tauz = u[3]

        val cphi = cos(phi);  val sphi = sin(phi)
        val cth  = cos(theta); val sth  = sin(theta)
        val cpsi = cos(x[8]); val spsi = sin(x[8])

        val ax = (cphi * sth * cpsi + sphi * spsi) * F / mass
        val ay = (cphi * sth * spsi - sphi * cpsi) * F / mass
        val az = (cphi * cth) * F / mass - g

        // Note: Euler angle kinematic equations have singularities at phi=±π/2 and
        // theta=±π/2. For normal quadrotor flight these are avoided; guards prevent NaN.
        val tphi   = sphi / cphi.coerceAtLeast(1e-6)
        val dphi   = p + (q * sphi + r * cphi) * tphi
        val dtheta = q * cphi - r * sphi
        val dpsi   = (q * sphi + r * cphi) / cth.coerceAtLeast(1e-6)

        val dp = (taux - (Izz - Iyy) * q * r) / Ixx
        val dq = (tauy - (Ixx - Izz) * p * r) / Iyy
        val dr = (tauz - (Iyy - Ixx) * p * q) / Izz

        return doubleArrayOf(vx, vy, vz, ax, ay, az, dphi, dtheta, dpsi, dp, dq, dr)
    }
}
