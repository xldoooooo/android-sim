import math

# ── 维度声明（必须） ──────────────────────────────────────────
STATE_DIM = 12   # 状态向量维度
CTRL_DIM  = 4    # 控制输入维度

# ── 模型参数 ──────────────────────────────────────────────────
mass = 1.0
Ixx, Iyy, Izz = 0.0196, 0.0196, 0.0264
g = 9.81

# ── 状态方程 ẋ = f(t, x, u) ──────────────────────────────────
# x[0..2]  : 位置 px, py, pz (m)
# x[3..5]  : 速度 vx, vy, vz (m/s)
# x[6..8]  : 欧拉角 roll, pitch, yaw (rad)
# x[9..11] : 角速度 p, q, r (rad/s)
# u[0]     : 总推力 F (N)
# u[1..3]  : 力矩 τx, τy, τz (N·m)
def deriv(t, x, u):
    vx, vy, vz     = x[3], x[4], x[5]
    phi, theta, psi = x[6], x[7], x[8]
    p, q, r        = x[9], x[10], x[11]
    F              = u[0]
    taux, tauy, tauz = u[1], u[2], u[3]

    cphi, sphi = math.cos(phi), math.sin(phi)
    cth,  sth  = math.cos(theta), math.sin(theta)
    cpsi, spsi = math.cos(psi),  math.sin(psi)

    ax = (cphi * sth * cpsi + sphi * spsi) * F / mass
    ay = (cphi * sth * spsi - sphi * cpsi) * F / mass
    az = (cphi * cth) * F / mass - g

    tphi   = sphi / max(abs(cphi), 1e-6) * (1 if cphi >= 0 else -1)
    dphi   = p + (q * sphi + r * cphi) * tphi
    dtheta = q * cphi - r * sphi
    dpsi   = (q * sphi + r * cphi) / max(abs(cth), 1e-6)

    dp = (taux - (Izz - Iyy) * q * r) / Ixx
    dq = (tauy - (Ixx - Izz) * p * r) / Iyy
    dr = (tauz - (Iyy - Ixx) * p * q) / Izz

    return [vx, vy, vz, ax, ay, az, dphi, dtheta, dpsi, dp, dq, dr]


# ── 控制器（可选） ────────────────────────────────────────────
# 若定义此函数，将替代 Kotlin 侧的默认 PID 控制器
# def controller(t, x):
#     # 示例：悬停 PID（简化）
#     goal_z = 2.0
#     ez = goal_z - x[2]
#     F  = mass * g + 5.0 * ez
#     return [F, 0.0, 0.0, 0.0]
