# android-sim

在 Android 手机上运行的轻量级物理仿真 App，用于随时随地测试**感知-规划-控制**算法。

## 特性

- ✅ 纯 Kotlin，无 NDK，无第三方原生依赖
- ✅ 内置 **RK4**（定步长）和 **RK45**（自适应步长，等价于 MATLAB `ode45`）积分器
- ✅ 内置 **PID 控制器**（带积分限幅、微分低通滤波）
- ✅ 仿真结果输出到 **CSV 文件** + **Logcat**
- ✅ 示例模型：四旋翼 6-DOF 动力学

## 项目结构

```
app/src/main/kotlin/com/xldoooooo/androidsim/
├── MainActivity.kt          # 主界面（Start/Stop + 日志滚动）
├── sim/
│   ├── DynamicsModel.kt     # 动力学模型接口 ẋ = f(t, x, u)
│   ├── RungeKutta4.kt       # RK4 定步长积分器
│   ├── RungeKutta45.kt      # RK45 自适应步长积分器（Dormand-Prince）
│   ├── SimLoop.kt           # 仿真主循环（协程驱动）
│   └── models/
│       └── QuadrotorModel.kt  # 四旋翼示例模型（12状态，4控制）
├── control/
│   └── PIDController.kt     # 单通道 PID 控制器
└── logger/
    └── CsvLogger.kt         # CSV 数据记录器
```

## 快速开始

1. 用 Android Studio 打开本项目
2. 连接手机或启动模拟器
3. 点击 **Run** 或 **Build → Generate APK**
4. App 启动后点击 **▶ Start** 开始仿真
5. 仿真结束后，CSV 文件保存在：
   ```
   /Android/data/com.xldoooooo.androidsim/files/sim_output.csv
   ```
6. 用 `adb pull` 取出后用 Python/MATLAB 画图分析

## 添加自定义模型

实现 `DynamicsModel` 接口：

```kotlin
class MyRobotModel : DynamicsModel {
    override val stateDim = 6   // 自定义状态维度
    override val ctrlDim  = 2   // 自定义控制维度

    override fun deriv(t: Double, x: DoubleArray, u: DoubleArray): DoubleArray {
        // 在这里写你的状态方程 ẋ = f(t, x, u)
        return doubleArrayOf(...)
    }
}
```

## 切换积分器

在 `MainActivity.kt` 的 `SimLoop` 构造中：

```kotlin
useRK45 = false   // RK4 定步长（快）
useRK45 = true    // RK45 自适应步长（精）
```

## 数据可视化（电脑端）

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("sim_output.csv")

fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
axes[0].plot(df['time'], df['x2'], label='z position (m)')
axes[1].plot(df['time'], df['x5'], label='vz (m/s)')
axes[2].plot(df['time'], df['u0'], label='thrust (N)')
for ax in axes:
    ax.legend(); ax.grid(True)
plt.xlabel('Time (s)')
plt.tight_layout()
plt.savefig('result.png', dpi=150)
plt.show()
```
