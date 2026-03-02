# 弹道求解器完整技术文档

> **源码版本**: ballistic-shooter 分支
> **原始 C++ 实现**: [ujinf74/ballistic-solver](https://github.com/ujinf74/ballistic-solver) (MIT License)
> **Java 移植**: Jiarui Wu, 2026

---

## 目录

1. [系统概述](#1-系统概述)
2. [物理常数](#2-物理常数)
3. [三档速度系统](#3-三档速度系统)
4. [求解器参数](#4-求解器参数)
5. [核心算法](#5-核心算法)
6. [TCP 远程求解协议](#6-tcp-远程求解协议)
7. [Hood 机械子系统](#7-hood-机械子系统)
8. [Turret 机械子系统](#8-turret-机械子系统)
9. [Flywheel 子系统](#9-flywheel-子系统)
10. [数据流与线程架构](#10-数据流与线程架构)
11. [可调参数 (NetworkTables)](#11-可调参数-networktables)
12. [3D 轨迹可视化](#12-3d-轨迹可视化)
13. [里程计与视觉相机标定](#13-里程计与视觉相机标定)

---

## 1. 系统概述

弹道求解器 (`BallisticSolver`) 的核心职责：给定场地坐标系下的**目标 3D 位置**、**机器人当前位姿与速度**，在考虑**重力**和**二次空气阻力**的条件下，实时计算三个输出量：

| 输出 | 含义 | 单位 |
|------|------|------|
| Hood 角度 (θ) | 射击仰角（从水平面算起） | degrees |
| Turret 角度 (φ) | 方位角（场地坐标系，后转为机器人坐标系） | degrees |
| 飞轮转速 | 对应速度档位的 RPM | RPM |

### 1.1 求解模式

系统支持两种求解模式，通过 `SolverMode` 枚举选择：

| 模式 | 说明 |
|------|------|
| `LOCAL` | 在 roboRIO 上本地运行 RK4 + 梯度下降求解 |
| `CLIENT` | 通过 TCP 连接将求解任务发送到远程服务器（coprocessor），接收结果 |

默认使用 `CLIENT` 模式（`Shooter` 构造函数默认参数）。若 TCP 连接失败，不会自动回退到 LOCAL，而是返回 `ShotData.INVALID` 并每 3 秒尝试重连。

### 1.2 接口定义

求解器实现 `ShooterSolver` 接口：

```java
public interface ShooterSolver {
    ShotData calculateShot(Pose3d launcherPose, ChassisSpeeds fieldSpeeds, Translation3d actualTarget);
}
```

返回的 `ShotData` 记录：

```java
public record ShotData(
    Rotation2d hoodAngle,      // Hood 仰角
    Rotation2d turretAngle,    // Turret 方位角（场地坐标系）
    double rpm,                // 飞轮目标 RPM
    Translation3d predictedTarget,  // 预测的目标位置
    boolean validShot          // 求解是否成功
) {}
```

无效结果常量：`ShotData.INVALID = new ShotData(0°, 0°, 0, (0,0,0), false)`

---

## 2. 物理常数

### 2.1 FRC 2026 Game Piece (FUEL) 参数

| 参数 | 符号 | 值 | 说明 |
|------|------|-----|------|
| 直径 | d | 0.15 m (5.91 in) | `GAME_PIECE_DIAMETER` |
| 半径 | r | 0.075 m | `GAME_PIECE_RADIUS = d/2` |
| 横截面积 | A | π × 0.075² ≈ 0.01767 m² | `GAME_PIECE_AREA = π r²` |
| 质量 | m | 0.215 kg | `GAME_PIECE_MASS`，官方范围 0.448–0.5 lbs (203–227 g)，取中点 ~215 g |
| 阻力系数 | Cd | 0.47 | `GAME_PIECE_CD`，社区测试的泡沫球经验值 |
| 材质 | - | 高密度聚氨酯泡沫 (黄色) | 带 FIRST/AndyMark 标志 |

### 2.2 环境参数

| 参数 | 符号 | 值 |
|------|------|-----|
| 空气密度 | ρ | 1.225 kg/m³ (`AIR_DENSITY`，海平面标准值) |
| 重力加速度 | g | 9.80665 m/s² (`BallisticConstants.g`，实例字段) |
| 风速向量 | wind | (0, 0, 0) m/s (`BallisticConstants.wind`，默认无风) |

### 2.3 二次阻力系数推导

求解器使用的阻力模型为**速度平方阻力**（quadratic drag）。加速度中的阻力项为：

$$
\vec{a}_{drag} = -k_{drag} \cdot |\vec{v}_{rel}| \cdot \vec{v}_{rel}
$$

其中 $k_{drag}$ 的物理推导：

$$
k_{drag} = \frac{1}{2} \cdot \frac{C_d \cdot \rho \cdot A}{m}
$$

代入数值：

$$
k_{drag} = \frac{1}{2} \times \frac{0.47 \times 1.225 \times 0.01767}{0.215}
= \frac{0.5 \times 0.010168}{0.215}
= \frac{0.005084}{0.215}
\approx 0.02365
$$

取近似值：**`DEFAULT_DRAG_COEFFICIENT = 0.024`**

### 2.4 Shooter 几何常数

| 参数 | 值 | 说明 |
|------|-----|------|
| 射手出口高度 | 0.554035 m | `SHOOTER_HEIGHT`，地面到发射出口的垂直距离（已修正，旧值 0.59673 m 有误，见下方说明） |
| 炮塔偏移 | (-0.12827, 0.05080) m | `TurretConstants.OFFSET`，炮塔中心相对于机器人中心的 2D 偏移 |
| 炮塔 3D 位置 | (-0.131341, 0.046547, 0.391529) m | `ShooterTurret.getComponentPose3d()` 中的硬编码值 |
| Hood 相对炮塔偏移 | (0.106, 0.0, 0.0713) m | `ShooterHood.getComponentPose3d()` 中的硬编码值 |

#### SHOOTER_HEIGHT 修正说明

旧值 `0.59673 m` 是错误的，实际测量值为 `0.554035 m`。差值约 4.27 cm。旧值偏高会导致弹道求解器认为发射点更高，从而计算出偏低的仰角，实际射击会打低。修正后弹道计算更准确。

---

## 3. 三档速度系统

### 3.1 速度档位定义

根据水平距离（发射器到目标的水平投影距离）选择三个速度档位：

| 档位 | 枚举值 | 飞轮 RPM | 水平距离范围 |
|------|--------|----------|-------------|
| 近距离 | `SpeedPreset.LOW` | 1800 RPM | < 3.0 m |
| 中距离 | `SpeedPreset.MEDIUM` | 2600 RPM | 3.0 m ≤ d < 6.0 m |
| 远距离 | `SpeedPreset.HIGH` | 4500 RPM | ≥ 6.0 m |

距离阈值常数：
- `LOW_TO_MEDIUM_DISTANCE = 3.0` m
- `MEDIUM_TO_HIGH_DISTANCE = 6.0` m

选择逻辑：

```java
public static SpeedPreset getSpeedPresetForDistance(double distance) {
    if (distance < LOW_TO_MEDIUM_DISTANCE) return SpeedPreset.LOW;
    else if (distance < MEDIUM_TO_HIGH_DISTANCE) return SpeedPreset.MEDIUM;
    else return SpeedPreset.HIGH;
}
```

### 3.2 飞轮 RPS → 球速转换

#### 插值查找表（当前版本）

`BALL_VEL_TO_FLYWHEEL_VEL` (InterpolatingDoubleTreeMap)，key = 球速 (m/s)，value = 飞轮速度 (rps)：

| 球速 (m/s) | 飞轮速度 (rps) |
|------------|---------------|
| 1.2381 | 10.0 |
| 4.6097 | 20.0 |
| 6.9112 | 30.0 |
| 7.2088 | 30.0 |
| 8.6967 | 40.0 |
| 8.6545 | 40.0 |
| 10.5219 | 50.0 |

#### 插值查找表（旧版本，已替换）

| 球速 (m/s) | 飞轮速度 (rps) |
|------------|---------------|
| 1.7303 | 10.0 |
| 4.7783 | 20.0 |
| 6.7228 | 30.0 |
| 8.0715 | 40.0 |
| 9.2410 | 61.0 |
| 9.7809 | 62.0 |
| 10.1543 | 70.0 |

#### 新旧数据集差异说明

新数据集相比旧数据集有以下变化：

1. **低速段球速降低**：10 rps 对应的球速从 1.73 m/s 降至 1.24 m/s，说明新测试中低速飞轮的出球效率更低
2. **引入重复飞轮速度**：30 rps 和 40 rps 各有两个数据点（对应不同的球速测量），反映了实际测试中的测量分散性
3. **最高飞轮速度降低**：旧数据最高到 70 rps，新数据最高到 50 rps，覆盖范围更保守
4. **旧数据在高速段不均匀**：61 rps 和 62 rps 两个点过于接近，新数据消除了这个问题

#### 二次拟合公式

两组数据分别拟合了 $y = ax^2 + bx + c$ 形式的二次方程（其中 $x$ = 飞轮 RPS，$y$ = 球速 m/s）：

**旧拟合（已替换）— 间接法**：

先拟合正向关系 $\text{rps} = f(v)$，再反解：

$$
\text{flywheelRPS} = 0.8593 \cdot v_{ball}^2 - 3.2464 \cdot v_{ball} + 13.6691
$$

反解得到 $v_{ball} = f(\text{rps})$：

$$
v_{ball} = \frac{3.2464 + \sqrt{(-3.2464)^2 - 4 \times 0.8593 \times (13.6691 - \text{rps})}}{2 \times 0.8593}
$$

旧代码：

```java
private static final double A = 0.8593;
private static final double B = -3.2464;
private static final double C = 13.6691;

public static double flywheelRPSToBallVel(double flywheelRPS) {
    double disc = B * B - 4.0 * A * (C - flywheelRPS);
    if (disc < 0) return 0;
    return (-B + Math.sqrt(disc)) / (2.0 * A);
}
```

**新拟合（当前使用）— 直接法**：

直接拟合 $v_{ball} = f(\text{rps})$，无需反解，更简洁：

$$
v_{ball} = -0.00276490 \cdot \text{rps}^2 + 0.38986381 \cdot \text{rps} - 2.25855714
$$

新代码：

```java
private static final double A = -0.00276490;
private static final double B = 0.38986381;
private static final double C = -2.25855714;

public static double flywheelRPSToBallVel(double flywheelRPS) {
    double v = A * flywheelRPS * flywheelRPS + B * flywheelRPS + C;
    return Math.max(v, 0);
}
```

**新拟合验证**（各数据点的拟合精度）：

| 飞轮 RPS | 实测球速 (m/s) | 拟合球速 (m/s) |
|----------|---------------|---------------|
| 10 | 1.2381 | 1.36 |
| 20 | 4.6097 | 4.43 |
| 30 | 6.91 / 7.21 | 6.95 |
| 40 | 8.70 / 8.65 | 8.91 |
| 50 | 10.5219 | 10.32 |

#### 各档位的发射速度

每档发射速度通过 `flywheelRPSToBallVel(RPM / 60.0)` 计算：

| 档位 | RPM | RPS (RPM/60) | 发射速度 (m/s) |
|------|-----|------|------|
| LOW | 1800 | 30.0 | `flywheelRPSToBallVel(30.0)` |
| MEDIUM | 2600 | 43.333 | `flywheelRPSToBallVel(43.333)` |
| HIGH | 4500 | 75.0 | `flywheelRPSToBallVel(75.0)` |

`DEFAULT_LAUNCH_SPEED` = `MEDIUM_SPEED_LAUNCH_VEL`（中档发射速度）。

### 3.3 速度缩放

发射速度和 RPM 均乘以可调参数 `LaunchSpeedScale`（默认 1.0）：

```java
double scale = launchSpeedScale.get();
double launchVelocity = BallisticConstants.getLaunchVelocityForPreset(preset) * scale;
double rpm = BallisticConstants.getRPMForPreset(preset) * scale;
```

---

## 4. 求解器参数

### 4.1 默认参数

`BallisticConstants` 实例字段（每次创建新实例时的默认值）：

| 参数 | 字段名 | 默认值 | 说明 |
|------|--------|--------|------|
| 高弧线模式 | `highArc` | `false` | true = 选择高弧线解，false = 低弧线解 |
| RK4 步长 | `dt` | 0.02 s | 每步积分的时间间隔 |
| 最大仿真时间 | `tMax` | 3.0 s | 超过此时间停止搜索最近接近点 |
| 命中容差 | `tolMiss` | 0.08 m | miss ≤ tolMiss 即视为成功 |
| 修正因子 | `beta` | 1.0 | 残差计算中的瞄准点修正缩放 |
| 最大迭代次数 | `maxIter` | 5 | 梯度下降的最大循环次数 |
| 最小俯仰角 | `thetaMin` | π/4 (45°) | 限制最低射击角 |
| 最大俯仰角 | `thetaMax` | π/2 (90°) | 限制最高射击角（正上方） |

**重要**：`BallisticSolver` 构造函数中会将 `highArc` 覆盖为 `true`：

```java
public BallisticSolver(SolverMode mode) {
    this.params = new BallisticConstants();
    this.params.highArc = true;  // 强制使用高弧线
    this.dragCoeff = BallisticConstants.DEFAULT_DRAG_COEFFICIENT;
    // ...
}
```

因此实际运行时 `highArc = true`，`thetaMin = 45°`，求解器只考虑 45°–90° 的高弧线弹道。

### 4.2 预设配置

| 预设 | `tMax` | `tolMiss` | `maxIter` | `highArc` | `thetaMin` | `thetaMax` |
|------|--------|-----------|-----------|-----------|------------|------------|
| 默认 | 3.0 s | 0.08 m | 5 | false | π/4 | π/2 |
| `closeRange()` | 1.5 s | 0.03 m | 10 | false | π/4 | π/2 |
| `longRange()` | 5.0 s | 0.08 m | 20 | false | π/4 | π/2 |
| `highArcShot()` | 5.0 s | 0.1 m | 20 | true | π/4 | π/2 |

注意：这些预设目前没有在 `Shooter.java` 中使用，实际代码使用的是默认参数 + `highArc = true` 的组合。

---

## 5. 核心算法

### 5.1 总体流程

```
输入: relPos0 (相对位置), relVel (相对速度), v0 (发射速度), kDrag (阻力系数)
  │
  ├─ 1. 真空弹道初始猜测 → (θ₀, φ₀)
  │
  ├─ 2. 梯度下降循环 (最多 maxIter 次):
  │   ├─ 计算残差 → miss 距离
  │   ├─ 若 miss ≤ tolMiss → 成功，退出
  │   ├─ 有限差分计算梯度 ∂miss/∂θ, ∂miss/∂φ
  │   ├─ 梯度下降步 + 线搜索
  │   └─ 更新 (θ, φ)
  │
  └─ 输出: SolverResult { theta, phi, miss, tStar, iterations, success }
```

### 5.2 坐标系与角度约定

- **坐标系**：WPILib 场地坐标系，X 正向为场地正前方，Y 正向为场地左方，Z 正向为上方
- **θ (theta)**：俯仰角 / 仰角，从水平面算起，正值为向上（弧度）
- **φ (phi)**：方位角 / 偏航角，从 X 轴正方向算起，逆时针为正（弧度）
- **发射方向向量**：`dir = (cos θ · cos φ, cos θ · sin φ, sin θ)`

角度与方向向量的互转：

```java
// 角度 → 方向向量
private static Vec3 anglesToDir(double theta, double phi) {
    double ct = Math.cos(theta);
    return new Vec3(ct * Math.cos(phi), ct * Math.sin(phi), Math.sin(theta));
}

// 方向向量 → 角度
private static double[] vecToAngles(Vec3 r) {
    double n = r.norm();
    if (n < 1e-12) return new double[]{0, 0};
    Vec3 d = r.times(1.0 / n);
    double theta = Math.asin(clamp(d.z, -1, 1));
    double phi = Math.atan2(d.y, d.x);
    return new double[]{theta, phi};
}
```

### 5.3 真空弹道初始猜测

#### 5.3.1 前馈补偿

先估算飞行时间，然后将目标位置沿相对速度方向推前：

```java
double t0 = relPos0.norm() / max(v0, 1e-6);
Vec3 R = relPos0.plus(relVel.times(t0));
```

#### 5.3.2 真空弹道求解 (`vacuumArcAnglesToPoint`)

给定目标点 R = (Rx, Ry, Rz) 和发射速度 v0，在无空气阻力条件下求解仰角：

**水平距离和方位角：**

$$
R_{xy} = \sqrt{R_x^2 + R_y^2}, \quad \phi = \text{atan2}(R_y, R_x)
$$

**判别式（决定是否有解）：**

$$
\Delta = v_0^4 - g \cdot (g \cdot R_{xy}^2 + 2 \cdot R_z \cdot v_0^2)
$$

- $\Delta < 0$：超出射程，无真空弹道解。退化为直线瞄准：$\theta = \text{atan2}(R_z, R_{xy})$
- $\Delta \geq 0$：有解

**仰角（两个解）：**

$$
\tan \theta = \frac{v_0^2 \pm \sqrt{\Delta}}{g \cdot R_{xy}}
$$

- **高弧线** (`highArc = true`)：取 $+\sqrt{\Delta}$（较大的 θ）
- **低弧线** (`highArc = false`)：取 $-\sqrt{\Delta}$（较小的 θ）

$$
\theta = \arctan(\tan \theta)
$$

如果结果不是有限值，退化为直线瞄准。

**角度限制：**

初始猜测后，θ 被 clamp 到 `[thetaMin, thetaMax]`（即 [45°, 90°]），φ 被 wrap 到 [-π, π]。

### 5.4 RK4 四阶龙格-库塔积分

#### 5.4.1 状态向量

```java
private static class State {
    Vec3 r;  // 位置 (m)
    Vec3 v;  // 速度 (m/s)
}
```

#### 5.4.2 微分方程（含二次空气阻力）

$$
\frac{d\vec{r}}{dt} = \vec{v}
$$

$$
\frac{d\vec{v}}{dt} = \vec{a} = (0, 0, -g) - k_{drag} \cdot |\vec{v}_{rel}| \cdot \vec{v}_{rel}
$$

其中 $\vec{v}_{rel} = \vec{v} - \vec{v}_{wind}$（风速默认为零）。

```java
private static void deriv(State s, double kDrag, double g, Vec3 wind, State ds) {
    ds.r = s.v.copy();
    Vec3 a = new Vec3(0, 0, -g);
    Vec3 vRelAir = s.v.minus(wind);
    double speed = vRelAir.norm();
    if (speed > 1e-12 && kDrag != 0.0) {
        a = a.minus(vRelAir.times(kDrag * speed));
    }
    ds.v = a;
}
```

#### 5.4.3 RK4 步进

标准四阶龙格-库塔方法，步长 h = dt = 0.02 s：

$$
k_1 = f(s_n)
$$
$$
k_2 = f(s_n + \frac{h}{2} k_1)
$$
$$
k_3 = f(s_n + \frac{h}{2} k_2)
$$
$$
k_4 = f(s_n + h \cdot k_3)
$$
$$
s_{n+1} = s_n + \frac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)
$$

对位置和速度分别应用：

```java
s.r = s.r + (h/6)(k1.r + 2·k2.r + 2·k3.r + k4.r)
s.v = s.v + (h/6)(k1.v + 2·k2.v + 2·k3.v + k4.v)
```

### 5.5 最近接近点查找 (`findClosestApproach`)

#### 5.5.1 目标运动模型

假设目标匀速运动（实际上目标一般是静止的，但机器人在移动，所以相对速度不为零）：

$$
\vec{r}_{target}(t) = \vec{r}_{relPos0} + \vec{v}_{rel} \cdot t
$$

#### 5.5.2 相对向量

弹丸位置相对于目标位置的偏差：

$$
\vec{r}_{miss}(t) = \vec{r}_{proj}(t) - \vec{r}_{target}(t)
$$

#### 5.5.3 最近接近点检测

定义标量函数 q(t)：

$$
q(t) = \vec{r}_{miss}(t) \cdot \vec{v}_{miss}(t)
$$

其中 $\vec{v}_{miss}(t) = \vec{v}_{proj}(t) - \vec{v}_{rel}$。

- q < 0：弹丸正在接近目标
- q > 0：弹丸正在远离目标
- q = 0：最近接近点

算法逐步积分，检测 q 的**符号翻转**（从负到正）：

```java
boolean allowCheck = isLow || curr.v.z <= 0;
if (allowCheck && qPrev <= 0 && qCurr >= 0) {
    // 找到最近接近点
}
```

**高弧线约束**：当 `highArc = true` 时，`isLow = false`，所以只在 `curr.v.z <= 0`（弹丸下落阶段）时才检测符号翻转。这确保了选择的是高弧线解的下降段接近点，而不是上升段。

#### 5.5.4 线性插值精确定位

在检测到符号翻转的两个时间步之间，用线性插值求精确时间：

$$
\tau = dt \cdot \frac{-q_{prev}}{q_{curr} - q_{prev} + 10^{-12}}
$$

$$
\tau = \text{clamp}(\tau, 0, dt)
$$

$$
t^* = t_{prev} + \tau
$$

然后用 RK4 从 $t_{prev}$ 精确步进 τ 时间，得到最近接近点的弹丸位置。

#### 5.5.5 未找到符号翻转

如果在整个仿真时间 `tMax` 内未检测到符号翻转，使用最后一个时间步的位置作为结果。

### 5.6 残差计算 (`computeResidual`)

残差函数将当前角度 (θ, φ) 映射到一个 miss 距离，并计算修正方向：

1. **发射并模拟弹道**：根据 (θ, φ) 和 v0 生成初速度向量，调用 `findClosestApproach`
2. **获取最近接近点**：得到 miss 向量 $\vec{r}_{miss}$ 和时间 $t^*$
3. **计算瞄准点**：$\vec{r}_{aim} = \vec{r}_{target}(t^*)$
4. **Beta 修正**：$\vec{r}_{aimCorr} = \vec{r}_{aim} - \beta \cdot \vec{r}_{miss}$

   其中 β = 1.0，意味着将瞄准点沿 miss 向量的反方向偏移一个 miss 距离

5. **分别求解两个瞄准点的真空弹道角度**：
   - $(θ_0, φ_0)$ = `vacuumArcAnglesToPoint(aim, v0, highArc, g)`
   - $(θ_1, φ_1)$ = `vacuumArcAnglesToPoint(aimCorr, v0, highArc, g)`

6. **残差**：
   - $F_0 = θ_1 - θ_0$（俯仰角修正量）
   - $F_1 = \text{wrap\_pi}(φ_1 - φ_0)$（方位角修正量，wrap 到 [-π, π]）

7. **返回**：`(F0, F1, miss距离, tStar)`

注意：梯度下降优化的实际目标是最小化 miss 距离，残差 F0/F1 在当前代码中仅被计算但未直接用于更新步。

### 5.7 梯度下降优化 (`solve` 主循环)

#### 5.7.1 初始化

```
stepSize = 0.1
bestTheta, bestPhi, bestMiss, bestTStar = 初始值
```

#### 5.7.2 迭代循环

对每次迭代（最多 `maxIter = 5` 次）：

**a) 收敛检查**

```java
if (miss <= tolMiss) {  // tolMiss = 0.08 m
    result.success = true;
    break;
}
```

**b) 有限差分梯度计算**

使用中心差分法，步长 $h = 10^{-4}$ 弧度：

$$
\frac{\partial \text{miss}}{\partial \theta} = \frac{\text{miss}(\theta + h, \phi) - \text{miss}(\theta - h, \phi)}{2h}
$$

$$
\frac{\partial \text{miss}}{\partial \phi} = \frac{\text{miss}(\theta, \phi + h) - \text{miss}(\theta, \phi - h)}{2h}
$$

每次迭代需要 4 次额外的 `computeResidual` 调用（θ+h, θ-h, φ+h, φ-h）。

**c) 梯度下降步**

$$
d\theta = -\text{stepSize} \cdot \frac{\partial \text{miss}}{\partial \theta}
$$

$$
d\phi = -\text{stepSize} \cdot \frac{\partial \text{miss}}{\partial \phi}
$$

**d) 回溯线搜索**

初始 α = 1.0，最多尝试 8 次，每次 α 减半：

```java
for (int ls = 0; ls < 8; ls++) {
    double newTheta = clamp(theta + alpha * dTheta, thetaMin, thetaMax);
    double newPhi = wrapPi(phi + alpha * dPhi);
    double newMiss = computeResidual(newTheta, newPhi, ...).miss;
    if (newMiss < miss) {
        // 接受此步
        break;
    }
    alpha *= 0.5;
}
```

**e) 步长衰减**

如果线搜索未找到改进方向：

```java
stepSize *= 0.5;
if (stepSize < 1e-6) break;  // 步长太小，提前退出
```

#### 5.7.3 计算复杂度

每次 `computeResidual` 调用涉及：
- 1 次 `findClosestApproach`（≤ `tMax/dt` = 150 次 RK4 步进）
- 2 次 `vacuumArcAnglesToPoint`

每次迭代需要：
- 4 次梯度计算 + 最多 8 次线搜索 = 最多 12 次 `computeResidual`

总计最多：`1（初始）+ 5 × 12 = 61` 次 `computeResidual` 调用。

### 5.8 后备方案

当求解器返回非有限值（`!Double.isFinite(result.theta)`）时，使用几何后备：

```java
double fallbackTurretDeg = Math.toDegrees(Math.atan2(relY, relX));
double fallbackShootingDeg = Math.toDegrees(Math.atan2(relZ, horizontalDistance))
        + horizontalDistance * 1.5;  // 经验补偿项
```

后备结果标记 `validShot = false`。

### 5.9 辅助函数

**wrapPi**：将角度归一化到 [-π, π]：

```java
private static double wrapPi(double a) {
    a = ((a + Math.PI) % (2.0 * Math.PI));
    if (a < 0) a += 2.0 * Math.PI;
    return a - Math.PI;
}
```

---

## 6. TCP 远程求解协议

### 6.1 连接参数

| 参数 | 值 |
|------|-----|
| 服务器地址 | `10.1.14.11` (coprocessor) |
| 端口 | `5805` |
| 连接超时 | 2000 ms |
| TCP_NODELAY | `true` |
| 字节序 | **Little Endian** |

### 6.2 二进制协议格式

#### 消息头 (Header) — 9 字节

| 偏移 | 大小 | 类型 | 值 / 说明 |
|------|------|------|-----------|
| 0 | 4 | uint32 | Magic Number: `0xABCDABCD` |
| 4 | 1 | uint8 | 消息类型 |
| 5 | 4 | uint32 | 负载长度 (字节) |

消息类型：
- `1` = SOLVE_REQUEST
- `129` (0x81) = SOLVE_RESPONSE

#### 求解请求 (SOLVE_REQUEST) — 负载 57 字节

| 偏移 | 大小 | 类型 | 字段 |
|------|------|------|------|
| 0 | 8 | double | relPos0X (m) |
| 8 | 8 | double | relPos0Y (m) |
| 16 | 8 | double | relPos0Z (m) |
| 24 | 8 | double | relVelX (m/s) |
| 32 | 8 | double | relVelY (m/s) |
| 40 | 8 | double | relVelZ (m/s) |
| 48 | 8 | double | v0 (m/s) — 发射速度 |
| 56 | 1 | bool | archModeHigh (1=高弧线, 0=低弧线) |

总大小：Header(9) + Payload(57) = **66 字节**

#### 求解响应 (SOLVE_RESPONSE) — 负载 25 字节

| 偏移 | 大小 | 类型 | 字段 |
|------|------|------|------|
| 0 | 8 | double | azimuth (φ, 弧度) — 方位角 |
| 8 | 8 | double | elevation (θ, 弧度) — 仰角 |
| 16 | 8 | double | time (s) — 飞行时间 |
| 24 | 1 | bool | success (1=成功) |

总大小：Header(9) + Payload(25) = **34 字节**

### 6.3 发送的参数

从 `BallisticSolver.solveViaClient` 中：

```java
new BallisticClient.ServerSolverParams(
    relX, relY, relZ,       // 相对位置
    relVelX, relVelY, 0,    // 相对速度 (Z 分量为 0)
    launchVelocity,         // 发射速度
    true                    // 始终使用高弧线
)
```

### 6.4 响应处理

```java
if (serverResult.success || Double.isFinite(serverResult.azimuth)) {
    double shootingAngleDeg = Math.toDegrees(serverResult.elevation);
    double turretAngleDeg = MathUtil.inputModulus(Math.toDegrees(serverResult.azimuth), -180, 180);
    return new ShotData(shootingAngleDeg, turretAngleDeg, rpm, actualTarget, serverResult.success);
}
```

即使 `success = false`，只要 azimuth 是有限值就返回结果（允许降级使用不完全收敛的解）。

### 6.5 错误处理与重连

- 任何异常 → 关闭 TCP 连接，返回 `ShotData.INVALID`
- 后台线程每 3 秒检测一次，若连接断开则尝试重连
- 重连间隔：`RECONNECT_INTERVAL_MS = 3000` ms

---

## 7. Hood 机械子系统

### 7.1 角度定义

Hood 角度定义为**射击角**（从水平面算起的仰角）：

- **75.11°** = 最水平位置（硬停位置，hood 打开最大）
- **26°** = 最倾斜位置（hood 关闭最多）

注意这是**反直觉**的：角度值越大 → hood 越水平 → 射击仰角越小。这是因为角度衡量的是发射管方向相对于水平面的夹角。

### 7.2 硬件配置

| 参数 | 值 | 说明 |
|------|-----|------|
| 电机 | TalonFX (Kraken X44) | CAN ID = 20 |
| CAN 总线 | `MECHANISM_CANBUS` | 独立 CAN 总线 |
| 齿轮比 | 362/32 × 20/15 × 48/16 = **45.25** | `SensorToMechanismRatio` |
| 电机反转 | `Clockwise_Positive` | |
| 中性模式 | `Brake` | |
| 最大电压 | ±1.5 V | 正反向均限制 |

### 7.3 PID 参数

| 参数 | 值 |
|------|-----|
| kP | 300.0 |
| kI | 0.0 |
| kD | 5.0 |
| kS | 0.0 |
| kV | 0.0 |
| kA | 0.0 |
| kG | 0.0 |

控制模式：`PositionVoltage`（位置控制，输出电压）。

### 7.4 软限位

| 限位 | 值 (degrees) | 值 (rotations) |
|------|-------------|----------------|
| Forward (正向) | 75.11° (MAX_ANGLE) | 75.11/360 |
| Reverse (反向) | 26° (MIN_ANGLE) | 26/360 |

手动电压模式下临时禁用软限位，退出时恢复。

### 7.5 零位校准

零位校准命令 `zero()`：

```java
motor.setPosition(Degrees.of(HoodConstants.OFFSET));  // OFFSET = 75.11°
zeroed = true;
```

在初始化时自动执行一次：`motor.setPosition(Degrees.of(HoodConstants.OFFSET))`。

`zeroed` 标志控制是否接受 `setGoal()` 指令——未校零时忽略所有目标设置。

### 7.6 角度容差

`ANGLE_TOLERANCE = 0.5°`，用于判断 Hood 是否到达目标位置：

```java
public boolean atTarget() {
    return getAngle().isNear(targetAngle, HoodConstants.ANGLE_TOLERANCE);
}
```

### 7.7 仿真

- 使用 `DCMotorSim` + `LinearSystemId.createDCMotorSystem`
- 电机模型：Kraken X44
- 转动惯量：0.001 kg·m²
- 仿真更新周期：0.020 s

### 7.8 3D 可视化位姿

Hood 的 3D 位姿基于 Turret 位姿计算：

```java
public Pose3d getComponentPose3d(Pose3d turretPose) {
    double offsetX = 0.106;
    double offsetY = 0.0;
    // 将偏移旋转到场地坐标系
    double fieldOffsetX = cos(yaw) * offsetX - sin(yaw) * offsetY;
    double fieldOffsetY = sin(yaw) * offsetX + cos(yaw) * offsetY;
    return new Pose3d(
        turretPose.X + fieldOffsetX,
        turretPose.Y + fieldOffsetY,
        turretPose.Z + 0.0713,
        new Rotation3d(0, -hoodAngle, yaw)
    );
}
```

---

## 8. Turret 机械子系统

### 8.1 硬件配置

| 参数 | 值 | 说明 |
|------|-----|------|
| 电机 | TalonFX (Kraken X60) | CAN ID = 19 |
| CAN 总线 | `MECHANISM_CANBUS` | |
| 齿轮比 | **9.2** | `SensorToMechanismRatio` |
| 电机反转 | `Clockwise_Positive` | |
| 中性模式 | `Brake` | |
| 最大电压 | ±2.0 V | |

### 8.2 PID 参数

| 参数 | 值 |
|------|-----|
| kP | 60.0 |
| kI | 15.0 |
| kD | 3.0 |
| kS | 0.15 |
| kV | 0.0 |
| kA | 0.0 |

控制模式：`PositionVoltage`。

### 8.3 旋转范围

| 参数 | 值 |
|------|-----|
| 正向限位 | +130° (Forward Limit) |
| 反向限位 | -230° (Backward Limit) |
| 总旋转范围 | 360° |
| 编码器偏移 | -0.5 rotations (-180°) |
| 收纳位置 | -0.5 rotations (-180°，传感器零位) |

### 8.4 场地角 → 机器人角转换

在 `Shooter.periodic()` 中：

```java
targetTurretAngleDeg = -(cachedFieldTurretDeg - robotPose.getRotation().getDegrees())
        + aimTrimSub.get();
```

公式含义：
- `cachedFieldTurretDeg`：求解器计算的场地坐标系方位角
- `robotPose.getRotation().getDegrees()`：机器人当前航向
- 取负号并加上 aimTrim 微调

### 8.5 安全范围归一化 (`normalizeToSafeRange`)

将任意角度映射到 Turret 的安全旋转范围 [-230°, +130°]：

```java
private static Angle normalizeToSafeRange(Angle angle) {
    double deg = angle.in(Degrees);
    double lo = -230.0;  // BACKWARD_LIMIT
    double hi = +130.0;  // FORWARD_LIMIT
    while (deg > hi) deg -= 360.0;
    while (deg < lo) deg += 360.0;
    if (deg > hi) deg = hi;
    else if (deg < lo) deg = lo;
    return Degrees.of(deg);
}
```

### 8.6 零位传感器

| 参数 | 值 |
|------|-----|
| 传感器类型 | 光电传感器 (Photoelectric) |
| DIO 端口 | 7 |
| 触发逻辑 | Active Low (遮挡时 DIO 输出 LOW → isZeroSensorGreen = true) |

零位校准：`motor.setPosition(TurretConstants.ENCODER_OFFSET)` → 将当前位置设为 -0.5 rotations。

### 8.7 角度容差

`ANGLE_TOLERANCE = 0.5°`：

```java
public boolean atTarget() {
    return motor.getPosition().getValue().isNear(targetAngle, TurretConstants.ANGLE_TOLERANCE);
}
```

### 8.8 仿真

- 使用 `DCMotorSim` + `LinearSystemId.createDCMotorSystem`
- 电机模型：Kraken X60
- 转动惯量：0.0818917055 kg·m²

### 8.9 3D 可视化位姿

```java
public Pose3d getComponentPose3d() {
    return new Pose3d(
        -0.131341, 0.046547, 0.391529,
        new Rotation3d(0, 0, -turretAngle_rad)
    );
}
```

注意 yaw 取负号（与电机正方向 Clockwise_Positive 配合）。

---

## 9. Flywheel 子系统

### 9.1 硬件配置

| 参数 | 值 | 说明 |
|------|-----|------|
| 电机 | TalonFX | CAN ID = 6 |
| CAN 总线 | `MECHANISM_CANBUS` | |
| 齿轮比 | 10/9 ≈ 1.111 | `SensorToMechanismRatio` |
| 电机反转 | `Clockwise_Positive` | |
| 中性模式 | `Coast` | |
| 最大电压 | ±8.0 V | |
| 电流限制 | 40 A | |
| 最大速度 | 70 rps | `MAX_VELOCITY_RPS` |

### 9.2 PID 参数

| 参数 | 值 |
|------|-----|
| kP | 12.0 |
| kI | 0.0 |
| kD | 0.0 |
| kS | 15.4 |
| kV | 0.2 |
| kA | 45.0 |

控制模式：`VelocityTorqueCurrentFOC`（FOC 速度控制，电流环）。

所有 PID 参数均可通过 NetworkTables `DogLog.tunable` 在线调节。

### 9.3 速度设置

输入为 RPM，内部转换为 RPS 并限幅：

```java
public void setVelocity(double targetVel) {
    double rps = Math.min(targetVel / 60.0, FlywheelConstants.MAX_VELOCITY_RPS);
    if (rps <= 0) {
        motor.stopMotor();
    } else {
        motor.setControl(velocityRequest.withVelocity(rps));
    }
}
```

### 9.4 速度容差

`VELOCITY_TOLERANCE = 2.0 rps`：

```java
public boolean atVelocity() {
    return motor.getVelocity().getValue()
            .isNear(targetVel, RotationsPerSecond.of(FlywheelConstants.VELOCITY_TOLERANCE));
}
```

### 9.5 空闲速度

`IDLE_VELOCITY = 0.0 RPM`（不射击时飞轮停转）。

---

## 10. 数据流与线程架构

### 10.1 整体架构

```
主线程 (Robot.periodic, 50Hz / 20ms)
  │
  ├─ Shooter.periodic()
  │   ├─ 读取最新求解结果 (AtomicReference, 无阻塞)
  │   ├─ 每 3 个周期提交新的 SolverInput
  │   ├─ 每周期都用缓存结果指令 Turret/Hood/Flywheel
  │   └─ 更新轨迹可视化 + 日志
  │
  └─ ShooterTurret/ShooterHood/ShooterFlywheels.periodic()
      └─ 各自的日志输出

后台线程 (BallisticSolver, daemon)
  │
  └─ solverLoop():
      ├─ 检测 CLIENT 重连需求 (每 3s)
      ├─ 消费 pendingSolverInput (AtomicReference)
      ├─ 调用 solver.calculateShot()
      └─ 将结果写入 latestSolverOutput (AtomicReference)
```

### 10.2 线程通信

使用两个 `AtomicReference` 实现无锁的生产者-消费者模式：

| AtomicReference | 生产者 | 消费者 | 方向 |
|-----------------|--------|--------|------|
| `pendingSolverInput` | 主线程 (set) | 后台线程 (getAndSet null) | 主 → 后台 |
| `latestSolverOutput` | 后台线程 (set) | 主线程 (getAndSet null) | 后台 → 主 |

### 10.3 提交节奏

```java
private static final int SOLVER_SUBMIT_PERIOD = 3;  // 每 3 次 periodic() 提交一次
```

即每 60ms 提交一次求解请求（50Hz × 3 = ~16.67Hz 求解频率）。

提交条件：
1. `periodicCounter >= SOLVER_SUBMIT_PERIOD`
2. 前一个输入已被消费 (`pendingSolverInput.get() == null`)

### 10.4 输入准备流程

每次提交时：

1. 计算发射器场地坐标位置：
   ```
   offsetField = TurretConstants.OFFSET.rotateBy(robotHeading)
   shooterPos2d = robotPos + offsetField
   shooterPos3d = (shooterPos2d.X, shooterPos2d.Y, SHOOTER_HEIGHT)
   ```

2. 计算水平距离和速度档位

3. 将机器人速度从机器人坐标系转换到场地坐标系：
   ```
   fieldVel = (vx, vy).rotateBy(robotHeading)
   ```

4. 应用目标偏移：
   ```
   adjustedTarget = targetPose + (targetOffsetX, targetOffsetY, targetOffsetZ)
   ```

### 10.5 输出应用流程

每个 periodic() 周期（20ms）：

1. 读取后台线程结果（如果有新结果）
2. 使用当前机器人航向重新计算机器人相对的 turret 角度（即使没有新的求解结果）
3. 指令 Turret、Hood、Flywheel

这确保了：
- Turret 角度实时跟踪机器人旋转（50Hz）
- Hood 和 Flywheel 使用缓存的目标值平滑运动

### 10.6 准备就绪判定

射击准备就绪需要同时满足三个条件：

```java
public boolean isReady() {
    return isTurretOnTarget() && isHoodOnTarget() && isFlywheelReady();
}
```

各子系统容差：
- Turret：误差 < 0.5°
- Hood：误差 < 0.5°
- Flywheel：误差 < 2.0 rps

### 10.7 Armed 机制

飞轮只在 `armed = true` **且** 求解结果有效时才启动：

```java
if (armed && solverReady) {
    flywheels.setVelocity(targetFlywheelVel);
} else {
    flywheels.setVelocity(IDLE_VELOCITY);  // 0 RPM
}
```

`armed` 标志由上层 Superstructure 控制。

### 10.8 Direct Turret 模式

可通过 NetworkTables 开关 `Shooter/DirectTurret` 启用直接瞄准模式：

```java
if (directTurret) {
    double dx = target.X - shooterPose.X;
    double dy = target.Y - shooterPose.Y;
    fieldTurretDeg = Math.toDegrees(Math.atan2(dy, dx));
} else {
    fieldTurretDeg = result.turretAngle().getDegrees();  // 弹道求解器输出
}
```

Direct 模式下忽略弹道求解的方位角，直接用几何角度指向目标。

---

## 11. 可调参数 (NetworkTables)

以下所有参数通过 `DogLog.tunable()` 注册，可在运行时通过 NetworkTables 动态调整：

### 11.1 Shooter 主参数

| NT 路径 | 默认值 | 类型 | 说明 |
|---------|--------|------|------|
| `Shooter/LaunchSpeedScale` | 1.0 | double | 发射速度和 RPM 的全局缩放因子 |
| `Shooter/AimTrimDeg` | 0.0 | double | Turret 角度微调偏移量 (度) |
| `Shooter/DirectTurret` | 0.0 | double | 非零 = 启用直接几何瞄准模式 |
| `Shooter/TargetOffsetX` | 0.0 | double | 目标位置 X 偏移 (m) |
| `Shooter/TargetOffsetY` | 0.0 | double | 目标位置 Y 偏移 (m) |
| `Shooter/TargetOffsetZ` | 0.0 | double | 目标位置 Z 偏移 (m) |

### 11.2 Drive 参数

| NT 路径 | 默认值 | 类型 | 说明 |
|---------|--------|------|------|
| `Drive/YawOffsetDeg` | 0.0 | double | 机器人航向偏移补偿 (度) |

### 11.3 Hood 参数

| NT 路径 | 默认值 | 类型 | 说明 |
|---------|--------|------|------|
| `ShooterHood/Request Degrees` | 75.11 | double | 手动设置 Hood 目标角度 |

### 11.4 Turret 参数

| NT 路径 | 默认值 | 类型 | 说明 |
|---------|--------|------|------|
| `ShooterTurret/TurretRequest Deg` | 0.0 | double | 手动设置 Turret 目标角度 |

### 11.5 Flywheel 参数

| NT 路径 | 默认值 | 类型 | 说明 |
|---------|--------|------|------|
| `ShooterFlywheels/kS` | 15.4 | double | Flywheel kS 前馈 |
| `ShooterFlywheels/kV` | 0.2 | double | Flywheel kV 前馈 |
| `ShooterFlywheels/kA` | 45.0 | double | Flywheel kA 前馈 |
| `ShooterFlywheels/kP` | 12.0 | double | Flywheel kP |
| `ShooterFlywheels/kI` | 0.0 | double | Flywheel kI |
| `ShooterFlywheels/kD` | 0.0 | double | Flywheel kD |
| `ShooterFlywheels/targetAccel` | 10000.0 | double | 目标加速度 (RPM/s²) |

---

## 12. 3D 轨迹可视化

### 12.1 概述

`Shooter` 类生成一组 `Pose3d` 数组，发布到 NetworkTables 供 AdvantageScope 3D Field 视图使用。

### 12.2 算法

使用简化的 **Euler 积分**（非 RK4）进行可视化轨迹采样：

1. **初始条件**：
   - 位置：发射器的场地坐标位置
   - 速度：发射速度（分解到场地坐标系）+ 机器人场地速度

   ```java
   double ct = cos(launchAngleRad);
   double vxF = launchVel * ct * cos(turretAngle) + fieldSpeeds.vx;
   double vyF = launchVel * ct * sin(turretAngle) + fieldSpeeds.vy;
   double vzF = launchVel * sin(launchAngleRad);
   ```

2. **飞行时间估算**：
   ```
   horizDist = |target - shooterPos|_xy
   horizSpeed = sqrt(vxF² + vyF²)
   tFlight = horizDist / horizSpeed
   ```

3. **采样**：均匀采样 **20 + 1 = 21** 个点，时间间隔 `dtVis = tFlight / 20`

4. **Euler 积分步进**（每步）：

   ```java
   double speed = sqrt(vx² + vy² + vz²);
   double df = kDrag * speed;
   vx -= df * vx * dtVis;
   vy -= df * vy * dtVis;
   vz -= (g + df * vz) * dtVis;
   px += vx * dtVis;
   py += vy * dtVis;
   pz += vz * dtVis;
   ```

   其中 `kDrag = DEFAULT_DRAG_COEFFICIENT = 0.024`，`g = 9.80665`。

   注意：可视化使用的是简化的 Euler 积分而非完整的 RK4，精度较低但足够用于可视化目的。

5. **Z 限制**：`pz = max(pz, 0)`，防止轨迹穿入地面。

### 12.3 日志输出

| NT 路径 | 类型 | 说明 |
|---------|------|------|
| `Shooter/PredictedTrajectory` | Pose3d[] | 预测弹道轨迹点数组 |
| `Shooter/TurretPose3d` | Pose3d[] | Turret 的场地 3D 位姿 |
| `Shooter/HoodPose3d` | Pose3d[] | Hood 的场地 3D 位姿 |
| `Shooter/TargetPose3d` | Pose3d[] | 目标位置 |

### 12.4 其他诊断日志

| NT 路径 | 类型 | 说明 |
|---------|------|------|
| `Shooter/HasTarget` | boolean | 是否有目标 |
| `Shooter/DistanceToTarget` | double | 到目标水平距离 (m) |
| `Shooter/SpeedPreset` | string | 当前速度档位名称 |
| `Shooter/LaunchVelocity_mps` | double | 当前发射速度 (m/s) |
| `Shooter/TargetTurretAngle_deg` | double | Turret 目标角度 (机器人坐标系, 度) |
| `Shooter/ActualTurretAngle_deg` | double | Turret 实际角度 (度) |
| `Shooter/TurretError_deg` | double | Turret 误差 (度) |
| `Shooter/FieldTurretAngle_deg` | double | Turret 场地角度 (度) |
| `Shooter/RobotYaw_deg` | double | 机器人航向 (度) |
| `Shooter/TargetHoodAngle_deg` | double | Hood 目标角度 (度) |
| `Shooter/ActualHoodAngle_deg` | double | Hood 实际角度 (度) |
| `Shooter/TargetFlywheelVel_rpm` | double | 飞轮目标速度 (RPM) |
| `Shooter/TurretOnTarget` | boolean | Turret 是否到位 |
| `Shooter/HoodOnTarget` | boolean | Hood 是否到位 |
| `Shooter/FlywheelReady` | boolean | Flywheel 是否到速 |
| `Shooter/ReadyToShoot` | boolean | 整体是否准备就绪 |
| `Shooter/Solver/Mode` | string | 当前求解模式 (LOCAL/CLIENT) |
| `Shooter/Solver/ClientConnected` | boolean | TCP 客户端是否已连接 |
| `Shooter/Solver/Success` | boolean | 上次求解是否成功 |
| `Shooter/Solver/HoodAngle_deg` | double | 求解器输出的 Hood 角度 |
| `Shooter/Solver/TurretAngle_deg` | double | 求解器输出的 Turret 角度 |
| `Shooter/Solver/RPM` | double | 求解器输出的 RPM |

---

## 13. 里程计与视觉相机标定

弹道求解器的精度直接依赖于机器人位姿估计（odometry + vision fusion）。相机安装角度的标定错误会导致位姿估计偏差，进而使弹道计算的目标相对位置不准确。

### 13.1 相机配置

机器人安装了 4 个视觉相机，分布在机器人后方两侧：

| 相机名称 | NT 名称 | 位置 (x, y, z) m | Pitch | Yaw |
|----------|---------|-------------------|-------|-----|
| 左后 | `LeftCameraBack` | (-0.3212, 0.2972, 0.2023) | -30° (-π/6) | **165°** |
| 左侧 | `LeftCamera` | (-0.2634, 0.3241, 0.2023) | -30° (-π/6) | **110°** |
| 右后 | `RightCameraBack` | (-0.3387, -0.2801, 0.277) | -10° | **-165°** |
| 右侧 | `RightCamera` | (-0.2782, -0.3387, 0.277) | -10° | **-110°** |

### 13.2 相机角度修正（旧值有误）

main 分支上的旧相机 yaw 角度是错误的，已在当前分支修正。以下是变更对比：

| 相机 | 旧 yaw (main, 错误) | 新 yaw (已修正) | 差值 |
|------|---------------------|----------------|------|
| LeftCameraBack | 160° | **165°** | +5° |
| LeftCamera | 93° | **110°** | +17° |
| RightCameraBack | -160° | **-165°** | -5° |
| RightCamera | -110° | -110° | 不变 |

旧的 yaw 角度（特别是 LeftCamera 的 93° → 110°，偏差达 17°）会导致：

1. 视觉位姿估计中 AprilTag 的方位计算出现系统性偏差
2. 融合后的机器人位姿在场地坐标系中偏移
3. 弹道求解器获取的目标相对位置不准确，导致 Turret 方位角和 Hood 仰角计算错误
4. 尤其在远距离射击时，角度偏差被放大，命中率显著下降

相关 commit：main 分支最新更新为 `88662e8`（2026-03-01 07:27，在 5 小时内），包含 commit `f29fd89`（Improved network and memory management）。

### 13.3 Yaw 偏移补偿

`EagleSwerveDrivetrain` 中新增了可调的 yaw 偏移参数：

| NT 路径 | 默认值 | 说明 |
|---------|--------|------|
| `Drive/YawOffsetDeg` | 0.0 | 机器人航向偏移补偿 (度) |

```java
public Rotation2d getAdjustedYaw() {
    return getState().Pose.getRotation().plus(Rotation2d.fromDegrees(yawOffsetSub.get()));
}
```

该参数用于在不重新标定 IMU 的情况下，通过 NetworkTables 实时补偿航向偏差。

### 13.4 对弹道计算的影响

相机标定误差通过以下链路影响弹道计算：

```
相机角度错误
  → 视觉位姿估计偏差
    → 机器人场地坐标偏移
      → 目标相对位置计算错误
        → 求解器输出的 θ (Hood) 和 φ (Turret) 不准
          → 射击偏差
```

修正相机角度后，视觉里程计的位姿估计精度提升，弹道求解器获取的输入更准确，射击命中率随之改善。

---

## 附录：源文件索引

| 文件 | 路径 | 职责 |
|------|------|------|
| BallisticSolver.java | `solver/BallisticSolver.java` | 核心求解器（RK4 + 梯度下降） |
| BallisticConstants.java | `solver/BallisticConstants.java` | 物理常数、速度档位、求解器参数 |
| BallisticClient.java | `solver2/BallisticClient.java` | TCP 远程求解客户端 |
| ShooterSolver.java | `solver/ShooterSolver.java` | 求解器接口 + ShotData 定义 |
| Shooter.java | `Shooter.java` | 主控类（线程管理、坐标变换、指令下发） |
| ShooterConstants.java | `ShooterConstants.java` | 飞轮/炮塔/Hood 硬件常数 |
| ShooterHood.java | `ShooterHood.java` | Hood 机械子系统 |
| ShooterTurret.java | `ShooterTurret.java` | Turret 机械子系统 |
| ShooterFlywheels.java | `ShooterFlywheels.java` | Flywheel 机械子系统 |
