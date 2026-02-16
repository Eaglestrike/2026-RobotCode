# CAN 与 Deploy 故障排查

## 一、日志里看到的两类问题

### 1. 代码导致崩溃（No robot code）

- **Flywheel kS = 0**  
  `Invalid parameter id (204) S 0: Feed Forward Static Gain ... invalid value is 0.000000`  
  → 本仓库已改为 `kS = 0.0001`，需**重新 build + deploy** 才会生效。

- **Hood 用错 ID**  
  若仍出现 `CAN ID 5, which is not a SPARK MAX`，说明跑的是旧代码；当前代码里 Hood 默认是 **ID 1**。

**操作：**

1. 在项目根目录执行 **clean + deploy**，确保部署的是最新代码：
   ```bash
   ./gradlew clean deploy -PteamNumber=114
   ```
2. 确认 `ShooterConstants.java` 里：
   - `FlywheelConstants.kS = 0.0001`
   - `HoodConstants.MOTOR_ID = 1`

---

### 2. CAN 总线完全不通（硬件/接线）

日志里大量出现：

- `CAN frame not received/too-stale`
- `Could not transmit CAN Frame`
- `Talon ID 1–8 failed config with error: TxFailed`
- `Encoder ID 9–12 failed config with error: TxFailed`
- `[phoenix] CANbus Failed to Connect: Drivebase` / `Mechanisms`
- `Device firmware could not be retrieved ... the device is powered`

说明 **roboRIO 和所有 CAN 设备之间无法正常通信**，这是硬件/接线问题，不是软件逻辑错误。

**请逐项检查：**

| 检查项 | 说明 |
|--------|------|
| **CAN 终端电阻** | 总线两端各有一个 **120Ω** 终端电阻（通常 roboRIO 一侧有一个，链末端设备或专用终端有一个）。缺了或错了都会导致通信异常。 |
| **CAN 线序与接线** | 使用 **CAN 总线**（CAN-H / CAN-L / GND），不要接错到其它串口或电源。确认所有设备都在**同一条 CAN 链**上，且没有断线、松动。 |
| **供电** | PDP/配电板、roboRIO、所有 Talon/Spark 等设备都要**上电**。电池或 12V 电源电压足够。 |
| **是否多条 CAN 总线** | 若代码里用了「Drivebase」「Mechanisms」等不同 CAN 总线名，确认每条总线的设备都接在对应物理总线上，且每条总线都有 120Ω 终端。 |
| **设备未接或未上电** | 若当前只接了部分设备做测试，Talon/Encoder 等未接的 ID 报 TxFailed 是正常的；但若**所有** ID 都 TxFailed，多半是整条 CAN 线或供电有问题。 |

**建议步骤：**

1. 断电，检查 CAN 线（含 120Ω 终端）和电源线。
2. 只接 **一个** CAN 设备（例如一个 Talon 或一个 Spark），确认其 ID 与代码一致，再上电、deploy，看该设备是否还能报 TxFailed。
3. 若单设备仍失败，换一根已知好的 CAN 线或换一个已知好的 CAN 设备再试。

---

## 二、当前代码中的电机 ID（供对照接线）

| 子系统 | 设备类型 | CAN ID |
|--------|----------|--------|
| Flywheel | Spark Flex | 3 |
| Turret | TalonFX | 7 |
| Hood | Spark Max | 1 |

若实车接线与上表不一致，请在 Shooter Tuner 里改 Motor ID 并 Apply，或直接改 `ShooterConstants.java` 里对应 `MOTOR_ID` 后重新 deploy。

---

## 三、总结

- **“No robot code”/程序崩溃**：确保 deploy 的是**当前仓库**（含 kS=0.0001、Hood ID=1），做一次 `./gradlew clean deploy`。
- **CAN TxFailed / frame not received / CANbus Failed to Connect**：是 **CAN 接线、终端电阻或供电** 问题，需在实车上按上面表格逐项检查。
