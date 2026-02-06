# Shooter Tuner (Streamlit)

Streamlit 调参页面：Flywheel、Turret、Hood、Controller、Ballistics。通过 NetworkTables 读写机器人端 DogLog 发布的键。

## 安装

```bash
cd tools/shooter_tuner
pip install -r requirements.txt
```

## 运行

```bash
streamlit run app.py
```

浏览器打开后，在 **roboRIO / Sim IP** 输入：

- 仿真：`localhost` 或 `127.0.0.1`
- 实机：roboRIO IP（如 `10.1.14.2`）

点击 **Reconnect** 连接。连接成功后各 Tab 会显示实时数据，并可在 “Tunables” 中修改参数并 **Apply** 写入 NetworkTables。

## 页面说明

| Tab | 内容 |
|-----|------|
| **Flywheel** | VelocityRPM、电流、温度；可调 kP/kI/kD/kS/kV/kA、maxAccel、targetRPM |
| **Turret** | 请求位置、电压；可调 CruiseVelo、Acceleration、kP/kD/kV/kS、TurretRequest |
| **Hood** | 当前角度、Goal、电压；可调 RequestDegrees |
| **Controller** | 目标角度、Ready、Solver 状态；可调 TunableTarget X/Y/Z、Enable |
| **Ballistics** | 参考：game piece、三档速度、solver 参数（实际常量在代码里） |

## 依赖

- `streamlit`：页面
- `pynetworktables`：连接 FRC NetworkTables
- `streamlit-autorefresh`：定时刷新
- `altair` / `pandas`：预留作图（可选）
