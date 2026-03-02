# C++ 弹道求解服务 (frc_server)

与 Java 端 CLIENT 模式配合，在 coprocessor（如树莓派 10.1.14.11）上运行，通过 TCP 5805 接收求解请求并返回俯仰/方位角。

## 编译

在带 g++ 的 Linux/macOS 上：

```bash
cd server
g++ -O3 -std=c++17 -o frc_server frc_server.cpp
```

无额外依赖，仅需标准库与 POSIX 网络。

## 运行

- 前台（含调试日志）：`./frc_server --debug`
- 前台（静默）：`./frc_server`
- 监听端口：**5805**

## 部署到 coprocessor（树莓派）

1. 在本机编译得到 `frc_server` 可执行文件。
2. 拷贝到树莓派（替换为你的 Pi 用户/IP）：
   ```bash
   scp frc_server pi@10.1.14.11:/home/pi/
   ```
3. 若使用 systemd，拷贝 service 并启用：
   ```bash
   scp frc_server.service pi@10.1.14.11:/home/pi/
   ssh pi@10.1.14.11
   sudo cp /home/pi/frc_server.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable frc_server
   sudo systemctl restart frc_server
   ```
4. 查看状态：`sudo systemctl status frc_server`

## 与 Java 的同步

- 重力、阻力、spin lift（球体内旋等效修正）与 Java 的 [BallisticConstants](src/main/java/frc/robot/subsystems/shooter/solver/BallisticConstants.java) / [BallisticSolver](src/main/java/frc/robot/subsystems/shooter/solver/BallisticSolver.java) 保持一致。
- C++ 端 `spinLiftFactor` 默认 0.0；若需与 Java 的 `Shooter/SpinLiftFactor` 一致，需在 C++ 中改为从环境变量读取或后续扩展协议传入。
