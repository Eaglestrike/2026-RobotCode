"""
Quick NT monitor - connects to the robot and reads shooter status.
Usage: python3 tools/nt_monitor.py [robot_ip]
Default IP: 10.1.14.2
"""
import sys
import time
from networktables import NetworkTables

ip = sys.argv[1] if len(sys.argv) > 1 else "10.1.14.2"

print(f"Connecting to {ip}...")
NetworkTables.initialize(server=ip)

time.sleep(2)

shooter = NetworkTables.getTable("Shooter")
flywheel = shooter.getSubTable("Flywheel")
turret = shooter.getSubTable("Turret")
hood = shooter.getSubTable("Hood")

print(f"\nNT Connected: {NetworkTables.isConnected()}")
print("=" * 60)

try:
    while True:
        connected = NetworkTables.isConnected()

        fw_vel = flywheel.getNumber("VelocityRPM", -1)
        fw_target = flywheel.getNumber("CurrentTargetRPM", -1)
        fw_enabled = flywheel.getBoolean("Enabled", False)
        fw_output = flywheel.getNumber("AppliedOutput", -1)

        tr_angle = turret.getNumber("CurrentAngle", -1)
        tr_req = turret.getNumber("RequestedPosition", -1)
        tr_at = turret.getBoolean("AtTarget", False)

        hd_angle = hood.getNumber("CurrAngDeg", -1)
        hd_goal = hood.getNumber("GoalDeg", -1)
        hd_volts = hood.getNumber("OutputVolts", -1)

        print(f"\r[NT {'OK' if connected else 'DISCONNECTED'}] "
              f"FW: {fw_vel:7.0f}/{fw_target:7.0f} RPM out={fw_output:5.2f} en={fw_enabled} | "
              f"TR: {tr_angle:6.1f}/{tr_req:6.1f}° at={tr_at} | "
              f"HD: {hd_angle:5.1f}/{hd_goal:5.1f}° V={hd_volts:5.2f}",
              end="", flush=True)

        time.sleep(0.2)
except KeyboardInterrupt:
    print("\nStopped.")
    NetworkTables.shutdown()
