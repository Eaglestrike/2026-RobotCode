"""Monitor hood + flywheel. User should press POV Up / POV Down / A / B"""
import sys, time
from networktables import NetworkTables

ip = sys.argv[1] if len(sys.argv) > 1 else "10.1.14.2"
NetworkTables.initialize(server=ip)
time.sleep(2)

dbg = NetworkTables.getTable("Debug")
fw = NetworkTables.getTable("Shooter").getSubTable("Flywheel")
hd = NetworkTables.getTable("Shooter").getSubTable("Hood")

print(f"Connected: {NetworkTables.isConnected()}")
print("=" * 60)
print("  POV Up   = Hood → 54.7° (最陡)")
print("  POV Down = Hood → 26.5° (最平)")
print("  A        = Flywheel 4500 RPM")
print("  B        = Flywheel 3000 RPM")
print("=" * 60)

for i in range(100):  # 20 seconds
    fw_vel = fw.getNumber("VelocityRPM", -1)
    fw_target = fw.getNumber("CurrentTargetRPM", -1)
    hd_angle = hd.getNumber("CurrAngDeg", -1)
    hd_goal = hd.getNumber("GoalDeg", -1)
    hd_volts = hd.getNumber("OutputVolts", -1)
    hd_pid = hd.getNumber("PIDVolts", -1)
    hd_vel = hd.getNumber("VelocityDegPerS", -1)
    hd_at = hd.getBoolean("AtTarget", False)

    at = "OK" if hd_at else ".."
    print(f"\r HD: {hd_angle:5.1f}/{hd_goal:5.1f}° V={hd_volts:5.1f} pid={hd_pid:5.1f} vel={hd_vel:6.1f}°/s [{at}] | FW:{fw_vel:6.0f}/{fw_target:6.0f}", end="", flush=True)
    time.sleep(0.2)

print("\nDone.")
NetworkTables.shutdown()
