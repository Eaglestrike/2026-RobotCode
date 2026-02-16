"""Just monitor buttons and flywheel for 12 seconds"""
import sys, time, json
from networktables import NetworkTables

LOG_PATH = "/Users/minecraftxz.com/Desktop/robotics/2026-RobotCode/.cursor/debug.log"
ip = sys.argv[1] if len(sys.argv) > 1 else "10.1.14.2"
NetworkTables.initialize(server=ip)
time.sleep(2)

dbg = NetworkTables.getTable("Debug")
fw = NetworkTables.getTable("Shooter").getSubTable("Flywheel")
hd = NetworkTables.getTable("Shooter").getSubTable("Hood")

def log(msg, data):
    with open(LOG_PATH, "a") as f:
        f.write(json.dumps({"timestamp": int(time.time()*1000), "message": msg, "data": data}) + "\n")

print(f"Connected: {NetworkTables.isConnected()}")
print("=" * 50)
print("  PRESS AND HOLD A FOR 5 SECONDS NOW!")
print("=" * 50)

for i in range(60):  # 12 seconds
    btn_a = dbg.getBoolean("H2_CtrlPort0_A", False)
    btn_b = dbg.getBoolean("H2_CtrlPort0_B", False)
    fw_vel = fw.getNumber("VelocityRPM", -1)
    fw_target = fw.getNumber("CurrentTargetRPM", -1)
    hd_angle = hd.getNumber("CurrAngDeg", -1)
    hd_goal = hd.getNumber("GoalDeg", -1)

    log("poll", {"A": btn_a, "B": btn_b, "fwVel": round(fw_vel), "fwTarget": round(fw_target), "hdAngle": round(hd_angle,1), "hdGoal": round(hd_goal,1)})

    a_str = "##A##" if btn_a else "  .  "
    b_str = "##B##" if btn_b else "  .  "
    print(f"\r [{a_str}] [{b_str}] FW:{fw_vel:6.0f}/{fw_target:6.0f} HD:{hd_angle:5.1f}/{hd_goal:5.1f}", end="", flush=True)
    time.sleep(0.2)

print("\nDone.")
NetworkTables.shutdown()
