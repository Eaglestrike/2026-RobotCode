"""Combined: monitor buttons + send NT flywheel command to verify motor response"""
import sys, time, json
from networktables import NetworkTables

LOG_PATH = "/Users/minecraftxz.com/Desktop/robotics/2026-RobotCode/.cursor/debug.log"
ip = sys.argv[1] if len(sys.argv) > 1 else "10.1.14.2"
NetworkTables.initialize(server=ip)
time.sleep(2)

dbg = NetworkTables.getTable("Debug")
fw = NetworkTables.getTable("Shooter").getSubTable("Flywheel")
hd = NetworkTables.getTable("Shooter").getSubTable("Hood")

def log(hyp, msg, data):
    with open(LOG_PATH, "a") as f:
        f.write(json.dumps({"timestamp": int(time.time()*1000), "message": msg, "data": data, "hypothesisId": hyp}) + "\n")

print(f"Connected: {NetworkTables.isConnected()}")
print("Phase 1: Monitoring buttons for 8s — PRESS A NOW!")

for i in range(40):
    ds_en = dbg.getBoolean("H1_DSEnabled", False)
    btn_a = dbg.getBoolean("H2_CtrlPort0_A", False)
    btn_b = dbg.getBoolean("H2_CtrlPort0_B", False)
    fw_vel = fw.getNumber("VelocityRPM", -1)
    fw_target = fw.getNumber("CurrentTargetRPM", -1)
    hd_angle = hd.getNumber("CurrAngDeg", -1)

    if i % 5 == 0:
        log("BTN", "ButtonMonitor", {"A": btn_a, "B": btn_b, "enabled": ds_en, "fwVel": fw_vel, "fwTarget": fw_target})

    en = "EN" if ds_en else "DIS"
    print(f"\r[{en}] A={'#' if btn_a else '.'} B={'#' if btn_b else '.'} FW:{fw_vel:5.0f}/{fw_target:5.0f} HD:{hd_angle:4.1f}", end="", flush=True)
    time.sleep(0.2)

print("\n\nPhase 2: Sending FW=3000 via NT ManualControl for 5s...")
fw.putBoolean("ManualControl", True)
fw.putNumber("TargetRPM", 3000)

for i in range(25):
    fw_vel = fw.getNumber("VelocityRPM", -1)
    fw_target = fw.getNumber("CurrentTargetRPM", -1)
    hd_angle = hd.getNumber("CurrAngDeg", -1)
    if i % 5 == 0:
        log("NT_CMD", "NTFlywheelTest", {"vel": fw_vel, "target": fw_target})
    print(f"\r  FW:{fw_vel:6.0f}/{fw_target:6.0f} HD:{hd_angle:5.1f}", end="", flush=True)
    time.sleep(0.2)

print("\n\nPhase 3: Stopping flywheel...")
fw.putNumber("TargetRPM", 0)
time.sleep(1)
fw.putBoolean("ManualControl", False)

fw_vel = fw.getNumber("VelocityRPM", -1)
log("FINAL", "AfterStop", {"vel": fw_vel})
print(f"FW vel after stop: {fw_vel:.0f}")
print("Done.")
NetworkTables.shutdown()
