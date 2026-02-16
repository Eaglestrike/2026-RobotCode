"""Quick 15-second NT debug check"""
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
        f.write(json.dumps({"timestamp": int(time.time()*1000), "location": "nt_quick_check.py",
            "message": msg, "data": data, "hypothesisId": hyp}) + "\n")

print(f"Connected: {NetworkTables.isConnected()}")

# Check if debug entries exist (proves new code is deployed)
has_debug = dbg.getBoolean("H1_DSEnabled", None) is not None
print(f"Debug NT entries exist: {has_debug}")

for i in range(75):  # 15 seconds
    ds_en = dbg.getBoolean("H1_DSEnabled", False)
    ds_teleop = dbg.getBoolean("H1_DSTeleop", False)
    btn_a = dbg.getBoolean("H2_CtrlPort0_A", False)
    btn_b = dbg.getBoolean("H2_CtrlPort0_B", False)
    ctrl_en = dbg.getBoolean("H3_ControllerEnabled", False)
    fw_target = dbg.getNumber("H4_FW_TargetRPM", -1)
    fw_vel = fw.getNumber("VelocityRPM", -1)
    fw_cur_target = fw.getNumber("CurrentTargetRPM", -1)
    hd_angle = hd.getNumber("CurrAngDeg", -1)
    hd_volts = hd.getNumber("OutputVolts", -1)

    if i % 5 == 0:
        log("H1", "RobotState", {"enabled": ds_en, "teleop": ds_teleop})
        log("H2", "Buttons", {"A": btn_a, "B": btn_b})
        log("H3", "Controller", {"enabled": ctrl_en, "fwTarget": fw_target})
        log("H4", "Flywheel", {"vel": fw_vel, "target": fw_cur_target})
        log("H5", "Hood", {"angle": hd_angle, "volts": hd_volts})

    en_str = "EN" if ds_en else "DIS"
    tel_str = "TEL" if ds_teleop else "---"
    print(f"\r[{en_str}|{tel_str}] A={'#' if btn_a else '.'} B={'#' if btn_b else '.'} ctrl={ctrl_en} FW:{fw_vel:5.0f}/{fw_cur_target:5.0f} HD:{hd_angle:4.1f}V={hd_volts:4.1f}", end="", flush=True)
    time.sleep(0.2)

print("\nDone.")
NetworkTables.shutdown()
