"""
NT Debug Monitor - reads debug NT entries from robot and writes to local debug.log
"""
import sys, time, json
from networktables import NetworkTables

LOG_PATH = "/Users/minecraftxz.com/Desktop/robotics/2026-RobotCode/.cursor/debug.log"
ip = sys.argv[1] if len(sys.argv) > 1 else "10.1.14.2"

print(f"Connecting to {ip}...")
NetworkTables.initialize(server=ip)
time.sleep(2)
print(f"NT Connected: {NetworkTables.isConnected()}")

dbg = NetworkTables.getTable("Debug")
fw = NetworkTables.getTable("Shooter").getSubTable("Flywheel")
hd = NetworkTables.getTable("Shooter").getSubTable("Hood")
tr = NetworkTables.getTable("Shooter").getSubTable("Turret")

def log(hyp, msg, data):
    entry = {
        "timestamp": int(time.time()*1000),
        "location": "nt_debug_monitor.py",
        "message": msg,
        "data": data,
        "hypothesisId": hyp
    }
    with open(LOG_PATH, "a") as f:
        f.write(json.dumps(entry) + "\n")

print("Monitoring for 30 seconds... Press Ctrl+C to stop early.")
print("Try pressing A, B, X, Y, bumpers on your controller.")
print("=" * 60)

try:
    for i in range(150):  # 30 seconds at 200ms intervals
        ds_en = dbg.getBoolean("H1_DSEnabled", False)
        ds_teleop = dbg.getBoolean("H1_DSTeleop", False)
        btn_a = dbg.getBoolean("H2_CtrlPort0_A", False)
        btn_b = dbg.getBoolean("H2_CtrlPort0_B", False)
        btn_x = dbg.getBoolean("H2_CtrlPort0_X", False)
        btn_y = dbg.getBoolean("H2_CtrlPort0_Y", False)
        btn_lb = dbg.getBoolean("H2_CtrlPort0_LB", False)
        btn_rb = dbg.getBoolean("H2_CtrlPort0_RB", False)
        ctrl_en = dbg.getBoolean("H3_ControllerEnabled", False)
        fw_target = dbg.getNumber("H4_FW_TargetRPM", -1)
        a_press_ts = dbg.getNumber("H3_A_Pressed", 0)
        fw_vel = fw.getNumber("VelocityRPM", -1)
        fw_out = fw.getNumber("AppliedOutput", -1)
        hd_angle = hd.getNumber("CurrAngDeg", -1)
        hd_goal = hd.getNumber("GoalDeg", -1)
        hd_volts = hd.getNumber("OutputVolts", -1)

        # Log every 1 second
        if i % 5 == 0:
            log("H1", "RobotState", {"enabled": ds_en, "teleop": ds_teleop})
            log("H2", "ButtonState", {"A": btn_a, "B": btn_b, "X": btn_x, "Y": btn_y, "LB": btn_lb, "RB": btn_rb})
            log("H3", "ControllerState", {"controllerEnabled": ctrl_en, "A_pressTimestamp": a_press_ts})
            log("H4", "FlywheelState", {"targetRPM": fw_target, "velocityRPM": fw_vel, "output": fw_out})
            log("H5", "HoodState", {"angleDeg": hd_angle, "goalDeg": hd_goal, "outputVolts": hd_volts})

        # Print live status
        btns = f"A={'#' if btn_a else '.'} B={'#' if btn_b else '.'} X={'#' if btn_x else '.'} Y={'#' if btn_y else '.'} LB={'#' if btn_lb else '.'} RB={'#' if btn_rb else '.'}"
        status = f"EN={'Y' if ds_en else 'N'} TEL={'Y' if ds_teleop else 'N'}"
        print(f"\r[{status}] [{btns}] ctrl={ctrl_en} FW:{fw_vel:5.0f}/{fw_target:5.0f} HD:{hd_angle:5.1f}/{hd_goal:5.1f}° V={hd_volts:5.1f}", end="", flush=True)

        time.sleep(0.2)
except KeyboardInterrupt:
    pass

print("\n\nFinal state:")
log("FINAL", "FinalSnapshot", {
    "enabled": dbg.getBoolean("H1_DSEnabled", False),
    "teleop": dbg.getBoolean("H1_DSTeleop", False),
    "btnA": dbg.getBoolean("H2_CtrlPort0_A", False),
    "controllerEnabled": dbg.getBoolean("H3_ControllerEnabled", False),
    "fwTarget": dbg.getNumber("H4_FW_TargetRPM", -1),
    "fwVel": fw.getNumber("VelocityRPM", -1),
})
print(f"Logs written to {LOG_PATH}")
NetworkTables.shutdown()
