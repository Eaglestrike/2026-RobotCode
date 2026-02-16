"""
Auto Hood Debug - Connects to robot NT, monitors hood state,
and sends test commands via NT RequestDeg to verify hood movement.
Logs all data to .cursor/debug.log in NDJSON format.

Usage: python3 tools/nt_auto_hood_debug.py [robot_ip]
Default IP: 10.1.14.2
"""
import sys, time, json, os

from networktables import NetworkTables

LOG_PATH = os.path.join(os.path.dirname(__file__), "..", ".cursor", "debug.log")
ip = sys.argv[1] if len(sys.argv) > 1 else "10.1.14.2"

def log(msg, data=None, hyp=""):
    entry = {
        "timestamp": int(time.time() * 1000),
        "location": "nt_auto_hood_debug.py",
        "message": msg,
        "hypothesisId": hyp,
    }
    if data:
        entry["data"] = data
    with open(LOG_PATH, "a") as f:
        f.write(json.dumps(entry) + "\n")
    # Also print
    short = json.dumps(data) if data else ""
    print(f"[{hyp or 'INFO'}] {msg} {short}")

print(f"Connecting to {ip}...")
NetworkTables.initialize(server=ip)
time.sleep(2)

connected = NetworkTables.isConnected()
log("NT connection status", {"connected": connected, "ip": ip}, "H1")

if not connected:
    print("ERROR: Not connected to robot! Is the robot on and connected?")
    log("ABORT: NT not connected", {}, "H1")
    sys.exit(1)

dbg = NetworkTables.getTable("Debug")
hd = NetworkTables.getTable("Shooter").getSubTable("Hood")
fw = NetworkTables.getTable("Shooter").getSubTable("Flywheel")

def read_state():
    """Read all relevant state from NT"""
    return {
        # H1: Robot enabled state
        "ds_enabled": dbg.getBoolean("H1_DSEnabled", False),
        "ds_teleop": dbg.getBoolean("H1_DSTeleop", False),
        # H2: Hood goal
        "hd_goal_deg": hd.getNumber("GoalDeg", -999),
        "hd_request_deg": hd.getNumber("RequestDeg", -999),
        # H3: PID output
        "hd_pid_volts": hd.getNumber("PIDVolts", -999),
        "hd_ff_volts": hd.getNumber("FFVolts", -999),
        "hd_output_volts": hd.getNumber("OutputVolts", -999),
        # H4: Encoder
        "hd_curr_deg": hd.getNumber("CurrAngDeg", -999),
        "hd_vel_dps": hd.getNumber("VelocityDegPerS", -999),
        "hd_error_deg": hd.getNumber("ErrorDeg", -999),
        "hd_at_target": hd.getBoolean("AtTarget", False),
        # H5: Controller state
        "ctrl_enabled": dbg.getBoolean("H3_ControllerEnabled", False),
        # Raw buttons
        "raw_buttons": dbg.getString("H2_RawButtons", "?"),
    }

# Phase 1: Read initial state (5 samples over 2.5 seconds)
log("=== Phase 1: Initial state monitoring (2.5s) ===", {})
for i in range(5):
    state = read_state()
    log(f"Initial sample {i}", state, "H1,H2,H3,H4,H5")
    time.sleep(0.5)

# Check H1: Is robot enabled? Wait up to 30 seconds
state = read_state()
if not state["ds_enabled"]:
    print("\n*** Robot NOT enabled yet. Waiting for Enable (up to 30s)... ***")
    log("H1: Robot not enabled, waiting...", state, "H1")
    for wait_i in range(60):
        time.sleep(0.5)
        state = read_state()
        if state["ds_enabled"]:
            break
        if wait_i % 10 == 0:
            print(f"  Still waiting... ({wait_i * 0.5:.0f}s)")
    if not state["ds_enabled"]:
        log("H1 CONFIRMED: Robot still NOT enabled after 30s.", state, "H1")
        print("\n*** TIMEOUT: Robot not enabled after 30s. Aborting. ***")
        NetworkTables.shutdown()
        sys.exit(0)

log("H1 REJECTED: Robot IS enabled.", state, "H1")
print(f"\n>>> Robot enabled! Teleop={state['ds_teleop']}. Starting hood test...")

# Phase 2: Send Hood to MAX_ANGLE (54.7°) via NT RequestDeg
MAX_ANGLE = 54.7
MIN_ANGLE = 26.5

log(f"=== Phase 2: Sending Hood to MAX_ANGLE={MAX_ANGLE}° via NT ===", {})
hd.putNumber("RequestDeg", MAX_ANGLE)
time.sleep(0.3)

# Monitor for 4 seconds
samples_phase2 = []
for i in range(20):
    state = read_state()
    samples_phase2.append(state)
    log(f"Phase2 sample {i}", state, "H2,H3,H4")
    time.sleep(0.2)

# Analyze phase 2
first = samples_phase2[0]
last = samples_phase2[-1]
log("Phase2 analysis", {
    "goal_changed": abs(last["hd_goal_deg"] - MAX_ANGLE) < 1.0,
    "first_angle": first["hd_curr_deg"],
    "last_angle": last["hd_curr_deg"],
    "angle_change": last["hd_curr_deg"] - first["hd_curr_deg"],
    "last_output_volts": last["hd_output_volts"],
    "last_error": last["hd_error_deg"],
}, "H2,H3,H4")

# Phase 3: Send Hood to MIN_ANGLE (26.5°) via NT RequestDeg
log(f"=== Phase 3: Sending Hood to MIN_ANGLE={MIN_ANGLE}° via NT ===", {})
hd.putNumber("RequestDeg", MIN_ANGLE)
time.sleep(0.3)

# Monitor for 4 seconds
samples_phase3 = []
for i in range(20):
    state = read_state()
    samples_phase3.append(state)
    log(f"Phase3 sample {i}", state, "H2,H3,H4")
    time.sleep(0.2)

# Analyze phase 3
first3 = samples_phase3[0]
last3 = samples_phase3[-1]
log("Phase3 analysis", {
    "goal_changed": abs(last3["hd_goal_deg"] - MIN_ANGLE) < 1.0,
    "first_angle": first3["hd_curr_deg"],
    "last_angle": last3["hd_curr_deg"],
    "angle_change": last3["hd_curr_deg"] - first3["hd_curr_deg"],
    "last_output_volts": last3["hd_output_volts"],
    "last_error": last3["hd_error_deg"],
}, "H2,H3,H4")

# Phase 4: Final summary
log("=== Final Summary ===", {
    "H1_robot_enabled": state["ds_enabled"],
    "H2_goal_tracks_request": abs(last["hd_goal_deg"] - MAX_ANGLE) < 1.0 and abs(last3["hd_goal_deg"] - MIN_ANGLE) < 1.0,
    "H3_pid_output_nonzero": abs(last["hd_output_volts"]) > 0.1 or abs(last3["hd_output_volts"]) > 0.1,
    "H4_encoder_changed": abs(last["hd_curr_deg"] - first["hd_curr_deg"]) > 0.5 or abs(last3["hd_curr_deg"] - first3["hd_curr_deg"]) > 0.5,
    "H5_controller_enabled": state["ctrl_enabled"],
})

print("\n\nDone! Log written to .cursor/debug.log")
NetworkTables.shutdown()
