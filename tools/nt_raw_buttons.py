"""Read raw button data from robot — press EVERY button on your controller"""
import sys, time
from networktables import NetworkTables

ip = sys.argv[1] if len(sys.argv) > 1 else "10.1.14.2"
NetworkTables.initialize(server=ip)
time.sleep(2)

dbg = NetworkTables.getTable("Debug")
fw = NetworkTables.getTable("Shooter").getSubTable("Flywheel")

print(f"Connected: {NetworkTables.isConnected()}")
print("=" * 55)
print("  PRESS EACH BUTTON ON YOUR CONTROLLER ONE BY ONE!")
print("  (A, B, X, Y, bumpers, triggers, sticks)")
print("=" * 55)

for i in range(75):  # 15 seconds
    raw = dbg.getString("H2_RawButtons", "N/A")
    ax0 = dbg.getNumber("H2_Axis0", -99)
    ax1 = dbg.getNumber("H2_Axis1", -99)
    btn_a = dbg.getBoolean("H2_CtrlPort0_A", False)
    fw_target = fw.getNumber("CurrentTargetRPM", -1)

    print(f"\r raw=[{raw}] ax0={ax0:5.2f} ax1={ax1:5.2f} A={btn_a} FW_t={fw_target:5.0f}", end="", flush=True)
    time.sleep(0.2)

print("\nDone.")
NetworkTables.shutdown()
