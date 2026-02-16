"""
NT Test Tool - connects to the robot, monitors status, and sends test commands.
Usage: python3 tools/nt_test.py [robot_ip]
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

print(f"NT Connected: {NetworkTables.isConnected()}")
print("=" * 60)

def status():
    fw_vel = flywheel.getNumber("VelocityRPM", -1)
    fw_target = flywheel.getNumber("CurrentTargetRPM", -1)
    fw_enabled = flywheel.getBoolean("Enabled", False)
    fw_output = flywheel.getNumber("AppliedOutput", -1)
    fw_manual = flywheel.getBoolean("ManualControl", False)

    tr_angle = turret.getNumber("CurrentAngle", -1)
    tr_req = turret.getNumber("RequestedPosition", -1)
    tr_at = turret.getBoolean("AtTarget", False)

    hd_angle = hood.getNumber("CurrAngDeg", -1)
    hd_goal = hood.getNumber("GoalDeg", -1)
    hd_volts = hood.getNumber("OutputVolts", -1)
    hd_pid = hood.getNumber("PIDVolts", -1)
    hd_ff = hood.getNumber("FFVolts", -1)

    print(f"  FW: vel={fw_vel:7.0f} target={fw_target:7.0f} RPM out={fw_output:6.3f} manual={fw_manual}")
    print(f"  TR: angle={tr_angle:7.1f} req={tr_req:7.1f}° atTarget={tr_at}")
    print(f"  HD: angle={hd_angle:7.1f} goal={hd_goal:7.1f}° outV={hd_volts:6.2f} pidV={hd_pid:6.2f} ffV={hd_ff:6.2f}")


def monitor(seconds=3):
    """Monitor for N seconds"""
    end = time.time() + seconds
    while time.time() < end:
        fw_vel = flywheel.getNumber("VelocityRPM", -1)
        fw_target = flywheel.getNumber("CurrentTargetRPM", -1)
        fw_output = flywheel.getNumber("AppliedOutput", -1)
        hd_angle = hood.getNumber("CurrAngDeg", -1)
        hd_goal = hood.getNumber("GoalDeg", -1)
        hd_volts = hood.getNumber("OutputVolts", -1)
        tr_angle = turret.getNumber("CurrentAngle", -1)
        print(f"\r  FW:{fw_vel:6.0f}/{fw_target:6.0f}RPM out={fw_output:5.2f} | "
              f"HD:{hd_angle:5.1f}/{hd_goal:5.1f}° V={hd_volts:5.1f} | "
              f"TR:{tr_angle:5.1f}°", end="", flush=True)
        time.sleep(0.3)
    print()


# ─── Initial status ───
print("\n>>> Initial status:")
status()
time.sleep(0.5)

# ─── Test 1: Flywheel via ManualControl + TargetRPM ───
print("\n>>> TEST 1: Flywheel - set ManualControl=true, TargetRPM=3000")
flywheel.putBoolean("ManualControl", True)
flywheel.putNumber("TargetRPM", 3000)
time.sleep(0.5)
print("  Monitoring flywheel response (5s)...")
monitor(5)

print("  Stopping flywheel...")
flywheel.putNumber("TargetRPM", 0)
time.sleep(1)
status()
flywheel.putBoolean("ManualControl", False)

# ─── Test 2: Turret via TargetDeg ───
print("\n>>> TEST 2: Turret - set TargetDeg=20")
turret.putNumber("TargetDeg", 20)
time.sleep(0.5)
print("  Monitoring turret response (3s)...")
monitor(3)
status()

print("  Turret back to 0...")
turret.putNumber("TargetDeg", 0)
time.sleep(0.5)
monitor(3)

# ─── Test 3: Hood via RequestDeg ───
print("\n>>> TEST 3: Hood - set RequestDeg=35")
hood.putNumber("RequestDeg", 35)
time.sleep(0.5)
print("  Monitoring hood response (3s)...")
monitor(3)
status()

print("  Hood to 30...")
hood.putNumber("RequestDeg", 30)
time.sleep(0.5)
monitor(3)
status()

# ─── Cleanup ───
print("\n>>> Cleanup: stop flywheel, zero targets")
flywheel.putNumber("TargetRPM", 0)
flywheel.putBoolean("ManualControl", False)
turret.putNumber("TargetDeg", 0)
hood.putNumber("RequestDeg", 26.5)
time.sleep(1)
status()

print("\n>>> Done!")
NetworkTables.shutdown()
