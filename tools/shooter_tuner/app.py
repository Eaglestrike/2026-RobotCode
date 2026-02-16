"""
Shooter Tuner - Streamlit app.
Pattern from JerryRoboticsArm2: robot & app share the SAME NT table.
  Shooter/Flywheel/*   Shooter/Turret/*   Shooter/Hood/*
Robot periodic() writes status + reads commands from these tables.
App reads status + writes commands to these tables.
No DogLog abstraction - pure NT3 compatible.
"""
import time
import threading
import json
from collections import deque

import streamlit as st
from streamlit_autorefresh import st_autorefresh
from networktables import NetworkTables
import pandas as pd
import altair as alt

# #region agent log
DEBUG_LOG_PATH = "/Users/minecraftxz.com/Desktop/robotics/2026-RobotCode/.cursor/debug.log"
def _dlog(location, message, data=None, hypothesisId=""):
    try:
        entry = {"timestamp": int(time.time()*1000), "location": location,
                 "message": message, "data": data or {}, "hypothesisId": hypothesisId}
        with open(DEBUG_LOG_PATH, "a") as f:
            f.write(json.dumps(entry) + "\n")
    except Exception:
        pass
# #endregion

st.set_page_config(page_title="Shooter Tuner", layout="wide")

ROBORIO_DEFAULT = "10.1.14.2"


@st.cache_resource
def nt_init(server: str):
    NetworkTables.initialize(server=server)
    return NetworkTables.getTable("Shooter")


def nt_reconnect(server: str):
    try:
        NetworkTables.shutdown()
    except Exception:
        pass
    NetworkTables.initialize(server=server)
    return NetworkTables.getTable("Shooter")


# -- Pollers --------------------------------------------------------
class FlywheelPoller:
    def __init__(self, table, hz=50.0, maxlen=2000):
        self.table = table
        self.hz = hz
        self.lock = threading.Lock()
        self.maxlen = maxlen
        self.t = deque(maxlen=maxlen)
        self.velocity = deque(maxlen=maxlen)
        self.target = deque(maxlen=maxlen)
        self.output = deque(maxlen=maxlen)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_maxlen(self, m):
        m = int(m)
        if m == self.maxlen:
            return
        with self.lock:
            self.maxlen = m
            self.t = deque(self.t, maxlen=m)
            self.velocity = deque(self.velocity, maxlen=m)
            self.target = deque(self.target, maxlen=m)
            self.output = deque(self.output, maxlen=m)

    def snapshot(self):
        with self.lock:
            return list(self.t), list(self.velocity), list(self.target), list(self.output)

    def _run(self):
        while True:
            ts = time.time()
            v = float(self.table.getNumber("VelocityRPM", 0.0))
            tg = float(self.table.getNumber("CurrentTargetRPM", 0.0))
            out = float(self.table.getNumber("AppliedOutput", 0.0))
            with self.lock:
                self.t.append(ts)
                self.velocity.append(v)
                self.target.append(tg)
                self.output.append(out)
            time.sleep(1.0 / self.hz)


class TurretPoller:
    def __init__(self, table, hz=50.0, maxlen=2000):
        self.table = table
        self.hz = hz
        self.lock = threading.Lock()
        self.maxlen = maxlen
        self.t = deque(maxlen=maxlen)
        self.angle = deque(maxlen=maxlen)
        self.target = deque(maxlen=maxlen)
        self.volts = deque(maxlen=maxlen)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_maxlen(self, m):
        m = int(m)
        if m == self.maxlen:
            return
        with self.lock:
            self.maxlen = m
            self.t = deque(self.t, maxlen=m)
            self.angle = deque(self.angle, maxlen=m)
            self.target = deque(self.target, maxlen=m)
            self.volts = deque(self.volts, maxlen=m)

    def snapshot(self):
        with self.lock:
            return list(self.t), list(self.angle), list(self.target), list(self.volts)

    def _run(self):
        while True:
            ts = time.time()
            a = float(self.table.getNumber("CurrentAngle", 0.0))
            tg = float(self.table.getNumber("RequestedPosition", 0.0))
            v = float(self.table.getNumber("MotorVoltage", 0.0))
            with self.lock:
                self.t.append(ts)
                self.angle.append(a)
                self.target.append(tg)
                self.volts.append(v)
            time.sleep(1.0 / self.hz)


class HoodPoller:
    def __init__(self, table, hz=50.0, maxlen=2000):
        self.table = table
        self.hz = hz
        self.lock = threading.Lock()
        self.maxlen = maxlen
        self.t = deque(maxlen=maxlen)
        self.angle = deque(maxlen=maxlen)
        self.goal = deque(maxlen=maxlen)
        self.volts = deque(maxlen=maxlen)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_maxlen(self, m):
        m = int(m)
        if m == self.maxlen:
            return
        with self.lock:
            self.maxlen = m
            self.t = deque(self.t, maxlen=m)
            self.angle = deque(self.angle, maxlen=m)
            self.goal = deque(self.goal, maxlen=m)
            self.volts = deque(self.volts, maxlen=m)

    def snapshot(self):
        with self.lock:
            return list(self.t), list(self.angle), list(self.goal), list(self.volts)

    def _run(self):
        while True:
            ts = time.time()
            a = float(self.table.getNumber("CurrAngDeg", 0.0))
            g = float(self.table.getNumber("GoalDeg", 0.0))
            v = float(self.table.getNumber("OutputVolts", 0.0))
            with self.lock:
                self.t.append(ts)
                self.angle.append(a)
                self.goal.append(g)
                self.volts.append(v)
            time.sleep(1.0 / self.hz)


# -- Connection ----------------------------------------------------
st.title("Shooter Tuner (NetworkTables)")
colA, colB, colC = st.columns([2, 2, 3])
with colA:
    server = st.text_input("roboRIO / Sim IP", value=ROBORIO_DEFAULT, key="server_ip")
    if st.button("Reconnect", use_container_width=True, key="btn_reconnect"):
        st.session_state["_force_reconnect"] = True
with colB:
    auto_refresh = st.toggle("Auto refresh", value=True, key="auto_refresh")
    refresh_ms = st.slider("Refresh (ms)", 200, 2000, 500, 100, disabled=not auto_refresh, key="refresh_ms")
with colC:
    st.caption("NT Table: `/Shooter/Flywheel/*`, `/Shooter/Turret/*`, `/Shooter/Hood/*`")

prev_server = st.session_state.get("_nt_server", "")
force = bool(st.session_state.pop("_force_reconnect", False))
if force or (prev_server and prev_server != server):
    root = nt_reconnect(server)
else:
    root = nt_init(server)
st.session_state["_nt_server"] = server

fw_table = root.getSubTable("Flywheel")
tr_table = root.getSubTable("Turret")
hd_table = root.getSubTable("Hood")

# Init pollers
if "_fw_poller" not in st.session_state:
    st.session_state["_fw_poller"] = FlywheelPoller(fw_table)
else:
    st.session_state["_fw_poller"].table = fw_table
if "_tr_poller" not in st.session_state:
    st.session_state["_tr_poller"] = TurretPoller(tr_table)
else:
    st.session_state["_tr_poller"].table = tr_table
if "_hd_poller" not in st.session_state:
    st.session_state["_hd_poller"] = HoodPoller(hd_table)
else:
    st.session_state["_hd_poller"].table = hd_table

connected = False
try:
    connected = bool(NetworkTables.isConnected())
except Exception:
    pass
st.caption(f"NT Connected: **{connected}**  |  Server: `{server}`")
if not connected:
    st.warning("NetworkTables not connected. Check IP, network, and that robot/sim is running.")

st.divider()

# -- Helper --------------------------------------------------------
def gn(table, key, default=0.0):
    return float(table.getNumber(key, default))

def gb(table, key, default=False):
    return bool(table.getBoolean(key, default))


# ========================== TABS ==================================
tab_fly, tab_turret, tab_hood, tab_ctrl, tab_ball = st.tabs([
    "Flywheel", "Turret", "Hood", "Controller", "Ballistics"
])

# -- Flywheel ------------------------------------------------------
with tab_fly:
    st.subheader("Flywheel")
    st.caption("NEO Vertex velocity control.")

    st.markdown("**Status**")
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Velocity (RPM)", f"{gn(fw_table, 'VelocityRPM'):.0f}")
    c2.metric("Target (RPM)", f"{gn(fw_table, 'CurrentTargetRPM'):.0f}")
    c3.metric("Applied output", f"{gn(fw_table, 'AppliedOutput'):.2f}")
    c4.metric("Motor temp (C)", f"{gn(fw_table, 'MotorTemp'):.1f}")
    c5, c6, c7, c8 = st.columns(4)
    c5.metric("Output current (A)", f"{gn(fw_table, 'OutputCurrent'):.2f}")
    c6.metric("Error (RPM)", f"{gn(fw_table, 'ErrorRPM'):.1f}")
    c7.metric("At setpoint", gb(fw_table, "AtSetpoint"))
    c8.metric("Enabled", gb(fw_table, "Enabled"))

    st.divider()
    st.markdown("**Controls**")

    # ManualControl toggle - blocks ShooterController from overriding NT target
    manual_ctrl = st.toggle(
        "ManualControl (block ShooterController override)",
        value=gb(fw_table, "ManualControl", False),
        key="fw_manual_ctrl",
        help="Enable to allow Streamlit to control flywheel RPM directly. "
             "When ON, ShooterController's setVelocity() calls are ignored."
    )
    fw_table.putBoolean("ManualControl", manual_ctrl)

    ctrl1, ctrl2 = st.columns(2)
    with ctrl1:
        draft_rpm = st.number_input("Draft target RPM", value=gn(fw_table, "TargetRPM", 0.0), step=100.0, key="fw_draft")
    with ctrl2:
        if st.button("Send TargetRPM", key="btn_fw_send", use_container_width=True):
            # Auto-enable ManualControl when sending from Streamlit
            fw_table.putBoolean("ManualControl", True)
            # #region agent log
            _dlog("app.py:send_btn", "Before putNumber", {"draft_rpm": draft_rpm, "connected": connected, "manual": True}, "H1")
            # #endregion
            fw_table.putNumber("TargetRPM", draft_rpm)
            # #region agent log
            readback = float(fw_table.getNumber("TargetRPM", -999))
            _dlog("app.py:send_btn", "After putNumber", {"draft_rpm": draft_rpm, "readback": readback}, "H5")
            # #endregion
            st.success(f"Sent {draft_rpm:.0f} RPM (ManualControl auto-enabled)")

    # #region agent log - Robot debug entries
    with st.expander("Debug (robot-side NT entries)", expanded=True):
        db1, db2, db3, db4, db5 = st.columns(5)
        dbg_ntRpm = gn(fw_table, "Debug_ntRpm", -999)
        dbg_prevNtRpm = gn(fw_table, "Debug_prevNtRpm", -999)
        dbg_isNaN = gb(fw_table, "Debug_isNaN", False)
        dbg_changed = gb(fw_table, "Debug_changed", False)
        dbg_override = gb(fw_table, "Debug_ntOverride", False)
        db1.metric("Debug_ntRpm", f"{dbg_ntRpm:.2f}")
        db2.metric("Debug_prevNtRpm", f"{dbg_prevNtRpm:.2f}")
        db3.metric("Debug_isNaN", dbg_isNaN)
        db4.metric("Debug_changed", dbg_changed)
        db5.metric("Debug_ntOverride", dbg_override)
        _dlog("app.py:refresh", "flywheel_debug_cycle", {
            "nt_TargetRPM": gn(fw_table, "TargetRPM", -999),
            "nt_CurrentTargetRPM": gn(fw_table, "CurrentTargetRPM", -999),
            "dbg_ntRpm": dbg_ntRpm,
            "dbg_prevNtRpm": dbg_prevNtRpm,
            "dbg_isNaN": dbg_isNaN,
            "dbg_changed": dbg_changed,
            "dbg_override": dbg_override,
            "manual_ctrl": manual_ctrl,
            "connected": connected,
        }, "H2,H3,H4")
    # #endregion

    with st.expander("Tunables", expanded=True):
        tune_live = st.toggle("TuneLive", value=gb(fw_table, "TuneLive"), key="fw_tune")
        fw_table.putBoolean("TuneLive", tune_live)

        st.markdown("**Hardware**")
        fw_mid = st.number_input("Motor ID (CAN)", value=gn(fw_table, "MotorID", 3.0), min_value=0.0, max_value=63.0, step=1.0, key="fw_mid")

        st.markdown("**PID**")
        g1, g2, g3 = st.columns(3)
        fw_kp = g1.number_input("kP", value=gn(fw_table, "kP", 0.0001), format="%.6f", key="fw_kp")
        fw_ki = g2.number_input("kI", value=gn(fw_table, "kI", 0.0), format="%.6f", key="fw_ki")
        fw_kd = g3.number_input("kD", value=gn(fw_table, "kD", 0.0), format="%.6f", key="fw_kd")

        st.markdown("**Feedforward**")
        f1, f2, f3 = st.columns(3)
        fw_ks = f1.number_input("kS", value=gn(fw_table, "kS", 0.0), format="%.6f", key="fw_ks")
        fw_kv = f2.number_input("kV", value=gn(fw_table, "kV", 0.000147), format="%.6f", key="fw_kv")
        fw_ka = f3.number_input("kA", value=gn(fw_table, "kA", 0.0), format="%.6f", key="fw_ka")

        fw_maxaccel = st.number_input("maxAccel (RPM/s)", value=gn(fw_table, "maxAccel", 10000.0), step=500.0, key="fw_maxaccel")

        if st.button("Apply Flywheel tunables", key="btn_fw_apply", use_container_width=True):
            fw_table.putNumber("MotorID", fw_mid)
            fw_table.putNumber("kP", fw_kp)
            fw_table.putNumber("kI", fw_ki)
            fw_table.putNumber("kD", fw_kd)
            fw_table.putNumber("kS", fw_ks)
            fw_table.putNumber("kV", fw_kv)
            fw_table.putNumber("kA", fw_ka)
            fw_table.putNumber("maxAccel", fw_maxaccel)
            st.success("Applied.")

    # Graphs
    st.divider()
    st.subheader("Graphs")
    gc1, gc2 = st.columns([2, 3])
    fw_window = gc1.slider("Show last (s)", 5, 120, 30, 5, key="fw_win")
    fw_hist = gc2.slider("History pts", 200, 5000, 2000, 200, key="fw_hist")
    poller = st.session_state["_fw_poller"]
    poller.set_maxlen(fw_hist)
    t, v, tg, out = poller.snapshot()
    ts = time.time()
    cut = ts - float(fw_window)
    start = next((i for i, tt in enumerate(t) if tt >= cut), 0)
    df = pd.DataFrame({"VelocityRPM": v[start:], "TargetRPM": tg[start:], "AppliedOutput": out[start:]},
                       index=pd.to_datetime(t[start:], unit="s"))
    ch1, ch2 = st.columns(2)
    with ch1:
        d = df[["VelocityRPM", "TargetRPM"]].reset_index().rename(columns={"index": "t"}).melt("t", var_name="s", value_name="rpm")
        st.altair_chart(alt.Chart(d).mark_line().encode(x="t:T", y="rpm:Q", color="s:N").properties(height=280), use_container_width=True)
    with ch2:
        d2 = df[["AppliedOutput"]].reset_index().rename(columns={"index": "t"})
        st.altair_chart(alt.Chart(d2).mark_line().encode(x="t:T", y="AppliedOutput:Q").properties(height=280), use_container_width=True)

# -- Turret --------------------------------------------------------
with tab_turret:
    st.subheader("Turret")
    st.caption("Turret position (Motion Magic).")

    st.markdown("**Status**")
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Current angle (deg)", f"{gn(tr_table, 'CurrentAngle'):.2f}")
    c2.metric("Target angle (deg)", f"{gn(tr_table, 'RequestedPosition'):.2f}")
    c3.metric("Error (deg)", f"{gn(tr_table, 'Error'):.2f}")
    c4.metric("At target", gb(tr_table, "AtTarget"))
    c5, c6, c7, c8 = st.columns(4)
    c5.metric("Motor voltage (V)", f"{gn(tr_table, 'MotorVoltage'):.2f}")
    c6.metric("Sim position (deg)", f"{gn(tr_table, 'SimPosition'):.2f}")
    c7.metric("Enabled", gb(tr_table, "Enabled"))
    c8.metric("Motor ID", f"{gn(tr_table, 'MotorID'):.0f}")

    st.divider()
    st.markdown("**Controls**")
    ctrl1, ctrl2, ctrl3 = st.columns(3)
    with ctrl1:
        tr_draft = st.number_input("Draft target (deg)", value=gn(tr_table, "TargetDeg", 0.0), step=5.0, key="tr_draft")
    with ctrl2:
        if st.button("Send TargetDeg", key="btn_tr_send", use_container_width=True):
            tr_table.putNumber("TargetDeg", tr_draft)
            st.success(f"Sent {tr_draft:.1f} deg")
    with ctrl3:
        if st.button("ZERO", type="primary", key="btn_tr_zero", use_container_width=True):
            tr_table.putBoolean("Zero", True)
            st.success("Sent Zero")

    with st.expander("Tunables", expanded=True):
        tune_live = st.toggle("TuneLive", value=gb(tr_table, "TuneLive"), key="tr_tune")
        tr_table.putBoolean("TuneLive", tune_live)

        st.markdown("**PID**")
        g1, g2, g3 = st.columns(3)
        tr_kp = g1.number_input("kP", value=gn(tr_table, "kP", 100.0), format="%.4f", key="tr_kp")
        tr_ki = g2.number_input("kI", value=gn(tr_table, "kI", 0.0), format="%.4f", key="tr_ki")
        tr_kd = g3.number_input("kD", value=gn(tr_table, "kD", 5.0), format="%.4f", key="tr_kd")

        st.markdown("**FF**")
        f1, f2, f3 = st.columns(3)
        tr_ks = f1.number_input("kS", value=gn(tr_table, "kS", 0.0), format="%.4f", key="tr_ks")
        tr_kv = f2.number_input("kV", value=gn(tr_table, "kV", 1.2), format="%.4f", key="tr_kv")
        tr_ka = f3.number_input("kA", value=gn(tr_table, "kA", 0.0), format="%.4f", key="tr_ka")

        st.markdown("**Motion**")
        m1, m2 = st.columns(2)
        tr_cruise = m1.number_input("CruiseVelo", value=gn(tr_table, "CruiseVelo", 20.0), step=1.0, key="tr_cruise")
        tr_accel = m2.number_input("Acceleration", value=gn(tr_table, "Acceleration", 20.0), step=1.0, key="tr_accel")

        if st.button("Apply Turret tunables", key="btn_tr_apply", use_container_width=True):
            tr_table.putNumber("kP", tr_kp)
            tr_table.putNumber("kI", tr_ki)
            tr_table.putNumber("kD", tr_kd)
            tr_table.putNumber("kS", tr_ks)
            tr_table.putNumber("kV", tr_kv)
            tr_table.putNumber("kA", tr_ka)
            tr_table.putNumber("CruiseVelo", tr_cruise)
            tr_table.putNumber("Acceleration", tr_accel)
            st.success("Applied.")

    # Graphs
    st.divider()
    st.subheader("Graphs")
    gc1, gc2 = st.columns([2, 3])
    tr_window = gc1.slider("Show last (s)", 5, 120, 30, 5, key="tr_win")
    tr_hist_n = gc2.slider("History pts", 200, 5000, 2000, 200, key="tr_hist")
    tp = st.session_state["_tr_poller"]
    tp.set_maxlen(tr_hist_n)
    t_tr, a_tr, tg_tr, v_tr = tp.snapshot()
    ts = time.time()
    cut_tr = ts - float(tr_window)
    start_tr = next((i for i, tt in enumerate(t_tr) if tt >= cut_tr), 0)
    df_tr = pd.DataFrame({"AngleDeg": a_tr[start_tr:], "TargetDeg": tg_tr[start_tr:], "Volts": v_tr[start_tr:]},
                          index=pd.to_datetime(t_tr[start_tr:], unit="s"))
    ch1, ch2 = st.columns(2)
    with ch1:
        d = df_tr[["AngleDeg", "TargetDeg"]].reset_index().rename(columns={"index": "t"}).melt("t", var_name="s", value_name="deg")
        st.altair_chart(alt.Chart(d).mark_line().encode(x="t:T", y="deg:Q", color="s:N").properties(height=280), use_container_width=True)
    with ch2:
        d2 = df_tr[["Volts"]].reset_index().rename(columns={"index": "t"})
        st.altair_chart(alt.Chart(d2).mark_line().encode(x="t:T", y="Volts:Q").properties(height=280), use_container_width=True)

# -- Hood ----------------------------------------------------------
with tab_hood:
    st.subheader("Hood")
    st.caption("Hood angle (ProfiledPID + ArmFeedforward).")

    st.markdown("**Status**")
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Current angle (deg)", f"{gn(hd_table, 'CurrAngDeg'):.2f}")
    c2.metric("Goal (deg)", f"{gn(hd_table, 'GoalDeg'):.2f}")
    c3.metric("Error (deg)", f"{gn(hd_table, 'ErrorDeg'):.2f}")
    c4.metric("Velocity (deg/s)", f"{gn(hd_table, 'VelocityDegPerS'):.1f}")
    c5, c6, c7, c8 = st.columns(4)
    c5.metric("PID volts", f"{gn(hd_table, 'PIDVolts'):.2f}")
    c6.metric("FF volts", f"{gn(hd_table, 'FFVolts'):.2f}")
    c7.metric("Output volts", f"{gn(hd_table, 'OutputVolts'):.2f}")
    c8.metric("At target", gb(hd_table, "AtTarget"))

    st.divider()
    st.markdown("**Controls**")
    ctrl1, ctrl2, ctrl3 = st.columns(3)
    with ctrl1:
        hd_draft = st.number_input("Draft target (deg)", value=gn(hd_table, "RequestDeg", 0.0), step=1.0, key="hd_draft")
    with ctrl2:
        if st.button("Send RequestDeg", key="btn_hd_send", use_container_width=True):
            hd_table.putNumber("RequestDeg", hd_draft)
            st.success(f"Sent {hd_draft:.1f} deg")
    with ctrl3:
        if st.button("ZERO", type="primary", key="btn_hd_zero", use_container_width=True):
            hd_table.putBoolean("Zero", True)
            st.success("Sent Zero")

    st.markdown("**Calibration**")
    cal1, cal2 = st.columns(2)
    with cal1:
        hd_setpos = st.number_input("SetPosDeg", value=gn(hd_table, "SetPosDeg", 0.0), step=1.0, key="hd_setpos")
    with cal2:
        if st.button("Set Position", key="btn_hd_setpos", use_container_width=True):
            hd_table.putNumber("SetPosDeg", hd_setpos)
            hd_table.putBoolean("SetPos", True)
            st.success("Sent SetPos.")

    with st.expander("Tunables", expanded=True):
        tune_live = st.toggle("TuneLive", value=gb(hd_table, "TuneLive"), key="hd_tune")
        hd_table.putBoolean("TuneLive", tune_live)

        st.markdown("**PID**")
        g1, g2, g3 = st.columns(3)
        hd_kp = g1.number_input("kP", value=gn(hd_table, "kP", 40.0), format="%.4f", key="hd_kp")
        hd_ki = g2.number_input("kI", value=gn(hd_table, "kI", 100.0), format="%.4f", key="hd_ki")
        hd_kd = g3.number_input("kD", value=gn(hd_table, "kD", 0.7), format="%.4f", key="hd_kd")

        st.markdown("**FF (ArmFeedforward)**")
        f1, f2, f3, f4 = st.columns(4)
        hd_ks = f1.number_input("kS", value=gn(hd_table, "kS", 0.19), format="%.4f", key="hd_ks")
        hd_kg = f2.number_input("kG", value=gn(hd_table, "kG", 0.0), format="%.4f", key="hd_kg")
        hd_kv = f3.number_input("kV", value=gn(hd_table, "kV", 3.0), format="%.4f", key="hd_kv")
        hd_ka = f4.number_input("kA", value=gn(hd_table, "kA", 0.01), format="%.4f", key="hd_ka")

        st.markdown("**Motion**")
        m1, m2 = st.columns(2)
        hd_maxvel = m1.number_input("MaxVel (rad/s)", value=gn(hd_table, "MaxVel", 14.0), step=1.0, key="hd_maxvel")
        hd_maxacc = m2.number_input("MaxAcc (rad/s^2)", value=gn(hd_table, "MaxAcc", 20.0), step=1.0, key="hd_maxacc")

        st.markdown("**Anti-windup**")
        aw1, aw2 = st.columns(2)
        hd_izone = aw1.slider("IZoneDeg", 0.0, 10.0, value=gn(hd_table, "IZoneDeg", 5.0), step=0.1, key="hd_izone")
        hd_imax = aw2.slider("IMaxVolts", 0.0, 3.0, value=gn(hd_table, "IMaxVolts", 1.0), step=0.1, key="hd_imax")

        if st.button("Apply Hood tunables", key="btn_hd_apply", use_container_width=True):
            hd_table.putNumber("kP", hd_kp)
            hd_table.putNumber("kI", hd_ki)
            hd_table.putNumber("kD", hd_kd)
            hd_table.putNumber("kS", hd_ks)
            hd_table.putNumber("kG", hd_kg)
            hd_table.putNumber("kV", hd_kv)
            hd_table.putNumber("kA", hd_ka)
            hd_table.putNumber("MaxVel", hd_maxvel)
            hd_table.putNumber("MaxAcc", hd_maxacc)
            hd_table.putNumber("IZoneDeg", hd_izone)
            hd_table.putNumber("IMaxVolts", hd_imax)
            st.success("Applied.")

    # Graphs
    st.divider()
    st.subheader("Graphs")
    gc1, gc2 = st.columns([2, 3])
    hd_window = gc1.slider("Show last (s)", 5, 120, 30, 5, key="hd_win")
    hd_hist_n = gc2.slider("History pts", 200, 5000, 2000, 200, key="hd_hist")
    hp = st.session_state["_hd_poller"]
    hp.set_maxlen(hd_hist_n)
    t_h, a_h, g_h, v_h = hp.snapshot()
    ts = time.time()
    cut_h = ts - float(hd_window)
    start_h = next((i for i, tt in enumerate(t_h) if tt >= cut_h), 0)
    df_h = pd.DataFrame({"AngleDeg": a_h[start_h:], "GoalDeg": g_h[start_h:], "Volts": v_h[start_h:]},
                         index=pd.to_datetime(t_h[start_h:], unit="s"))
    ch1, ch2 = st.columns(2)
    with ch1:
        d = df_h[["AngleDeg", "GoalDeg"]].reset_index().rename(columns={"index": "t"}).melt("t", var_name="s", value_name="deg")
        st.altair_chart(alt.Chart(d).mark_line().encode(x="t:T", y="deg:Q", color="s:N").properties(height=280), use_container_width=True)
    with ch2:
        d2 = df_h[["Volts"]].reset_index().rename(columns={"index": "t"})
        st.altair_chart(alt.Chart(d2).mark_line().encode(x="t:T", y="Volts:Q").properties(height=280), use_container_width=True)

# -- Controller ----------------------------------------------------
with tab_ctrl:
    st.subheader("Shooter Controller")
    st.caption("Read-only status from ShooterController (via DogLog /Robot table)")
    st.info("Controller status is read from DogLog /Robot table. To enable live updates, "
            "refactor ShooterController.java to also use /Shooter/Controller/* table.")

# -- Ballistics ----------------------------------------------------
with tab_ball:
    st.subheader("Ballistics")
    st.caption("Ballistic parameters reference (read-only).")

    st.markdown("**FRC 2026 Game Piece**")
    gp1, gp2, gp3, gp4 = st.columns(4)
    gp1.metric("Diameter (m)", "0.15")
    gp2.metric("Weight (kg)", "0.363")
    gp3.metric("Cd", "0.50")
    gp4.metric("kDrag", "0.015")

    st.markdown("**Three-Speed Presets**")
    sp1, sp2, sp3 = st.columns(3)
    sp1.markdown("**LOW**: 3000 RPM, 12 m/s")
    sp2.markdown("**MEDIUM**: 4500 RPM, 18 m/s")
    sp3.markdown("**HIGH**: 6000 RPM, 24 m/s")

if auto_refresh:
    st_autorefresh(interval=refresh_ms, key="shooter_refresh")
