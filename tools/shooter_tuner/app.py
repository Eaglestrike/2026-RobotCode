"""
Shooter Tuner - Streamlit app for Flywheel, Turret, Hood, Controller, Ballistics.
Uses NetworkTables (pynetworktables) to read/write DogLog keys from robot/sim.
Layout and graphs reference JerryRoboticsArm2 (PID/FF in rows, NTPoller + Altair charts).
"""
import time
import threading
from collections import deque

import streamlit as st
from streamlit_autorefresh import st_autorefresh
from networktables import NetworkTables
import pandas as pd
import altair as alt

st.set_page_config(page_title="Shooter Tuner", layout="wide")

ROBORIO_DEFAULT = "10.1.14.2"  # TODO: set your team IP; use localhost for sim
ROOT_TABLE = "DogLog"


@st.cache_resource
def nt_init(server: str):
    NetworkTables.initialize(server=server)
    return NetworkTables.getTable(ROOT_TABLE)


def nt_reconnect(server: str):
    try:
        NetworkTables.shutdown()
    except Exception:
        pass
    NetworkTables.initialize(server=server)
    return NetworkTables.getTable(ROOT_TABLE)


def get_nt():
    return st.session_state.get("_nt_table")


def get_number(key: str, default: float = 0.0) -> float:
    t = get_nt()
    if t is None:
        return default
    return float(t.getNumber(key, default))


def get_boolean(key: str, default: bool = False) -> bool:
    t = get_nt()
    if t is None:
        return default
    return bool(t.getBoolean(key, default))


def get_string(key: str, default: str = "") -> str:
    t = get_nt()
    if t is None:
        return default
    return str(t.getString(key, default))


def put_number(key: str, value: float):
    t = get_nt()
    if t is not None:
        t.putNumber(key, value)


def put_boolean(key: str, value: bool):
    t = get_nt()
    if t is not None:
        t.putBoolean(key, value)


# ---------- NTPoller for Flywheel (velocity / target / output) ----------
class FlywheelPoller:
    def __init__(self, table, hz: float = 50.0, maxlen: int = 5000):
        self.table = table
        self.hz = hz
        self.maxlen = maxlen
        self.lock = threading.Lock()
        self.t = deque(maxlen=maxlen)
        self.velocity_rpm = deque(maxlen=maxlen)
        self.target_rpm = deque(maxlen=maxlen)
        self.applied_output = deque(maxlen=maxlen)
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_maxlen(self, maxlen: int):
        maxlen = int(maxlen)
        if maxlen == self.maxlen:
            return
        with self.lock:
            self.maxlen = maxlen
            self.t = deque(self.t, maxlen=maxlen)
            self.velocity_rpm = deque(self.velocity_rpm, maxlen=maxlen)
            self.target_rpm = deque(self.target_rpm, maxlen=maxlen)
            self.applied_output = deque(self.applied_output, maxlen=maxlen)

    def snapshot(self):
        with self.lock:
            return (
                list(self.t),
                list(self.velocity_rpm),
                list(self.target_rpm),
                list(self.applied_output),
            )

    def _run(self):
        while not self._stop:
            ts = time.time()
            v = float(self.table.getNumber("ShooterFlywheels/VelocityRPM", 0.0))
            tg = float(self.table.getNumber("ShooterFlywheels/targetRPM", 0.0))
            out = float(self.table.getNumber("ShooterFlywheels/AppliedOutput", 0.0))
            with self.lock:
                self.t.append(ts)
                self.velocity_rpm.append(v)
                self.target_rpm.append(tg)
                self.applied_output.append(out)
            time.sleep(1.0 / self.hz)


# ---------- NTPoller for Hood (angle / goal / volts) ----------
class HoodPoller:
    def __init__(self, table, hz: float = 50.0, maxlen: int = 5000):
        self.table = table
        self.hz = hz
        self.maxlen = maxlen
        self.lock = threading.Lock()
        self.t = deque(maxlen=maxlen)
        self.angle_deg = deque(maxlen=maxlen)
        self.goal_deg = deque(maxlen=maxlen)
        self.output_volts = deque(maxlen=maxlen)
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_maxlen(self, maxlen: int):
        maxlen = int(maxlen)
        if maxlen == self.maxlen:
            return
        with self.lock:
            self.maxlen = maxlen
            self.t = deque(self.t, maxlen=maxlen)
            self.angle_deg = deque(self.angle_deg, maxlen=maxlen)
            self.goal_deg = deque(self.goal_deg, maxlen=maxlen)
            self.output_volts = deque(self.output_volts, maxlen=maxlen)

    def snapshot(self):
        with self.lock:
            return (
                list(self.t),
                list(self.angle_deg),
                list(self.goal_deg),
                list(self.output_volts),
            )

    def _run(self):
        while not self._stop:
            ts = time.time()
            ang = float(self.table.getNumber("ShooterHood/CurrAngDeg", 0.0))
            goal_rad = float(self.table.getNumber("ShooterHood/GoalRad", 0.0))
            goal_deg = goal_rad * 180.0 / 3.141592653589793
            out = float(self.table.getNumber("ShooterHood/OutputVolts", 0.0))
            with self.lock:
                self.t.append(ts)
                self.angle_deg.append(ang)
                self.goal_deg.append(goal_deg)
                self.output_volts.append(out)
            time.sleep(1.0 / self.hz)


# ---------- NTPoller for Turret (angle / target / volts) ----------
class TurretPoller:
    def __init__(self, table, hz: float = 50.0, maxlen: int = 5000):
        self.table = table
        self.hz = hz
        self.maxlen = maxlen
        self.lock = threading.Lock()
        self.t = deque(maxlen=maxlen)
        self.angle_deg = deque(maxlen=maxlen)
        self.target_deg = deque(maxlen=maxlen)
        self.output_volts = deque(maxlen=maxlen)
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set_maxlen(self, maxlen: int):
        maxlen = int(maxlen)
        if maxlen == self.maxlen:
            return
        with self.lock:
            self.maxlen = maxlen
            self.t = deque(self.t, maxlen=maxlen)
            self.angle_deg = deque(self.angle_deg, maxlen=maxlen)
            self.target_deg = deque(self.target_deg, maxlen=maxlen)
            self.output_volts = deque(self.output_volts, maxlen=maxlen)

    def snapshot(self):
        with self.lock:
            return (
                list(self.t),
                list(self.angle_deg),
                list(self.target_deg),
                list(self.output_volts),
            )

    def _run(self):
        while not self._stop:
            ts = time.time()
            ang = float(self.table.getNumber("ShooterTurret/CurrentAngle", 0.0))
            tgt = float(self.table.getNumber("ShooterTurret/RequestedPosition", 0.0))
            out = float(self.table.getNumber("ShooterTurret/MotorVoltage", 0.0))
            with self.lock:
                self.t.append(ts)
                self.angle_deg.append(ang)
                self.target_deg.append(tgt)
                self.output_volts.append(out)
            time.sleep(1.0 / self.hz)


# --- Connection UI ---
st.title("Shooter Tuner (NetworkTables)")

col_a, col_b, col_c = st.columns([2, 2, 3])
with col_a:
    server = st.text_input("roboRIO / Sim IP", value=ROBORIO_DEFAULT, key="server_ip")
    if st.button("Reconnect", use_container_width=True, key="btn_reconnect"):
        st.session_state["_force_reconnect"] = True
with col_b:
    auto_refresh = st.toggle("Auto refresh", value=True, key="auto_refresh")
    refresh_ms = st.slider("Refresh (ms)", 200, 2000, 500, 100, disabled=not auto_refresh, key="refresh_ms")
with col_c:
    st.caption("Table: DogLog. Keys: ShooterFlywheels/*, ShooterTurret/*, ShooterHood/*, ShooterController/*")

prev_server = st.session_state.get("_nt_server", "")
force = bool(st.session_state.pop("_force_reconnect", False))
if force or (prev_server and prev_server != server):
    root = nt_reconnect(server)
    st.session_state["_nt_table"] = root
    if "_fw_poller" in st.session_state:
        st.session_state["_fw_poller"].table = root
    if "_hood_poller" in st.session_state:
        st.session_state["_hood_poller"].table = root
    if "_turret_poller" in st.session_state:
        st.session_state["_turret_poller"].table = root
else:
    try:
        root = nt_init(server)
        st.session_state["_nt_table"] = root
    except Exception as e:
        st.error(f"NT init failed: {e}")
        st.stop()
st.session_state["_nt_server"] = server

if "_fw_poller" not in st.session_state:
    st.session_state["_fw_poller"] = FlywheelPoller(root, hz=50.0, maxlen=2000)
else:
    st.session_state["_fw_poller"].table = root
if "_hood_poller" not in st.session_state:
    st.session_state["_hood_poller"] = HoodPoller(root, hz=50.0, maxlen=2000)
else:
    st.session_state["_hood_poller"].table = root
if "_turret_poller" not in st.session_state:
    st.session_state["_turret_poller"] = TurretPoller(root, hz=50.0, maxlen=2000)
else:
    st.session_state["_turret_poller"].table = root

connected = False
try:
    connected = bool(NetworkTables.isConnected())
except Exception:
    connected = False
st.caption(f"NT Connected: **{connected}**  |  Server: `{server}`")
if not connected:
    st.warning("NetworkTables 未连接：确认 IP、网络，以及仿真/机器人程序是否在运行。可点 Reconnect 重连。")

st.divider()

# --- Tabs ---
tab_fly, tab_turret, tab_hood, tab_ctrl, tab_ball = st.tabs([
    "Flywheel", "Turret", "Hood", "Controller", "Ballistics"
])

# ---------- Flywheel ----------
with tab_fly:
    st.subheader("Flywheel")
    st.caption("NEO Vertex velocity control. PID 一行，FF 一行。")

    # Status
    st.markdown("**Status**")
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        st.metric("Velocity (RPM)", f"{get_number('ShooterFlywheels/VelocityRPM', 0):.0f}")
    with c2:
        st.metric("Target (RPM)", f"{get_number('ShooterFlywheels/targetRPM', 0):.0f}")
    with c3:
        st.metric("Applied output", f"{get_number('ShooterFlywheels/AppliedOutput', 0):.2f}")
    with c4:
        st.metric("Motor temp (°C)", f"{get_number('ShooterFlywheels/MotorTemp', 0):.1f}")
    c5, c6, c7, c8 = st.columns(4)
    with c5:
        st.metric("Output current (A)", f"{get_number('ShooterFlywheels/OutputCurrent', 0):.2f}")
    with c6:
        st.metric("Error (RPM)", f"{get_number('ShooterFlywheels/ErrorRPM', 0):.1f}")
    with c7:
        st.metric("At setpoint", get_boolean("ShooterFlywheels/AtSetpoint", False))
    with c8:
        st.metric("Enabled", get_boolean("ShooterFlywheels/Enabled", False))

    # Controls (draft target pattern)
    st.divider()
    st.markdown("**Controls**")
    ctrl1, ctrl2 = st.columns(2)
    with ctrl1:
        draft_rpm = st.number_input("Draft target RPM (local)", value=get_number("ShooterFlywheels/targetRPM", 0.0), step=100.0, key="fw_draft_rpm")
    with ctrl2:
        if st.button("Send targetRPM", key="btn_send_fw_rpm", use_container_width=True):
            put_number("ShooterFlywheels/targetRPM", draft_rpm)
            st.success("Sent.")

    # Tunables
    with st.expander("Tunables (write to NT)", expanded=True):
        # TuneLive toggle
        tune_live = st.toggle("TuneLive (apply while running)", value=get_boolean("ShooterFlywheels/TuneLive", False), key="fw_tunelive")
        put_boolean("ShooterFlywheels/TuneLive", tune_live)

        # Limits
        st.markdown("**Limits**")
        lim1, lim2 = st.columns(2)
        with lim1:
            max_volts = st.slider("MaxVolts (clamp output)", 0.0, 12.0, value=get_number("ShooterFlywheels/MaxVolts", 12.0), step=0.1, key="fw_maxvolts")
        with lim2:
            max_accel = st.number_input("maxAccel (RPM/s)", value=get_number("ShooterFlywheels/maxAccel", 10000.0), step=500.0, key="fw_maxaccel")

        # Anti-windup
        st.markdown("**Anti-windup (I term)**")
        aw1, aw2 = st.columns(2)
        with aw1:
            izone = st.slider("IZone (RPM, 0=disable)", 0.0, 500.0, value=get_number("ShooterFlywheels/IZone", 0.0), step=10.0, key="fw_izone")
        with aw2:
            imax = st.slider("IMax (clamp)", 0.0, 1.0, value=get_number("ShooterFlywheels/IMax", 0.1), step=0.01, key="fw_imax")

        # PID 一行
        st.markdown("**PID**")
        g1, g2, g3 = st.columns(3)
        with g1:
            kp = st.number_input("kP", value=get_number("ShooterFlywheels/kP", 0.0001), format="%.6f", key="fw_kp")
        with g2:
            ki = st.number_input("kI", value=get_number("ShooterFlywheels/kI", 0.0), format="%.6f", key="fw_ki")
        with g3:
            kd = st.number_input("kD", value=get_number("ShooterFlywheels/kD", 0.0), format="%.6f", key="fw_kd")

        # FF 一行
        st.markdown("**Feedforward**")
        f1, f2, f3 = st.columns(3)
        with f1:
            ks = st.number_input("kS", value=get_number("ShooterFlywheels/kS", 0.0), format="%.6f", key="fw_ks")
        with f2:
            kv = st.number_input("kV", value=get_number("ShooterFlywheels/kV", 0.000147), format="%.6f", key="fw_kv")
        with f3:
            ka = st.number_input("kA", value=get_number("ShooterFlywheels/kA", 0.0), format="%.6f", key="fw_ka")

        if st.button("Apply Flywheel tunables", key="btn_apply_fw", use_container_width=True):
            put_number("ShooterFlywheels/MaxVolts", max_volts)
            put_number("ShooterFlywheels/maxAccel", max_accel)
            put_number("ShooterFlywheels/IZone", izone)
            put_number("ShooterFlywheels/IMax", imax)
            put_number("ShooterFlywheels/kP", kp)
            put_number("ShooterFlywheels/kI", ki)
            put_number("ShooterFlywheels/kD", kd)
            put_number("ShooterFlywheels/kS", ks)
            put_number("ShooterFlywheels/kV", kv)
            put_number("ShooterFlywheels/kA", ka)
            st.success("Applied.")

    # Graphs (Velocity / Volts)
    st.divider()
    st.subheader("Graphs")
    gopt1, gopt2 = st.columns([2, 3])
    with gopt1:
        window_s_fw = st.slider("Show last (seconds)", 5, 120, 30, 5, key="fw_window_s")
    with gopt2:
        hist_points_fw = st.slider("History points kept", 200, 5000, 2000, 200, key="fw_hist")
    fw_poller = st.session_state["_fw_poller"]
    fw_poller.set_maxlen(hist_points_fw)
    t_fw, v_fw, tg_fw, out_fw = fw_poller.snapshot()
    ts = time.time()
    cut = ts - float(window_s_fw)
    start = 0
    for i, tt in enumerate(t_fw):
        if tt >= cut:
            start = i
            break
    df_fw = pd.DataFrame(
        {"VelocityRPM": v_fw[start:], "TargetRPM": tg_fw[start:], "AppliedOutput": out_fw[start:]},
        index=pd.to_datetime(t_fw[start:], unit="s"),
    )
    chart_fw1, chart_fw2 = st.columns(2)
    with chart_fw1:
        df_fw1 = df_fw[["VelocityRPM", "TargetRPM"]].reset_index().rename(columns={"index": "t"})
        df_fw1 = df_fw1.melt("t", var_name="series", value_name="rpm")
        st.altair_chart(
            alt.Chart(df_fw1)
            .mark_line()
            .encode(
                x=alt.X("t:T", title="Time"),
                y=alt.Y("rpm:Q", title="RPM"),
                color=alt.Color("series:N", title=""),
            )
            .properties(height=280),
            use_container_width=True,
        )
    with chart_fw2:
        df_fw2 = df_fw[["AppliedOutput"]].reset_index().rename(columns={"index": "t"})
        st.altair_chart(
            alt.Chart(df_fw2)
            .mark_line()
            .encode(
                x=alt.X("t:T", title="Time"),
                y=alt.Y("AppliedOutput:Q", title="Applied output"),
            )
            .properties(height=280),
            use_container_width=True,
        )
    st.caption("Velocity/Target RPM 与 Applied output 随时间。若图不更新，检查 NT 连接与机器人是否发布 ShooterFlywheels/*。")

# ---------- Turret ----------
with tab_turret:
    st.subheader("Turret")
    st.caption("Turret position (Motion Magic). PID/FF 分行。")

    # Status
    st.markdown("**Status**")
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        st.metric("Current angle (°)", f"{get_number('ShooterTurret/CurrentAngle', 0):.2f}")
    with c2:
        st.metric("Target angle (°)", f"{get_number('ShooterTurret/RequestedPosition', 0):.2f}")
    with c3:
        st.metric("Motor voltage (V)", f"{get_number('ShooterTurret/MotorVoltage', 0):.2f}")
    with c4:
        st.metric("Sim position (°)", f"{get_number('ShooterTurret/SimPosition', 0):.2f}")
    c5, c6, c7, c8 = st.columns(4)
    with c5:
        st.metric("Error (°)", f"{get_number('ShooterTurret/Error', 0):.2f}")
    with c6:
        st.metric("Velocity (°/s)", f"{get_number('ShooterTurret/Velocity', 0):.1f}")
    with c7:
        st.metric("At target", get_boolean("ShooterTurret/AtTarget", False))
    with c8:
        st.metric("Enabled", get_boolean("ShooterTurret/Enabled", False))

    # Controls (draft target pattern)
    st.divider()
    st.markdown("**Controls**")
    ctrl1, ctrl2, ctrl3 = st.columns(3)
    with ctrl1:
        draft_deg = st.number_input("Draft target (°, local)", value=get_number("TurretRequest", 0.0), step=5.0, key="tr_draft_deg")
    with ctrl2:
        if st.button("Send TurretRequest", key="btn_send_tr_req", use_container_width=True):
            put_number("TurretRequest", draft_deg)
            st.success("已发送。")
    with ctrl3:
        if st.button("ZERO (清零 encoder)", type="primary", key="btn_turret_zero", use_container_width=True):
            put_boolean("ShooterTurret/Zero", True)
            st.success("已发送 Zero，机器人端执行后会将 encoder 置零。")

    # Tunables
    with st.expander("Tunables (write to NT)", expanded=True):
        # TuneLive toggle
        tune_live = st.toggle("TuneLive (apply while running)", value=get_boolean("ShooterTurret/TuneLive", False), key="tr_tunelive")
        put_boolean("ShooterTurret/TuneLive", tune_live)

        # Motion / Limits
        st.markdown("**Motion Profile**")
        m1, m2, m3 = st.columns(3)
        with m1:
            cruise = st.number_input("CruiseVelo (rot/s)", value=get_number("ShooterTurret/CruiseVelo", 20.0), step=1.0, key="tr_cruise")
        with m2:
            accel = st.number_input("Acceleration (rot/s²)", value=get_number("ShooterTurret/Acceleration", 20.0), step=1.0, key="tr_accel")
        with m3:
            jerk = st.number_input("Jerk (rot/s³)", value=get_number("ShooterTurret/Jerk", 0.0), step=10.0, key="tr_jerk")

        # Limits
        st.markdown("**Limits**")
        lim1, lim2, lim3 = st.columns(3)
        with lim1:
            max_volts = st.slider("MaxVolts (clamp)", 0.0, 12.0, value=get_number("ShooterTurret/MaxVolts", 12.0), step=0.1, key="tr_maxvolts")
        with lim2:
            min_deg = st.number_input("MinDeg", value=get_number("ShooterTurret/MinDeg", -180.0), step=5.0, key="tr_mindeg")
        with lim3:
            max_deg = st.number_input("MaxDeg", value=get_number("ShooterTurret/MaxDeg", 180.0), step=5.0, key="tr_maxdeg")

        # Anti-windup
        st.markdown("**Anti-windup (I term)**")
        aw1, aw2 = st.columns(2)
        with aw1:
            izone = st.slider("IZone (°, 0=disable)", 0.0, 30.0, value=get_number("ShooterTurret/IZone", 5.0), step=0.5, key="tr_izone")
        with aw2:
            imax = st.slider("IMax (clamp)", 0.0, 1.0, value=get_number("ShooterTurret/IMax", 0.1), step=0.01, key="tr_imax")

        # PID
        st.markdown("**PID**")
        g1, g2, g3 = st.columns(3)
        with g1:
            kp = st.number_input("kP", value=get_number("ShooterTurret/kP", 100.0), step=1.0, format="%.4f", key="tr_kp")
        with g2:
            ki = st.number_input("kI", value=get_number("ShooterTurret/kI", 0.0), step=0.01, format="%.4f", key="tr_ki")
        with g3:
            kd = st.number_input("kD", value=get_number("ShooterTurret/kD", 5.0), step=0.5, format="%.4f", key="tr_kd")

        # FF
        st.markdown("**Feedforward**")
        f1, f2, f3, f4 = st.columns(4)
        with f1:
            tr_ks = st.number_input("kS", value=get_number("ShooterTurret/kS", 0.0), step=0.01, format="%.4f", key="tr_ks")
        with f2:
            tr_kv = st.number_input("kV", value=get_number("ShooterTurret/kV", 1.2), step=0.1, format="%.4f", key="tr_kv")
        with f3:
            tr_ka = st.number_input("kA", value=get_number("ShooterTurret/kA", 0.0), step=0.01, format="%.4f", key="tr_ka")
        with f4:
            tr_kg = st.number_input("kG (gravity)", value=get_number("ShooterTurret/kG", 0.0), step=0.01, format="%.4f", key="tr_kg")

        if st.button("Apply Turret tunables", key="btn_apply_tr", use_container_width=True):
            put_number("ShooterTurret/CruiseVelo", cruise)
            put_number("ShooterTurret/Acceleration", accel)
            put_number("ShooterTurret/Jerk", jerk)
            put_number("ShooterTurret/MaxVolts", max_volts)
            put_number("ShooterTurret/MinDeg", min_deg)
            put_number("ShooterTurret/MaxDeg", max_deg)
            put_number("ShooterTurret/IZone", izone)
            put_number("ShooterTurret/IMax", imax)
            put_number("ShooterTurret/kP", kp)
            put_number("ShooterTurret/kI", ki)
            put_number("ShooterTurret/kD", kd)
            put_number("ShooterTurret/kS", tr_ks)
            put_number("ShooterTurret/kV", tr_kv)
            put_number("ShooterTurret/kA", tr_ka)
            put_number("ShooterTurret/kG", tr_kg)
            st.success("Applied.")

    # Graphs (Angle / Volts)
    st.divider()
    st.subheader("Graphs")
    gopt1, gopt2 = st.columns([2, 3])
    with gopt1:
        window_s_tr = st.slider("Show last (seconds)", 5, 120, 30, 5, key="tr_window_s")
    with gopt2:
        hist_points_tr = st.slider("History points kept", 200, 5000, 2000, 200, key="tr_hist")
    turret_poller = st.session_state["_turret_poller"]
    turret_poller.set_maxlen(hist_points_tr)
    t_tr, ang_tr, tgt_tr, out_tr = turret_poller.snapshot()
    ts = time.time()
    cut_tr = ts - float(window_s_tr)
    start_tr = 0
    for i, tt in enumerate(t_tr):
        if tt >= cut_tr:
            start_tr = i
            break
    df_turret = pd.DataFrame(
        {"AngleDeg": ang_tr[start_tr:], "TargetDeg": tgt_tr[start_tr:], "MotorVolts": out_tr[start_tr:]},
        index=pd.to_datetime(t_tr[start_tr:], unit="s"),
    )
    chart_tr1, chart_tr2 = st.columns(2)
    with chart_tr1:
        df_tr1 = df_turret[["AngleDeg", "TargetDeg"]].reset_index().rename(columns={"index": "t"})
        df_tr1 = df_tr1.melt("t", var_name="series", value_name="deg")
        st.altair_chart(
            alt.Chart(df_tr1)
            .mark_line()
            .encode(
                x=alt.X("t:T", title="Time"),
                y=alt.Y("deg:Q", title="Degrees"),
                color=alt.Color("series:N", title=""),
            )
            .properties(height=280),
            use_container_width=True,
        )
    with chart_tr2:
        df_tr2 = df_turret[["MotorVolts"]].reset_index().rename(columns={"index": "t"})
        st.altair_chart(
            alt.Chart(df_tr2)
            .mark_line()
            .encode(
                x=alt.X("t:T", title="Time"),
                y=alt.Y("MotorVolts:Q", title="Motor Volts"),
            )
            .properties(height=280),
            use_container_width=True,
        )
    st.caption("Turret 角度/目标 与 电机电压 随时间。")

# ---------- Hood ----------
with tab_hood:
    st.subheader("Hood")
    st.caption("Hood angle (ProfiledPID + ArmFeedforward).")

    # Status
    st.markdown("**Status**")
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        st.metric("Current angle (°)", f"{get_number('ShooterHood/CurrAngDeg', 0):.2f}")
    with c2:
        goal_rad = get_number('ShooterHood/GoalRad', 0)
        goal_deg = goal_rad * 180.0 / 3.141592653589793
        st.metric("Goal (°)", f"{goal_deg:.2f}")
    with c3:
        st.metric("Error (°)", f"{get_number('ShooterHood/ErrorDeg', 0):.2f}")
    with c4:
        st.metric("Velocity (°/s)", f"{get_number('ShooterHood/VelocityDegPerS', 0):.1f}")
    c5, c6, c7, c8 = st.columns(4)
    with c5:
        st.metric("PID volts", f"{get_number('ShooterHood/PIDVolts', 0):.2f}")
    with c6:
        st.metric("FF volts", f"{get_number('ShooterHood/FFVolts', 0):.2f}")
    with c7:
        st.metric("Output volts", f"{get_number('ShooterHood/OutputVolts', 0):.2f}")
    with c8:
        st.metric("At target", get_boolean("ShooterHood/AtTarget", False))

    # Controls (draft target pattern)
    st.divider()
    st.markdown("**Controls**")
    ctrl1, ctrl2, ctrl3 = st.columns(3)
    with ctrl1:
        draft_deg = st.number_input("Draft target (°, local)", value=get_number("ShooterHood/RequestDegrees", 0.0), step=1.0, key="hood_draft_deg")
    with ctrl2:
        if st.button("Send RequestDegrees", key="btn_send_hood_req", use_container_width=True):
            put_number("ShooterHood/RequestDegrees", draft_deg)
            st.success("已发送。")
    with ctrl3:
        if st.button("ZERO (清零 Hood encoder)", type="primary", key="btn_hood_zero", use_container_width=True):
            put_boolean("ShooterHood/Zero", True)
            st.success("已发送 Zero，机器人端执行后会将 encoder 置为 ENCODER_OFFSET。")

    # Calibration (set current position)
    st.divider()
    st.markdown("**Calibration**")
    cal1, cal2 = st.columns(2)
    with cal1:
        set_pos_deg = st.number_input("SetPosDeg (set encoder to this °)", value=get_number("ShooterHood/SetPosDeg", 0.0), step=1.0, key="hood_setpos")
    with cal2:
        if st.button("Set Position", key="btn_hood_setpos", use_container_width=True):
            put_number("ShooterHood/SetPosDeg", set_pos_deg)
            put_boolean("ShooterHood/SetPos", True)
            st.success("已发送 SetPos。")

    # Tunables
    with st.expander("Tunables (write to NT)", expanded=True):
        # TuneLive toggle
        tune_live = st.toggle("TuneLive (apply while running)", value=get_boolean("ShooterHood/TuneLive", False), key="hood_tunelive")
        put_boolean("ShooterHood/TuneLive", tune_live)

        # Motion Profile
        st.markdown("**Motion Profile (ProfiledPIDController)**")
        mp1, mp2 = st.columns(2)
        with mp1:
            max_vel = st.number_input("MaxVel (rad/s)", value=get_number("ShooterHood/MaxVel", 14.0), step=1.0, key="hood_maxvel")
        with mp2:
            max_acc = st.number_input("MaxAcc (rad/s²)", value=get_number("ShooterHood/MaxAcc", 20.0), step=1.0, key="hood_maxacc")

        # Limits
        st.markdown("**Limits**")
        lim1, lim2, lim3 = st.columns(3)
        with lim1:
            max_volts = st.slider("MaxVolts (clamp)", 0.0, 12.0, value=get_number("ShooterHood/MaxVolts", 12.0), step=0.1, key="hood_maxvolts")
        with lim2:
            min_deg = st.number_input("MinDeg", value=get_number("ShooterHood/MinDeg", 0.0), step=5.0, key="hood_mindeg")
        with lim3:
            max_deg = st.number_input("MaxDeg", value=get_number("ShooterHood/MaxDeg", 90.0), step=5.0, key="hood_maxdeg")

        # Anti-windup
        st.markdown("**Anti-windup (I term)**")
        aw1, aw2 = st.columns(2)
        with aw1:
            izone = st.slider("IZoneDeg (0=disable)", 0.0, 10.0, value=get_number("ShooterHood/IZoneDeg", 5.0), step=0.1, key="hood_izone")
        with aw2:
            imax = st.slider("IMaxVolts (clamp)", 0.0, 3.0, value=get_number("ShooterHood/IMaxVolts", 1.0), step=0.1, key="hood_imax")

        # PID
        st.markdown("**PID**")
        g1, g2, g3 = st.columns(3)
        with g1:
            kp = st.number_input("kP", value=get_number("ShooterHood/kP", 40.0), step=1.0, format="%.4f", key="hood_kp")
        with g2:
            ki = st.number_input("kI", value=get_number("ShooterHood/kI", 100.0), step=1.0, format="%.4f", key="hood_ki")
        with g3:
            kd = st.number_input("kD", value=get_number("ShooterHood/kD", 0.7), step=0.1, format="%.4f", key="hood_kd")

        # FF (ArmFeedforward)
        st.markdown("**Feedforward (ArmFeedforward)**")
        f1, f2, f3, f4 = st.columns(4)
        with f1:
            ks = st.number_input("kS", value=get_number("ShooterHood/kS", 0.19), step=0.01, format="%.4f", key="hood_ks")
        with f2:
            kg = st.number_input("kG", value=get_number("ShooterHood/kG", 0.0), step=0.01, format="%.4f", key="hood_kg")
        with f3:
            kv = st.number_input("kV", value=get_number("ShooterHood/kV", 3.0), step=0.1, format="%.4f", key="hood_kv")
        with f4:
            ka = st.number_input("kA", value=get_number("ShooterHood/kA", 0.01), step=0.001, format="%.4f", key="hood_ka")

        if st.button("Apply Hood tunables", key="btn_apply_hood", use_container_width=True):
            put_number("ShooterHood/MaxVel", max_vel)
            put_number("ShooterHood/MaxAcc", max_acc)
            put_number("ShooterHood/MaxVolts", max_volts)
            put_number("ShooterHood/MinDeg", min_deg)
            put_number("ShooterHood/MaxDeg", max_deg)
            put_number("ShooterHood/IZoneDeg", izone)
            put_number("ShooterHood/IMaxVolts", imax)
            put_number("ShooterHood/kP", kp)
            put_number("ShooterHood/kI", ki)
            put_number("ShooterHood/kD", kd)
            put_number("ShooterHood/kS", ks)
            put_number("ShooterHood/kG", kg)
            put_number("ShooterHood/kV", kv)
            put_number("ShooterHood/kA", ka)
            st.success("Applied.")

    # Graphs (Angle / Volts)
    st.divider()
    st.subheader("Graphs")
    gopt1, gopt2 = st.columns([2, 3])
    with gopt1:
        window_s_hood = st.slider("Show last (seconds)", 5, 120, 30, 5, key="hood_window_s")
    with gopt2:
        hist_points_hood = st.slider("History points kept", 200, 5000, 2000, 200, key="hood_hist")
    hood_poller = st.session_state["_hood_poller"]
    hood_poller.set_maxlen(hist_points_hood)
    t_h, ang_h, goal_h, out_h = hood_poller.snapshot()
    ts = time.time()
    cut_h = ts - float(window_s_hood)
    start_h = 0
    for i, tt in enumerate(t_h):
        if tt >= cut_h:
            start_h = i
            break
    df_hood = pd.DataFrame(
        {"AngleDeg": ang_h[start_h:], "GoalDeg": goal_h[start_h:], "OutputVolts": out_h[start_h:]},
        index=pd.to_datetime(t_h[start_h:], unit="s"),
    )
    chart_h1, chart_h2 = st.columns(2)
    with chart_h1:
        df_h1 = df_hood[["AngleDeg", "GoalDeg"]].reset_index().rename(columns={"index": "t"})
        df_h1 = df_h1.melt("t", var_name="series", value_name="deg")
        st.altair_chart(
            alt.Chart(df_h1)
            .mark_line()
            .encode(
                x=alt.X("t:T", title="Time"),
                y=alt.Y("deg:Q", title="Degrees"),
                color=alt.Color("series:N", title=""),
            )
            .properties(height=280),
            use_container_width=True,
        )
    with chart_h2:
        df_h2 = df_hood[["OutputVolts"]].reset_index().rename(columns={"index": "t"})
        st.altair_chart(
            alt.Chart(df_h2)
            .mark_line()
            .encode(
                x=alt.X("t:T", title="Time"),
                y=alt.Y("OutputVolts:Q", title="Output (Volts)"),
            )
            .properties(height=280),
            use_container_width=True,
        )
    st.caption("Hood 角度/目标 与 输出电压 随时间。")

# ---------- Controller ----------
with tab_ctrl:
    st.subheader("Shooter Controller")
    st.caption("Unified aiming. Tunable target: TunableTarget/X, Y, Z, Enable")

    # Status row 1
    st.markdown("**Status**")
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        st.metric("Enabled", get_boolean("ShooterController/Enabled", False))
    with c2:
        st.metric("Distance (m)", f"{get_number('ShooterController/DistanceToTarget', 0):.2f}")
    with c3:
        st.metric("Speed preset", get_string("ShooterController/SpeedPreset", "—"))
    with c4:
        st.metric("Launch vel (m/s)", f"{get_number('ShooterController/LaunchVelocity', 0):.2f}")

    # Status row 2 - target outputs
    st.markdown("**Target Outputs**")
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        st.metric("Turret (°)", f"{get_number('ShooterController/TargetTurretAngle', 0):.2f}")
    with c2:
        st.metric("Hood (°)", f"{get_number('ShooterController/TargetHoodAngle', 0):.2f}")
    with c3:
        st.metric("Flywheel RPM", f"{get_number('ShooterController/TargetFlywheelRPM', 0):.0f}")
    with c4:
        st.metric("Theta (solver, °)", f"{get_number('ShooterController/SolverTheta', 0):.2f}")

    # Status row 3 - ready states
    st.markdown("**Ready States**")
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        st.metric("Turret on target", get_boolean("ShooterController/TurretOnTarget", False))
    with c2:
        st.metric("Hood on target", get_boolean("ShooterController/HoodOnTarget", False))
    with c3:
        st.metric("Flywheel ready", get_boolean("ShooterController/FlywheelReady", False))
    with c4:
        ready = get_boolean("ShooterController/ReadyToShoot", False)
        if ready:
            st.success("Ready to shoot!")
        else:
            st.metric("Ready to shoot", False)

    # Solver info
    st.markdown("**Solver Info**")
    c1, c2, c3, c4 = st.columns(4)
    with c1:
        st.metric("Solver success", get_boolean("ShooterController/SolverSuccess", False))
    with c2:
        st.metric("Predicted miss (m)", f"{get_number('ShooterController/PredictedMiss', 0):.3f}")
    with c3:
        st.metric("Time of flight (s)", f"{get_number('ShooterController/TimeOfFlight', 0):.3f}")
    with c4:
        st.metric("Iterations", f"{get_number('ShooterController/SolverIterations', 0):.0f}")

    # Controls
    st.divider()
    st.markdown("**Controls**")
    ctrl1, ctrl2 = st.columns(2)
    with ctrl1:
        if st.button("Enable Controller", key="btn_ctrl_enable", use_container_width=True):
            put_boolean("ShooterController/Enabled", True)
            st.success("Enabled.")
    with ctrl2:
        if st.button("Disable Controller", key="btn_ctrl_disable", use_container_width=True):
            put_boolean("ShooterController/Enabled", False)
            st.success("Disabled.")

    # Tunable target
    with st.expander("Tunable target (sim test)", expanded=True):
        st.markdown("**Target Position**")
        t1, t2, t3 = st.columns(3)
        with t1:
            tx = st.number_input("X (m)", value=get_number("ShooterController/TunableTarget/X", 5.0), step=0.5, key="ctrl_x")
        with t2:
            ty = st.number_input("Y (m)", value=get_number("ShooterController/TunableTarget/Y", 0.0), step=0.5, key="ctrl_y")
        with t3:
            tz = st.number_input("Z (m)", value=get_number("ShooterController/TunableTarget/Z", 2.0), step=0.1, key="ctrl_z")
        enable = st.toggle("TunableTarget Enable", value=get_boolean("ShooterController/TunableTarget/Enable", False), key="ctrl_enable")
        if st.button("Apply Tunable Target", key="btn_apply_ctrl", use_container_width=True):
            put_number("ShooterController/TunableTarget/X", tx)
            put_number("ShooterController/TunableTarget/Y", ty)
            put_number("ShooterController/TunableTarget/Z", tz)
            put_boolean("ShooterController/TunableTarget/Enable", enable)
            st.success("Applied.")

    # Controller tunables
    with st.expander("Controller tunables", expanded=False):
        st.markdown("**Tolerances**")
        tol1, tol2, tol3 = st.columns(3)
        with tol1:
            turret_tol = st.number_input("TurretTolerance (°)", value=get_number("ShooterController/TurretTolerance", 1.0), step=0.1, key="ctrl_turret_tol")
        with tol2:
            hood_tol = st.number_input("HoodTolerance (°)", value=get_number("ShooterController/HoodTolerance", 1.0), step=0.1, key="ctrl_hood_tol")
        with tol3:
            flywheel_tol = st.number_input("FlywheelTolerance (RPM)", value=get_number("ShooterController/FlywheelTolerance", 50.0), step=10.0, key="ctrl_fw_tol")
        if st.button("Apply Controller tunables", key="btn_apply_ctrl_tol", use_container_width=True):
            put_number("ShooterController/TurretTolerance", turret_tol)
            put_number("ShooterController/HoodTolerance", hood_tol)
            put_number("ShooterController/FlywheelTolerance", flywheel_tol)
            st.success("Applied.")

# ---------- Ballistics ----------
with tab_ball:
    st.subheader("Ballistics")
    st.caption("Ballistic solver parameters. Constants 在 BallisticConstants.java，输出在 Controller tab。")

    # Game piece info (read-only display)
    st.markdown("**FRC 2026 Game Piece**")
    gp1, gp2, gp3, gp4 = st.columns(4)
    with gp1:
        st.metric("Diameter (m)", "0.15")
    with gp2:
        st.metric("Weight (kg)", "0.363")
    with gp3:
        st.metric("Cd", "0.50")
    with gp4:
        st.metric("kDrag", "0.015")

    # Three-speed presets (read-only)
    st.divider()
    st.markdown("**Three-Speed Presets**")
    sp1, sp2, sp3 = st.columns(3)
    with sp1:
        st.markdown("**LOW**")
        st.caption("3000 RPM, 12 m/s")
    with sp2:
        st.markdown("**MEDIUM**")
        st.caption("4500 RPM, 18 m/s")
    with sp3:
        st.markdown("**HIGH**")
        st.caption("6000 RPM, 24 m/s")

    st.markdown("**Distance Thresholds**")
    dt1, dt2 = st.columns(2)
    with dt1:
        st.metric("LOW→MEDIUM (m)", "4.0")
    with dt2:
        st.metric("MEDIUM→HIGH (m)", "7.0")

    # Solver tunables
    st.divider()
    with st.expander("Solver Tunables (write to NT)", expanded=True):
        st.markdown("**Solver Parameters**")
        s1, s2, s3, s4 = st.columns(4)
        with s1:
            gravity = st.number_input("Gravity (m/s²)", value=get_number("Ballistics/Gravity", 9.80665), step=0.01, format="%.5f", key="ball_gravity")
        with s2:
            dt = st.number_input("dt (s)", value=get_number("Ballistics/dt", 0.005), step=0.001, format="%.4f", key="ball_dt")
        with s3:
            t_max = st.number_input("tMax (s)", value=get_number("Ballistics/tMax", 3.0), step=0.5, key="ball_tmax")
        with s4:
            tol_miss = st.number_input("tolMiss (m)", value=get_number("Ballistics/tolMiss", 0.05), step=0.01, format="%.3f", key="ball_tolmiss")

        st.markdown("**Drag / Air**")
        d1, d2, d3 = st.columns(3)
        with d1:
            k_drag = st.number_input("kDrag", value=get_number("Ballistics/kDrag", 0.015), step=0.001, format="%.4f", key="ball_kdrag")
        with d2:
            air_density = st.number_input("Air density (kg/m³)", value=get_number("Ballistics/AirDensity", 1.225), step=0.01, format="%.3f", key="ball_airdensity")
        with d3:
            high_arc = st.toggle("highArc (抛物模式)", value=get_boolean("Ballistics/highArc", True), key="ball_higharc")

        st.markdown("**Hood Calibration**")
        h1, h2 = st.columns(2)
        with h1:
            hood_offset = st.number_input("HOOD_LAUNCH_ANGLE_OFFSET (°)", value=get_number("Ballistics/HoodLaunchAngleOffset", 3.5), step=0.1, format="%.2f", key="ball_hoodoffset")
        with h2:
            st.caption("θ_launch = θ_hood + offset")

        st.markdown("**Speed Preset Distances**")
        spd1, spd2 = st.columns(2)
        with spd1:
            low_to_med = st.number_input("LOW→MEDIUM (m)", value=get_number("Ballistics/LowToMediumDistance", 4.0), step=0.5, key="ball_lowtomed")
        with spd2:
            med_to_high = st.number_input("MEDIUM→HIGH (m)", value=get_number("Ballistics/MediumToHighDistance", 7.0), step=0.5, key="ball_medtohigh")

        if st.button("Apply Ballistics tunables", key="btn_apply_ball", use_container_width=True):
            put_number("Ballistics/Gravity", gravity)
            put_number("Ballistics/dt", dt)
            put_number("Ballistics/tMax", t_max)
            put_number("Ballistics/tolMiss", tol_miss)
            put_number("Ballistics/kDrag", k_drag)
            put_number("Ballistics/AirDensity", air_density)
            put_boolean("Ballistics/highArc", high_arc)
            put_number("Ballistics/HoodLaunchAngleOffset", hood_offset)
            put_number("Ballistics/LowToMediumDistance", low_to_med)
            put_number("Ballistics/MediumToHighDistance", med_to_high)
            st.success("Applied.")

if auto_refresh:
    st_autorefresh(interval=refresh_ms, key="shooter_refresh")
