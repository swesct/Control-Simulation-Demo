"""
Cruise Control Helpers
======================
Simulation engine, scenario generation, and Plotly figure builders for the
cruise control student demo.

Vehicle model:
    m * dv/dt = F_engine - F_drag - F_rolling - F_hill
where:
    F_engine  = throttle * engine_force_max          [N]
    F_drag    = 0.5 * Cd * A * rho * v^2             [N]
    F_rolling = Cr * m * g                           [N]
    F_hill    = m * g * sin(grade)                   [N]

Throttle is clipped to [0, 1] (0% to 100%).
"""

import numpy as np


# ---------------------------------------------------------------------------
# Default parameters
# ---------------------------------------------------------------------------

def default_params():
    """Return a dict of baseline simulation parameters."""
    return dict(
        # Vehicle
        m=1200.0,              # Vehicle mass [kg]
        Cd=0.3,                # Drag coefficient
        A=2.5,                 # Frontal area [m^2]
        rho=1.225,             # Air density [kg/m^3]
        Cr=0.01,               # Rolling resistance coefficient
        engine_force_max=4000.0,  # Max engine force at full throttle [N]
        g=9.81,                # Gravity [m/s^2]
        # PID gains
        Kp=0.05,
        Ki=0.01,
        Kd=0.005,
        # Simulation
        t_end=120.0,           # Duration [s]
        dt=0.05,               # Time step [s]
        v0=0.0,                # Initial speed [m/s]
    )


# ---------------------------------------------------------------------------
# Scenario generation
# ---------------------------------------------------------------------------

def build_scenario(t, scenario="flat", target_speed_kmh=80.0):
    """Build target speed [m/s] and road grade [rad] arrays over time.

    Scenarios:
        flat             - constant target, zero grade
        uphill           - constant target, grade ramp up at t=30s
        downhill         - constant target, grade ramp down at t=30s
        hill_profile     - constant target, up then down
        speed_change     - target speed changes mid-run, flat road
    """
    n = len(t)
    target = np.full(n, target_speed_kmh / 3.6)  # km/h -> m/s
    grade = np.zeros(n)

    if scenario == "uphill":
        grade[t >= 30] = 0.05           # 5% grade after 30s

    elif scenario == "downhill":
        grade[t >= 30] = -0.05          # -5% grade after 30s

    elif scenario == "hill_profile":
        grade[(t >= 30) & (t < 60)] = 0.06   # uphill 30-60s
        grade[(t >= 60) & (t < 90)] = -0.04  # downhill 60-90s

    elif scenario == "speed_change":
        target[t >= 40] = (target_speed_kmh + 20) / 3.6
        target[t >= 80] = (target_speed_kmh - 10) / 3.6

    return target, grade


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def simulate(params, target, grade):
    """Run cruise control simulation.

    Args:
        params: dict from default_params() (may be modified).
        target: array of target speed [m/s].
        grade:  array of road grade [rad].

    Returns:
        dict with keys: t, v, target, throttle, grade, F_engine, F_drag, F_hill
    """
    m = params["m"]
    Cd = params["Cd"]
    A = params["A"]
    rho = params["rho"]
    Cr = params["Cr"]
    engine_force_max = params["engine_force_max"]
    g = params["g"]
    Kp = params["Kp"]
    Ki = params["Ki"]
    Kd = params["Kd"]
    dt = params["dt"]
    t_end = params["t_end"]
    v0 = params["v0"]

    t = np.arange(0, t_end, dt)
    n = len(t)
    v = np.zeros(n)
    throttle = np.zeros(n)
    F_engine_arr = np.zeros(n)
    F_drag_arr = np.zeros(n)
    F_hill_arr = np.zeros(n)

    v[0] = v0
    integral_e = 0.0
    prev_e = 0.0

    for i in range(1, n):
        # PID error
        e = target[i - 1] - v[i - 1]
        integral_e += e * dt
        derivative_e = (e - prev_e) / dt
        prev_e = e

        # Throttle command (0 = no power, 1 = full power)
        throttle_raw = Kp * e + Ki * integral_e + Kd * derivative_e
        throttle[i] = np.clip(throttle_raw, 0.0, 1.0)

        # Anti-windup: undo integration when saturated
        if throttle[i] != throttle_raw:
            integral_e -= e * dt

        # Forces
        F_engine = throttle[i] * engine_force_max
        F_drag = 0.5 * Cd * A * rho * v[i - 1] ** 2
        F_rolling = Cr * m * g
        F_hill = m * g * np.sin(grade[i - 1])

        # Dynamics: m * dv/dt = net force
        dv = (F_engine - F_drag - F_rolling - F_hill) / m
        v[i] = max(v[i - 1] + dv * dt, 0.0)  # speed can't go negative

        F_engine_arr[i] = F_engine
        F_drag_arr[i] = F_drag
        F_hill_arr[i] = F_hill

    return dict(
        t=t, v=v, target=target[:n], throttle=throttle,
        grade=grade[:n],
        F_engine=F_engine_arr, F_drag=F_drag_arr, F_hill=F_hill_arr,
    )


# ---------------------------------------------------------------------------
# Plotting (Plotly)
# ---------------------------------------------------------------------------

def build_figure(result):
    """Build a 3-subplot Plotly figure from simulation results."""
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    t = result["t"]
    v_kmh = result["v"] * 3.6
    target_kmh = result["target"] * 3.6
    throttle_pct = result["throttle"] * 100
    grade_pct = result["grade"] * 100

    fig = make_subplots(
        rows=3, cols=1, shared_xaxes=True,
        subplot_titles=(
            "Vehicle Speed vs Target",
            "Throttle (Control Effort)",
            "Road Grade (Disturbance)",
        ),
        vertical_spacing=0.08,
    )

    # Row 1: speed
    fig.add_trace(go.Scatter(
        x=t, y=target_kmh, mode="lines",
        name="Target speed", line=dict(color="black", dash="dash", width=1.5),
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=t, y=v_kmh, mode="lines",
        name="Vehicle speed", line=dict(color="#636EFA", width=2),
    ), row=1, col=1)

    # Row 2: throttle
    fig.add_trace(go.Scatter(
        x=t, y=throttle_pct, mode="lines",
        name="Throttle", line=dict(color="#EF553B", width=1.5),
    ), row=2, col=1)

    # Row 3: grade
    fig.add_trace(go.Scatter(
        x=t, y=grade_pct, mode="lines",
        name="Road grade", line=dict(color="#00CC96", width=1.5),
        fill="tozeroy", fillcolor="rgba(0,204,150,0.15)",
    ), row=3, col=1)

    fig.update_yaxes(title_text="Speed [km/h]", row=1, col=1)
    fig.update_yaxes(title_text="Throttle [%]", range=[-5, 105], row=2, col=1)
    fig.update_yaxes(title_text="Grade [%]", row=3, col=1)
    fig.update_xaxes(title_text="Time [s]", row=3, col=1)

    fig.update_layout(
        template="plotly_white",
        height=700,
        showlegend=True,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="center", x=0.5),
        margin=dict(t=60, b=40),
    )

    return fig
