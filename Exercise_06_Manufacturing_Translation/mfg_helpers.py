"""
Manufacturing Translation Helpers
==================================
Simple PID-controlled manufacturing plant simulations for three common
systems: conveyor belt, industrial oven, and tank level.

Each simulator uses explicit Euler integration with an anti-windup PID
controller, matching the style used in the cruise control and conveyor
belt demos earlier in this course.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Conveyor belt speed control
# ---------------------------------------------------------------------------

def default_conveyor_params():
    """Return baseline conveyor belt parameters."""
    return dict(
        # Plant
        m_belt=50.0,           # Belt + roller mass [kg]
        friction=5.0,          # Viscous friction [N*s/m]
        motor_gain=20.0,       # Motor force per unit command [N/unit]
        # PID
        Kp=4.0, Ki=1.5, Kd=0.3,
        u_min=0.0, u_max=10.0,  # Motor command limits [unit]
        # Sim
        dt=0.01, t_end=60.0,
        y0=0.0,                # Initial speed [m/s]
    )


def simulate_conveyor(params, target, load_disturbance):
    """Simulate conveyor belt speed control under varying box load.

    Args:
        params: dict from default_conveyor_params()
        target: array of target speed [m/s]
        load_disturbance: array of extra friction force from boxes [N]

    Returns:
        dict with keys: t, y, ref, u, dist
    """
    m = params["m_belt"]
    b = params["friction"]
    K = params["motor_gain"]
    Kp, Ki, Kd = params["Kp"], params["Ki"], params["Kd"]
    u_min, u_max = params["u_min"], params["u_max"]
    dt, t_end = params["dt"], params["t_end"]

    t = np.arange(0, t_end, dt)
    n = len(t)
    y = np.zeros(n)
    u = np.zeros(n)
    y[0] = params["y0"]

    integral_e = 0.0
    prev_e = 0.0

    for i in range(1, n):
        e = target[i - 1] - y[i - 1]
        integral_e += e * dt
        de = (e - prev_e) / dt
        prev_e = e

        u_raw = Kp * e + Ki * integral_e + Kd * de
        u[i] = np.clip(u_raw, u_min, u_max)
        if u[i] != u_raw:
            integral_e -= e * dt  # anti-windup

        # m * dv/dt = motor_force - friction*v - load
        dv = (K * u[i] - b * y[i - 1] - load_disturbance[i - 1]) / m
        y[i] = max(y[i - 1] + dv * dt, 0.0)

    return dict(t=t, y=y, ref=target[:n], u=u, dist=load_disturbance[:n])


# ---------------------------------------------------------------------------
# Industrial oven temperature control
# ---------------------------------------------------------------------------

def default_oven_params():
    """Return baseline oven / furnace parameters."""
    return dict(
        # Plant (first-order thermal model)
        tau=30.0,              # Thermal time constant [s]
        K_heater=25.0,         # Heater gain [°C per unit power]
        T_ambient=25.0,        # Ambient temperature [°C]
        # PID
        Kp=2.0, Ki=0.15, Kd=1.0,
        u_min=0.0, u_max=10.0,  # Heater power limits [unit]
        # Sim
        dt=0.1, t_end=300.0,
        y0=25.0,               # Initial temperature [°C]
    )


def simulate_oven(params, target, heat_loss):
    """Simulate oven temperature control with heat-loss disturbance.

    Plant model:
        tau * dT/dt = -T + T_ambient + K_heater * u - heat_loss

    Args:
        params: dict from default_oven_params()
        target: array of target temperature [°C]
        heat_loss: array of extra heat loss [°C] (e.g. door opened)

    Returns:
        dict with keys: t, y, ref, u, dist
    """
    tau = params["tau"]
    K = params["K_heater"]
    T_amb = params["T_ambient"]
    Kp, Ki, Kd = params["Kp"], params["Ki"], params["Kd"]
    u_min, u_max = params["u_min"], params["u_max"]
    dt, t_end = params["dt"], params["t_end"]

    t = np.arange(0, t_end, dt)
    n = len(t)
    y = np.zeros(n)
    u = np.zeros(n)
    y[0] = params["y0"]

    integral_e = 0.0
    prev_e = 0.0

    for i in range(1, n):
        e = target[i - 1] - y[i - 1]
        integral_e += e * dt
        de = (e - prev_e) / dt
        prev_e = e

        u_raw = Kp * e + Ki * integral_e + Kd * de
        u[i] = np.clip(u_raw, u_min, u_max)
        if u[i] != u_raw:
            integral_e -= e * dt

        # First-order thermal dynamics
        dT = (-y[i - 1] + T_amb + K * u[i] - heat_loss[i - 1]) / tau
        y[i] = y[i - 1] + dT * dt

    return dict(t=t, y=y, ref=target[:n], u=u, dist=heat_loss[:n])


# ---------------------------------------------------------------------------
# Tank level control
# ---------------------------------------------------------------------------

def default_tank_params():
    """Return baseline tank level parameters."""
    return dict(
        # Plant
        area=2.0,              # Tank cross-section [m^2]
        outflow_coeff=0.1,     # Drain coefficient [m^2.5/s]
        pump_gain=0.05,        # Inflow per unit pump command [m^3/s per unit]
        # PID
        Kp=10.0, Ki=3.0, Kd=1.0,
        u_min=0.0, u_max=10.0,  # Pump command limits [unit]
        # Sim
        dt=0.05, t_end=120.0,
        y0=0.5,                # Initial level [m]
    )


def simulate_tank(params, target, outflow_disturbance):
    """Simulate tank level control with variable outflow.

    Plant model (nonlinear):
        A * dh/dt = pump_gain * u - outflow_coeff * sqrt(h) - outflow_disturbance

    Args:
        params: dict from default_tank_params()
        target: array of target level [m]
        outflow_disturbance: array of extra outflow [m^3/s]

    Returns:
        dict with keys: t, y, ref, u, dist
    """
    A = params["area"]
    c = params["outflow_coeff"]
    Kpump = params["pump_gain"]
    Kp, Ki, Kd = params["Kp"], params["Ki"], params["Kd"]
    u_min, u_max = params["u_min"], params["u_max"]
    dt, t_end = params["dt"], params["t_end"]

    t = np.arange(0, t_end, dt)
    n = len(t)
    y = np.zeros(n)
    u = np.zeros(n)
    y[0] = params["y0"]

    integral_e = 0.0
    prev_e = 0.0

    for i in range(1, n):
        e = target[i - 1] - y[i - 1]
        integral_e += e * dt
        de = (e - prev_e) / dt
        prev_e = e

        u_raw = Kp * e + Ki * integral_e + Kd * de
        u[i] = np.clip(u_raw, u_min, u_max)
        if u[i] != u_raw:
            integral_e -= e * dt

        # Nonlinear tank dynamics
        h = max(y[i - 1], 0.0)
        dh = (Kpump * u[i] - c * np.sqrt(h) - outflow_disturbance[i - 1]) / A
        y[i] = max(y[i - 1] + dh * dt, 0.0)

    return dict(t=t, y=y, ref=target[:n], u=u, dist=outflow_disturbance[:n])


# ---------------------------------------------------------------------------
# Plotly figure builder (shared by all three systems)
# ---------------------------------------------------------------------------

def build_mfg_figure(result, labels, title="Manufacturing Control"):
    """Build a 3-subplot Plotly figure from a simulation result dict.

    Args:
        result: dict with keys t, y, ref, u, dist
        labels: dict with keys y_label, u_label, dist_label, y_unit, u_unit, dist_unit
        title:  figure title string
    """
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    t = result["t"]
    fig = make_subplots(
        rows=3, cols=1, shared_xaxes=True,
        subplot_titles=(
            f"{labels['y_label']} vs Target",
            f"Control Effort ({labels['u_label']})",
            f"Disturbance ({labels['dist_label']})",
        ),
        vertical_spacing=0.08,
    )

    # Row 1: output tracking
    fig.add_trace(go.Scatter(
        x=t, y=result["ref"], mode="lines",
        name="Target", line=dict(color="black", dash="dash", width=1.5),
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=t, y=result["y"], mode="lines",
        name=labels["y_label"], line=dict(color="#636EFA", width=2),
    ), row=1, col=1)

    # Row 2: control effort
    fig.add_trace(go.Scatter(
        x=t, y=result["u"], mode="lines",
        name=labels["u_label"], line=dict(color="#EF553B", width=1.5),
    ), row=2, col=1)

    # Row 3: disturbance
    fig.add_trace(go.Scatter(
        x=t, y=result["dist"], mode="lines",
        name=labels["dist_label"], line=dict(color="#00CC96", width=1.5),
        fill="tozeroy", fillcolor="rgba(0,204,150,0.15)",
    ), row=3, col=1)

    fig.update_yaxes(title_text=f"{labels['y_label']} [{labels['y_unit']}]", row=1, col=1)
    fig.update_yaxes(title_text=f"{labels['u_label']} [{labels['u_unit']}]", row=2, col=1)
    fig.update_yaxes(title_text=f"{labels['dist_label']} [{labels['dist_unit']}]", row=3, col=1)
    fig.update_xaxes(title_text="Time [s]", row=3, col=1)

    fig.update_layout(
        template="plotly_white",
        height=700,
        title_text=title,
        showlegend=True,
        legend=dict(orientation="h", yanchor="bottom", y=1.02,
                    xanchor="center", x=0.5),
        margin=dict(t=80, b=40),
    )
    return fig
