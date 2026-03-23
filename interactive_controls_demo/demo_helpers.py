"""
Demo Helpers — Conveyor Belt Speed Control
===========================================
Simulation engine, parameter builders, and Plotly figure generators for
the interactive controls demo.

Plant model (conveyor belt driven by a DC motor):
    Transfer function:  G(s) = K / (tau*s + 1)*(J*s + B)
    Simplified 2nd-order form:
        G(s) = K_plant / (s^2 + 2*zeta*wn*s + wn^2)

    States: [belt_speed, motor_current]
    Input:  voltage command u  (clipped to actuator limits)
    Output: belt speed y  [m/s]

    Disturbance: a step load (product weight hitting the belt).
"""

import numpy as np


# ---------------------------------------------------------------------------
# Default parameters
# ---------------------------------------------------------------------------

def default_plant_params():
    """Return baseline conveyor belt plant parameters."""
    return dict(
        # Motor + belt dynamics
        J=0.5,            # Combined rotor + belt inertia [kg*m^2]
        B=0.8,            # Viscous friction [N*m*s/rad]
        K_motor=2.0,      # Motor torque constant [N*m/V]
        R_belt=0.1,       # Belt roller radius [m]
        # Equivalent second-order
        wn=2.0,           # Natural frequency [rad/s]
        zeta=0.4,         # Damping ratio
        K_plant=1.0,      # DC gain [m/s per V]
    )


def default_controller_params():
    """Return baseline PID controller parameters."""
    return dict(
        Kp=5.0,
        Ki=2.0,
        Kd=0.5,
        u_min=0.0,        # Actuator lower limit [V]
        u_max=10.0,       # Actuator upper limit [V]
        anti_windup=True,
    )


def default_sim_params():
    """Return baseline simulation parameters."""
    return dict(
        t_end=30.0,
        dt=0.005,
        y0=0.0,           # Initial belt speed [m/s]
    )


# ---------------------------------------------------------------------------
# Scenario generation
# ---------------------------------------------------------------------------

def build_scenario(t, scenario="step", target_speed=1.0):
    """Build target speed [m/s] and disturbance load [N] arrays.

    Scenarios:
        step            - constant target, no disturbance
        load_step       - constant target, load disturbance at t=10s
        speed_change    - target changes mid-run, no disturbance
        full            - target changes AND load disturbance
    """
    n = len(t)
    ref = np.full(n, target_speed)
    dist = np.zeros(n)

    if scenario == "load_step":
        dist[t >= 10.0] = 3.0   # Product load hits belt at t=10s

    elif scenario == "speed_change":
        ref[t >= 10.0] = target_speed * 1.5
        ref[t >= 20.0] = target_speed * 0.8

    elif scenario == "full":
        ref[t >= 10.0] = target_speed * 1.5
        ref[t >= 20.0] = target_speed * 0.8
        dist[(t >= 8.0) & (t < 15.0)] = 3.0
        dist[(t >= 22.0)] = 2.0

    return ref, dist


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def simulate(plant, controller, sim, ref, dist):
    """Run closed-loop conveyor speed control simulation.

    Uses a second-order plant model integrated with Euler method and an
    explicit PID controller.

    Args:
        plant:      dict from default_plant_params()
        controller: dict from default_controller_params()
        sim:        dict from default_sim_params()
        ref:        target speed array [m/s]
        dist:       disturbance load array [N]

    Returns:
        dict with keys: t, y, ref, u, u_raw, dist, error
    """
    wn = plant["wn"]
    zeta = plant["zeta"]
    K_plant = plant["K_plant"]

    Kp = controller["Kp"]
    Ki = controller["Ki"]
    Kd = controller["Kd"]
    u_min = controller["u_min"]
    u_max = controller["u_max"]
    anti_windup = controller["anti_windup"]

    dt = sim["dt"]
    t_end = sim["t_end"]

    t = np.arange(0, t_end, dt)
    n = len(t)

    # State: y (speed), dy (acceleration proxy)
    y = np.zeros(n)
    dy = np.zeros(n)
    u = np.zeros(n)
    u_raw = np.zeros(n)
    y[0] = sim["y0"]

    integral_e = 0.0
    prev_e = 0.0

    for i in range(1, n):
        # PID on error
        e = ref[i - 1] - y[i - 1]
        integral_e += e * dt
        derivative_e = (e - prev_e) / dt
        prev_e = e

        # Control law
        u_cmd = Kp * e + Ki * integral_e + Kd * derivative_e
        u_raw[i] = u_cmd
        u_clipped = np.clip(u_cmd, u_min, u_max)
        u[i] = u_clipped

        # Anti-windup: undo integration when saturated
        if anti_windup and u_clipped != u_cmd:
            integral_e -= e * dt

        # Plant dynamics:  y'' + 2*zeta*wn*y' + wn^2*y = wn^2*K_plant*u - dist/J
        J = plant["J"]
        ddy = (wn ** 2 * K_plant * u[i]
               - 2 * zeta * wn * dy[i - 1]
               - wn ** 2 * y[i - 1]
               - dist[i - 1] / J)
        dy[i] = dy[i - 1] + ddy * dt
        y[i] = max(y[i - 1] + dy[i] * dt, 0.0)  # speed >= 0

    return dict(
        t=t, y=y, ref=ref[:n], u=u, u_raw=u_raw,
        dist=dist[:n], error=ref[:n] - y,
    )


def simulate_open_loop(plant, sim, u_const, dist):
    """Run open-loop simulation with constant control input."""
    wn = plant["wn"]
    zeta = plant["zeta"]
    K_plant = plant["K_plant"]
    J = plant["J"]
    dt = sim["dt"]
    t_end = sim["t_end"]

    t = np.arange(0, t_end, dt)
    n = len(t)
    y = np.zeros(n)
    dy = np.zeros(n)
    y[0] = sim["y0"]

    for i in range(1, n):
        ddy = (wn ** 2 * K_plant * u_const
               - 2 * zeta * wn * dy[i - 1]
               - wn ** 2 * y[i - 1]
               - dist[i - 1] / J)
        dy[i] = dy[i - 1] + ddy * dt
        y[i] = max(y[i - 1] + dy[i] * dt, 0.0)

    return dict(t=t, y=y)


# ---------------------------------------------------------------------------
# Plotly figure builders
# ---------------------------------------------------------------------------

def build_figure(result, title="Conveyor Belt Speed Control"):
    """Build a 3-subplot Plotly figure from simulation result dict."""
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    t = result["t"]
    fig = make_subplots(
        rows=3, cols=1, shared_xaxes=True,
        subplot_titles=(
            "Belt Speed vs Target",
            "Control Effort (Voltage)",
            "Disturbance Load",
        ),
        vertical_spacing=0.08,
    )

    # Row 1: speed tracking
    fig.add_trace(go.Scatter(
        x=t, y=result["ref"], mode="lines",
        name="Target speed", line=dict(color="black", dash="dash", width=1.5),
    ), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=t, y=result["y"], mode="lines",
        name="Belt speed", line=dict(color="#636EFA", width=2),
    ), row=1, col=1)

    # Row 2: control effort
    fig.add_trace(go.Scatter(
        x=t, y=result["u"], mode="lines",
        name="Applied voltage", line=dict(color="#EF553B", width=1.5),
    ), row=2, col=1)
    if "u_raw" in result:
        fig.add_trace(go.Scatter(
            x=t, y=result["u_raw"], mode="lines",
            name="Commanded (pre-clip)", line=dict(color="#EF553B", dash="dot", width=1),
        ), row=2, col=1)

    # Row 3: disturbance
    fig.add_trace(go.Scatter(
        x=t, y=result["dist"], mode="lines",
        name="Load disturbance", line=dict(color="#00CC96", width=1.5),
        fill="tozeroy", fillcolor="rgba(0,204,150,0.15)",
    ), row=3, col=1)

    fig.update_yaxes(title_text="Speed [m/s]", row=1, col=1)
    fig.update_yaxes(title_text="Voltage [V]", row=2, col=1)
    fig.update_yaxes(title_text="Load [N]", row=3, col=1)
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


def build_comparison_figure(t, traces, title="Controller Comparison"):
    """Build a 2-subplot figure comparing multiple controller results.

    Args:
        t:      time array
        traces: list of dicts with keys 'label', 'y', 'u', 'color'
        title:  figure title
    """
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    fig = make_subplots(
        rows=2, cols=1, shared_xaxes=True,
        subplot_titles=("Belt Speed", "Control Effort"),
        vertical_spacing=0.10,
    )

    for tr in traces:
        fig.add_trace(go.Scatter(
            x=t, y=tr["y"], mode="lines",
            name=tr["label"], line=dict(color=tr["color"], width=2),
        ), row=1, col=1)
        fig.add_trace(go.Scatter(
            x=t, y=tr["u"], mode="lines",
            name=tr["label"], line=dict(color=tr["color"], width=1.5),
            showlegend=False,
        ), row=2, col=1)

    fig.update_yaxes(title_text="Speed [m/s]", row=1, col=1)
    fig.update_yaxes(title_text="Voltage [V]", row=2, col=1)
    fig.update_xaxes(title_text="Time [s]", row=2, col=1)

    fig.update_layout(
        template="plotly_white", height=550, title_text=title,
    )
    return fig
