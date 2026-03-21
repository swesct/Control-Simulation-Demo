# Controls & Dynamics Lecture Notebooks

Jupyter notebooks for a 2.5-hour manufacturing-focused controls lecture.

## Setup

```bash
pip install numpy matplotlib jupyter
```

## Run Order

| # | Notebook | Topic |
|---|----------|-------|
| 1 | `01_first_order_dynamics.ipynb` | Thermal process time constant |
| 2 | `02_second_order_axis_model.ipynb` | Mass-spring-damper axis, wn, zeta |
| 3 | `03_damping_sweep.ipynb` | Under/critical/overdamped comparison |
| 4 | `04_step_response_metrics.ipynb` | Rise time, settling time, overshoot, SSE |
| 5 | `05_proportional_control.ipynb` | Feedback control, Kp sweep |
| 6 | `06_pi_pid_control.ipynb` | P vs PI vs PID |
| 7 | `07_disturbance_rejection.ipynb` | Load disturbance recovery |
| 8 | `08_discrete_time_pid.ipynb` | Sampled loop, sample-time effects |

### Stretch / Optional

| # | Notebook | Topic |
|---|----------|-------|
| 9 | `09_actuator_saturation.ipynb` | Torque limits and clipping |
| 10 | `10_anti_windup.ipynb` | Integrator windup fix |
| 11 | `11_sensor_noise.ipynb` | Noise effect on derivative action |

## Usage

Each notebook is self-contained. Open any notebook and run all cells top to bottom. Editable parameters are in the first code cell.

```bash
jupyter notebook
```
