# Control & Simulation Demo

A hands-on lecture series using Python for control systems and simulation in manufacturing applications. All exercises use open-source libraries.

## Exercise Roadmap

Work through the exercises in order. Start with Exercise 0 to verify your setup.

| Exercise | Folder | Topic |
|----------|--------|-------|
| 0 | `Exercise_00_Environment_Setup` | Environment verification — confirm all libraries are installed |
| 1 | `Exercise_01_System_Dynamics` | First & second order dynamics, damping sweeps, step response metrics |
| 2 | `Exercise_02_Feedback_Control` | Proportional, PI, PID control & disturbance rejection |
| 3 | `Exercise_03_Digital_Control` | Discrete-time PID, actuator saturation, anti-windup, sensor noise |
| 4 | `Exercise_04_Interactive_Controls` | Interactive conveyor belt speed control exploration |
| 5 | `Exercise_05_Cruise_Control` | Capstone — PID cruise control application |
| 6 | `Exercise_06_Manufacturing_Translation` | Conveyor, oven, and tank — controls meet manufacturing |
| 7 | `Exercise_07_Faults_vs_Disturbances` | Diagnosing abnormal responses: faults, disturbances, sensor issues |
| 8 | `Exercise_08_PdM_and_Control_Decisions` | Control metrics to maintenance decisions and manufacturing KPIs |

## Quick Start

1. Create and activate a Python 3.10+ environment.
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Launch Jupyter and open `Exercise_00_Environment_Setup/verify_environment.ipynb`.
4. Once all checks pass, proceed to Exercise 01.

## Requirements

See `requirements.txt` for the full list. Core packages:

`numpy` · `scipy` · `matplotlib` · `control` · `plotly` · `ipywidgets` · `sympy` · `anywidget`
