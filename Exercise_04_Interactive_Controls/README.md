# Interactive Controls & Dynamics Demo

Interactive Jupyter notebook for exploring PID control, transfer functions, and state-space models on a conveyor belt speed control system.

## Setup

```bash
pip install -r requirements.txt
jupyter notebook interactive_controls_demo.ipynb
```

## Required Packages

numpy, matplotlib, scipy, control, sympy, ipywidgets, plotly, anywidget

## What's Inside

| File | Purpose |
|------|---------|
| `interactive_controls_demo.ipynb` | Main student notebook with 7 sections |
| `demo_helpers.py` | Simulation engine, scenario builder, Plotly figure generators |
| `requirements.txt` | Package dependencies |

## Learning Objectives

- Build and interpret a plant transfer function and state-space model
- Simulate open-loop step and impulse responses and identify key parameters (poles, wn, zeta)
- Close the feedback loop and compare P, PI, and PID control performance
- Observe how actuator saturation and disturbances affect real control loops
- Interactively tune PID gains and see immediate effects on speed tracking and control effort
- Connect control theory to manufacturing applications (conveyor speed, spindle regulation, pump control)

## Notebook Sections

1. **Setup and Imports** - verify all libraries
2. **Plant Definition** - transfer function (python-control), state-space, and SymPy symbolic
3. **Open-Loop Dynamics** - step/impulse response, poles, natural frequency
4. **Closed-Loop Control** - P vs PI vs PID comparison with pole plots
5. **Disturbance and Realism** - load disturbance, actuator saturation, anti-windup
6. **Interactive Exploration** - slider-based tuning with live Plotly updates
7. **Student Challenges** - tuning targets with automated pass/fail scoring
