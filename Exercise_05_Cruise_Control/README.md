# Exercise 5 — Cruise Control (Capstone)

Interactive Jupyter notebook for exploring PID cruise control on a simplified vehicle model.

## Setup

```bash
pip install numpy plotly ipywidgets
jupyter notebook cruise_control_student_demo.ipynb
```

## What's Inside

| File | Purpose |
|------|---------|
| `cruise_control_student_demo.ipynb` | Main student notebook with interactive widgets |
| `cruise_control_helpers.py` | Simulation engine, scenario builder, and plotting functions |

## How to Use

1. Open the notebook and run all cells top to bottom.
2. **Section 1** runs a baseline simulation so you can see the system before interacting.
3. **Section 2** gives you interactive sliders for PID gains, vehicle mass, drag, hill grade, and target speed.  Move sliders and watch the plots update in real time.
4. **Section 3** is a tuning challenge — try to meet the performance targets.

## Manufacturing Bridge

Cruise control is directly analogous to industrial speed regulation problems:
- Conveyor belt speed control under varying product load
- Spindle speed regulation during material engagement
- Pump/fan speed control with changing system resistance
- Production line-rate regulation under varying demand
