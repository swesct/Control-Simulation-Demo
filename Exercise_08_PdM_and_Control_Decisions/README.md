# Exercise 8 — Predictive Maintenance & Control Decisions

Connect control system behavior to manufacturing operational decisions.

## What's Inside

| File | Purpose |
|------|---------|
| `pdm_and_control_decisions.ipynb` | Main notebook — metrics, KPIs, and recommendations |
| `pdm_helpers.py` | Metrics extraction, KPI mapping, and recommendation engine |

## Metrics Extracted

Overshoot, settling time, steady-state error, oscillation count, time outside tolerance band, actuator saturation exposure

## Manufacturing KPIs

| KPI | Driven By |
|-----|-----------|
| Throughput risk | Slow settling, time off-target |
| Quality risk | Overshoot, oscillation |
| Energy penalty | Saturation, high error |
| Maintenance stress | Oscillation, actuator cycling |

## Learning Objectives

- Extract standard control performance metrics from a simulation
- Map metrics to manufacturing-relevant KPIs
- Make and justify operational recommendations using both control and manufacturing language
