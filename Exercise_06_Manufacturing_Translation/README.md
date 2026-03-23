# Exercise 6 — Manufacturing Translation

Take control concepts from Exercises 1–5 and apply them to three real manufacturing systems.

## What's Inside

| File | Purpose |
|------|---------|
| `manufacturing_translation.ipynb` | Main notebook — three manufacturing examples |
| `mfg_helpers.py` | Simulation engine for conveyor, oven, and tank systems |

## Manufacturing Examples

| System | Controlled Variable | Key Disturbance |
|--------|-------------------|-----------------|
| Conveyor belt | Belt speed [m/s] | Box/product loading |
| Curing oven | Temperature [°C] | Door-open heat loss |
| Mixing tank | Liquid level [m] | Variable downstream demand |

## Learning Objectives

- Map control loop elements (MV, CV, disturbance, sensor) to physical systems
- Simulate each system under realistic disturbance scenarios
- Describe what poor tuning looks like in production terms
