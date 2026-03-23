"""
Predictive Maintenance & Control Decision Helpers
===================================================
Metrics extraction, KPI mapping, and recommendation logic for connecting
control system behavior to manufacturing operational decisions.

Designed for teaching — intentionally simple thresholds and scoring.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Control performance metrics
# ---------------------------------------------------------------------------

def compute_metrics(t, y, ref, u, u_max, tolerance_band=0.02):
    """Extract standard control performance metrics from a simulation.

    Args:
        t:     time array [s]
        y:     output array
        ref:   reference/target array
        u:     control effort array
        u_max: actuator upper limit (for saturation %)
        tolerance_band: fraction of final target for settling (default 2%)

    Returns:
        dict with keys:
            overshoot_pct, settling_time, sse, oscillation_count,
            time_outside_band_pct, saturation_pct, rms_error
    """
    error = ref - y

    # Steady-state: use last 20% of the signal
    n_tail = max(int(0.2 * len(y)), 1)
    sse = float(np.mean(np.abs(error[-n_tail:])))

    # Overshoot (relative to final target)
    ref_final = ref[-1]
    if ref_final != 0:
        peak = np.max(y)
        overshoot_pct = max(0.0, (peak - ref_final) / abs(ref_final) * 100)
    else:
        overshoot_pct = 0.0

    # Settling time (last time output leaves tolerance band)
    band = tolerance_band * abs(ref_final) if ref_final != 0 else tolerance_band
    outside = np.abs(error) > band
    if np.any(outside):
        settling_time = float(t[np.max(np.where(outside))])
    else:
        settling_time = 0.0

    # Oscillation count (zero-crossings of error after first settling)
    crossings = np.diff(np.sign(error))
    oscillation_count = int(np.sum(crossings != 0) // 2)

    # Time outside acceptable band
    time_outside_band_pct = float(np.sum(outside) / len(outside) * 100)

    # Actuator saturation exposure
    at_limit = (u >= u_max * 0.99) | (u <= 0.01)
    saturation_pct = float(np.sum(at_limit) / len(u) * 100)

    # RMS error
    rms_error = float(np.sqrt(np.mean(error ** 2)))

    return dict(
        overshoot_pct=round(overshoot_pct, 1),
        settling_time=round(settling_time, 2),
        sse=round(sse, 4),
        oscillation_count=oscillation_count,
        time_outside_band_pct=round(time_outside_band_pct, 1),
        saturation_pct=round(saturation_pct, 1),
        rms_error=round(rms_error, 4),
    )


# ---------------------------------------------------------------------------
# Manufacturing KPI mapping
# ---------------------------------------------------------------------------

def compute_kpis(metrics):
    """Map control metrics to manufacturing-style KPI risk scores.

    Each KPI is scored 0 (no concern) to 3 (high concern).

    Returns:
        dict with keys: throughput_risk, quality_risk, energy_penalty, maintenance_stress
    """
    # Throughput risk: driven by settling time and time outside band
    throughput_risk = 0
    if metrics["settling_time"] > 30:
        throughput_risk += 1
    if metrics["settling_time"] > 60:
        throughput_risk += 1
    if metrics["time_outside_band_pct"] > 15:
        throughput_risk += 1

    # Quality risk: driven by overshoot, oscillation, and steady-state error
    quality_risk = 0
    if metrics["overshoot_pct"] > 10:
        quality_risk += 1
    if metrics["overshoot_pct"] > 25:
        quality_risk += 1
    if metrics["oscillation_count"] > 5:
        quality_risk += 1

    # Energy penalty: driven by saturation and high control effort
    energy_penalty = 0
    if metrics["saturation_pct"] > 10:
        energy_penalty += 1
    if metrics["saturation_pct"] > 30:
        energy_penalty += 1
    if metrics["rms_error"] > 0.5:
        energy_penalty += 1

    # Maintenance stress: driven by oscillation and saturation
    maintenance_stress = 0
    if metrics["oscillation_count"] > 3:
        maintenance_stress += 1
    if metrics["oscillation_count"] > 8:
        maintenance_stress += 1
    if metrics["saturation_pct"] > 20:
        maintenance_stress += 1

    return dict(
        throughput_risk=min(throughput_risk, 3),
        quality_risk=min(quality_risk, 3),
        energy_penalty=min(energy_penalty, 3),
        maintenance_stress=min(maintenance_stress, 3),
    )


# ---------------------------------------------------------------------------
# Operational recommendation
# ---------------------------------------------------------------------------

_RISK_LABELS = {0: "Low", 1: "Moderate", 2: "Elevated", 3: "High"}


def recommend_action(metrics, kpis):
    """Suggest an operational action based on metrics and KPIs.

    Returns:
        dict with keys: action, reasoning (list of strings)
    """
    reasons = []
    score = 0  # higher = more urgent

    if kpis["maintenance_stress"] >= 2:
        reasons.append("High actuator cycling suggests mechanical wear")
        score += 2
    if kpis["quality_risk"] >= 2:
        reasons.append("Overshoot/oscillation risks product quality")
        score += 2
    if kpis["throughput_risk"] >= 2:
        reasons.append("Slow settling reduces effective throughput")
        score += 1
    if kpis["energy_penalty"] >= 2:
        reasons.append("Sustained saturation wastes energy and stresses actuator")
        score += 1
    if metrics["sse"] > 0.5:
        reasons.append("Persistent steady-state error — possible sensor drift or gain change")
        score += 1
    if metrics["oscillation_count"] > 8:
        reasons.append("Excessive oscillation — controller may need retuning")
        score += 1

    if score == 0:
        action = "Continue running"
    elif score <= 2:
        action = "Inspect soon / increase monitoring"
    elif score <= 4:
        action = "Retune controller or reduce load"
    else:
        action = "Schedule maintenance shutdown"

    if not reasons:
        reasons.append("All metrics within acceptable bounds")

    return dict(action=action, reasoning=reasons)


def print_dashboard(metrics, kpis, recommendation):
    """Print a formatted text dashboard of metrics, KPIs, and recommendation."""
    print("=" * 60)
    print("  CONTROL PERFORMANCE METRICS")
    print("=" * 60)
    for k, v in metrics.items():
        label = k.replace("_", " ").title()
        print(f"  {label:.<35} {v}")

    print()
    print("=" * 60)
    print("  MANUFACTURING KPIs")
    print("=" * 60)
    for k, v in kpis.items():
        label = k.replace("_", " ").title()
        risk = _RISK_LABELS[v]
        bar = "|" + "#" * v + "-" * (3 - v) + "|"
        print(f"  {label:.<35} {bar} {risk}")

    print()
    print("=" * 60)
    print(f"  RECOMMENDATION: {recommendation['action'].upper()}")
    print("=" * 60)
    for r in recommendation["reasoning"]:
        print(f"  - {r}")
    print()
