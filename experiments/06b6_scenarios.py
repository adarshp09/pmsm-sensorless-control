#!/usr/bin/env python3
"""Phase 06b-6 Step 2 (stress, beyond paper) + Step 3 (scenario suite + sensitivity).

Suite: the four printed scenarios (S1-S4), chain B, matched cell, advance off/on; FLC
nominal on the two headline load-step scenarios. Encoder baseline at the same alpha_w
gives the sensorless cost.

Sensitivity at the 50 % step (S2): alpha_w x {0.5,1,2} x advance {off,on}; f_n x {0.5,2}
at alpha_w* (advance off).

Stress (informational, not gates): 500 rpm 20 %/50 % steps, rated step @1000,
start-up-under-load; plus the 06b-5 floor results for reference.

Output: experiments/output/06b6_scenarios/scenarios.json
"""

from __future__ import annotations

import dataclasses
import json
import sys
from pathlib import Path

try:
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DIR))

import sensorless_eval as E  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque  # noqa: E402

ANGLE_LIMIT_DEG = 30.0


def _flat(entry, spec):
    """Flatten one sensorless run into a report row."""
    row = {
        "handed_over": entry["handed_over"],
        "switch_time_s": entry["switch_time_s"],
        "fll_status": entry["fll_status"],
        "peak_theta_err_deg": entry["peak_theta_err_deg"],
        "steady_theta_max_deg": entry["steady_theta_err"]["max"],
        "steady_theta_mean_deg": entry["steady_theta_err"]["mean"],
        "steady_theta_rms_deg": entry["steady_theta_err"]["rms"],
        "steady_omega_max_rpm": entry["steady_omega_err"]["max"],
        "dynamic_theta_max_deg": entry["dynamic_theta_err"]["max"],
        "dynamic_theta_rms_deg": entry["dynamic_theta_err"]["rms"],
        "dynamic_omega_max_rpm": entry["dynamic_omega_err"]["max"],
        "peak_i_q_a": entry["peak_i_q_a"],
        "saturation_after_transient": entry["saturation_after_transient"],
    }
    for k in ("dip_rpm", "overshoot_rpm", "recovery_10rpm_s", "recovery_1pct_s"):
        if k in entry:
            row[k] = entry[k]
    row["passed"] = bool(
        entry["handed_over"]
        and entry["fll_status"] == "ok"
        and (
            entry["peak_theta_err_deg"] is not None
            and entry["peak_theta_err_deg"] < ANGLE_LIMIT_DEG
        )
        and entry["saturation_after_transient"] == 0.0
    )
    return row


def _sensorless(
    stack, params, plant, spec, advance, *, cell="matched_sigmoid", f_n=None, alpha_w=None
):
    aw = spec["alpha_w"] if alpha_w is None else alpha_w
    src, cs, res = E.run_sensorless(
        stack,
        params,
        plant,
        speed_ref=spec["speed_ref"],
        load_torque=spec["load"],
        duration=spec["duration"],
        alpha_w=aw,
        advance=advance,
        clamp=True,
        cell=cell,
        f_n=f_n,
    )
    m = E.estimate_metrics(src, cs, res, dynamic=spec["dynamic"], steady=spec["steady"])
    if spec["disturbance_s"] is not None:
        m.update(E.speed_metrics(stack, res, {"disturbance_s": spec["disturbance_s"]}))
    return m


def _baseline(stack, params, plant, spec, advance=False):
    res = E.run_encoder(
        stack,
        params,
        plant,
        speed_ref=spec["speed_ref"],
        load_torque=spec["load"],
        duration=spec["duration"],
        alpha_w=spec["alpha_w"],
        advance=advance,
    )
    if spec["disturbance_s"] is None:
        return {}
    return E.speed_metrics(stack, res, {"disturbance_s": spec["disturbance_s"]})


def main() -> int:
    stack, params, plant, rule = E.load()
    scenarios = E.paper_scenarios(rule)
    suite = {}
    for name, spec in scenarios.items():
        cells = {}
        for advance in (False, True):
            m = _sensorless(stack, params, plant, spec, advance)
            cells[f"matched_adv{'on' if advance else 'off'}"] = _flat(m, spec)
        # FLC nominal on the headline load-step scenarios
        if name in ("S2_sim_50pct_1000", "S3_exp_20pct_1000"):
            m = _sensorless(stack, params, plant, spec, False, cell="flc_nominal")
            cells["flc_nominal_advoff"] = _flat(m, spec)
        base = _baseline(stack, params, plant, spec)
        if base:
            best = cells["matched_advoff"]
            cells["encoder_baseline"] = base
            cells["sensorless_cost"] = {
                "dip_delta_rpm": best["dip_rpm"] - base["dip_rpm"],
                "recovery_10rpm_delta_s": best["recovery_10rpm_s"] - base["recovery_10rpm_s"],
            }
        cells["paper"] = spec["paper"]
        suite[name] = cells

    # Sensitivity at the 50 % step
    s2 = scenarios["S2_sim_50pct_1000"]
    sensitivity = {}
    for mult in (0.5, 1.0, 2.0):
        for advance in (False, True):
            m = _sensorless(stack, params, plant, s2, advance, alpha_w=s2["alpha_w"] * mult)
            sensitivity[f"aw{mult:g}x_adv{'on' if advance else 'off'}"] = _flat(m, s2)
    for mult in (0.5, 2.0):
        m = _sensorless(stack, params, plant, s2, False, f_n=20.0 * mult)
        sensitivity[f"fn{mult:g}x_advoff"] = _flat(m, s2)

    # Step 2 stress table (informational, beyond paper)
    def _stress(label, speed, load, duration=2.0, step_s=0.5, alpha_w=None):
        spec = dict(
            speed_ref=float(speed),
            load=load,
            duration=duration,
            alpha_w=rule.get(speed, rule[1000]) if alpha_w is None else alpha_w,
            dynamic=(step_s, min(step_s + 0.5, duration)),
            steady=(duration - 0.2, duration),
            disturbance_s=step_s,
        )
        m = _sensorless(stack, params, plant, spec, False)
        row = _flat(m, spec)
        if not row["handed_over"] or row["fll_status"] != "ok":
            row["failure_mode"] = "guard-trip/pre-handover"
        elif row["peak_theta_err_deg"] and row["peak_theta_err_deg"] >= ANGLE_LIMIT_DEG:
            row["failure_mode"] = "synchronism-loss"
        elif row["saturation_after_transient"] > 0:
            row["failure_mode"] = "voltage-saturation"
        else:
            row["failure_mode"] = "none"
        return row

    stress = {
        "500rpm_20pct_step": _stress("500/20", 500, StepLoadTorque(0.0, 0.2 * E.RATED_TORQUE, 0.5)),
        "500rpm_50pct_step": _stress("500/50", 500, StepLoadTorque(0.0, 0.5 * E.RATED_TORQUE, 0.5)),
        "1000rpm_rated_step": _stress("1000/100", 1000, StepLoadTorque(0.0, E.RATED_TORQUE, 0.5)),
        "500rpm_startup_under_load": _stress(
            "500/load-start",
            500,
            ConstantLoadTorque(0.2 * E.RATED_TORQUE),
            duration=1.6,
            step_s=0.0,
        ),
    }
    stress["floor_reference_06b5"] = {
        "note": "500 rpm 20 % step, floor sweep from D-062",
        "none": "guard trip, peak 35.9 deg",
        "100rpm": "diverges, peak 180 deg, sat 0.89",
        "200rpm": "diverges, 180 deg",
        "300rpm": "persistent 36 deg (pinned)",
        "400rpm": "diverges, 180 deg",
    }

    out = {
        "note": (
            "Phase 06b-6 suite: paper scenarios at the decoupled bandwidths, chain B, "
            "matched cell, clamp ON; advance off/on; FLC nominal on the load steps. "
            "Sensorless cost vs the encoder baseline at the same alpha_w. Stress table "
            "is informational (beyond paper)."
        ),
        "rule": {str(k): v for k, v in rule.items()},
        "suite": suite,
        "sensitivity": sensitivity,
        "stress": stress,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b6_scenarios"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "scenarios.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== suite ==")
    for name, cells in suite.items():
        for cell, row in cells.items():
            if isinstance(row, str) or "peak_theta_err_deg" not in row:
                continue
            print(
                f"  {name:20s} {cell:22s} fll={row['fll_status']:9s} "
                f"steady|dth|max={row['steady_theta_max_deg']:5.2f}° steady|dw|max={row['steady_omega_max_rpm']:6.2f} "
                f"dyn|dth|max={row['dynamic_theta_max_deg']:6.2f}° dyn|dw|max={row['dynamic_omega_max_rpm']:7.1f} "
                f"dip={row.get('dip_rpm', float('nan')):8.1f} sat={row['saturation_after_transient']:.3f} "
                f"{'PASS' if row['passed'] else 'FAIL'}"
            )
    print("\n== stress (informational) ==")
    for k, v in stress.items():
        if isinstance(v, dict) and "peak_theta_err_deg" in v:
            print(
                f"  {k:28s} peak={v['peak_theta_err_deg']:7.1f} fll={v['fll_status']:9s} "
                f"sat={v['saturation_after_transient']:.3f} mode={v['failure_mode']} {'PASS' if v['passed'] else 'FAIL'}"
            )
    print("\n== sensitivity (S2 50 %) ==")
    for k, v in sensitivity.items():
        print(
            f"  {k:22s} peak={v['peak_theta_err_deg']:7.1f} dip={v.get('dip_rpm', float('nan')):8.1f} "
            f"fll={v['fll_status']:9s} {'PASS' if v['passed'] else 'FAIL'}"
        )
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
