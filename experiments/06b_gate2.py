#!/usr/bin/env python3
"""Phase 06b GATE 2: sensorless handover + rated-load step at 1000 rpm.

Runs the encoder start-up with the chain in the background, hands over when the
chain-internal lock criterion holds, applies a 0 -> rated load step at 1000 rpm, and
checks synchronism (|theta_hat - theta| < 30 deg throughout, no voltage-limit
saturation after the transient) for the matched cell with the angle advance off and
on. At the frozen Phase 06a speed-loop bandwidth (alpha_w = 50 Hz) the gate FAILS:
the *estimated speed* in the speed loop destabilises the loop. Traces are recorded
and the fix (a separate approved step) is characterised by a bandwidth sweep.

Output: experiments/output/06b_gate2/gate2.json
"""

from __future__ import annotations

import dataclasses
import json
import math
import sys
from pathlib import Path

try:
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DIR))

import chain as C  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.plant.load_models import StepLoadTorque  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters, run_drive  # noqa: E402

TWO_PI = 2.0 * math.pi
LOAD_STEP_S = 0.6
DURATION = 1.2
RATED_TORQUE = 7.7
ANGLE_LIMIT_DEG = 30.0


class RecordingHandover(C.HandoverSource):
    """HandoverSource that records the post-switch synchronism traces."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.t: list[float] = []
        self.theta_err_deg: list[float] = []
        self.omega_err_rpm: list[float] = []
        self.i_q: list[float] = []

    def __call__(self, sample, truth):
        out = super().__call__(sample, truth)
        if self.mode != "encoder":
            self.t.append(sample.t)
            self.theta_err_deg.append(math.degrees((out[0] - truth.theta_e + math.pi) % TWO_PI - math.pi))
            self.omega_err_rpm.append((out[1] - truth.omega_e) * 60.0 / (TWO_PI * self.pole_pairs))
        return out


def _run(stack, params, plant, *, alpha_w, advance):
    prm = dataclasses.replace(params, alpha_w_hz=alpha_w) if alpha_w != params.alpha_w_hz else params
    assump = C.handover_assumptions(stack)
    chain_source = C.EstimatorSource(
        observer=C.build_observer("matched_sigmoid", stack),
        fll=C.build_fll(stack.initial_omega_at(assump.omega_h_rpm)),
        qpll=C.build_qpll(stack), pole_pairs=stack.pole_pairs)
    source = RecordingHandover(chain_source, pole_pairs=stack.pole_pairs, assumptions=assump)
    res = run_drive(
        plant, prm, duration=DURATION, speed_ref_rpm=1000.0,
        load_torque=StepLoadTorque(0.0, RATED_TORQUE, LOAD_STEP_S), source=source,
        angle_advance=advance)
    theta_err = np.asarray(source.theta_err_deg) if source.theta_err_deg else np.zeros(0)
    after = res.t >= LOAD_STEP_S
    entry = {
        "advance": advance,
        "handed_over": source.handed_over,
        "switch_time_s": source.switch_time_s,
        "fll_status": chain_source.status,
        "guard_fail_time_s": chain_source.first_guard_fail_s,
        "peak_theta_err_deg": float(np.max(np.abs(theta_err))) if theta_err.size else 0.0,
        "peak_omega_err_rpm": float(np.max(np.abs(source.omega_err_rpm))) if source.omega_err_rpm else 0.0,
        "saturation_fraction_after_step": float(np.mean(res.saturated[after])),
        "peak_voltage_v": float(np.max(res.u_cmd_magnitude)),
        "v_limit_v": res.v_limit,
    }
    entry["passed"] = bool(
        entry["handed_over"] and entry["fll_status"] == "ok"
        and entry["peak_theta_err_deg"] < ANGLE_LIMIT_DEG
        and entry["saturation_fraction_after_step"] == 0.0)
    entry["verdict"] = "PASS" if entry["passed"] else "FAIL"
    traces = {k: [round(v, 6) for v in vals] for k, vals in (
        ("theta_err_deg", source.theta_err_deg), ("omega_err_rpm", source.omega_err_rpm))}
    return entry, traces


def _sweep(stack, params, plant):
    rows = {}
    for aw in (5.0, 10.0, 15.0, 18.0, 20.0, 25.0, 50.0):
        entry, _ = _run(stack, params, plant, alpha_w=aw, advance=False)
        rows[f"alpha_w={aw:g}Hz"] = {"verdict": entry["verdict"],
                                    "peak_theta_err_deg": entry["peak_theta_err_deg"],
                                    "fll_status": entry["fll_status"]}
    return rows


def main() -> int:
    directory = config.default_config_dir()
    stack = C.build_stack(directory)
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=directory),
        config.load_config("drive_assumptions", directory=directory),
        stack.pmsm_config)
    plant = PMSMPlant(PMSMParameters.from_config(stack.pmsm_config), integrate_mechanics=True,
                      viscous_friction=0.0)

    results, traces = {}, {}
    for advance in (False, True):
        entry, tr = _run(stack, params, plant, alpha_w=params.alpha_w_hz, advance=advance)
        results[f"advance_{'on' if advance else 'off'}"] = entry
        traces[f"advance_{'on' if advance else 'off'}"] = tr
    sweep = _sweep(stack, params, plant)

    gate2 = all(e["passed"] for e in results.values())
    out = {
        "note": ("Phase 06b GATE 2 (sensorless synchronism). Matched cell, handover then a "
                 "0 -> rated load step at 1000 rpm; advance off/on. Encoder start-up is the "
                 "labelled truth path; the lock criterion reads no ground truth."),
        "assumptions": {"omega_h_rpm": C.handover_assumptions(stack).omega_h_rpm,
                        "speed_tolerance_rpm": C.handover_assumptions(stack).speed_tolerance_rpm,
                        "lock_time_s": C.handover_assumptions(stack).lock_time_s,
                        "alpha_w_hz": params.alpha_w_hz, "alpha_c_hz": params.alpha_c_hz,
                        "f_n_hz": 20.0, "load_step_s": LOAD_STEP_S,
                        "angle_limit_deg": ANGLE_LIMIT_DEG},
        "gate2": "PASS" if gate2 else "FAIL",
        "results": results,
        "bandwidth_sweep": sweep,
        "traces": traces,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b_gate2"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "gate2.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print(f"== GATE 2: {out['gate2']} ==")
    for key, e in results.items():
        print(f"  {key:11s} handover={e['handed_over']} switch={e['switch_time_s']} "
              f"fll={e['fll_status']} peak|dth|={e['peak_theta_err_deg']:.1f} deg "
              f"peak|dw|={e['peak_omega_err_rpm']:.0f} rpm sat={e['saturation_fraction_after_step']:.3f}")
    print("  bandwidth sweep (peak |theta_hat - theta|, matched cell):")
    for key, row in sweep.items():
        print(f"    {key:14s} {row['verdict']:4s} peak={row['peak_theta_err_deg']:7.2f} deg fll={row['fll_status']}")
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
