#!/usr/bin/env python3
"""Phase 06b-3 Step 2: staged sensorless gate at the decoupled bandwidths.

Rule: alpha_w = 5 Hz at 500 rpm, 10 Hz at 1000 rpm (configs/drive_assumptions.yaml,
D-059). Chain B (DSOGI v' -> Q-PLL, f_n = 20 Hz), matched cell, handover as in 06b,
advance off and on.

2a: handover then 1.0 s steady at 500/1000 rpm, no-load and 20 % load.
    Pass: |theta_hat - theta| < 30 deg throughout and < 5 deg after handover+0.2 s; no
    voltage saturation.
2b: 20 % load step at 1000 and 500 rpm.
2c: 50 % (rated) load step at 1000 rpm.
    Pass: |theta_hat - theta| < 30 deg throughout, no steady saturation after the
    transient; report true-speed dip and recovery to +/-10 rpm.

Output: experiments/output/06b3_gate/gate.json
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
from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters, run_drive  # noqa: E402

TWO_PI = 2.0 * math.pi
RATED_TORQUE = 7.7
ANGLE_LIMIT_DEG = 30.0
STEADY_ANGLE_DEG = 5.0
RECOVERY_RPM = 10.0


class RecordingHandover(C.HandoverSource):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.theta_err_deg: list[float] = []
        self.t: list[float] = []

    def __call__(self, sample, truth):
        out = super().__call__(sample, truth)
        if self.mode != "encoder":
            self.t.append(sample.t)
            self.theta_err_deg.append(
                math.degrees((out[0] - truth.theta_e + math.pi) % TWO_PI - math.pi)
            )
        return out


def _rule(config_dir) -> dict:
    cfg = config.load_config("drive_assumptions", directory=config_dir)
    return {
        500: config.resolve_parameter(cfg, "speed_loop.alpha_w_hz_at_500rpm", unit="Hz"),
        1000: config.resolve_parameter(cfg, "speed_loop.alpha_w_hz_at_1000rpm", unit="Hz"),
    }


def _run(stack, params, plant, *, speed_rpm, load, speed_ref, duration, advance, alpha_w):
    prm = dataclasses.replace(params, alpha_w_hz=alpha_w)
    assump = C.handover_assumptions(stack)
    chain_source = C.EstimatorSource(
        observer=C.build_observer("matched_sigmoid", stack),
        fll=C.build_fll(stack.initial_omega_at(assump.omega_h_rpm)),
        qpll=C.build_qpll(stack),
        pole_pairs=stack.pole_pairs,
    )
    source = RecordingHandover(chain_source, pole_pairs=stack.pole_pairs, assumptions=assump)
    res = run_drive(
        plant,
        prm,
        duration=duration,
        speed_ref_rpm=speed_ref,
        load_torque=load,
        source=source,
        angle_advance=advance,
    )
    return source, chain_source, res


def _gate_entry(
    source,
    chain_source,
    res,
    *,
    step_s=None,
    alpha_w=None,
    advance=None,
    speed_rpm=None,
    require_steady_deg=STEADY_ANGLE_DEG,
):
    th = np.abs(np.asarray(source.theta_err_deg)) if source.theta_err_deg else np.zeros(0)
    t_est = np.asarray(source.t) if source.t else np.zeros(0)
    switch = source.switch_time_s
    post = th[t_est >= (switch + 0.2)] if switch is not None else np.zeros(0)
    steady_max = float(np.max(post)) if post.size else None
    steady_ok = require_steady_deg is None or (
        steady_max is not None and steady_max < require_steady_deg
    )
    peak = float(np.max(th)) if th.size else 0.0
    sat = (
        float(np.mean(res.saturated[res.t >= (step_s + 0.1)]))
        if step_s is not None
        else float(np.mean(res.saturated[res.t >= 0.2]))
    )
    entry = {
        "speed_rpm": speed_rpm,
        "alpha_w_hz": alpha_w,
        "advance": advance,
        "handed_over": source.handed_over,
        "switch_time_s": switch,
        "fll_status": chain_source.status,
        "peak_theta_err_deg": peak,
        "steady_theta_err_max_deg": steady_max,
        "saturation_frac_after_transient": sat,
        "peak_voltage_v": float(np.max(res.u_cmd_magnitude)),
        "v_limit_v": res.v_limit,
    }
    if step_s is not None:
        err_rpm = (res.omega_m - res.omega_m_ref) * 60.0 / TWO_PI
        after = res.t >= step_s
        entry["speed_dip_rpm"] = float(np.min(err_rpm[after]))
        bad = np.where(np.abs(err_rpm[after]) > RECOVERY_RPM)[0]
        entry["recovery_to_10rpm_s"] = (
            float(res.t[np.where(after)[0][bad[-1] + 1]] - step_s)
            if len(bad) and bad[-1] + 1 < int(after.sum())
            else None
        )
    entry["passed"] = bool(
        entry["handed_over"]
        and entry["fll_status"] == "ok"
        and entry["peak_theta_err_deg"] < ANGLE_LIMIT_DEG
        and steady_ok
        and entry["saturation_frac_after_transient"] <= 0.0
    )
    return entry


def main() -> int:
    directory = config.default_config_dir()
    stack = C.build_stack(directory)
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=directory),
        config.load_config("drive_assumptions", directory=directory),
        stack.pmsm_config,
    )
    plant = PMSMPlant(
        PMSMParameters.from_config(stack.pmsm_config),
        integrate_mechanics=True,
        viscous_friction=0.0,
    )
    rule = _rule(directory)

    results: dict = {}
    # 2a: steady at 500/1000 rpm, no-load and 20 % load
    for speed in (500, 1000):
        for load_frac in (0.0, 0.2):
            for advance in (False, True):
                src, chain_source, res = _run(
                    stack,
                    params,
                    plant,
                    speed_rpm=speed,
                    load=ConstantLoadTorque(load_frac * RATED_TORQUE),
                    speed_ref=float(speed),
                    duration=1.6,
                    advance=advance,
                    alpha_w=rule[speed],
                )
                results[
                    f"2a|{speed}rpm|load{int(load_frac * 100)}|adv{'on' if advance else 'off'}"
                ] = _gate_entry(
                    src, chain_source, res, alpha_w=rule[speed], advance=advance, speed_rpm=speed
                )
    # 2b: 20 % load step at 500/1000 rpm; 2c: 50 % (rated) at 1000 rpm
    for tag, speed, frac in (("2b", 1000, 0.2), ("2b", 500, 0.2), ("2c", 1000, 0.5)):
        for advance in (False, True):
            src, chain_source, res = _run(
                stack,
                params,
                plant,
                speed_rpm=speed,
                load=StepLoadTorque(0.0, frac * RATED_TORQUE, 0.5),
                speed_ref=float(speed),
                duration=2.0,
                advance=advance,
                alpha_w=rule[speed],
            )
            results[f"{tag}|{speed}rpm|load{int(frac * 100)}|adv{'on' if advance else 'off'}"] = (
                _gate_entry(
                    src,
                    chain_source,
                    res,
                    step_s=0.5,
                    alpha_w=rule[speed],
                    advance=advance,
                    speed_rpm=speed,
                    require_steady_deg=None,
                )
            )

    # GATE: 2a-2c pass with advance OFF
    gate_off = [k for k, v in results.items() if k.endswith("advoff")]
    gate_pass = all(results[k]["passed"] for k in gate_off)
    out = {
        "note": (
            "Phase 06b-3 staged gate at the decoupled bandwidths (500 rpm: 5 Hz, "
            "1000 rpm: 10 Hz); chain B, matched cell, f_n = 20 Hz. Advance off/on."
        ),
        "rule": {str(k): v for k, v in rule.items()},
        "gate": "PASS" if gate_pass else "FAIL",
        "results": results,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b3_gate"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "gate.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print(f"== GATE 2 (06b-3): {out['gate']} ==")
    print(f"   rule: {out['rule']}")
    for key, e in results.items():
        rec = f" rec10={e['recovery_to_10rpm_s']}" if "recovery_to_10rpm_s" in e else ""
        dip = f" dip={e['speed_dip_rpm']:.1f}" if "speed_dip_rpm" in e else ""
        print(
            f"  {key:28s} hand={e['handed_over']} fll={e['fll_status']:9s} "
            f"peak={e['peak_theta_err_deg']:7.1f} steady={e['steady_theta_err_max_deg'] if e['steady_theta_err_max_deg'] is not None else -1:7.1f} "
            f"sat={e['saturation_frac_after_transient']:.3f}{dip}{rec} {'PASS' if e['passed'] else 'FAIL'}"
        )
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
