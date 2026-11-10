#!/usr/bin/env python3
"""Phase 06b-5: DSOGI-FLL minimum-frequency clamp + staged gate.

Step 1 check: with the clamp set far below the operating frequency (100 rpm floor),
the static open-loop chain (observer -> FLL -> Q-PLL) is bit-identical to the unclamped
chain at 500 and 1000 rpm.

Step 2 gates: decoupled bandwidths (500 rpm: 2.5 Hz, 1000 rpm: 10 Hz), chain B,
FLL clamp enabled, advance off/on.
  2a: handover then 1.0 s steady (no-load, 20 % load incl. start-up-under-load) at
      500/1000 rpm; |theta_hat-theta| < 30 deg throughout, no FLL guard trip, no sat.
  2b: 20 % load step at 500/1000 rpm; stable recovery, no guard trip.
  2c: 50 % (rated) load step at 1000 rpm.

Output: experiments/output/06b5_gate/gate.json
"""

from __future__ import annotations

import dataclasses
import importlib.util
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

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters, run_drive  # noqa: E402
from pmsm_sensorless.simulation.nominal_drive import nominal_drive_rollout  # noqa: E402


def _load(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, _DIR / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


G = _load("g06b3", "06b3_gate.py")
C = G.C
RATED = G.RATED_TORQUE
TS = 100e-6


def _clamp_check(stack, plant, min_omega):
    """Static open-loop: clamped vs unclamped chain must be identical at 500/1000 rpm."""
    out = {}
    for rpm in (500.0, 1000.0):
        r = nominal_drive_rollout(plant, i_q=stack.i_rated, dt=TS, duration=0.4,
                                  rpm_start=rpm, rpm_end=rpm, ramp_time=1.0)
        series = {}
        for label, mo in (("unclamped", None), ("clamped", min_omega)):
            observer = C.build_observer("matched_sigmoid", stack)
            fll = C.build_fll(stack.initial_omega_at(rpm), min_omega=mo)
            n = len(r.t)
            ea, eb = np.empty(n), np.empty(n)
            freq = np.empty(n)
            for k in range(n):
                s = observer.step(r.u_alpha[k], r.u_beta[k], r.i_alpha[k], r.i_beta[k], TS)
                ea[k], eb[k] = s.back_emf_alpha, s.back_emf_beta
                freq[k] = fll.step(ea[k], eb[k], TS).frequency_hz
            series[label] = (ea.copy(), eb.copy(), freq.copy())
        a, b = series["unclamped"], series["clamped"]
        out[f"rpm{int(rpm)}"] = {
            "max_abs_e_hat_diff": float(max(np.max(np.abs(a[0] - b[0])), np.max(np.abs(a[1] - b[1])))),
            "max_abs_freq_diff_hz": float(np.max(np.abs(a[2] - b[2]))),
        }
    out["identical"] = all(v["max_abs_e_hat_diff"] == 0.0 and v["max_abs_freq_diff_hz"] == 0.0
                           for k, v in out.items() if k.startswith("rpm"))
    return out


def _run(stack, params, plant, *, speed_rpm, load, speed_ref, duration, advance, alpha_w, min_omega):
    prm = dataclasses.replace(params, alpha_w_hz=alpha_w)
    assump = C.handover_assumptions(stack)
    chain_source = C.EstimatorSource(
        observer=C.build_observer("matched_sigmoid", stack),
        fll=C.build_fll(stack.initial_omega_at(assump.omega_h_rpm), min_omega=min_omega),
        qpll=C.build_qpll(stack), pole_pairs=stack.pole_pairs)
    source = G.RecordingHandover(chain_source, pole_pairs=stack.pole_pairs, assumptions=assump)
    res = run_drive(plant, prm, duration=duration, speed_ref_rpm=speed_ref,
                    load_torque=load, source=source, angle_advance=advance)
    return source, chain_source, res


def main() -> int:
    directory = config.default_config_dir()
    stack = C.build_stack(directory)
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=directory),
        config.load_config("drive_assumptions", directory=directory),
        stack.pmsm_config)
    plant = PMSMPlant(PMSMParameters.from_config(stack.pmsm_config), integrate_mechanics=True,
                      viscous_friction=0.0)
    rule = G._rule(directory)
    min_omega = C.fll_min_omega_rad_s(stack)
    clamp = _clamp_check(stack, plant, min_omega)

    results = {}
    # 2a: steady at 500/1000 rpm, no-load and 20 % load (incl. start-up-under-load)
    for speed in (500, 1000):
        for load_frac in (0.0, 0.2):
            for advance in (False, True):
                src, cs, res = _run(stack, params, plant, speed_rpm=speed,
                                    load=ConstantLoadTorque(load_frac * RATED), speed_ref=float(speed),
                                    duration=1.6, advance=advance, alpha_w=rule[speed], min_omega=min_omega)
                results[f"2a|{speed}rpm|load{int(load_frac*100)}|adv{'on' if advance else 'off'}"] = G._gate_entry(
                    src, cs, res, alpha_w=rule[speed], advance=advance, speed_rpm=speed)
    # 2b (20 %) at 500/1000; 2c (50 %) at 1000
    for tag, speed, frac in (("2b", 500, 0.2), ("2b", 1000, 0.2), ("2c", 1000, 0.5)):
        for advance in (False, True):
            src, cs, res = _run(stack, params, plant, speed_rpm=speed,
                                load=StepLoadTorque(0.0, frac * RATED, 0.5), speed_ref=float(speed),
                                duration=2.0, advance=advance, alpha_w=rule[speed], min_omega=min_omega)
            results[f"{tag}|{speed}rpm|load{int(frac*100)}|adv{'on' if advance else 'off'}"] = G._gate_entry(
                src, cs, res, step_s=0.5, alpha_w=rule[speed], advance=advance, speed_rpm=speed,
                require_steady_deg=None)

    # A guard trip must fail the gate even if the angle happened to recover.
    for e in results.values():
        if e["fll_status"] != "ok":
            e["passed"] = False

    off = [k for k in results if k.endswith("advoff")]
    gate = all(results[k]["passed"] for k in off) and clamp["identical"]
    out = {
        "note": ("Phase 06b-5 gates with the DSOGI-FLL min_omega clamp enabled "
                 "(100 rpm floor); decoupled bandwidths, chain B, f_n = 20 Hz, advance off/on."),
        "rule": {str(k): v for k, v in rule.items()},
        "fll_min_omega_rad_s": min_omega,
        "static_clamp_check": clamp,
        "gate": "PASS" if gate else "FAIL",
        "results": results,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b5_gate"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "gate.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print(f"== static clamp check: {'IDENTICAL' if clamp['identical'] else 'DIFFERS'} ==")
    for k, v in clamp.items():
        if k.startswith("rpm"):
            print(f"  {k}: max|dE_hat|={v['max_abs_e_hat_diff']:.3e}  max|df|={v['max_abs_freq_diff_hz']:.3e} Hz")
    print(f"\n== GATE (06b-5): {out['gate']} ==  rule={out['rule']}  min_omega={min_omega:.1f} rad/s")
    for key, e in results.items():
        dip = f" dip={e['speed_dip_rpm']:7.1f} rec10={e['recovery_to_10rpm_s']}" if "speed_dip_rpm" in e else ""
        print(f"  {key:30s} hand={e['handed_over']} fll={e['fll_status']:9s} "
              f"peak={e['peak_theta_err_deg']:7.1f} steady={e['steady_theta_err_max_deg'] if e['steady_theta_err_max_deg'] is not None else -1:7.1f} "
              f"sat={e['saturation_frac_after_transient']:.3f}{dip} {'PASS' if e['passed'] else 'FAIL'}")
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
