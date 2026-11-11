#!/usr/bin/env python3
"""Phase 06e Step 1: normalized Q-PLL lock-point check (open loop, no speed loop).

Open-loop nominal drive (matched cell), constant 500/1000 rpm, v' input. Steady theta_hat
- theta with normalization OFF vs ON (f_n = 20), and lock-in from a +90 deg start. If ON
converges to a constant offset -> the normalization option has an implementation/convention
bug (reported, not fixed); if ON locks at 0 -> the closed-loop instability is genuine.

Output: experiments/output/06e_step1/lockpoint.json
"""

from __future__ import annotations

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

import chain as C  # noqa: E402
import sensorless_eval as E  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.estimation import QPLL, QPLLParameters, wrap_angle_pi_closed  # noqa: E402


def _load(name, filename):
    spec = importlib.util.spec_from_file_location(name, _DIR / filename)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


DT = _load("d06c", "06c_deadtime.py")


def _run(stack, plant, rpm, *, normalize, theta0_offset=0.0):
    roll = DT._rollout(plant, i_q=stack.i_rated, rpm0=rpm, dead_time_s=0.0, duration=0.6)
    observer = C.build_observer("matched_sigmoid", stack)
    fll = C.build_fll(stack.initial_omega_at(rpm))
    qpll = QPLL(QPLLParameters.from_design(zeta=0.707, f_n_hz=20.0,
                                           amplitude_v=stack.design_amp_v, normalize=normalize),
                initial_theta=roll["theta_e"][0] + theta0_offset,
                initial_omega=stack.pole_pairs * rpm * 2.0 * math.pi / 60.0)
    n = len(roll["t"])
    th = np.empty(n)
    for k in range(n):
        s = observer.step(roll["u_alpha"][k], roll["u_beta"][k], roll["i_alpha"][k], roll["i_beta"][k], DT.TS)
        f = fll.step(s.back_emf_alpha, s.back_emf_beta, DT.TS)
        th[k] = qpll.step(f.v_alpha_prime, f.v_beta_prime, DT.TS).theta_hat
    err = np.degrees([wrap_angle_pi_closed(a - b) for a, b in zip(th, roll["theta_e"])])
    f_e = stack.pole_pairs * rpm / 60.0
    m = roll["t"] >= 0.6 - 8.0 / f_e
    return {"steady_mean_deg": float(np.mean(err[m])), "steady_std_deg": float(np.std(err[m])),
            "final_deg": float(err[-1]),
            "lock_in_deg": float(np.max(np.abs(err[m])))}


def main() -> int:
    directory = config.default_config_dir()
    stack = C.build_stack(directory)
    from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters
    plant = PMSMPlant(PMSMParameters.from_config(stack.pmsm_config))
    out = {"note": ("Phase 06e Step 1: Q-PLL normalization lock-point, open-loop nominal "
                    "drive, matched cell, v' input, f_n=20."), "runs": {}}
    for rpm in (500.0, 1000.0):
        for normalize in (False, True):
            out["runs"][f"rpm{int(rpm)}|norm{'on' if normalize else 'off'}|theta0=0"] = _run(
                stack, plant, rpm, normalize=normalize)
            out["runs"][f"rpm{int(rpm)}|norm{'on' if normalize else 'off'}|theta0=+90"] = _run(
                stack, plant, rpm, normalize=normalize, theta0_offset=math.pi / 2.0)
    out_dir = config.find_project_root() / "experiments" / "output" / "06e_step1"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "lockpoint.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
    print("== Step 1: Q-PLL lock point ==")
    for k, v in out["runs"].items():
        print(f"  {k:32s} steady_mean={v['steady_mean_deg']:8.2f}° std={v['steady_std_deg']:6.2f} "
              f"final={v['final_deg']:8.2f}° lock_in_max={v['lock_in_deg']:8.2f}°")
    print(f"Outputs: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
