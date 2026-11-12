#!/usr/bin/env python3
"""Phase 06f Step 1: record trajectories + GATE R (replay fidelity).

Sensorless closed loop, nominal chain (matched fixed sigmoid -> DSOGI-FLL v' ->
unnormalized Q-PLL f_n=20, t_h=0.30 s, advance off): S2 at alpha_w=4.4 Hz, S3 at
alpha_w=10 Hz. Saves t, applied u_ab, sampled i_ab, theta_e, omega_e, omega', theta_hat,
omega_hat, t_step. Also the encoder baseline (TrueSource) at the same alpha_w.

GATE R: a fresh identical chain driven by the recorded (u, i) from t=0 reproduces
theta_hat/omega_hat to < 1e-9 (rad, rad/s). Failure = STOP.

Output: experiments/output/06f_record/record.npz + record.json
"""

from __future__ import annotations

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
from pmsm_sensorless import config, config as cfg  # noqa: E402
from pmsm_sensorless.simulation.drive import run_drive  # noqa: E402

GATE_R_TOL = 1e-9


class RecordingSource(C.HandoverSource):
    """HandoverSource that records the drive/chain signals for offline replay."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.rows: dict[str, list[float]] = {
            k: []
            for k in (
                "t",
                "u_alpha",
                "u_beta",
                "i_alpha",
                "i_beta",
                "theta_e",
                "omega_e",
                "theta_hat",
                "omega_hat",
                "omega_prime",
            )
        }

    def __call__(self, sample, truth):
        out = super().__call__(sample, truth)
        r = self.rows
        r["t"].append(sample.t)
        r["u_alpha"].append(sample.u_alpha)
        r["u_beta"].append(sample.u_beta)
        r["i_alpha"].append(sample.i_alpha)
        r["i_beta"].append(sample.i_beta)
        r["theta_e"].append(truth.theta_e)
        r["omega_e"].append(truth.omega_e)
        r["theta_hat"].append(self.chain.theta_hat)
        r["omega_hat"].append(self.chain.omega_hat)
        r["omega_prime"].append(self.chain.omega_prime_rad_s)
        return out


def _chain(stack, fn=20.0):
    a = C.handover_assumptions(stack)
    return C.EstimatorSource(
        observer=C.build_observer("matched_sigmoid", stack),
        fll=C.build_fll(
            stack.initial_omega_at(a.omega_h_rpm), min_omega=C.fll_min_omega_rad_s(stack)
        ),
        qpll=C.QPLL(
            C.QPLLParameters.from_design(zeta=0.707, f_n_hz=fn, amplitude_v=stack.design_amp_v)
        ),
        pole_pairs=stack.pole_pairs,
    )


def main() -> int:
    stack, params, plant, rule = E.load()
    scenarios = E.paper_scenarios(rule)
    runs = {
        "S2_aw4p4": (dict(scenarios["S2_sim_50pct_1000"]), 4.4),
        "S3_aw10": (dict(scenarios["S3_exp_20pct_1000"]), 10.0),
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06f_record"
    out_dir.mkdir(parents=True, exist_ok=True)
    arrays: dict[str, np.ndarray] = {}
    meta: dict = {
        "note": (
            "Phase 06f Step 1 nominal chain: matched -> DSOGI-FLL v' -> "
            "unnormalized Q-PLL f_n=20, t_h=0.30 s, advance off."
        ),
        "gate_r_tol": GATE_R_TOL,
        "runs": {},
    }
    gate = {}
    for name, (spec, aw) in runs.items():
        from dataclasses import replace

        prm = replace(params, alpha_w_hz=aw)
        rec = RecordingSource(
            _chain(stack),
            pole_pairs=stack.pole_pairs,
            assumptions=C.handover_assumptions(stack),
            fixed_time_s=0.30,
        )
        res = run_drive(
            plant,
            prm,
            duration=spec["duration"],
            speed_ref_rpm=spec["speed_ref"],
            load_torque=spec["load"],
            source=rec,
        )
        for k, v in rec.rows.items():
            arrays[f"{name}|{k}"] = np.asarray(v)
        meta["runs"][name] = {
            "alpha_w_hz": aw,
            "speed_ref_rpm": spec["speed_ref"],
            "t_step_s": spec["disturbance_s"],
            "switch_time_s": rec.switch_time_s,
            "n": len(rec.rows["t"]),
            "dt": float(rec.rows["t"][1] - rec.rows["t"][0]),
        }
        # encoder baseline
        eres = run_drive(
            plant,
            prm,
            duration=spec["duration"],
            speed_ref_rpm=spec["speed_ref"],
            load_torque=spec["load"],
            source=None,
        )
        arrays[f"{name}|enc|omega_m"] = (
            eres.omega_m_raw if hasattr(eres, "omega_m_raw") else eres.omega_m
        )
        arrays[f"{name}|enc|dip_rpm"] = np.asarray([np.min(eres.omega_m)])

        # GATE R
        dt = meta["runs"][name]["dt"]
        fresh = _chain(stack)
        rep = C.run_chain_open_loop(
            fresh.observer,
            fresh.fll,
            fresh.qpll,
            u_alpha=np.asarray(rec.rows["u_alpha"]),
            u_beta=np.asarray(rec.rows["u_beta"]),
            i_alpha=np.asarray(rec.rows["i_alpha"]),
            i_beta=np.asarray(rec.rows["i_beta"]),
            dt=dt,
        )
        dmax_t = float(np.max(np.abs(rep["theta_hat"] - np.asarray(rec.rows["theta_hat"]))))
        dmax_w = float(np.max(np.abs(rep["omega_hat"] - np.asarray(rec.rows["omega_hat"]))))
        gate[name] = {
            "max_d_theta_rad": dmax_t,
            "max_d_omega_rad_s": dmax_w,
            "pass": bool(dmax_t < GATE_R_TOL and dmax_w < GATE_R_TOL),
        }
    meta["gate_r"] = gate
    np.savez_compressed(out_dir / "record.npz", **arrays)
    (out_dir / "record.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")

    print("== GATE R (replay fidelity) ==")
    ok = True
    for k, v in gate.items():
        print(
            f"  {k:12s} max|dtheta|={v['max_d_theta_rad']:.3e} rad  max|domega|={v['max_d_omega_rad_s']:.3e} rad/s  pass={v['pass']}"
        )
        ok = ok and v["pass"]
    for k, v in meta["runs"].items():
        print(
            f"  {k}: n={v['n']} dt={v['dt']:.2e} t_step={v['t_step_s']} switch={v['switch_time_s']} alpha_w={v['alpha_w_hz']}"
        )
    print("GATE R:", "PASS" if ok else "FAIL -- STOP")
    print(f"Outputs: {out_dir}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
