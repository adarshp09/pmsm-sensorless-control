#!/usr/bin/env python3
"""Phase 06c-2 Step 4: dead time in the closed loop — GATE D' + sensorless.

Gate D': encoder-fed drive (control = truth) with the matched chain in the background,
500/1000 rpm, I_q = I_rated and 20 %; dead time OFF / ON / ON+compensation. Compares the
E_hat fundamental (amplitude/phase) and the 6th-harmonic raw-arctan angle ripple to the
dead-time-free run.

Sensorless: matched chain under the fixed-time handover, dead time {OFF, ON, ON+comp} on
S1, S3, S4 and S2 at the Step-2 matched alpha_w.

Output: experiments/output/06c2_deadtime/deadtime.json
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

import c2_protocol as P  # noqa: E402
import chain as C  # noqa: E402
import sensorless_eval as E  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.plant.load_models import ConstantLoadTorque  # noqa: E402
from pmsm_sensorless.simulation.drive import run_drive  # noqa: E402
from validation_metrics import vector_metrics  # noqa: E402

TWO_PI = 2.0 * math.pi
T_D = 2e-6
PRED = 4.0 / math.pi * (311.0 * T_D * 10000.0)  # 7.92 V


def _observer_ehat(stack, res):
    observer = C.build_observer("matched_sigmoid", stack)
    n = len(res.t)
    ea, eb = np.empty(n), np.empty(n)
    for k in range(n):
        s = observer.step(
            res.u_alpha[k], res.u_beta[k], res.i_alpha[k], res.i_beta[k], float(res.t[1] - res.t[0])
        )
        ea[k], eb[k] = s.back_emf_alpha, s.back_emf_beta
    return ea, eb


def _gate_d(stack, plant, params, *, rpm, frac, dead, comp):
    res = run_drive(
        plant,
        params,
        duration=0.6,
        speed_ref_rpm=float(rpm),
        load_torque=ConstantLoadTorque(frac * E.RATED_TORQUE),
        dead_time_s=dead,
        dead_time_compensation=comp,
    )
    ea, eb = _observer_ehat(stack, res)
    f_e = stack.pole_pairs * rpm / 60.0
    m = res.t >= 0.3
    e_true = np.abs(stack.pole_pairs * rpm * TWO_PI / 60.0 * stack.psi_f)
    vm = vector_metrics(
        ea[m],
        eb[m],
        -(res.omega_e[m] * stack.psi_f) * np.sin(res.theta_e[m]),
        (res.omega_e[m] * stack.psi_f) * np.cos(res.theta_e[m]),
        res.theta_e[m],
    )
    theta_hat = np.degrees(np.arctan2(-ea[m], eb[m]))
    theta_true = np.degrees(res.theta_e[m])
    err = (theta_hat - theta_true + 180.0) % 360.0 - 180.0
    err = err - np.mean(err)
    n = len(err)
    spec = np.abs(np.fft.rfft(err)) * 2.0 / n
    freqs = np.fft.rfftfreq(n, float(res.t[1] - res.t[0]))
    i6 = np.argmin(np.abs(freqs - 6.0 * f_e))
    return {
        "abs_E_v": e_true,
        "fundamental_gain": vm["fundamental_gain"],
        "amplitude_pct": 100.0 * (vm["fundamental_gain"] - 1.0),
        "phase_deg": vm["fundamental_phase_deg"],
        "h6_deg": float(spec[i6]),
        "pred_amp_pct": 100.0 * PRED / e_true,
    }


def main() -> int:
    stack, params, plant, rule = E.load()
    suite_path = config.find_project_root() / "experiments" / "output" / "06c2_suite" / "suite.json"
    s2_alpha = 5.0
    if suite_path.is_file():
        s2_alpha = float(json.loads(suite_path.read_text()).get("s2_matched_alpha_w", 5.0))

    # ---- Gate D'
    gate = {}
    for rpm in (500.0, 1000.0):
        for frac in (1.0, 0.2):
            for label, dead, comp in (
                ("off", 0.0, False),
                ("on", T_D, False),
                ("on_comp", T_D, True),
            ):
                gate[f"rpm{int(rpm)}_iq{int(frac * 100)}|{label}"] = _gate_d(
                    stack, plant, params, rpm=rpm, frac=frac, dead=dead, comp=comp
                )

    # ---- sensorless dead time
    scenarios = E.paper_scenarios(rule)
    s2 = dict(scenarios["S2_sim_50pct_1000"])
    s2["alpha_w"] = s2_alpha
    runs = {
        "S1": (dict(scenarios["S1_no_load_ramp"]), 10.0),
        "S3": (dict(scenarios["S3_exp_20pct_1000"]), 10.0),
        "S4": (dict(scenarios["S4_steady_1000"]), 10.0),
        "S2_matched": (s2, s2_alpha),
    }

    def matched():
        a = C.handover_assumptions(stack)
        return C.EstimatorSource(
            observer=C.build_observer("matched_sigmoid", stack),
            fll=C.build_fll(
                stack.initial_omega_at(a.omega_h_rpm), min_omega=C.fll_min_omega_rad_s(stack)
            ),
            qpll=C.build_qpll(stack),
            pole_pairs=stack.pole_pairs,
        )

    sensorless = {}
    for rname, (spec, aw) in runs.items():
        for label, dead, comp in (("off", 0.0, False), ("on", T_D, False), ("on_comp", T_D, True)):
            rec, src, res = P.run_fixed(
                matched, stack, params, plant, spec, alpha_w=aw, dead_time=dead, compensation=comp
            )
            m = P.metrics(
                rec,
                src,
                res,
                dynamic=spec["dynamic"],
                steady=spec["steady"],
                disturbance_s=spec["disturbance_s"],
            )
            ok, peak = P.lock_valid(rec)
            sensorless[f"{rname}|{label}"] = {
                "lock_valid": ok,
                "lock_peak_deg": peak,
                "steady_theta_max_deg": m["steady_theta"]["max"],
                "dynamic_theta_max_deg": m["dynamic_theta"]["max"],
                "dynamic_omega_max_rpm": m["dynamic_omega"]["max"],
                "dip_rpm": m.get("dip_rpm"),
                "saturation": m["saturation_after_transient"],
                "fll_status": m["fll_status"],
                "handed_over": m["handed_over"],
                "steady_theta_rad": (
                    math.radians(m["steady_theta"]["max"])
                    if m["steady_theta"]["max"] is not None
                    else None
                ),
            }

    out = {
        "note": (
            "Phase 06c-2 Step 4: dead time OFF/ON/ON+comp; observer sees the "
            "commanded voltage. Gate D' prediction dE1=7.92 V. Fixed-time handover "
            "t_h=0.30 s."
        ),
        "prediction": {"dE1_v": PRED, "formula": "4/pi Vdc td fsw"},
        "gate_d_prime": gate,
        "sensorless": sensorless,
        "s2_matched_alpha_w": s2_alpha,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06c2_deadtime"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "deadtime.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== GATE D' (encoder-fed, chain in background) ==")
    for k, v in gate.items():
        print(
            f"  {k:22s} |E|={v['abs_E_v']:5.1f}V amp={v['amplitude_pct']:+6.1f}% "
            f"(pred {v['pred_amp_pct']:5.1f}%) phase={v['phase_deg']:+6.2f}° h6={v['h6_deg']:.2f}°"
        )
    print("\n== sensorless dead time (matched) ==")
    for k, v in sensorless.items():
        print(
            f"  {k:18s} lv={v['lock_valid']} steady={v['steady_theta_max_deg']} "
            f"dyn_dth={v['dynamic_theta_max_deg']} dyn_dw={v['dynamic_omega_max_rpm']} "
            f"sat={v['saturation']:.3f} fll={v['fll_status']}"
        )
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
