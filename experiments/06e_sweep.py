#!/usr/bin/env python3
"""Phase 06e Step 2: PLL bandwidth sweep at the paper-matched dips.

Chain B (v' -> unnormalized Q-PLL, zeta=0.707), matched cell, fixed-time handover
t_h=0.30 s, advance off (on reported). f_n (designed at |E| of 1000 rpm) in
{20,40,80,160} Hz x alpha_w in {3,4.4,7,10,15,20} Hz x {S2,S3}.

Model prediction (frozen before the runs): after a load step the Q-PLL frequency error
peaks at 0.456*a0/omega_n_eff at t=1.1/omega_n_eff, independent of alpha_w, with
omega_n_eff = 2*pi*f_n*sqrt(omega_peak/omega_1000) (unnormalized loop), a0 = p*dT/J.

Output: experiments/output/06e_sweep/sweep.json
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

import c2_protocol as P  # noqa: E402
import chain as C  # noqa: E402
import sensorless_eval as E  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque  # noqa: E402

TWO_PI = 2.0 * math.pi
F_N_GRID = (20.0, 40.0, 80.0, 160.0)
ALPHA_GRID = (3.0, 4.4, 7.0, 10.0, 15.0, 20.0)
PAPER = {"S2_sim_50pct_1000": (23.0, 350.0, 0.5), "S3_exp_20pct_1000": (10.0, 46.0, 0.2)}
STACK = None


def predict_peak_rpm(f_n_hz, a_e, rpm0):
    p = STACK.pole_pairs
    w0 = p * rpm0 * TWO_PI / 60.0
    wn0 = TWO_PI * f_n_hz
    wn = wn0
    for _ in range(60):
        t = 1.1 / wn
        w_peak = max(w0 - a_e * t, 0.02 * w0)
        wn = wn0 * math.sqrt(w_peak / (p * 1000.0 * TWO_PI / 60.0))
    peak_e = 0.456 * a_e / wn
    return peak_e * 60.0 / (TWO_PI * p)


def matched(fn):
    a = C.handover_assumptions(STACK)
    return C.EstimatorSource(observer=C.build_observer("matched_sigmoid", STACK),
                             fll=C.build_fll(STACK.initial_omega_at(a.omega_h_rpm),
                                             min_omega=C.fll_min_omega_rad_s(STACK)),
                             qpll=C.QPLL(C.QPLLParameters.from_design(zeta=0.707, f_n_hz=fn,
                                                                      amplitude_v=STACK.design_amp_v)),
                             pole_pairs=STACK.pole_pairs)


def run_cell(stack, params, plant, spec, *, fn, aw, advance):
    rec, src, res = P.run_fixed(lambda: matched(fn), stack, params, plant, spec, alpha_w=aw, advance=advance)
    m = P.metrics(rec, src, res, dynamic=spec["dynamic"], steady=spec["steady"],
                  disturbance_s=spec["disturbance_s"])
    ok, peak = P.lock_valid(rec)
    peak_theta = m["peak_theta_err_deg"]
    stable = bool(ok and peak_theta is not None and peak_theta < 30.0
                  and m["saturation_after_transient"] == 0.0)
    dip = m.get("dip_rpm")
    dyn = m["dynamic_omega"]["max"]
    return {"lock_valid": ok, "lock_peak_deg": peak, "stable": stable,
            "peak_theta_err_deg": peak_theta, "dynamic_omega_max_rpm": dyn,
            "dip_rpm": dip,
            "ratio": (abs(dyn) / abs(dip) if dip else None),
            "steady_theta_max_deg": m["steady_theta"]["max"],
            "saturation_after_transient": m["saturation_after_transient"],
            "fll_status": m["fll_status"], "handed_over": m["handed_over"]}


def main() -> int:
    global STACK
    stack, params, plant, rule = E.load()
    STACK = stack
    scenarios = E.paper_scenarios(rule)
    p = stack.pole_pairs
    J = params.inertia
    # frozen predictions
    predictions = {}
    for scen_name, (paper_err, paper_dip, frac) in PAPER.items():
        a_e = p * (frac * 7.7 / J)
        for fn in F_N_GRID:
            predictions[f"{scen_name}|fn{fn:g}"] = {
                "predicted_peak_rpm": predict_peak_rpm(fn, a_e, 1000.0),
                "paper_error_rpm": paper_err, "paper_dip_rpm": paper_dip}
    # paper-implied f_n
    implied = {}
    for scen_name, (paper_err, paper_dip, frac) in PAPER.items():
        a_e = p * (frac * 7.7 / J)
        target_e = paper_err * TWO_PI * p / 60.0
        wn_needed = 0.456 * a_e / target_e
        implied[scen_name] = {"f_n_hz_ignoring_sqrt": wn_needed / TWO_PI,
                              "predicted_peak_at_f20_rpm": predictions[f"{scen_name}|fn20"]["predicted_peak_rpm"]}

    cells = {}
    for scen_name in PAPER:
        spec = scenarios[scen_name]
        for fn in F_N_GRID:
            for aw in ALPHA_GRID:
                for advance in (False, True):
                    key = f"{scen_name}|fn{fn:g}|aw{aw:g}|adv{'on' if advance else 'off'}"
                    cells[key] = run_cell(stack, params, plant, spec, fn=fn, aw=aw, advance=advance)

    # (i) matched alpha_w per scenario (interpolate dip vs alpha_w on the stable, adv-off cells)
    def matched_alpha(scen_name):
        pts = []
        for aw in ALPHA_GRID:
            c = cells[f"{scen_name}|fn20|aw{aw:g}|advoff"]
            if c["dip_rpm"] and c["stable"]:
                pts.append((aw, abs(c["dip_rpm"])))
        if not pts:
            return None, None
        aws = np.array([x[0] for x in pts]); inv = np.array([1.0 / x[1] for x in pts])
        target = 1.0 / PAPER[scen_name][1]
        a_match = float(np.interp(target, inv[::-1], aws[::-1])) if target >= inv.min() else None
        return a_match, pts

    matched_info = {}
    for scen_name in PAPER:
        a_match, pts = matched_alpha(scen_name)
        entry = {"alpha_w_hz": a_match, "stable_points": pts}
        if scen_name == "S3_exp_20pct_1000" and (a_match is None or a_match > 15.0):
            entry["note"] = "S3 matched dip beyond the stable range"
        matched_info[scen_name] = entry

    # (ii) max stable alpha_w per f_n
    max_stable = {}
    for scen_name in PAPER:
        for fn in F_N_GRID:
            vals = [aw for aw in ALPHA_GRID
                    if cells[f"{scen_name}|fn{fn:g}|aw{aw:g}|advoff"]["stable"]]
            max_stable[f"{scen_name}|fn{fn:g}"] = max(vals) if vals else None

    # (iv) S4 steady/ripple with dead time x f_n
    s4 = dict(scenarios["S4_steady_1000"])
    dead_rows = {}
    for fn in (20.0, 80.0):
        for label, dead, comp in (("off", 0.0, False), ("on", 2e-6, False), ("on_comp", 2e-6, True)):
            rec, src, res = P.run_fixed(lambda fn=fn: matched(fn), stack, params, plant, s4,
                                        alpha_w=10.0, dead_time=dead, compensation=comp)
            m = P.metrics(rec, src, res, dynamic=s4["dynamic"], steady=s4["steady"],
                          disturbance_s=None)
            t = np.asarray(rec.t); e = np.asarray(rec.theta_err); post = np.asarray(rec.post, bool)
            mm = (t >= 0.6) & post
            dead_rows[f"fn{fn:g}|{label}"] = {
                "steady_theta_max_deg": m["steady_theta"]["max"],
                "theta_ripple_rms_deg": float(np.sqrt(np.mean((e[mm] - np.mean(e[mm])) ** 2))) if mm.any() else None}

    out = {"note": ("Phase 06e Step 2: PLL f_n sweep at the paper-matched dips. Chain B "
                    "unnormalized Q-PLL, zeta=0.707, matched cell, t_h=0.30 s. Predictions "
                    "frozen before the runs."),
           "config": {"f_n_grid": list(F_N_GRID), "alpha_grid": list(ALPHA_GRID),
                      "t_h_s": P.T_H, "zeta": 0.707, "design_amp_v": stack.design_amp_v},
           "predictions": predictions, "paper_implied_f_n": implied,
           "cells": cells, "matched_alpha": matched_info, "max_stable_alpha": max_stable,
           "s4_deadtime_ripple": dead_rows}
    out_dir = config.find_project_root() / "experiments" / "output" / "06e_sweep"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "sweep.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== predictions (rpm) ==")
    for k, v in predictions.items():
        print(f"  {k:32s} predicted {v['predicted_peak_rpm']:6.0f} rpm (paper {v['paper_error_rpm']:.0f})")
    print("== paper-implied f_n ==", {k: round(v['f_n_hz_ignoring_sqrt'], 1) for k, v in implied.items()})
    print("== matched alpha ==", {k: (round(v['alpha_w_hz'], 2) if v['alpha_w_hz'] else None) for k, v in matched_info.items()})
    print("== max stable alpha per f_n ==", max_stable)
    print("== cells (adv off): fn, aw, S2/S3 dip, dyn|dw|, predicted, ratio, stable ==")
    for fn in F_N_GRID:
        for aw in ALPHA_GRID:
            a = cells[f"S2_sim_50pct_1000|fn{fn:g}|aw{aw:g}|advoff"]
            print(f"  fn={fn:5g} aw={aw:4g}: S2 dip={a['dip_rpm']} dyn={a['dynamic_omega_max_rpm']} "
                  f"pred={predictions[f'S2_sim_50pct_1000|fn{fn:g}']['predicted_peak_rpm']:.0f} "
                  f"r={a['ratio']} st={a['stable']} | "
                  f"S3 dyn={cells[f'S3_exp_20pct_1000|fn{fn:g}|aw{aw:g}|advoff']['dynamic_omega_max_rpm']}")
    print("== S4 dead-time ripple ==")
    for k, v in dead_rows.items():
        print(f"  {k:16s} steady={v['steady_theta_max_deg']} ripple={v['theta_ripple_rms_deg']}")
    print(f"Outputs: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
