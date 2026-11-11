#!/usr/bin/env python3
"""Phase 06f Step 4: closed-loop cross-checks (S2 matched cell, advance off, t_h=0.30 s).

(a) alpha_w in {2,3} Hz x f_n in {20,80,160}: stable?, first-peak |omega_hat-omega|, peak
    |theta_hat-theta| (compare with the Step 2 V2 values at the same f_n).
(b) Channel isolation at f_n in {20,80}, alpha_w=10: (theta_enc, omega_chain),
    (theta_chain, omega_enc), (chain, chain) -- which path carries the f_n instability.
(c) Cost of high f_n at alpha_w=3: S4 steady |theta_hat-theta| mean/RMS and theta ripple,
    dead time {OFF, ON+comp}, for every f_n.

Output: experiments/output/06f_closed/closed.json
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import replace
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
from pmsm_sensorless.estimation import QPLL, QPLLParameters  # noqa: E402

TWO_PI = 2.0 * math.pi
STACK = None


def matched(fn):
    a = C.handover_assumptions(STACK)
    return C.EstimatorSource(observer=C.build_observer("matched_sigmoid", STACK),
                             fll=C.build_fll(STACK.initial_omega_at(a.omega_h_rpm),
                                             min_omega=C.fll_min_omega_rad_s(STACK)),
                             qpll=QPLL(QPLLParameters.from_design(zeta=0.707, f_n_hz=fn,
                                                                  amplitude_v=STACK.design_amp_v)),
                             pole_pairs=STACK.pole_pairs)


class MixSource(P.FullRecorder):
    """HandoverSource with theta/omega channels independently from encoder or chain."""

    def __init__(self, *args, theta_channel="chain", omega_channel="chain", **kwargs):
        super().__init__(*args, **kwargs)
        self.theta_channel, self.omega_channel = theta_channel, omega_channel

    def __call__(self, sample, truth):
        out = super().__call__(sample, truth)
        if self.handed_over:
            th = truth.theta_e if self.theta_channel == "encoder" else self.chain.theta_hat
            om = truth.omega_e if self.omega_channel == "encoder" else self.chain.omega_hat
            return th, om
        return out


def window(rec, t_step, *, dt_max=0.05, p=4):
    t = np.asarray(rec.t); post = np.asarray(rec.post, bool)
    m = (t >= t_step) & (t <= t_step + dt_max) & post
    de = np.asarray(rec.theta_err)[m]
    dw = np.asarray(rec.omega_err)[m]
    return {"first_peak_rpm": float(np.max(np.abs(dw))) if m.any() else None,
            "peak_theta_deg": float(np.max(np.abs(de))) if m.any() else None,
            "max_theta_deg_200ms": float(np.max(np.abs(np.asarray(rec.theta_err)[
                (t >= t_step) & (t <= t_step + 0.2) & post]))) if m.any() else None}


def main() -> int:
    global STACK
    stack, params, plant, rule = E.load()
    STACK = stack
    scenarios = E.paper_scenarios(rule)
    s2 = dict(scenarios["S2_sim_50pct_1000"]); s4 = dict(scenarios["S4_steady_1000"])
    t_step = s2["disturbance_s"]
    out = {"note": ("Phase 06f Step 4 closed-loop cross-checks; matched cell, advance off, "
                    "t_h=0.30 s, dead time OFF (S4 also ON+comp)."),
           "a_alpha_w_low": {}, "b_channel_isolation": {}, "c_high_fn_cost": {},
           "step2_v2_reference_rpm": {"fn20": 135, "fn80": 130, "fn160": 108}}

    def run_chain(fn, spec, aw, *, dead=0.0, comp=False, mix=None):
        prm = replace(params, alpha_w_hz=aw)
        src = matched(fn)
        rec = (MixSource(src, pole_pairs=stack.pole_pairs, assumptions=C.handover_assumptions(stack),
                         fixed_time_s=0.30, theta_channel=mix[0], omega_channel=mix[1])
               if mix else P.FullRecorder(src, pole_pairs=stack.pole_pairs,
                                          assumptions=C.handover_assumptions(stack), fixed_time_s=0.30))
        from pmsm_sensorless.simulation.drive import run_drive
        res = run_drive(plant, prm, duration=spec["duration"], speed_ref_rpm=spec["speed_ref"],
                        load_torque=spec["load"], source=rec, dead_time_s=dead,
                        dead_time_compensation=comp)
        return rec, src, res

    # (a)
    for aw in (2.0, 3.0):
        for fn in (20.0, 80.0, 160.0):
            rec, src, res = run_chain(fn, s2, aw)
            ok, peak = P.lock_valid(rec)
            w = window(rec, t_step, p=stack.pole_pairs)
            stable = bool(ok and src.status == "ok" and w["peak_theta_deg"] is not None
                          and w["peak_theta_deg"] < 90.0)
            out["a_alpha_w_low"][f"aw{aw:g}|fn{fn:g}"] = {
                "lock_valid": ok, "lock_peak_deg": peak, "fll_status": src.status,
                "stable": stable, **w}

    # (b)
    for fn in (20.0, 80.0):
        for label, mix in (("theta_enc_omega_chain", ("encoder", "chain")),
                           ("theta_chain_omega_enc", ("chain", "encoder")),
                           ("chain_chain", ("chain", "chain"))):
            rec, src, res = run_chain(fn, s2, 10.0, mix=mix)
            ok, peak = P.lock_valid(rec)
            w = window(rec, t_step, p=stack.pole_pairs)
            out["b_channel_isolation"][f"fn{fn:g}|{label}"] = {
                "lock_valid": ok, "lock_peak_deg": peak, "fll_status": src.status, **w}

    # (c)
    for fn in (20.0, 40.0, 80.0, 160.0):
        for label, dead, comp in (("off", 0.0, False), ("on_comp", 2e-6, True)):
            rec, src, res = run_chain(fn, s4, 3.0, dead=dead, comp=comp)
            t = np.asarray(rec.t); post = np.asarray(rec.post, bool)
            lo, hi = s4["steady"]
            e = np.asarray(rec.theta_err)[(t >= lo) & (t <= hi) & post]
            out["c_high_fn_cost"][f"fn{fn:g}|{label}"] = {
                "steady_mean_deg": float(np.mean(np.abs(e))) if e.size else None,
                "steady_rms_deg": float(np.sqrt(np.mean(e ** 2))) if e.size else None,
                "theta_ripple_rms_deg": float(np.sqrt(np.mean((e - np.mean(e)) ** 2))) if e.size else None,
                "fll_status": src.status, "lock_valid": P.lock_valid(rec)[0]}

    out_dir = config.find_project_root() / "experiments" / "output" / "06f_closed"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "closed.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== (a) low alpha_w, S2 (Step 2 V2 reference: fn20 135, fn80 130, fn160 108 rpm) ==")
    for k, v in out["a_alpha_w_low"].items():
        print(f"  {k:16s} stable={v['stable']} lv={v['lock_valid']}({v['lock_peak_deg']}) "
              f"fp={v['first_peak_rpm']} theta={v['peak_theta_deg']} fll={v['fll_status']}")
    print("\n== (b) channel isolation (S2, alpha_w=10) ==")
    for k, v in out["b_channel_isolation"].items():
        print(f"  {k:34s} lv={v['lock_valid']}({v['lock_peak_deg']}) fp={v['first_peak_rpm']} "
              f"theta={v['peak_theta_deg']} fll={v['fll_status']}")
    print("\n== (c) high f_n cost at alpha_w=3, S4 ==")
    for k, v in out["c_high_fn_cost"].items():
        print(f"  {k:16s} mean={v['steady_mean_deg']} rms={v['steady_rms_deg']} "
              f"ripple={v['theta_ripple_rms_deg']} fll={v['fll_status']}")
    print(f"Outputs: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
