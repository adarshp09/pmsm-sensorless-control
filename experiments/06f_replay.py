#!/usr/bin/env python3
"""Phase 06f Steps 2-3: replay decomposition (open loop, recorded (u, i)).

Variants (f_n in {20,40,80,160} Hz, designed at |E|_1000, unnormalized, zeta=0.707):
 V0 E_true -> Q-PLL                     (ideal back-EMF)
 V1 E_hat  -> Q-PLL                     (observer only)
 V2 E_hat  -> DSOGI-FLL v' -> Q-PLL     (nominal)
 V3 E_true -> DSOGI-FLL v' -> Q-PLL     (observer removed, SOGI+FLL only)
 V4 E_hat  -> DSOGI (oracle omega') -> Q-PLL  (FLL removed)

Step 3: V2 with Gamma in {50,500,5000} at f_n in {20,80} (DIAGNOSTIC, not adopted).

Output: experiments/output/06f_replay/replay.json
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
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.estimation import QPLL, QPLLParameters  # noqa: E402
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters  # noqa: E402

TWO_PI = 2.0 * math.pi
K_S, GAMMA = math.sqrt(2.0), 50.0
F_N_GRID = (20.0, 40.0, 80.0, 160.0)
# frozen 06e model values (rpm)
MODEL_FROZEN = {
    "S2_aw4p4": {20: 100, 40: 46, 80: 22, 160: 11},
    "S3_aw10": {20: 37, 40: 18, 80: 9, 160: 4},
}
PAPER = {"S2_aw4p4": 23.0, "S3_aw10": 10.0}


class OracleFLL(DSOGIFLL):
    """DSOGI band-pass with the FLL replaced by the true omega (oracle omega')."""

    def step_oracle(self, v_alpha, v_beta, dt, omega_true):
        self._omega = omega_true
        out = self.step(v_alpha, v_beta, dt)
        self._omega = omega_true
        return out


def rpm_from_elec(w_e, p):
    """Electrical rad/s -> mechanical rpm."""
    return w_e * 60.0 / (TWO_PI * p)


def metrics(t, th_hat, om_hat, th_e, om_e, *, t_step, p, om_prime=None):
    m = t >= t_step
    t_ = t[m]
    de = (th_hat[m] - th_e[m] + math.pi) % TWO_PI - math.pi
    dw = (om_hat[m] - om_e[m]) * 60.0 / (TWO_PI * p)
    w50 = t_ <= t_step + 0.05
    w200 = t_ <= t_step + 0.20
    fp_idx = int(np.argmax(np.abs(dw[w50]))) if w50.any() else 0
    out = {
        "first_peak_rpm": float(np.max(np.abs(dw[w50]))) if w50.any() else None,
        "first_peak_time_s": float(t_[w50][fp_idx]) if w50.any() else None,
        "window_max_rpm": float(np.max(np.abs(dw[w200]))) if w200.any() else None,
        "peak_theta_deg": float(np.max(np.abs(np.degrees(de[w200])))) if w200.any() else None,
    }
    if om_prime is not None:
        lag = (om_e[m] - om_prime[m]) * 60.0 / (TWO_PI * p)
        out["fll_lag_peak_rpm"] = float(np.max(np.abs(lag[w200]))) if w200.any() else None
    return out


def replay(stack, rec, *, variant, fn, spec):
    t = rec["t"]
    dt = float(t[1] - t[0])
    ua, ub, ia, ib = rec["u_alpha"], rec["u_beta"], rec["i_alpha"], rec["i_beta"]
    th_e, om_e = rec["theta_e"], rec["omega_e"]
    n = len(t)
    obs = C.build_observer("matched_sigmoid", stack) if variant in ("V1", "V2", "V4") else None
    a = C.handover_assumptions(stack)
    fll = None
    if variant in ("V2", "V3"):
        fll = C.build_fll(
            stack.initial_omega_at(a.omega_h_rpm), min_omega=C.fll_min_omega_rad_s(stack)
        )
    elif variant == "V4":
        fll = OracleFLL(DSOGIFLLParameters(K_S, GAMMA, stack.initial_omega_at(a.omega_h_rpm)))
    qpll = QPLL(QPLLParameters.from_design(zeta=0.707, f_n_hz=fn, amplitude_v=stack.design_amp_v))
    th_hat = np.empty(n)
    om_hat = np.empty(n)
    om_prime = np.empty(n) if fll is not None else None
    for k in range(n):
        if variant in ("V0", "V3"):
            ea = -om_e[k] * stack.psi_f * math.sin(th_e[k])
            eb = om_e[k] * stack.psi_f * math.cos(th_e[k])
        else:
            s = obs.step(float(ua[k]), float(ub[k]), float(ia[k]), float(ib[k]), dt)
            ea, eb = s.back_emf_alpha, s.back_emf_beta
        if fll is None:
            va, vb = ea, eb
        elif variant == "V4":
            f = fll.step_oracle(ea, eb, dt, om_e[k])
            va, vb = f.v_alpha_prime, f.v_beta_prime
            om_prime[k] = om_e[k]
        else:
            f = fll.step(ea, eb, dt)
            va, vb = f.v_alpha_prime, f.v_beta_prime
            om_prime[k] = f.omega
        q = qpll.step(va, vb, dt)
        th_hat[k], om_hat[k] = q.theta_hat, q.omega_hat
    return th_hat, om_hat, om_prime


def model_value(rec, *, fn, t_step, t_peak, p):
    om_e = rec["omega_e"]
    t = rec["t"]
    dt = float(t[1] - t[0])
    i0 = int(round(t_step / dt))
    i2 = int(round((t_step + 0.002) / dt))
    a0 = (om_e[i2] - om_e[i0]) / (t[i2] - t[i0])  # electrical rad/s^2
    w_peak = om_e[int(round(t_peak / dt))]
    wn = TWO_PI * fn * math.sqrt(max(w_peak, 0.0) / (400.0 * math.pi / 3.0))
    return rpm_from_elec(0.456 * a0 / wn, p), rpm_from_elec(a0, p)


def main() -> int:
    stack, params, plant, rule = E.load()
    root = config.find_project_root() / "experiments" / "output" / "06f_record"
    data = np.load(root / "record.npz")
    meta = json.loads((root / "record.json").read_text())

    out = {
        "note": (
            "Phase 06f Steps 2-3: replay decomposition on the recorded (u,i); "
            "dead time OFF; same initial conditions as the chain. Model values "
            "frozen from 06e."
        ),
        "model_frozen_rpm": MODEL_FROZEN,
        "paper_error_rpm": PAPER,
        "variants": {},
        "gamma_diag": {},
    }
    for scen in ("S2_aw4p4", "S3_aw10"):
        rec = {
            k.split("|")[1]: data[f"{scen}|{k.split('|')[1]}"].copy()
            for k in data.files
            if k.startswith(f"{scen}|") and "|enc|" not in k
        }
        t_step = meta["runs"][scen]["t_step_s"]
        for variant in ("V0", "V1", "V2", "V3", "V4"):
            for fn in F_N_GRID:
                th, om, omp = replay(stack, rec, variant=variant, fn=fn, spec=None)
                m = metrics(
                    rec["t"],
                    th,
                    om,
                    rec["theta_e"],
                    rec["omega_e"],
                    t_step=t_step,
                    p=stack.pole_pairs,
                    om_prime=omp,
                )
                mv, a0 = model_value(
                    rec,
                    fn=fn,
                    t_step=t_step,
                    t_peak=(m["first_peak_time_s"] or t_step + 0.01),
                    p=stack.pole_pairs,
                )
                m["model_rpm_at_peak"] = mv
                m["recorded_a0_rpm_s"] = a0
                m["first_peak_over_model"] = (m["first_peak_rpm"] / mv) if mv else None
                out["variants"][f"{scen}|{variant}|fn{fn:g}"] = m
        for fn in (20.0, 80.0):
            for gamma in (50.0, 500.0, 5000.0):
                n = len(rec["t"])
                dt = float(rec["t"][1] - rec["t"][0])
                obs = C.build_observer("matched_sigmoid", stack)
                fll = DSOGIFLL(DSOGIFLLParameters(K_S, gamma, stack.initial_omega_at(500.0)))
                qpll = QPLL(
                    QPLLParameters.from_design(
                        zeta=0.707, f_n_hz=fn, amplitude_v=stack.design_amp_v
                    )
                )
                th = np.empty(n)
                om = np.empty(n)
                omp = np.empty(n)
                status = "ok"
                for k in range(n):
                    s = obs.step(
                        float(rec["u_alpha"][k]),
                        float(rec["u_beta"][k]),
                        float(rec["i_alpha"][k]),
                        float(rec["i_beta"][k]),
                        dt,
                    )
                    try:
                        f = fll.step(s.back_emf_alpha, s.back_emf_beta, dt)
                        va, vb = f.v_alpha_prime, f.v_beta_prime
                        omp[k] = f.omega
                    except ValueError:
                        status = "guard-fail"
                        va, vb = s.back_emf_alpha, s.back_emf_beta
                        omp[k] = fll.omega
                    q = qpll.step(va, vb, dt)
                    th[k], om[k] = q.theta_hat, q.omega_hat
                m = metrics(
                    rec["t"],
                    th,
                    om,
                    rec["theta_e"],
                    rec["omega_e"],
                    t_step=t_step,
                    p=stack.pole_pairs,
                    om_prime=omp,
                )
                m["fll_status"] = status
                out["gamma_diag"][f"{scen}|V2|fn{fn:g}|gamma{gamma:g}"] = m

    out_dir = config.find_project_root() / "experiments" / "output" / "06f_replay"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "replay.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== Step 2: first-peak |omega_hat-omega| rpm (model / paper) ==")
    for scen in ("S2_aw4p4", "S3_aw10"):
        print(f"  {scen} (paper {PAPER[scen]:.0f} rpm; frozen model {MODEL_FROZEN[scen]})")
        for variant in ("V0", "V1", "V2", "V3", "V4"):
            row = []
            for fn in F_N_GRID:
                m = out["variants"][f"{scen}|{variant}|fn{fn:g}"]
                row.append(f"f{fn:.0f}:{m['first_peak_rpm']:.0f}/{m['model_rpm_at_peak']:.0f}")
            print(f"    {variant}: " + "  ".join(row))
    print("\n== Step 2 detail (S2, fn20): theta peak, window max, fll lag ==")
    for variant in ("V0", "V1", "V2", "V3", "V4"):
        m = out["variants"][f"S2_aw4p4|{variant}|fn20"]
        print(
            f"  {variant}: theta={m['peak_theta_deg']:.2f}° winmax={m['window_max_rpm']:.0f} "
            f"fp@{m['first_peak_time_s']:.3f}s lag={m.get('fll_lag_peak_rpm')}"
        )
    print("\n== Step 3: Gamma diagnostic (first-peak rpm / fll lag / status) ==")
    for k, v in out["gamma_diag"].items():
        print(
            f"  {k:28s} fp={v['first_peak_rpm']:.0f} lag={v.get('fll_lag_peak_rpm')} st={v['fll_status']} theta={v['peak_theta_deg']:.1f}"
        )
    print(f"Outputs: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
