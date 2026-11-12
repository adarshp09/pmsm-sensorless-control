#!/usr/bin/env python3
"""Phase 06d: estimation-bandwidth diagnosis (experiments only; no src changes).

Arms (0 = 06b-6 nominal; 1-3 DIAGNOSTIC — not paper):
  0  chain B nominal: DSOGI v' -> Q-PLL (f_n = 20 Hz).
  1  chain A: raw E_hat -> Q-PLL (no filter).
  2  ORACLE: DSOGI with prescribed true omega' (FLL bypassed) -> Q-PLL.
  3  normalised FLL: d omega'/dt = -Gamma*omega'*eps_f/(sum|v|^2 + eps) -> v' -> Q-PLL.
     (Fig.5 prints a "gain normalization" block but NOT the algebra -> ASSUMPTION.)

Step 1: open-loop T_est (06b-2 modulation) at 250/500/1000 rpm.
Step 2: closed-loop grid (S2/S3) alpha_w x f_n, stability/performance, stress.

Output: experiments/output/06d_diagnosis/diagnosis.json
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

import sensorless_eval as E  # noqa: E402
import chain as C  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.estimation import QPLL, QPLLParameters  # noqa: E402
from pmsm_sensorless.filters.dsogi_fll import DSOGIFLL, DSOGIFLLParameters  # noqa: E402
from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters, run_drive  # noqa: E402

TWO_PI = 2.0 * math.pi
TS = 100e-6
GAMMA = 50.0
EPS_GUARD = 1e-12


def _qpll_params(stack, f_n):
    return QPLLParameters.from_design(zeta=0.707, f_n_hz=f_n, amplitude_v=stack.design_amp_v)


# --- arm FLL variants (experiment-local subclasses) --------------------------------


class OracleDSOGI(DSOGIFLL):
    """DSOGI whose omega' is prescribed (FLL bypassed) — isolates SOGI vs FLL."""

    def __init__(self, parameters: DSOGIFLLParameters) -> None:
        super().__init__(parameters)
        self.omega_override: float | None = None

    def step(self, v_alpha, v_beta, dt):
        if self.omega_override is not None:
            self._omega = float(self.omega_override)
        out = super().step(v_alpha, v_beta, dt)
        omega = self._omega if self.omega_override is None else float(self.omega_override)
        return dataclasses.replace(out, omega=omega, frequency_hz=omega / TWO_PI)


class NormalizedDSOGI(DSOGIFLL):
    """FLL with gain normalisation (ASSUMPTION; algebra not printed, Fig.5 block only)."""

    def step(self, v_alpha, v_beta, dt):
        omega_prev = self._omega
        out = super().step(v_alpha, v_beta, dt)
        if self.parameters.normalize_error:
            return out  # library normalisation path already applied; do not double
        denom = (
            out.v_alpha_prime**2
            + out.qv_alpha**2
            + out.v_beta_prime**2
            + out.qv_beta**2
            + EPS_GUARD
        )
        omega = omega_prev - GAMMA * omega_prev * out.epsilon_f * dt / denom
        if self.parameters.min_omega is not None:
            omega = max(omega, self.parameters.min_omega)
        if self.parameters.max_omega is not None:
            omega = min(omega, self.parameters.max_omega)
        self._omega = omega
        return dataclasses.replace(out, omega=omega, frequency_hz=omega / TWO_PI)


class RawEstimatorSource(C.EstimatorSource):
    """Raw E_hat -> Q-PLL (no DSOGI-FLL) — diagnostic arm 1."""

    def __call__(self, sample, truth):  # noqa: ARG002
        s = self.observer.step(
            sample.u_alpha, sample.u_beta, sample.i_alpha, sample.i_beta, sample.dt
        )
        self.last_freq_hz = math.nan
        q = self.qpll.step(s.back_emf_alpha, s.back_emf_beta, sample.dt)
        self.theta_hat, self.omega_hat = q.theta_hat, q.omega_hat
        return q.theta_hat, q.omega_hat


def make_fll(arm, stack, *, initial_omega, min_omega, oracle=False):
    params = DSOGIFLLParameters(C.K_S, GAMMA, initial_omega, min_omega=min_omega)
    if arm == 2 or oracle:
        return OracleDSOGI(params)
    if arm == 3:
        return NormalizedDSOGI(params)
    return DSOGIFLL(params)


def make_source(arm, stack, *, f_n, initial_omega, min_omega):
    observer = C.build_observer("matched_sigmoid", stack)
    qpll = QPLL(_qpll_params(stack, f_n))
    if arm == 1:
        return RawEstimatorSource(
            observer=observer, fll=None, qpll=qpll, pole_pairs=stack.pole_pairs
        )
    fll = make_fll(arm, stack, initial_omega=initial_omega, min_omega=min_omega)
    return C.EstimatorSource(observer=observer, fll=fll, qpll=qpll, pole_pairs=stack.pole_pairs)


# --- Step 1: open-loop T_est ------------------------------------------------------


def _rollout(plant, *, i_q, dt, duration, rpm0, eps, f_m):
    from pmsm_sensorless.plant.pmsm import PMSMState

    p = plant.parameters
    steps = int(round(duration / dt))
    t = np.arange(steps) * dt
    omega_m = rpm0 * (1.0 + eps * np.sin(TWO_PI * f_m * t))
    omega_e = p.pole_pairs * omega_m * TWO_PI / 60.0
    out = {
        k: np.empty(steps) for k in ("u_alpha", "u_beta", "i_alpha", "i_beta", "theta_e", "omega_e")
    }
    i_ref0 = i_q * np.array([-math.sin(0.0), math.cos(0.0)])
    state = PMSMState(float(i_ref0[0]), float(i_ref0[1]), float(omega_e[0]), 0.0)
    for n in range(steps):
        state.omega_e = float(omega_e[n])
        theta = state.theta_e
        i_ref = i_q * np.array([-math.sin(theta), math.cos(theta)])
        di_ref = i_q * state.omega_e * np.array([-math.cos(theta), -math.sin(theta)])
        emf = plant.back_emf(theta, state.omega_e)
        u = p.stator_resistance * i_ref + p.ld * di_ref + emf
        out["u_alpha"][n], out["u_beta"][n] = u
        out["i_alpha"][n], out["i_beta"][n] = state.i_alpha, state.i_beta
        out["theta_e"][n], out["omega_e"][n] = theta, state.omega_e
        state = plant.step(state, float(u[0]), float(u[1]), dt)
    out["t"], out["omega_m"] = t, omega_m
    return out


def _fit_gain_phase(t, x, y, f_m, mask):
    w = TWO_PI * f_m
    dt = float(t[1] - t[0])
    e = np.exp(-1j * w * t[mask])
    ax = 2.0 * dt * np.sum(x[mask] * e)
    ay = 2.0 * dt * np.sum(y[mask] * e)
    T = ay / ax if abs(ax) > 0 else complex("nan")
    return float(abs(T)), float(math.degrees(np.angle(T)))


def open_loop_arm(arm, stack, plant, rpm, fm, eps=0.005):
    duration = 0.25 + max(2.0 / fm, 8.0 / fm)
    roll = _rollout(plant, i_q=stack.i_rated, dt=TS, duration=duration, rpm0=rpm, eps=eps, f_m=fm)
    observer = C.build_observer("matched_sigmoid", stack)
    init_omega = stack.initial_omega_at(rpm)
    if arm == 1:
        filt = None
    elif arm == 2:
        filt = make_fll(2, stack, initial_omega=init_omega, min_omega=None)
    else:
        filt = make_fll(arm, stack, initial_omega=init_omega, min_omega=None)
    qpll = QPLL(_qpll_params(stack, 20.0))
    n = len(roll["t"])
    om = np.empty(n)
    for k in range(n):
        s = observer.step(
            roll["u_alpha"][k], roll["u_beta"][k], roll["i_alpha"][k], roll["i_beta"][k], TS
        )
        if arm == 1:
            va, vb = s.back_emf_alpha, s.back_emf_beta
        else:
            if arm == 2:
                filt.omega_override = roll["omega_e"][k]
            f = filt.step(s.back_emf_alpha, s.back_emf_beta, TS)
            va, vb = f.v_alpha_prime, f.v_beta_prime
        om[k] = qpll.step(va, vb, TS).omega_hat
    mask = roll["t"] >= 0.25
    x = roll["omega_m"] - np.mean(roll["omega_m"][mask])
    y = om / stack.pole_pairs * 60.0 / TWO_PI - np.mean(om[mask] / stack.pole_pairs * 60.0 / TWO_PI)
    return _fit_gain_phase(roll["t"], x, y, fm, mask)


def _interp(freqs, values, f):
    return float(np.interp(math.log10(f), [math.log10(x) for x in freqs], values))


def _minus3db(freqs, gains):
    for i in range(len(freqs) - 1):
        floor = 1.0 / math.sqrt(2.0)
        if (gains[i] - floor) * (gains[i + 1] - floor) <= 0 and gains[i] != gains[i + 1]:
            lo, hi = math.log10(freqs[i]), math.log10(freqs[i + 1])
            return float(10 ** (lo + (hi - lo) * (gains[i] - floor) / (gains[i] - gains[i + 1])))
    return None


# --- Step 2: closed loop ----------------------------------------------------------


class OracleRecorder(E.ChainRecorder):
    """Diagnostic recorder that also feeds the true omega_e to the oracle FLL."""

    def __call__(self, sample, truth):
        self.chain.fll.omega_override = truth.omega_e
        return super().__call__(sample, truth)


def closed_loop(arm, stack, params, plant, spec, *, alpha_w, f_n, advance=False):
    assump = C.handover_assumptions(stack)
    prm = dataclasses.replace(params, alpha_w_hz=alpha_w)
    min_omega = C.fll_min_omega_rad_s(stack)
    chain_source = make_source(
        arm,
        stack,
        f_n=f_n,
        initial_omega=stack.initial_omega_at(assump.omega_h_rpm),
        min_omega=min_omega,
    )
    RecClass = OracleRecorder if arm == 2 else E.ChainRecorder
    rec = RecClass(chain_source, pole_pairs=stack.pole_pairs, assumptions=assump)
    res = run_drive(
        plant,
        prm,
        duration=spec["duration"],
        speed_ref_rpm=spec["speed_ref"],
        load_torque=spec["load"],
        source=rec,
        angle_advance=advance,
    )
    m = E.estimate_metrics(rec, chain_source, res, dynamic=spec["dynamic"], steady=spec["steady"])
    if spec["disturbance_s"] is not None:
        m.update(E.speed_metrics(stack, res, {"disturbance_s": spec["disturbance_s"]}))
    m["minimum_speed_rpm"] = float(np.min(res.omega_m) * 60.0 / TWO_PI)
    return m


class _Status:
    status = "ok"
    first_guard_fail_s = None


def _stable(m):
    return bool(
        m["handed_over"]
        and m["fll_status"] == "ok"
        and (m["peak_theta_err_deg"] is not None and m["peak_theta_err_deg"] < 30.0)
        and m["saturation_after_transient"] == 0.0
    )


def main() -> int:
    stack, params, plant, rule = E.load()
    scenarios = E.paper_scenarios(rule)
    freqs = (2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 50.0, 80.0, 120.0, 200.0)
    arms = {
        0: "nominal chain B",
        1: "chain A raw E_hat (DIAGNOSTIC)",
        2: "ORACLE omega' (DIAGNOSTIC)",
        3: "normalised FLL (DIAGNOSTIC)",
    }

    # ---- Step 1: open-loop T_est
    t_est = {}
    for arm in (0, 1, 2, 3):
        for rpm in (250.0, 500.0, 1000.0):
            gains, phases = [], []
            for fm in freqs:
                g, p = open_loop_arm(arm, stack, plant, rpm, fm)
                gains.append(g)
                phases.append(p)
            u = np.degrees(np.unwrap(np.radians(phases)))
            t_est[f"arm{arm}|rpm{int(rpm)}"] = {
                "gain_vs_fm": dict(zip((f"{f:g}" for f in freqs), [round(g, 4) for g in gains])),
                "phase_unwrapped_vs_fm": dict(
                    zip((f"{f:g}" for f in freqs), [round(x, 2) for x in u])
                ),
                "minus3db_hz": _minus3db(list(freqs), gains),
                "phase_at_25Hz": _interp(list(freqs), u, 25.0),
                "phase_at_50Hz": _interp(list(freqs), u, 50.0),
            }

    # ---- Step 2: closed-loop grid
    grid = {}
    for arm in (0, 1, 2, 3):
        for scenario in ("S2_sim_50pct_1000", "S3_exp_20pct_1000"):
            spec = scenarios[scenario]
            for aw in (2.5, 5.0, 10.0, 20.0, 40.0):
                for fn in (5.0, 10.0, 20.0, 40.0):
                    m = closed_loop(arm, stack, params, plant, spec, alpha_w=aw, f_n=fn)
                    grid[f"arm{arm}|{scenario}|aw{aw:g}|fn{fn:g}"] = {
                        "stable": _stable(m),
                        "peak_theta_err_deg": m["peak_theta_err_deg"],
                        "dynamic_omega_err_max": m["dynamic_omega_err"]["max"],
                        "dip_rpm": m.get("dip_rpm"),
                        "recovery_10rpm_s": m.get("recovery_10rpm_s"),
                        "fll_status": m["fll_status"],
                        "saturation": m["saturation_after_transient"],
                        "minimum_speed_rpm": m["minimum_speed_rpm"],
                    }

    # per-arm summary
    summary = {}
    for arm in (0, 1, 2, 3):
        max_stable = {}
        for fn in (5.0, 10.0, 20.0, 40.0):
            vals = [
                aw
                for aw in (2.5, 5.0, 10.0, 20.0, 40.0)
                if grid[f"arm{arm}|S2_sim_50pct_1000|aw{aw:g}|fn{fn:g}"]["stable"]
                and grid[f"arm{arm}|S3_exp_20pct_1000|aw{aw:g}|fn{fn:g}"]["stable"]
            ]
            max_stable[f"fn{fn:g}"] = max(vals) if vals else None
        best = None
        for aw in (2.5, 5.0, 10.0, 20.0, 40.0):
            for fn in (5.0, 10.0, 20.0, 40.0):
                k = f"arm{arm}|S2_sim_50pct_1000|aw{aw:g}|fn{fn:g}"
                if grid[k]["stable"]:
                    err = grid[k]["dynamic_omega_err_max"]
                    if err is not None and (best is None or err < best["dynamic_omega_err_max"]):
                        best = {
                            "aw": aw,
                            "fn": fn,
                            "dynamic_omega_err_max": err,
                            "s3_err": grid[f"arm{arm}|S3_exp_20pct_1000|aw{aw:g}|fn{fn:g}"][
                                "dynamic_omega_err_max"
                            ],
                        }
        summary[f"arm{arm}"] = {"max_stable_alpha_w_by_fn": max_stable, "best_cell_S2": best}

    # ---- stress per arm at nominal and best stable cell
    stress = {}
    for arm in (0, 1, 2, 3):
        best = summary[f"arm{arm}"]["best_cell_S2"]
        cells = [("nominal", 10.0, 20.0)]
        if best:
            cells.append(("best", best["aw"], best["fn"]))
        for label, aw, fn in cells:
            for name, speed, load, step in (
                ("1000rpm_rated", 1000, StepLoadTorque(0.0, E.RATED_TORQUE, 0.5), 0.5),
                ("500rpm_20", 500, StepLoadTorque(0.0, 0.2 * E.RATED_TORQUE, 0.5), 0.5),
                ("500rpm_50", 500, StepLoadTorque(0.0, 0.5 * E.RATED_TORQUE, 0.5), 0.5),
            ):
                spec = dict(
                    speed_ref=float(speed),
                    load=load,
                    duration=2.0,
                    alpha_w=aw,
                    dynamic=(0.5, 1.0),
                    steady=(1.8, 2.0),
                    disturbance_s=step,
                )
                m = closed_loop(arm, stack, params, plant, spec, alpha_w=aw, f_n=fn)
                stress[f"arm{arm}|{label}|{name}"] = {
                    "stable": _stable(m),
                    "peak_theta_err_deg": m["peak_theta_err_deg"],
                    "minimum_speed_rpm": m["minimum_speed_rpm"],
                    "fll_status": m["fll_status"],
                    "sat": m["saturation_after_transient"],
                    "aw": aw,
                    "fn": fn,
                }

    out = {
        "note": (
            "Phase 06d bandwidth diagnosis. Arm 0 = 06b-6 nominal; arms 1-3 are "
            "DIAGNOSTIC — not paper. Arm 3 normalization is ASSUMPTION (Fig.5 prints "
            "only a labelled block). No src change."
        ),
        "arms": arms,
        "grid": {
            "alpha_w_hz": [2.5, 5, 10, 20, 40],
            "f_n_hz": [5, 10, 20, 40],
            "scenarios": ["S2_sim_50pct_1000", "S3_exp_20pct_1000"],
        },
        "test": t_est,
        "grid_cells": grid,
        "summary": summary,
        "stress": stress,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06d_diagnosis"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "diagnosis.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== Step 1: T_est -3 dB / phase ==")
    for key, v in t_est.items():
        m3 = v["minus3db_hz"]
        print(
            f"  {key:14s} -3dB={'%.1f Hz' % m3 if m3 else '>200 Hz':>9s} "
            f"ph25={v['phase_at_25Hz']:7.1f} ph50={v['phase_at_50Hz']:7.1f}"
        )
    print("\n== Step 2: max stable alpha_w (both S2 & S3) by f_n ==")
    for arm_key, s in summary.items():
        arm = int(arm_key[3:])
        print(f"  arm{arm} {arms[arm]:32s} {s['max_stable_alpha_w_by_fn']}")
        print(f"        best S2 cell: {s['best_cell_S2']}")
    print("\n== stress ==")
    for k, v in stress.items():
        print(
            f"  {k:34s} stable={v['stable']} peak={v['peak_theta_err_deg']} min_speed={v['minimum_speed_rpm']:.0f} fll={v['fll_status']}"
        )
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
