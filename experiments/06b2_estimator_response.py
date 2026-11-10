#!/usr/bin/env python3
"""Phase 06b-2 Step 1: measured estimator response T_est(j w) for the speed loop.

Open-loop nominal-drive scenario (04c ideal-FOC feedforward generator, I_q = I_rated)
with a prescribed sinusoidal speed ``omega_e(t) = omega_0 (1 + eps sin(2 pi f_m t))``,
eps = 0.5 %, omega_0 in {500, 1000} rpm, f_m in {2 .. 200} Hz, Ts = 100 us, matched
sigmoid cell. The chain sees (applied voltage, current); we fit the complex ratio
``omega_hat_m / omega_m`` (gain, phase) over a window after the settling interval
(>= 8 modulation periods).

Chains: A = raw E_hat -> Q-PLL (no filter), f_n in {20, 200} Hz;
        B = DSOGI-FLL v' -> Q-PLL (paper-literal), f_n in {10, 20, 50, 200} Hz.
All Q-PLL f_n are experiment-level overrides of the frozen 05b design (recorded).

Hypotheses (stated before running):
  H1: the DSOGI-FLL envelope caps T_est, so raising f_n in chain B does not widen it.
      (envelope time constant ~ 2/(k_s w') = 3.4 ms at 1000 rpm -> corner ~ 47 Hz,
       proportional to speed)
  H2: chain A (no filter) is markedly wider than chain B at equal f_n.

Output: experiments/output/06b2_estimator_response/estimator_response.json
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
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.estimation import QPLL, QPLLParameters, wrap_angle_pi_closed  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters, PMSMState  # noqa: E402

TWO_PI = 2.0 * math.pi
RPM_TO_RAD_S = TWO_PI / 60.0
TS = 100e-6
EPS = 0.005
SETTLE_S = 0.25
MIN_PERIODS = 8
SPEEDS = (500.0, 1000.0)
FREQS = (2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 50.0, 80.0, 120.0, 200.0)
CHAIN_A_FN = (20.0, 200.0)
CHAIN_B_FN = (10.0, 20.0, 50.0, 200.0)


def _rollout(plant: PMSMPlant, *, i_q: float, dt: float, duration: float,
             rpm0: float, eps: float, f_m: float) -> dict:
    """04c ideal-FOC feedforward with a prescribed sinusoidal mechanical speed."""
    p = plant.parameters
    steps = int(round(duration / dt))
    t = np.arange(steps) * dt
    omega_m = rpm0 * (1.0 + eps * np.sin(TWO_PI * f_m * t))          # mechanical rpm
    omega_e = p.pole_pairs * omega_m * RPM_TO_RAD_S
    out = {k: np.empty(steps) for k in ("u_alpha", "u_beta", "i_alpha", "i_beta", "e_alpha", "e_beta", "theta_e", "omega_e")}
    i_ref = i_q * np.array([-math.sin(0.0), math.cos(0.0)])
    state = PMSMState(float(i_ref[0]), float(i_ref[1]), float(omega_e[0]), 0.0)
    for n in range(steps):
        state.omega_e = float(omega_e[n])
        theta = state.theta_e
        i_ref = i_q * np.array([-math.sin(theta), math.cos(theta)])
        di_ref = i_q * state.omega_e * np.array([-math.cos(theta), -math.sin(theta)])
        emf = plant.back_emf(theta, state.omega_e)
        u = p.stator_resistance * i_ref + p.ld * di_ref + emf
        out["u_alpha"][n], out["u_beta"][n] = u
        out["i_alpha"][n], out["i_beta"][n] = state.i_alpha, state.i_beta
        out["e_alpha"][n], out["e_beta"][n] = emf
        out["theta_e"][n], out["omega_e"][n] = theta, state.omega_e
        state = plant.step(state, float(u[0]), float(u[1]), dt)
    out["t"], out["omega_m"] = t, omega_m
    return out


def _observer_fll(stack: C.Stack, plant: PMSMPlant, roll: dict, *, rpm0: float):
    """Deterministic observer + DSOGI-FLL pass (shared across Q-PLL f_n)."""
    observer = C.build_observer("matched_sigmoid", stack)
    fll = C.build_fll(stack.initial_omega_at(rpm0))
    n = len(roll["t"])
    ea, eb, va, vb, freq = (np.empty(n) for _ in range(5))
    for k in range(n):
        s = observer.step(roll["u_alpha"][k], roll["u_beta"][k], roll["i_alpha"][k], roll["i_beta"][k], TS)
        ea[k], eb[k] = s.back_emf_alpha, s.back_emf_beta
        f = fll.step(ea[k], eb[k], TS)
        va[k], vb[k], freq[k] = f.v_alpha_prime, f.v_beta_prime, f.frequency_hz
    return ea, eb, va, vb, freq


def _qpll(stack: C.Stack, va, vb, dt: float, *, f_n: float):
    params = QPLLParameters.from_design(zeta=0.707, f_n_hz=f_n, amplitude_v=stack.design_amp_v)
    q = QPLL(params, initial_theta=0.0, initial_omega=0.0)
    omega = np.empty(len(va))
    for k in range(len(va)):
        omega[k] = q.step(float(va[k]), float(vb[k]), dt).omega_hat
    return omega


def _fit(t, x, y, f_m, mask) -> dict:
    w = TWO_PI * f_m
    e = np.exp(-1j * w * t[mask])
    dt = float(t[1] - t[0])
    ax = 2.0 * dt * np.sum(x[mask] * e)
    ay = 2.0 * dt * np.sum(y[mask] * e)
    T = ay / ax if abs(ax) > 0 else complex("nan")
    return {"gain": float(abs(T)), "phase_deg": float(math.degrees(np.angle(T)))}


def _interp_cross(freqs, gains, floor=1.0 / math.sqrt(2.0)):
    """Frequency where gain crosses ``floor`` (log-log), or None if outside the grid."""
    for i in range(len(freqs) - 1):
        g0, g1 = gains[i], gains[i + 1]
        if (g0 - floor) * (g1 - floor) <= 0 and g0 != g1:
            lo, hi = math.log10(freqs[i]), math.log10(freqs[i + 1])
            f = 10 ** (lo + (hi - lo) * (g0 - floor) / (g0 - g1))
            return float(f)
    return None


def _interp(freqs, values, f):
    lf = [math.log10(x) for x in freqs]
    return float(np.interp(math.log10(f), lf, values))


def main() -> int:
    directory = config.default_config_dir()
    stack = C.build_stack(directory)
    plant = PMSMPlant(PMSMParameters.from_config(stack.pmsm_config))

    rows: dict = {}
    for rpm0 in SPEEDS:
        for f_m in FREQS:
            periods = MIN_PERIODS / f_m
            duration = SETTLE_S + max(2.0 / f_m, periods)
            roll = _rollout(plant, i_q=stack.i_rated, dt=TS, duration=duration,
                            rpm0=rpm0, eps=EPS, f_m=f_m)
            t = roll["t"]
            mask = t >= SETTLE_S
            x = roll["omega_m"] - np.mean(roll["omega_m"][mask])
            ea, eb, va, vb, freq = _observer_fll(stack, plant, roll, rpm0=rpm0)
            entry = {"rpm": rpm0, "f_m": f_m, "duration_s": duration,
                     "window_s": [SETTLE_S, float(t[-1])],
                     "fll_status": "guard-fail" if not np.all(np.isfinite(freq)) else "locked",
                     "e_hat_ripple_v": None}
            chains = {}
            for name, fns, basis in (("A", CHAIN_A_FN, (ea, eb)), ("B", CHAIN_B_FN, (va, vb))):
                for f_n in fns:
                    omega_hat = _qpll(stack, basis[0], basis[1], TS, f_n=f_n)
                    y = omega_hat / stack.pole_pairs * 60.0 / TWO_PI - np.mean(
                        omega_hat[mask] / stack.pole_pairs * 60.0 / TWO_PI)
                    fit = _fit(t, x, y, f_m, mask)
                    chains[f"{name}_fn{f_n:g}"] = fit
            entry["chains"] = chains
            rows[f"rpm{int(rpm0)}_fm{f_m:g}"] = entry

    # ---- linearity check at f_m = 20 Hz (gain within 5 %)
    linearity = {}
    for rpm0 in SPEEDS:
        for eps in (0.0025, 0.005, 0.01):
            roll = _rollout(plant, i_q=stack.i_rated, dt=TS,
                            duration=SETTLE_S + max(2.0 / 20.0, MIN_PERIODS / 20.0),
                            rpm0=rpm0, eps=eps, f_m=20.0)
            mask = roll["t"] >= SETTLE_S
            x = roll["omega_m"] - np.mean(roll["omega_m"][mask])
            _ea, _eb, va, vb, _freq = _observer_fll(stack, plant, roll, rpm0=rpm0)
            omega_hat = _qpll(stack, va, vb, TS, f_n=20.0)
            y = omega_hat / stack.pole_pairs * 60.0 / TWO_PI - np.mean(
                omega_hat[mask] / stack.pole_pairs * 60.0 / TWO_PI)
            linearity[f"rpm{int(rpm0)}_eps{eps:g}"] = _fit(roll["t"], x, y, 20.0, mask)

    # ---- summaries: per chain/speed, gain & phase vs f_m; -3 dB; phase at 25/50 Hz
    summary = {}
    for rpm0 in SPEEDS:
        for chain in ("A_fn20", "A_fn200", "B_fn10", "B_fn20", "B_fn50", "B_fn200"):
            freqs = list(FREQS)
            gains = [rows[f"rpm{int(rpm0)}_fm{f:g}"]["chains"][chain]["gain"] for f in freqs]
            phases = list(np.degrees(np.unwrap(np.radians(
                [rows[f"rpm{int(rpm0)}_fm{f:g}"]["chains"][chain]["phase_deg"] for f in freqs]))))
            summary[f"rpm{int(rpm0)}_{chain}"] = {
                "gain_vs_fm": dict(zip((f"{f:g}" for f in freqs), [round(g, 4) for g in gains])),
                "phase_deg_vs_fm": dict(zip((f"{f:g}" for f in freqs), [round(p, 2) for p in phases])),
                "minus3db_hz": _interp_cross(freqs, gains),
                "phase_at_25Hz_deg": _interp(freqs, phases, 25.0),
                "phase_at_50Hz_deg": _interp(freqs, phases, 50.0),
            }

    out = {
        "note": ("Phase 06b-2 measured estimator response; open-loop nominal drive, matched "
                 "cell; chain A = raw E_hat -> Q-PLL, chain B = DSOGI v' -> Q-PLL. "
                 "All Q-PLL f_n are experiment overrides of the frozen 05b design."),
        "config": {"ts_us": TS * 1e6, "eps": EPS, "settle_s": SETTLE_S, "min_periods": MIN_PERIODS,
                   "speeds_rpm": list(SPEEDS), "freqs_hz": list(FREQS),
                   "chain_a_fn_hz": list(CHAIN_A_FN), "chain_b_fn_hz": list(CHAIN_B_FN),
                   "i_q_a": stack.i_rated, "frozen_f_n_hz": 20.0},
        "hypotheses": {
            "H1": ("the DSOGI envelope (tau ~ 2/(k_s w') = 3.4 ms at 1000 rpm, corner ~47 Hz, "
                   "proportional to speed) caps T_est, so raising f_n in chain B does not widen it"),
            "H2": "chain A (no filter) is markedly wider than chain B at equal f_n",
        },
        "rows": rows, "summary": summary, "linearity": linearity,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b2_estimator_response"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "estimator_response.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print("== T_est gain / phase (matched cell, eps=0.5 %) ==")
    for key, s in summary.items():
        m3 = s["minus3db_hz"]
        print(f"  {key:16s} -3dB={'%.1f Hz' % m3 if m3 else '>200 Hz':>8s} "
              f"ph(25Hz)={s['phase_at_25Hz_deg']:7.1f}  ph(50Hz)={s['phase_at_50Hz_deg']:7.1f}")
    print("\n== gain vs f_m (1000 rpm) ==")
    for chain in ("A_fn20", "A_fn200", "B_fn20", "B_fn200"):
        s = summary[f"rpm1000_{chain}"]
        print(f"  {chain:8s} " + " ".join(
            f"{k}:{v:.2f}" for k, v in s["gain_vs_fm"].items()))
    print("\n== linearity (f_m=20 Hz) ==")
    for key, v in linearity.items():
        print(f"  {key:18s} gain={v['gain']:.3f} phase={v['phase_deg']:+.2f} deg")
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
