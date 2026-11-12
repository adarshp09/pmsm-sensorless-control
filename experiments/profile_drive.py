#!/usr/bin/env python3
"""Phase 06g-2 prescribed transient profiles and chain comparison.

The plant is already a prescribed-kinematics plant: the caller supplies
``state.omega_e`` before each step. This experiment-side generator therefore
adds no ``src`` API and leaves ``nominal_drive_rollout`` unchanged.
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
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
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMState  # noqa: E402
from pmsm_sensorless.simulation.nominal_drive import _SpeedProfile  # noqa: E402

TWO_PI = 2.0 * math.pi
RPM_TO_RAD_S = TWO_PI / 60.0
DT = 1.0e-4
T0 = 0.30
DURATION = 1.50
TAU_TOL = 0.40
PAPER = {"S2": {"speed_rpm": 23.0, "position_rad": 0.22},
         "S3": {"speed_rpm": 10.0, "position_rad": 0.152}}


@dataclass(frozen=True)
class ProfileResult:
    t: np.ndarray
    u_alpha: np.ndarray
    u_beta: np.ndarray
    i_alpha: np.ndarray
    i_beta: np.ndarray
    theta_e: np.ndarray
    omega_e: np.ndarray


class TransientProfile:
    """1000 rpm transient with peak deceleration ``a0`` at ``t0 + tau_d``."""

    def __init__(self, *, drop_rpm: float, a0_rpm_s: float, t0: float = T0):
        self.drop_rpm = drop_rpm
        self.a0_rpm_s = a0_rpm_s
        self.t0 = t0
        self.tau_d = drop_rpm * math.e / a0_rpm_s

    def rpm(self, t: float) -> float:
        if t < self.t0:
            return 1000.0
        x = (t - self.t0) / self.tau_d
        return 1000.0 - self.drop_rpm * x * math.exp(1.0 - x)


def _rollout(plant: PMSMPlant, profile, *, i_q: float, dt: float,
             duration: float) -> ProfileResult:
    steps = int(round(duration / dt))
    t = np.arange(steps) * dt
    names = ("u_alpha", "u_beta", "i_alpha", "i_beta", "theta_e", "omega_e")
    out = {name: np.empty(steps) for name in names}
    p = plant.parameters
    omega = profile.rpm(0.0) * p.pole_pairs * RPM_TO_RAD_S
    state = PMSMState(0.0, i_q, omega, 0.0)
    state.i_alpha, state.i_beta = (
        i_q * -math.sin(state.theta_e), i_q * math.cos(state.theta_e)
    )
    for n, now in enumerate(t):
        omega = profile.rpm(float(now)) * p.pole_pairs * RPM_TO_RAD_S
        state.omega_e = omega
        theta = state.theta_e
        i_ref = i_q * np.array([-math.sin(theta), math.cos(theta)])
        di_ref = i_q * omega * np.array([-math.cos(theta), -math.sin(theta)])
        emf = plant.back_emf(theta, omega)
        u = p.stator_resistance * i_ref + p.ld * di_ref + emf
        out["u_alpha"][n], out["u_beta"][n] = u
        out["i_alpha"][n], out["i_beta"][n] = state.i_alpha, state.i_beta
        out["theta_e"][n], out["omega_e"][n] = theta, omega
        state = plant.step(state, float(u[0]), float(u[1]), dt)
    return ProfileResult(t=t, **out)


def rollout_transient(plant: PMSMPlant, *, drop_rpm: float, a0_rpm_s: float,
                      i_q: float, dt: float = DT, duration: float = DURATION) -> ProfileResult:
    return _rollout(plant, TransientProfile(drop_rpm=drop_rpm, a0_rpm_s=a0_rpm_s),
                    i_q=i_q, dt=dt, duration=duration)


def regression_linear(stack, plant) -> dict:
    from pmsm_sensorless.simulation.nominal_drive import nominal_drive_rollout

    expected = nominal_drive_rollout(
        plant, i_q=stack.i_rated, dt=DT, duration=0.20,
        rpm_start=500.0, rpm_end=1000.0, ramp_time=0.10,
    )
    actual = _rollout(
        plant, _SpeedProfile(500.0, 1000.0, 0.10, stack.pole_pairs),
        i_q=stack.i_rated, dt=DT, duration=0.20,
    )
    diffs = {
        name: float(np.max(np.abs(getattr(expected, name) - getattr(actual, name))))
        for name in ("u_alpha", "u_beta", "i_alpha", "i_beta", "theta_e", "omega_e")
    }
    return {"pass": bool(max(diffs.values()) == 0.0), "max_abs_diffs": diffs}


def _oracle_fll(stack, omega0):
    return DSOGIFLL(DSOGIFLLParameters(math.sqrt(2.0), 50.0, omega0))


def _replay(stack, rec: ProfileResult, variant: str, f_n: float):
    n = len(rec.t)
    observer = C.build_observer("matched_sigmoid", stack) if variant in ("V1", "V2", "V4") else None
    fll = None
    if variant == "V2":
        fll = C.build_fll(rec.omega_e[0], min_omega=None)
    elif variant == "V4":
        fll = _oracle_fll(stack, rec.omega_e[0])
    qpll = QPLL(QPLLParameters.from_design(
        zeta=0.707, f_n_hz=f_n, amplitude_v=stack.design_amp_v,
    ), initial_theta=float(rec.theta_e[0]), initial_omega=float(rec.omega_e[0]))
    theta_hat = np.empty(n)
    omega_hat = np.empty(n)
    for k in range(n):
        if variant == "V0":
            ea, eb = -rec.omega_e[k] * stack.psi_f * math.sin(rec.theta_e[k]), \
                rec.omega_e[k] * stack.psi_f * math.cos(rec.theta_e[k])
        else:
            s = observer.step(rec.u_alpha[k], rec.u_beta[k], rec.i_alpha[k], rec.i_beta[k], DT)
            ea, eb = s.back_emf_alpha, s.back_emf_beta
        if variant == "V2":
            filt = fll.step(ea, eb, DT)
            ea, eb = filt.v_alpha_prime, filt.v_beta_prime
        elif variant == "V4":
            fll._omega = rec.omega_e[k]
            filt = fll.step(ea, eb, DT)
            ea, eb = filt.v_alpha_prime, filt.v_beta_prime
        q = qpll.step(ea, eb, DT)
        theta_hat[k], omega_hat[k] = q.theta_hat, q.omega_hat
    return theta_hat, omega_hat


def _metrics(rec, theta_hat, omega_hat, *, t0: float, a0_rpm_s: float):
    m = rec.t >= t0
    speed_err = (omega_hat[m] - rec.omega_e[m]) * 60.0 / (TWO_PI * 4)
    angle_err = (theta_hat[m] - rec.theta_e[m] + math.pi) % TWO_PI - math.pi
    first = rec.t[m] <= t0 + 0.05
    window = rec.t[m] <= t0 + 0.20
    first_peak = float(np.max(np.abs(speed_err[first])))
    window_max = float(np.max(np.abs(speed_err[window])))
    peak_theta = float(np.max(np.abs(angle_err[window])))
    peak_speed_rad_s = float(np.max(np.abs(speed_err[window])) * TWO_PI * 4 / 60)
    return {
        "first_peak_rpm": first_peak,
        "window_max_rpm": window_max,
        "peak_theta_rad": peak_theta,
        "r_ms": (1000.0 * peak_theta / peak_speed_rad_s) if peak_speed_rad_s else None,
        "e_over_a0_ms": 1000.0 * window_max / a0_rpm_s,
    }


def main() -> int:
    stack, params, plant, _ = E.load()
    out_dir = config.find_project_root() / "experiments" / "output" / "06g2_profiles"
    out_dir.mkdir(parents=True, exist_ok=True)
    a0_grid = {
        "S2": (190.0, 235.0, 350.0, (1400.0, 2500.0, 5000.0, 10000.0, 24027.0)),
        "S3": (46.0, 46.0, 46.0, (700.0, 1400.0, 2800.0, 5600.0, 9611.0)),
    }
    frozen = {
        "status": "predictions_frozen",
        "criteria": {
            "tolerance_fraction": TAU_TOL,
            "joint_match": "speed and position each within 2x paper value",
            "step0_estimate_fraction": 0.50,
            "chains": ["V0", "V1", "V2", "V4"],
            "dead_time": "OFF",
        },
        "predictions": {
            "V2_tau_eff_ms": {"S2": 5.63, "S3": 4.20},
            "V0_tau_eff_ms": {"S2": 3.62, "S3": 3.44},
            "V0_r_ms": "2.29 / omega_n_eff for tau_d >= 50 ms; smaller for shorter profiles",
        },
        "paper_targets": PAPER,
        "profiles": {},
    }
    frozen["profiles"] = {
        "S2": {"drop_rpm": [190.0, 235.0, 350.0],
               "a0_rpm_s": list(a0_grid["S2"][3])},
        "S3": {"drop_rpm": [46.0], "a0_rpm_s": list(a0_grid["S3"][3])},
    }
    results = []
    for scenario, (first_drop, second_drop, third_drop, a0s) in a0_grid.items():
        drops = (first_drop, second_drop, third_drop) if scenario == "S2" else (46.0,)
        for drop in drops:
            for a0 in a0s:
                rec = rollout_transient(plant, drop_rpm=drop, a0_rpm_s=a0,
                                        i_q=stack.i_rated)
                for variant in ("V0", "V1", "V2", "V4"):
                    fns = (20.0,)
                    if variant in ("V0", "V2") and a0 in (24027.0, 5000.0, 1400.0):
                        fns = (20.0, 80.0)
                    for f_n in fns:
                        th, om = _replay(stack, rec, variant, f_n)
                        tau_prediction = None
                        if variant == "V2":
                            tau_prediction = {"S2": 5.63, "S3": 4.20}[scenario]
                        elif variant in ("V0", "V4"):
                            tau_prediction = {"S2": 3.62, "S3": 3.44}[scenario]
                        results.append({
                            "scenario": scenario, "drop_rpm": drop, "a0_rpm_s": a0,
                            "tau_d_ms": 1000.0 * drop * math.e / a0,
                            "variant": variant, "f_n_hz": f_n,
                            "predicted_tau_eff_ms": tau_prediction,
                            "predicted_e_over_a0_ms": tau_prediction,
                            "predicted_speed_error_rpm": (
                                a0 * tau_prediction / 1000.0
                                if tau_prediction is not None else None
                            ),
                            **_metrics(rec, th, om, t0=T0, a0_rpm_s=a0),
                        })
    closed_loop = []
    for rise_s in (0.1, 0.2):
        def load(t, rise_s=rise_s):
            if t < 0.5:
                return 0.0
            return min((t - 0.5) / rise_s, 1.0) * 0.5 * 7.7

        source, chain_source, result = E.run_sensorless(
            stack, params, plant, speed_ref=1000.0, load_torque=load,
            duration=2.0, alpha_w=10.0, f_n=20.0,
        )
        post = result.t >= 0.5
        speed_rpm = result.omega_m * 60.0 / TWO_PI
        decel = np.gradient(speed_rpm, result.t)
        frozen_target = next(
            r for r in results
            if r["scenario"] == "S2" and r["drop_rpm"] == 235.0
            and r["a0_rpm_s"] == 5000.0 and r["variant"] == "V2"
            and r["f_n_hz"] == 20.0
        )
        closed_loop.append({
            "load_ramp_s": rise_s,
            "true_dip_rpm": float(np.min(speed_rpm[post]) - 1000.0),
            "peak_deceleration_rpm_s": float(np.min(decel[post])),
            "peak_theta_err_deg": float(np.max(np.abs(source.theta_err_deg))),
            "peak_omega_err_rpm": float(np.max(np.abs(source.omega_err_rpm))),
            "fll_status": chain_source.status,
            "handover_s": source.switch_time_s,
            "comparison_profile": {
                "drop_rpm": 235.0, "a0_rpm_s": 5000.0,
                "v2_first_peak_rpm": frozen_target["first_peak_rpm"],
                "v2_peak_theta_rad": frozen_target["peak_theta_rad"],
            },
        })
    frozen["closed_loop_confirmation"] = {
        "assumptions": "nominal chain, alpha_w=10 Hz, t_h=0.30 s, dead time OFF, ramp starts at 0.5 s",
        "rows": closed_loop,
    }
    frozen["status"] = "complete"
    frozen["generator_regression"] = regression_linear(stack, plant)
    frozen["results"] = results
    (out_dir / "profiles.json").write_text(json.dumps(frozen, indent=2), encoding="utf-8")
    print(json.dumps(frozen["generator_regression"], indent=2))
    print(f"Wrote {len(results)} profile rows to {out_dir / 'profiles.json'}")
    return 0 if frozen["generator_regression"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
