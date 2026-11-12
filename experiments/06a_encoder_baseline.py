#!/usr/bin/env python3
"""Phase 06a: closed-loop drive with encoder (true-angle) feedback.

Runs the paper's class-B scenarios (from ``docs/estimator_spec.md``) plus a speed
step, rated-load steps at 1000/500 rpm and the no-load variable-speed ramp, all on
the encoder baseline (no observer/filter/estimator). Reports speed dip/overshoot,
recovery times, peak i_q, i_d error and the voltage-limit saturation fraction, with
the paper's printed values beside ours. A 3x3 sensitivity on the current-loop and
speed-loop bandwidths is run on the headline rated-load step.

Each load/ramp scenario runs one *continuous* rollout with a short settle period
before the disturbance (so the start-up transient and the controller integral state
are handled correctly) and is analysed only after the disturbance. The speed step
starts from rest (its own transient is the scenario). The speed figures are *control*
dips, not the paper's estimator errors.

Design (all ASSUMPTIONS; D-047): V_dc = 311 V, one-sample delay, alpha_c = 2pi 500 Hz
(Kp_c = alpha_c Ld, Ki_c = alpha_c Rs), alpha_w = 2pi 50 Hz, zeta = 1
(Kp_w = 2 zeta alpha_w J/Kt, Ki_w = alpha_w^2 J/Kt), Kt = (3/2) p psi_f, b = 0,
i_q* clamp = 2 I_rated, decoupling ON, back-EMF feed-forward OFF, 4 RK4 sub-steps.

Output: experiments/output/06a_encoder_baseline/encoder_baseline.json
"""

from __future__ import annotations

import dataclasses
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

try:
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_DIR))

import config_diff  # noqa: E402
from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.plant.load_models import ConstantLoadTorque, StepLoadTorque  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters, PMSMState  # noqa: E402
from pmsm_sensorless.simulation.drive import DriveParameters, run_drive  # noqa: E402

TWO_PI = 2.0 * math.pi
RPM_TO_RAD_S = TWO_PI / 60.0
RATED_TORQUE = 7.7
GRID_MULTS = (0.5, 1.0, 2.0)


@dataclass(frozen=True)
class Scenario:
    """One drive scenario definition (times are scenario-relative, tau >= 0)."""

    name: str
    kind: str  # "speed_step" | "load_step" | "ramp"
    settle_s: float
    duration: float
    reference: float | Callable[[float], float]
    load: Callable[[float], float]
    initial_rpm: float
    disturbance_s: float
    window_s: tuple[float, float]
    paper_value: str
    note: str = ""


def _ramp(t: float) -> float:
    return 500.0 + min(t / 1.5, 1.0) * 500.0


def _scenarios() -> list[Scenario]:
    return [
        Scenario(
            "speed_step_0_1000",
            "speed_step",
            0.0,
            1.0,
            1000.0,
            ConstantLoadTorque(0.0),
            0.0,
            0.0,
            (0.8, 1.0),
            "n/a (not a paper scenario)",
            "0 -> 1000 rpm step, no load",
        ),
        Scenario(
            "load_step_rated_1000",
            "load_step",
            0.1,
            1.0,
            1000.0,
            StepLoadTorque(0.0, RATED_TORQUE, 0.5),
            1000.0,
            0.5,
            (0.8, 1.0),
            "n/a",
            "HEADLINE: 0 -> rated load step at 1000 rpm",
        ),
        Scenario(
            "load_step_rated_500",
            "load_step",
            0.1,
            1.5,
            500.0,
            StepLoadTorque(0.0, RATED_TORQUE, 0.5),
            500.0,
            0.5,
            (1.3, 1.5),
            "n/a",
            "0 -> rated load step at 500 rpm",
        ),
        Scenario(
            "sim_load_50pct_1000",
            "load_step",
            0.1,
            2.0,
            1000.0,
            StepLoadTorque(0.0, 0.5 * RATED_TORQUE, 1.5),
            1000.0,
            1.5,
            (1.8, 2.0),
            "dynamic max speed err 23 rpm; steady mean speed err 5 rpm (Sec. IV-D, p.827)",
            "paper SIM: 1000 rpm, load steps to 50% rated at 1.5 s (Fig.13)",
        ),
        Scenario(
            "exp_load_20pct_1000",
            "load_step",
            0.1,
            1.0,
            1000.0,
            StepLoadTorque(0.0, 0.2 * RATED_TORQUE, 0.5),
            1000.0,
            0.5,
            (0.8, 1.0),
            "speed err ~10 rpm, position err 0.152 rad (Sec. V-D, p.829)",
            "DEVIATION: paper is a hardware experiment; this is a sim replica",
        ),
        Scenario(
            "ramp_500_1000_noload",
            "ramp",
            0.1,
            2.0,
            _ramp,
            ConstantLoadTorque(0.0),
            500.0,
            0.0,
            (1.8, 2.0),
            "max speed err ~5 rpm; steady pos err ~0.07 rad (Sec. IV-C, p.827)",
            "paper sim/exp no-load 500 -> 1000 rpm ramp (Fig.9/11/17/18)",
        ),
    ]


def _loop_margins(p: DriveParameters) -> dict:
    """Discrete open-loop crossover and phase margin of the two loops."""
    ts = p.ts_current
    a = math.exp(-p.stator_resistance * ts / p.ld)
    b = (1.0 - a) / p.stator_resistance

    def current(f):
        z = np.exp(1j * 2 * math.pi * np.asarray(f, dtype=float) * ts)
        return (p.kp_current + p.ki_current * ts / (z - 1)) * b / (z - a) * z**-1

    tsw = p.ts_speed

    def speed(f):  # ZOH half-sample delay (the speed loop applies i_q* in-step)
        z = np.exp(1j * 2 * math.pi * np.asarray(f, dtype=float) * tsw)
        return (
            (p.kp_speed + p.ki_speed * tsw / (z - 1))
            * (p.torque_constant * tsw / p.inertia / (z - 1))
            * z**-0.5
        )

    def cross(fn, lo, hi):
        f = np.linspace(lo, hi, 200000)
        m = np.abs(fn(f)) - 1.0
        k = np.where(np.diff(np.sign(m)) != 0)[0]
        if len(k) == 0:
            return {"crossover_hz": None, "phase_margin_deg": None}
        k = k[0]
        fc = f[k] - m[k] * (f[k + 1] - f[k]) / (m[k + 1] - m[k])
        L = complex(fn(fc))
        return {
            "crossover_hz": float(fc),
            "phase_margin_deg": float(180 + math.degrees(math.atan2(L.imag, L.real))),
        }

    return {
        "current": cross(current, 10.0, 5000.0),
        "speed": cross(speed, 5.0, 900.0),
        "design_alpha_c_hz": p.alpha_c_hz,
        "design_alpha_w_hz": p.alpha_w_hz,
        "note": "current includes the 1-sample computation delay; speed includes a ZOH half-sample delay",
    }


def _run(p: DriveParameters, plant: PMSMPlant, sc: Scenario) -> dict:
    settle = sc.settle_s

    def ref_abs(t: float) -> float:
        tau = max(t - settle, 0.0)
        return sc.reference(tau) if callable(sc.reference) else float(sc.reference)

    def load_abs(t: float) -> float:
        return sc.load(max(t - settle, 0.0))

    total = settle + sc.duration
    res = run_drive(
        plant,
        p,
        duration=total,
        speed_ref_rpm=ref_abs,
        load_torque=load_abs,
        initial_state=PMSMState(0.0, 0.0, 4.0 * sc.initial_rpm * RPM_TO_RAD_S, 0.0),
    )
    t = res.t
    err = (res.omega_m - res.omega_m_ref) * 60.0 / TWO_PI
    ref_rpm = float(np.mean(np.asarray(res.omega_m_ref) * 60.0 / TWO_PI))
    d0 = settle + sc.disturbance_s  # disturbance instant
    after = t >= d0
    win = (t >= settle + sc.window_s[0]) & (t <= settle + sc.window_s[1])

    def recovery(threshold):
        bad = np.where(np.abs(err[after]) > threshold)[0]
        if len(bad) == 0:
            return 0.0
        idx = bad[-1] + 1
        if idx >= int(after.sum()):
            return None
        return float(t[np.where(after)[0][idx]] - d0)

    entry = {
        "scenario": sc.name,
        "kind": sc.kind,
        "peak_i_q_a": float(np.max(np.abs(res.i_q[after])))
        if sc.kind != "speed_step"
        else float(np.max(np.abs(res.i_q))),
        "i_d_error_max_pct": float(100.0 * np.max(np.abs(res.i_d)) / (p.i_q_clamp / 2.0)),
        "i_d_error_settled_pct": float(100.0 * np.max(np.abs(res.i_d[win])) / (p.i_q_clamp / 2.0)),
        "saturation_fraction": float(np.mean(res.saturated)),
        "saturation_fraction_settled": float(np.mean(res.saturated[win])),
        "peak_voltage_v": float(np.max(res.u_cmd_magnitude)),
        "v_limit_v": res.v_limit,
        "steady_speed_err_rpm": float(np.mean(err[win])),
        "steady_i_q_a": float(np.mean(res.i_q[win])),
        "note": sc.note,
        "paper_value": sc.paper_value,
    }
    if sc.kind == "speed_step":
        rpm = np.asarray(res.omega_m) * 60.0 / TWO_PI
        reach = np.where(rpm >= 0.9 * ref_rpm)[0]
        settle_idx = np.where(np.abs(err) > 0.01 * ref_rpm)[0]
        entry.update(
            {
                "speed_dip_rpm": None,
                "speed_overshoot_rpm": float(np.max(err)),
                "speed_overshoot_pct": float(100.0 * np.max(err) / ref_rpm),
                "t_rise_90pct_s": float(t[reach[0]]) if len(reach) else None,
                "settle_to_1pct_s": float(t[settle_idx[-1] + 1])
                if len(settle_idx) and settle_idx[-1] + 1 < len(t)
                else 0.0,
                "recovery_to_10rpm_s": None,
                "recovery_to_1pct_s": None,
            }
        )
        first = np.where(np.abs(err) <= 0.01 * ref_rpm)[0]
        if len(first):
            entry["speed_overshoot_after_reach_rpm"] = float(np.max(err[t >= t[first[0]]]))
    else:
        entry.update(
            {
                "speed_dip_rpm": float(np.min(err[after])),
                "speed_overshoot_rpm": float(np.max(err[after])),
                "speed_dip_pct": float(100.0 * np.min(err[after]) / ref_rpm) if ref_rpm else None,
                "recovery_to_10rpm_s": recovery(10.0),
                "recovery_to_1pct_s": recovery(0.01 * ref_rpm),
            }
        )
    return entry


def _stable(entry: dict) -> bool:
    rec = entry["recovery_to_10rpm_s"]
    return (
        rec is not None
        and abs(entry["steady_speed_err_rpm"]) < 5.0
        and abs(entry["speed_overshoot_rpm"]) < 200.0
        and abs(entry["speed_dip_rpm"]) < 500.0
    )


def _fmt(value) -> str:
    return "None" if value is None else f"{value:.4f}"


def main() -> int:
    directory = config.default_config_dir()
    pmsm_cfg = config.load_config("pmsm", directory=directory)
    drive_cfg = config.load_config("drive", directory=directory)
    assumptions_cfg = config.load_config("drive_assumptions", directory=directory)
    base = DriveParameters.from_configs(drive_cfg, assumptions_cfg, pmsm_cfg)

    def make_plant(b):
        return PMSMPlant(
            PMSMParameters.from_config(pmsm_cfg), integrate_mechanics=True, viscous_friction=b
        )

    scenarios = _scenarios()
    results = {sc.name: _run(base, make_plant(base.viscous_friction), sc) for sc in scenarios}

    headline = next(sc for sc in scenarios if sc.name == "load_step_rated_1000")
    sensitivity: dict[str, dict] = {}
    for mc in GRID_MULTS:
        for mw in GRID_MULTS:
            cell = dataclasses.replace(
                base, alpha_c_hz=base.alpha_c_hz * mc, alpha_w_hz=base.alpha_w_hz * mw
            )
            row = _run(cell, make_plant(cell.viscous_friction), headline)
            row.update(
                {
                    "alpha_c_mult": mc,
                    "alpha_w_mult": mw,
                    "alpha_c_hz": cell.alpha_c_hz,
                    "alpha_w_hz": cell.alpha_w_hz,
                    "stable": _stable(row),
                }
            )
            sensitivity[f"ac{mc:g}_aw{mw:g}"] = row

    output = {
        "note": (
            "Phase 06a encoder baseline (true angle/speed fed back); no observer/filter/"
            "estimator. All drive gains/options are ASSUMPTIONS (D-047); no tuning toward "
            "the paper's numbers. The speed figures are control dips, not the paper's "
            "estimator errors."
        ),
        "assumptions": {
            "v_dc_v": base.v_dc,
            "v_limit_v": base.v_limit,
            "alpha_c_hz": base.alpha_c_hz,
            "alpha_w_hz": base.alpha_w_hz,
            "zeta_speed": base.zeta_speed,
            "decoupling": base.decoupling,
            "back_emf_compensation": base.back_emf_compensation,
            "delay_samples": base.delay_samples,
            "n_substeps": base.n_substeps,
            "viscous_friction_b": base.viscous_friction,
            "i_q_clamp_a": base.i_q_clamp,
            "torque_constant": base.torque_constant,
            "kp_current": base.kp_current,
            "ki_current": base.ki_current,
            "kp_speed": base.kp_speed,
            "ki_speed": base.ki_speed,
        },
        "loop_margins": _loop_margins(base),
        "config_diff": config_diff.summary(
            "06a_encoder_baseline",
            {
                "epsilon_filter_tau": None,
                "min_omega": None,
                "max_omega": None,
                "normalize_error": False,
                "positive_sequence": True,
            },
            scenario={
                "ts_us": base.ts_current * 1e6,
                "scenarios": [s.name for s in scenarios],
                "sensitivity_cells": len(sensitivity),
            },
        ),
        "scenarios": results,
        "sensitivity": sensitivity,
        "speed_reversal": "not run: docs/estimator_spec.md lists no reversal scenario",
    }

    out_dir = config.find_project_root() / "experiments" / "output" / "06a_encoder_baseline"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "encoder_baseline.json").write_text(json.dumps(output, indent=2), encoding="utf-8")
    config_diff.emit(output["config_diff"])

    m = output["loop_margins"]
    print("\n== discrete loop margins ==")
    print(
        f"  current: crossover {m['current']['crossover_hz']:.1f} Hz, PM {m['current']['phase_margin_deg']:.1f} deg"
        f"  (design alpha_c = {m['design_alpha_c_hz']:.0f} Hz)"
    )
    print(
        f"  speed:   crossover {m['speed']['crossover_hz']:.1f} Hz, PM {m['speed']['phase_margin_deg']:.1f} deg"
        f"  (design alpha_w = {m['design_alpha_w_hz']:.0f} Hz)"
    )
    print("\n== scenarios ==")
    print(
        f"  {'scenario':22s} {'dip':>8s} {'ovs':>8s} {'rec10':>8s} {'rec1%':>8s} "
        f"{'pkIq':>7s} {'id%':>6s} {'sat':>6s}"
    )
    for sc in scenarios:
        r = results[sc.name]
        dip = r["speed_dip_rpm"]
        print(
            f"  {sc.name:22s} {('%.2f' % dip) if dip is not None else '     n/a':>8s} "
            f"{r['speed_overshoot_rpm']:8.2f} {_fmt(r['recovery_to_10rpm_s']):>8s} "
            f"{_fmt(r['recovery_to_1pct_s']):>8s} {r['peak_i_q_a']:7.2f} "
            f"{r['i_d_error_settled_pct']:6.2f} {r['saturation_fraction']:6.3f}"
        )
    print(
        f"  speed_step: t_rise_90% = {results['speed_step_0_1000']['t_rise_90pct_s']:.4f} s, "
        f"settle 1% = {_fmt(results['speed_step_0_1000']['settle_to_1pct_s'])} s, "
        f"overshoot = {results['speed_step_0_1000']['speed_overshoot_pct']:.2f} %"
    )
    print("\n== sensitivity (headline: rated-load step @1000 rpm) ==")
    for key, r in sensitivity.items():
        print(
            f"  {key:12s} ac={r['alpha_c_hz']:6.0f} aw={r['alpha_w_hz']:4.0f} "
            f"dip={r['speed_dip_rpm']:8.2f} rec10={_fmt(r['recovery_to_10rpm_s']):>8s} "
            f"pkIq={r['peak_i_q_a']:7.2f} sat={r['saturation_fraction']:.3f} stable={r['stable']}"
        )
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
