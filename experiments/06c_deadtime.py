#!/usr/bin/env python3
"""Phase 06c GATE D: open-loop nominal drive + inverter dead time (matched cell).

Ideal-FOC feedforward at constant speed (500/1000 rpm, I_q = I_rated and 20 % of it);
the **plant** sees u_cmd + dead-time distortion, the **observer** sees u_cmd (primary,
no compensation). Reports the E_hat fundamental amplitude/phase error and the 5th/7th
harmonic ripple, compared with the prediction dE ~ 4/pi * V_dc t_d f_sw = 7.9 V
(14 % / 28 % of |E| at 1000 / 500 rpm).

Output: experiments/output/06c_deadtime/gate_d.json
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
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters, PMSMState  # noqa: E402
from pmsm_sensorless.simulation.drive import (  # noqa: E402
    dead_time_delta_alpha_beta, dead_time_phase_error)
from validation_metrics import vector_metrics  # noqa: E402

TWO_PI = 2.0 * math.pi
TS = 100e-6
V_DC = 311.0
T_D = 2e-6
F_SW = 10000.0
I_TH = 0.1


def _rollout(plant, *, i_q, rpm0, dead_time_s, duration=0.5):
    p = plant.parameters
    steps = int(round(duration / TS))
    t = np.arange(steps) * TS
    omega_e = p.pole_pairs * rpm0 * TWO_PI / 60.0
    names = ("u_alpha", "u_beta", "i_alpha", "i_beta", "e_alpha", "e_beta", "theta_e")
    out = {k: np.empty(steps) for k in names}
    i_ref0 = i_q * np.array([0.0, 1.0])
    state = PMSMState(float(i_ref0[0]), float(i_ref0[1]), float(omega_e), 0.0)
    for n in range(steps):
        state.omega_e = omega_e
        theta = state.theta_e
        i_ref = i_q * np.array([-math.sin(theta), math.cos(theta)])
        di_ref = i_q * omega_e * np.array([-math.cos(theta), -math.sin(theta)])
        emf = plant.back_emf(theta, omega_e)
        u_cmd = p.stator_resistance * i_ref + p.ld * di_ref + emf
        out["u_alpha"][n], out["u_beta"][n] = u_cmd
        out["i_alpha"][n], out["i_beta"][n] = state.i_alpha, state.i_beta
        out["e_alpha"][n], out["e_beta"][n] = emf
        out["theta_e"][n] = theta
        u_plant = u_cmd
        if dead_time_s > 0.0:
            da, db = dead_time_delta_alpha_beta(state.i_alpha, state.i_beta, v_dc=V_DC,
                                                dead_time_s=dead_time_s, switching_hz=F_SW,
                                                current_threshold_a=I_TH)
            u_plant = u_cmd + np.array([da, db])
        state = plant.step(state, float(u_plant[0]), float(u_plant[1]), TS)
    out["t"] = t
    return out


def _harmonics(signal, f0_hz, dt, mask):
    n = int(mask.sum())
    x = signal[mask] - np.mean(signal[mask])
    spec = np.abs(np.fft.rfft(x)) * 2.0 / n
    freqs = np.fft.rfftfreq(n, dt)
    f_lo, f_hi = 10.0, 4000.0

    def amp(f):
        sel = (freqs >= f_lo) & (freqs <= f_hi)
        if not sel.any():
            return 0.0
        i = np.argmin(np.abs(freqs[sel] - f))
        return float(spec[sel][i])

    fund = amp(f0_hz)
    return {"fund_v": fund, "h5_rel": amp(5 * f0_hz) / fund if fund else None,
            "h7_rel": amp(7 * f0_hz) / fund if fund else None}


def main() -> int:
    directory = config.default_config_dir()
    stack = C.build_stack(directory)
    plant = PMSMPlant(PMSMParameters.from_config(stack.pmsm_config))
    pred = 4.0 / math.pi * dead_time_phase_error(V_DC, T_D, F_SW)
    rows = {}
    for rpm in (500.0, 1000.0):
        for frac in (1.0, 0.2):
            i_q = frac * stack.i_rated
            roll = _rollout(plant, i_q=i_q, rpm0=rpm, dead_time_s=T_D)
            observer = C.build_observer("matched_sigmoid", stack)
            n = len(roll["t"])
            ea, eb = np.empty(n), np.empty(n)
            for k in range(n):
                s = observer.step(roll["u_alpha"][k], roll["u_beta"][k], roll["i_alpha"][k], roll["i_beta"][k], TS)
                ea[k], eb[k] = s.back_emf_alpha, s.back_emf_beta
            f_e = stack.pole_pairs * rpm / 60.0
            mask = roll["t"] >= 0.2
            vm = vector_metrics(ea[mask], eb[mask], roll["e_alpha"][mask], roll["e_beta"][mask],
                                roll["theta_e"][mask])
            harm = _harmonics(ea, f_e, TS, mask)
            e_mag = abs(stack.pole_pairs * rpm * TWO_PI / 60.0 * stack.psi_f)
            rows[f"rpm{int(rpm)}_iq{int(frac*100)}pct"] = {
                "i_q_a": i_q, "abs_E_v": e_mag,
                "fundamental_gain": vm["fundamental_gain"],
                "amplitude_error_pct": 100.0 * (vm["fundamental_gain"] - 1.0),
                "phase_error_deg": vm["fundamental_phase_deg"],
                "h5_rel": harm["h5_rel"], "h7_rel": harm["h7_rel"],
                "predicted_dE_v": pred, "predicted_pct_of_E": 100.0 * pred / e_mag,
            }
    out = {
        "note": ("Phase 06c GATE D: average-value dead time (V_dc=311 V, t_d=2 us, "
                 "f_sw=10 kHz, i_th=0.1 A ASSUMPTION); observer sees the commanded voltage."),
        "prediction": {"dE_v": pred, "formula": "4/pi * V_dc * t_d * f_sw"},
        "rows": rows,
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06c_deadtime"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "gate_d.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    print(f"== GATE D (dead time, matched cell) — prediction dE={pred:.2f} V ==")
    for k, r in rows.items():
        print(f"  {k:18s} |E|={r['abs_E_v']:5.1f}V  amp_err={r['amplitude_error_pct']:+6.1f}% "
              f"(pred {r['predicted_pct_of_E']:5.1f}%)  phase={r['phase_error_deg']:+6.2f}°  "
              f"h5={r['h5_rel']:.3f} h7={r['h7_rel']:.3f}" if r['h5_rel'] is not None else f"  {k}: {r}")
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
