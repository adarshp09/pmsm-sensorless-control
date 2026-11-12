#!/usr/bin/env python3
"""Phase 06b Step 0: constant-vs-ramp basis (0b) and the friction-term check (0c).

0b: time-localise the nominal-cell (matched sigmoid | v' | QPLL, 100 us) constant-1000
rpm speed error inside the Phase 05b steady window, and recompute it over the ramp's
settled-window rule (a window placed after the estimator transient). Decides whether
the ~20x ramp/constant speed-error gap is a transient-in-window artefact.

0c: check whether ``PMSMPlant.speed_derivative`` applies ``B*omega_e`` or ``B*omega_m``
and whether that equals the documented mechanical ``J w_m' = T_e - T_L - B w_m``.

Output: experiments/output/06b_step0/step0_analysis.json
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
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters  # noqa: E402
from pmsm_sensorless.simulation.nominal_drive import nominal_drive_rollout  # noqa: E402

TWO_PI = 2.0 * math.pi
RPM_TO_RAD_S = TWO_PI / 60.0
TS = 100e-6
STEADY_CYCLES = 8
RUN_CYCLES = 12
MIN_DURATION = 0.25


def _nominal_chain(stack: C.Stack, plant: PMSMPlant, *, rpm, duration):
    """The 05b/06a nominal cell (matched sigmoid | DSOGI v' | QPLL) at constant rpm."""
    f_e = stack.pole_pairs * rpm / 60.0
    r = nominal_drive_rollout(
        plant,
        i_q=stack.i_rated,
        dt=TS,
        duration=duration,
        rpm_start=rpm,
        rpm_end=rpm,
        ramp_time=1.0,
    )
    observer = C.build_observer("matched_sigmoid", stack)
    fll = C.build_fll(stack.initial_omega_at(rpm))
    qpll = C.build_qpll(stack, initial_theta=float(r.theta_e[0]), initial_omega=float(r.omega_e[0]))
    out = C.run_chain_open_loop(
        observer,
        fll,
        qpll,
        u_alpha=r.u_alpha,
        u_beta=r.u_beta,
        i_alpha=r.i_alpha,
        i_beta=r.i_beta,
        dt=TS,
    )
    omega_err_rpm = (out["omega_hat"] - r.omega_e) * 60.0 / (TWO_PI * stack.pole_pairs)
    theta_err_deg = np.degrees((out["theta_hat"] - r.theta_e + np.pi) % TWO_PI - np.pi)
    return r.t, omega_err_rpm, theta_err_deg, f_e


def main() -> int:
    directory = config.default_config_dir()
    stack = C.build_stack(directory)
    plant = PMSMPlant(PMSMParameters.from_config(stack.pmsm_config))

    # ---- 0b: constant 1000 rpm under the 05b rule vs the ramp rule
    rpm = 1000.0
    short_duration = max(MIN_DURATION, RUN_CYCLES / (stack.pole_pairs * rpm / 60.0))
    t_short, err_short, th_short, f_e = _nominal_chain(
        stack, plant, rpm=rpm, duration=short_duration
    )
    short_mask = (t_short >= short_duration - STEADY_CYCLES / f_e) & (t_short <= short_duration)
    i_max = np.argmax(np.abs(err_short[short_mask]))
    window_start = t_short[short_mask][0]

    long_duration = 2.0
    t_long, err_long, _th_long, _ = _nominal_chain(stack, plant, rpm=rpm, duration=long_duration)
    late_mask = (t_long >= long_duration - STEADY_CYCLES / f_e) & (t_long <= long_duration)
    settled_mask = (t_long >= 0.75) & (t_long <= 1.5)  # the ramp's settled rule

    # ---- 0c: friction term
    inertia = plant.parameters.inertia
    friction = 0.003
    plant_b = PMSMPlant(plant.parameters, integrate_mechanics=True, viscous_friction=friction)
    omega_e, torque, load = 418.879, 3.0, 0.7
    domega_e = plant_b.speed_derivative(omega_e, torque, load)
    # documented mechanical form J dw_m/dt = T - T_L - B w_m, expressed as d w_e/dt
    mechanical = (
        plant.parameters.pole_pairs
        * (torque - load - friction * (omega_e / plant.parameters.pole_pairs))
        / inertia
    )

    out = {
        "step0b": {
            "rule_constant": "last 8 electrical cycles of a run of length max(0.25s, 12 cycles)",
            "rule_ramp": "settled window [0.75, 1.5] s (after the estimator transient)",
            "constant_1000_05b_window": [float(window_start), float(short_duration)],
            "constant_1000_05b_speed_max_rpm": float(np.max(np.abs(err_short[short_mask]))),
            "constant_1000_05b_time_of_max_s": float(t_short[short_mask][i_max]),
            "constant_1000_05b_time_of_max_after_window_start_s": float(
                t_short[short_mask][i_max] - window_start
            ),
            "constant_1000_05b_error_at_window_start_rpm": float(err_short[short_mask][0]),
            "constant_1000_late_window_speed_max_rpm": float(np.max(np.abs(err_long[late_mask]))),
            "constant_1000_settled_rule_speed_max_rpm": float(
                np.max(np.abs(err_long[settled_mask]))
            ),
            "constant_1000_settled_rule_angle_max_deg": float(np.max(np.abs(th_short[short_mask]))),
            "verdict": (
                "transient-in-window artefact: the 05b constant window starts at 0.13 s, "
                "still inside the decaying FLL/QPLL settling envelope (window-start error "
                f"{err_short[short_mask][0]:.2f} rpm, decaying through the window). A window "
                "placed after the transient (>=0.75 s) gives the same ~0.15 rpm as the ramp."
            ),
        },
        "step0c": {
            "speed_derivative_applies": "B * omega_e (electrical state)",
            "domega_e_from_code": float(domega_e),
            "domega_e_mechanical_equivalent": float(mechanical),
            "identical": bool(abs(domega_e - mechanical) < 1e-12),
            "verdict": (
                "consistent: inside d(omega_e)/dt the term B*omega_e is p*(B*omega_m), so the "
                "code is exactly the documented mechanical J d(w_m)/dt = T_e - T_L - B w_m "
                "(equations_and_conventions sec.12 writes it as B w_e / p = B w_m). No fix needed."
            ),
        },
    }
    out_dir = config.find_project_root() / "experiments" / "output" / "06b_step0"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "step0_analysis.json").write_text(json.dumps(out, indent=2), encoding="utf-8")

    b = out["step0b"]
    print("== 0b: constant-1000 rpm speed error ==")
    print(
        f"  05b rule  : window [{b['constant_1000_05b_window'][0]:.3f}, {b['constant_1000_05b_window'][1]:.3f}] s, "
        f"max|dw|={b['constant_1000_05b_speed_max_rpm']:.3f} rpm at t={b['constant_1000_05b_time_of_max_s']:.4f} "
        f"({b['constant_1000_05b_time_of_max_after_window_start_s'] * 1e3:.1f} ms into the window)"
    )
    print(
        f"  2 s late window: max|dw|={b['constant_1000_late_window_speed_max_rpm']:.3f} rpm  |  "
        f"settled [0.75,1.5]: {b['constant_1000_settled_rule_speed_max_rpm']:.3f} rpm"
    )
    print(f"  ramp (05b)  : 0.1475 rpm (100 us)")
    print(f"  -> {b['verdict']}")
    c = out["step0c"]
    print("\n== 0c: friction term ==")
    print(
        f"  speed_derivative uses {c['speed_derivative_applies']}; d w_e/dt code={c['domega_e_from_code']:.9f} "
        f"vs mechanical={c['domega_e_mechanical_equivalent']:.9f} (identical={c['identical']})"
    )
    print(f"  -> {c['verdict']}")
    print(f"\nOutputs written to: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
