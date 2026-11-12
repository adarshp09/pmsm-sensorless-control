#!/usr/bin/env python3
"""Phase 01 validation experiment for the PMSM plant.

This script runs short, deterministic simulations with the *resolved* motor
parameters from ``configs/pmsm.yaml`` and checks the plant against closed-form
solutions of the equations it implements. It does **not** compare against the
paper's figures or reproduce published results -- Phase 01 has no observer,
controller or DSOGI-FLL, and the paper's experimental conditions are unresolved.

Scenarios
---------
A. Standstill step response: validates the electrical equation Eq.(1), p.2,
   against ``i(t) = (U/Rs)(1 - exp(-t/tau))`` with ``tau = Ld/Rs``.
B. Rotating back-EMF with short-circuited terminals (``u = 0``): validates the
   back-EMF Eq.(2), p.2, against the closed-form current
   ``i(t) = A exp(j w_e t) + C exp(-t/tau)``, ``A = -j w_e psi_f / (Rs + j w_e Ld)``.
C. Opt-in mechanical extension (derived, *not* printed in the paper): friction
   decay against ``w_e(t) = w_e0 exp(-t/tau_m)``, ``tau_m = J/B``. The drive
   ``u = E`` is held constant over each step (zero-order hold) and evaluated at
   the mid-step angle; this scenario therefore uses ``dt/10`` to isolate the
   plant from the control-rate error that any fixed-step drive introduces.

The viscous friction ``B`` and the demonstration speed/voltage levels are local
choices (the paper leaves ``B`` unresolved); they are passed explicitly, never
silently. See ``docs/implementation_decisions.md``.

Outputs are written to ``experiments/output/01_plant_validation/``.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

try:  # allow running the script without an editable install
    import pmsm_sensorless  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover - developer convenience
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from pmsm_sensorless import config  # noqa: E402
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters, PMSMState  # noqa: E402

RPM_TO_RAD_S = 2.0 * math.pi / 60.0

# Local demonstration settings (not paper values).
STANDSTILL_VOLTAGE = 5.0  # V, drives scenario A
MECH_FRICTION_DEMO = 2.0e-3  # N.m.s/rad, local assumption for scenario C
MECH_SPEED_DEMO_RPM = 500.0  # rpm, initial mechanical speed for scenario C


def load_configs() -> tuple[dict, dict]:
    directory = config.default_config_dir()
    return (
        config.load_config("pmsm", directory=directory),
        config.load_config("experiments", directory=directory),
    )


def simulate(plant, initial_state, control, duration, dt):
    """Deterministic fixed-step rollout. ``control(t, state) -> (ua, ub, load)``."""
    steps = int(round(duration / dt))
    t = np.arange(steps + 1) * dt
    i_alpha = np.empty(steps + 1)
    i_beta = np.empty(steps + 1)
    omega_e = np.empty(steps + 1)
    theta_e = np.empty(steps + 1)
    state = initial_state
    i_alpha[0], i_beta[0] = state.i_alpha, state.i_beta
    omega_e[0], theta_e[0] = state.omega_e, state.theta_e
    for k in range(steps):
        u_alpha, u_beta, load = control(t[k], state)
        state = plant.step(state, u_alpha, u_beta, dt, load_torque=load)
        i_alpha[k + 1], i_beta[k + 1] = state.i_alpha, state.i_beta
        omega_e[k + 1], theta_e[k + 1] = state.omega_e, state.theta_e
    return {
        "t": t,
        "i_alpha": i_alpha,
        "i_beta": i_beta,
        "omega_e": omega_e,
        "theta_e": theta_e,
        "state": state,
    }


def scenario_a(plant: PMSMPlant, dt: float):
    """Standstill step response of Eq.(1)."""
    tau = plant.parameters.ld / plant.parameters.stator_resistance
    duration = 8.0 * tau
    result = simulate(
        plant,
        PMSMState(0.0, 0.0, 0.0, 0.0),
        control=lambda t, s: (STANDSTILL_VOLTAGE, 0.0, 0.0),
        duration=duration,
        dt=dt,
    )
    i_ss = STANDSTILL_VOLTAGE / plant.parameters.stator_resistance
    analytic = i_ss * (1.0 - np.exp(-result["t"] / tau))
    result["analytic_i_alpha"] = analytic
    result["tau"] = tau
    result["max_abs_error"] = float(np.max(np.abs(result["i_alpha"] - analytic)))
    return result


def scenario_b(plant: PMSMPlant, dt: float, duration: float = 0.1):
    """Short-circuited rotating back-EMF."""
    parameters = plant.parameters
    tau = parameters.ld / parameters.stator_resistance
    omega_e = parameters.pole_pairs * MECH_SPEED_DEMO_RPM * RPM_TO_RAD_S
    result = simulate(
        plant,
        PMSMState(0.0, 0.0, omega_e, 0.0),
        control=lambda t, s: (0.0, 0.0, 0.0),
        duration=duration,
        dt=dt,
    )
    theta = omega_e * result["t"]
    a_ss = (
        -1j
        * omega_e
        * parameters.psi_f
        / (parameters.stator_resistance + 1j * omega_e * parameters.ld)
    )
    complex_current = a_ss * np.exp(1j * theta) - a_ss * np.exp(-result["t"] / tau)
    result["analytic_i_alpha"] = complex_current.real
    result["analytic_i_beta"] = complex_current.imag
    result["emf"] = np.array(
        [plant.back_emf(float(th), omega_e) for th in np.mod(theta, 2 * math.pi)]
    )
    result["omega_e"] = omega_e
    result["steady_amplitude"] = abs(a_ss)
    result["final_magnitude"] = float(math.hypot(result["i_alpha"][-1], result["i_beta"][-1]))
    result["max_abs_error"] = float(
        np.max(
            np.hypot(
                result["i_alpha"] - result["analytic_i_alpha"],
                result["i_beta"] - result["analytic_i_beta"],
            )
        )
    )
    return result


def scenario_c(plant: PMSMPlant, dt: float, duration: float = 0.5):
    """Opt-in mechanical friction decay (derived extension)."""
    parameters = plant.parameters
    omega_e0 = parameters.pole_pairs * MECH_SPEED_DEMO_RPM * RPM_TO_RAD_S
    tau_m = parameters.inertia / MECH_FRICTION_DEMO
    result = simulate(
        plant,
        PMSMState(0.0, 0.0, omega_e0, 0.0),
        control=lambda t, s: (
            *plant.back_emf(float(s.theta_e + 0.5 * s.omega_e * dt), s.omega_e),
            0.0,
        ),
        duration=duration,
        dt=dt,
    )
    result["analytic_omega_e"] = omega_e0 * np.exp(-result["t"] / tau_m)
    result["analytic_theta_e"] = omega_e0 * tau_m * (1.0 - np.exp(-result["t"] / tau_m))
    result["tau_m"] = tau_m
    result["omega_e0"] = omega_e0
    result["max_rel_omega_error"] = float(
        np.max(np.abs(result["omega_e"] - result["analytic_omega_e"]) / result["analytic_omega_e"])
    )
    return result


def make_plots(a, b, c, param, output_dir: Path) -> None:
    rpm = lambda we: we / param.pole_pairs / RPM_TO_RAD_S  # noqa: E731

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    axes[0].plot(a["t"] * 1e3, a["i_alpha"], label="numeric")
    axes[0].plot(a["t"] * 1e3, a["analytic_i_alpha"], "--", label="analytic")
    axes[0].set(
        xlabel="time (ms)", ylabel="i_alpha (A)", title="A. Standstill step response of Eq.(1)"
    )
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(a["t"] * 1e3, np.abs(a["i_alpha"] - a["analytic_i_alpha"]), color="crimson")
    axes[1].set(xlabel="time (ms)", ylabel="|i_alpha - analytic| (A)", title="A. Numerical error")
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "scenario_a_standstill_step.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    axes[0].plot(b["t"] * 1e3, b["i_alpha"], label="i_alpha")
    axes[0].plot(b["t"] * 1e3, b["i_beta"], label="i_beta")
    axes[0].plot(b["t"] * 1e3, b["analytic_i_alpha"], "--", label="analytic alpha")
    axes[0].plot(b["t"] * 1e3, b["analytic_i_beta"], "--", label="analytic beta")
    axes[0].set(xlabel="time (ms)", ylabel="current (A)", title="B. Rotating back-EMF (u=0)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(b["t"] * 1e3, np.hypot(b["i_alpha"], b["i_beta"]), label="|i| numeric")
    axes[1].axhline(b["steady_amplitude"], color="k", ls=":", label="analytic |A|")
    axes[1].set(xlabel="time (ms)", ylabel="|i| (A)", title="B. Current magnitude")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(b["emf"][:, 0], b["emf"][:, 1])
    axes[2].set(
        xlabel="E_alpha (V)", ylabel="E_beta (V)", title="B. Back-EMF locus (Eq.2)", aspect="equal"
    )
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "scenario_b_rotating_emf.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    axes[0].plot(c["t"], rpm(c["omega_e"]), label="numeric")
    axes[0].plot(c["t"], rpm(c["analytic_omega_e"]), "--", label="analytic")
    axes[0].set(
        xlabel="time (s)", ylabel="mechanical speed (rpm)", title="C. Mechanical friction decay"
    )
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(c["t"], c["theta_e"], label="numeric")
    axes[1].plot(c["t"], c["analytic_theta_e"], "--", label="analytic")
    axes[1].set(xlabel="time (s)", ylabel="electrical angle (rad)", title="C. Electrical angle")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "scenario_c_mechanical.png", dpi=130)
    plt.close(fig)


def main() -> int:
    pmsm_cfg, experiments_cfg = load_configs()
    parameters = PMSMParameters.from_config(pmsm_cfg)
    dt = config.resolve_parameter(experiments_cfg, "sim.timestep_ts", unit="s")
    plant = PMSMPlant(parameters)
    mechanical_plant = PMSMPlant(
        parameters, integrate_mechanics=True, viscous_friction=MECH_FRICTION_DEMO
    )

    a = scenario_a(plant, dt)
    b = scenario_b(plant, dt)
    c = scenario_c(mechanical_plant, dt / 10.0)

    # Determinism: an independent re-run must be bit-for-bit identical.
    a_repeat = scenario_a(plant, dt)
    determinism_max_diff = float(np.max(np.abs(a["i_alpha"] - a_repeat["i_alpha"])))

    output_dir = config.find_project_root() / "experiments" / "output" / "01_plant_validation"
    output_dir.mkdir(parents=True, exist_ok=True)
    make_plots(a, b, c, parameters, output_dir)

    np.savez(
        output_dir / "states.npz",
        t_a=a["t"],
        i_alpha_a=a["i_alpha"],
        analytic_i_alpha_a=a["analytic_i_alpha"],
        t_b=b["t"],
        i_alpha_b=b["i_alpha"],
        i_beta_b=b["i_beta"],
        analytic_i_alpha_b=b["analytic_i_alpha"],
        analytic_i_beta_b=b["analytic_i_beta"],
        e_alpha=b["emf"][:, 0],
        e_beta=b["emf"][:, 1],
        t_c=c["t"],
        omega_e_c=c["omega_e"],
        analytic_omega_e_c=c["analytic_omega_e"],
        theta_e_c=c["theta_e"],
        analytic_theta_e_c=c["analytic_theta_e"],
    )

    diagnostics = {
        "note": (
            "Phase 01 plant validation against closed-form solutions of Eq.(1)-(2); "
            "not a reproduction of the paper's results."
        ),
        "parameters": {
            "pole_pairs": parameters.pole_pairs,
            "stator_resistance_ohm": parameters.stator_resistance,
            "ld_h": parameters.ld,
            "lq_h": parameters.lq,
            "psi_f_wb": parameters.psi_f,
            "inertia_kg_m2": parameters.inertia,
            "viscous_friction_paper_value": parameters.viscous_friction,
        },
        "timestep_s": dt,
        "scenario_a": {
            "tau_s": a["tau"],
            "max_abs_error_a": a["max_abs_error"],
        },
        "scenario_b": {
            "electrical_speed_rad_s": b["omega_e"],
            "steady_current_amplitude_a": b["steady_amplitude"],
            "final_current_magnitude_a": b["final_magnitude"],
            "max_abs_error_a": b["max_abs_error"],
        },
        "scenario_c": {
            "local_friction_assumption_n_m_s_per_rad": MECH_FRICTION_DEMO,
            "initial_speed_rpm": MECH_SPEED_DEMO_RPM,
            "timestep_s": dt / 10.0,
            "tau_m_s": c["tau_m"],
            "max_rel_omega_error": c["max_rel_omega_error"],
            "note": (
                "residual is dominated by the zero-order-held u=E drive, "
                "not by the Runge-Kutta integrator"
            ),
        },
        "determinism_max_abs_diff_a": determinism_max_diff,
    }
    (output_dir / "diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2), encoding="utf-8"
    )

    print(json.dumps(diagnostics, indent=2))
    print(f"\nOutputs written to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
