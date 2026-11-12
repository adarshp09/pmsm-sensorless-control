"""Phase 06c: average-value inverter dead-time model (drive.py flags, default OFF).

Checks the model formula (magnitude, sign, zero at i=0), the harmonic content under a
sinusoidal current (fundamental 4/pi, 5th/7th 1/5, 1/7), and that the default (dead
time OFF) is bit-identical to the Phase 06a encoder baseline.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from pmsm_sensorless import config
from pmsm_sensorless.plant.load_models import ConstantLoadTorque
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters
from pmsm_sensorless.simulation.drive import (
    DriveParameters,
    dead_time_delta_alpha_beta,
    dead_time_phase_error,
    run_drive,
)

_FLOAT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_FLOAT_ROOT / "experiments"))

V_DC = 311.0
T_D = 2e-6
F_SW = 10_000.0
I_TH = 0.1


def test_phase_error_magnitude() -> None:
    assert dead_time_phase_error(V_DC, T_D, F_SW) == pytest.approx(6.22, rel=1e-6)


def test_sign_opposes_current_and_zero_at_zero() -> None:
    # i_alpha > 0 with i_beta = 0 -> phase a current positive -> dv_a negative
    da, db = dead_time_delta_alpha_beta(
        5.0, 0.0, v_dc=V_DC, dead_time_s=T_D, switching_hz=F_SW, current_threshold_a=I_TH
    )
    assert da < 0.0
    # zero current -> zero distortion
    assert dead_time_delta_alpha_beta(
        0.0, 0.0, v_dc=V_DC, dead_time_s=T_D, switching_hz=F_SW, current_threshold_a=I_TH
    ) == (0.0, 0.0)
    # below threshold: proportional
    da_small, _ = dead_time_delta_alpha_beta(
        0.05, 0.0, v_dc=V_DC, dead_time_s=T_D, switching_hz=F_SW, current_threshold_a=I_TH
    )
    assert 0.0 > da_small > da


def test_harmonics_of_square_wave_distortion() -> None:
    """Under sinusoidal 3-phase current the per-phase distortion is a square wave:
    fundamental 4/pi*6.22 V, 5th/7th = 1/5, 1/7 of it."""
    amp = dead_time_phase_error(V_DC, T_D, F_SW)
    theta = np.linspace(0.0, 2.0 * math.pi, 4096, endpoint=False)
    i_a = 5.0 * np.cos(theta)
    i_b = 5.0 * np.cos(theta - 2.0 * math.pi / 3.0)
    i_c = 5.0 * np.cos(theta + 2.0 * math.pi / 3.0)
    dv_a = -amp * np.clip(i_a / I_TH, -1.0, 1.0)
    spec = np.abs(np.fft.rfft(dv_a)) * 2.0 / len(theta)
    fundamental = spec[1]
    assert fundamental == pytest.approx(4.0 / math.pi * amp, rel=0.02)
    assert spec[5] / fundamental == pytest.approx(1.0 / 5.0, rel=0.05)
    assert spec[7] / fundamental == pytest.approx(1.0 / 7.0, rel=0.05)


def test_alpha_beta_fundamental_matches_4_over_pi() -> None:
    """The alpha-beta distortion vector's fundamental magnitude is also 4/pi*6.22 V."""
    amp = dead_time_phase_error(V_DC, T_D, F_SW)
    theta = np.linspace(0.0, 2.0 * math.pi, 4096, endpoint=False)
    ia = 5.0 * np.cos(theta)
    ib = 5.0 * np.cos(theta - 2.0 * math.pi / 3.0)
    ic = 5.0 * np.cos(theta + 2.0 * math.pi / 3.0)
    # alpha/beta from the abc square waves (amplitude-invariant Clarke)
    dv = -amp * np.clip(np.vstack([ia, ib, ic]) / I_TH, -1.0, 1.0)
    dal = (2.0 / 3.0) * (dv[0] - 0.5 * dv[1] - 0.5 * dv[2])
    dbet = (1.0 / math.sqrt(3.0)) * (dv[1] - dv[2])
    za = np.abs(np.fft.rfft(dal)) * 2.0 / len(theta)
    zb = np.abs(np.fft.rfft(dbet)) * 2.0 / len(theta)
    # amplitude-invariant Clarke: each alpha/beta component carries the phase amplitude
    assert za[1] == pytest.approx(4.0 / math.pi * amp, rel=0.02)
    assert zb[1] == pytest.approx(4.0 / math.pi * amp, rel=0.02)


def test_dead_time_compensation_cancels_distortion() -> None:
    """With ideal parameters the residual plant-side voltage error is < 5 % of dV,
    except where any phase current is below i_th (linear sat region)."""
    from pmsm_sensorless.simulation.drive import dead_time_compensation_alpha_beta

    amp = dead_time_phase_error(V_DC, T_D, F_SW)
    theta = np.linspace(0.0, 2.0 * math.pi, 4096, endpoint=False)
    ia = 5.0 * np.cos(theta)
    ib = 5.0 * np.cos(theta - 2.0 * math.pi / 3.0)
    ic = 5.0 * np.cos(theta + 2.0 * math.pi / 3.0)
    i_alpha = ia
    i_beta = (ib - ic) / math.sqrt(3.0)
    d = np.array(
        [
            dead_time_delta_alpha_beta(
                i_alpha[k],
                i_beta[k],
                v_dc=V_DC,
                dead_time_s=T_D,
                switching_hz=F_SW,
                current_threshold_a=I_TH,
            )
            for k in range(len(theta))
        ]
    )
    ip = np.roll(i_alpha, 1)
    jp = np.roll(i_beta, 1)
    c = np.array(
        [
            dead_time_compensation_alpha_beta(
                ip[k],
                jp[k],
                v_dc=V_DC,
                dead_time_s=T_D,
                switching_hz=F_SW,
                current_threshold_a=I_TH,
            )
            for k in range(len(theta))
        ]
    )
    net = np.hypot(*(d + c).T)
    away = np.min(np.vstack([np.abs(ia), np.abs(ib), np.abs(ic)]), axis=0) >= I_TH
    assert np.max(net[away]) < 0.05 * amp


def test_defaults_are_bit_identical_regression(config_dir) -> None:
    """dead_time_s = 0 (default) reproduces the dead-time-OFF run exactly."""
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=config_dir),
        config.load_config("drive_assumptions", directory=config_dir),
        config.load_config("pmsm", directory=config_dir),
    )
    plant = PMSMPlant(
        PMSMParameters.from_config(config.load_config("pmsm", directory=config_dir)),
        integrate_mechanics=True,
        viscous_friction=0.0,
    )
    kw = dict(duration=0.2, speed_ref_rpm=1000.0, load_torque=ConstantLoadTorque(7.7))
    a = run_drive(plant, params, **kw)
    b = run_drive(plant, params, dead_time_s=0.0, **kw)
    assert np.array_equal(a.i_alpha, b.i_alpha)
    assert np.array_equal(a.u_alpha, b.u_alpha)


def test_dead_time_changes_the_plant() -> None:
    """A non-zero dead time perturbs the current (so the flag is wired)."""
    import pmsm_sensorless.simulation.drive as dr

    plant = PMSMPlant(
        PMSMParameters.from_config(
            config.load_config("pmsm", directory=config.default_config_dir())
        ),
        integrate_mechanics=True,
        viscous_friction=0.0,
    )
    params = DriveParameters.from_configs(
        config.load_config("drive", directory=config.default_config_dir()),
        config.load_config("drive_assumptions", directory=config.default_config_dir()),
        config.load_config("pmsm", directory=config.default_config_dir()),
    )
    kw = dict(duration=0.3, speed_ref_rpm=1000.0, load_torque=ConstantLoadTorque(7.7))
    a = dr.run_drive(plant, params, **kw)
    b = dr.run_drive(plant, params, dead_time_s=2e-6, **kw)
    assert not np.array_equal(a.i_alpha, b.i_alpha)
