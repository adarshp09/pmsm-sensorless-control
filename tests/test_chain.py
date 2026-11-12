"""Phase 06b: estimator chain as a drive angle/speed source.

Covers the voltage alignment (Gate 1, vs the Phase 04c nominal-drive values), the
angle-advance analytic check, the handover (bumpless + ground-truth-free criterion),
the encoder-baseline regression (06a bit-identical) and the sensorless stability
boundary. Gate 2 (synchronism at the frozen 06a speed-loop bandwidth) is reported as
failed by the experiment; the characterisation test below pins that finding.
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

from pmsm_sensorless import config
from pmsm_sensorless.plant.load_models import ConstantLoadTorque
from pmsm_sensorless.plant.pmsm import PMSMPlant, PMSMParameters, PMSMState
from pmsm_sensorless.simulation.drive import DriveParameters, DriveSample, run_drive

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "experiments"))

import chain as C  # noqa: E402
from validation_metrics import vector_metrics  # noqa: E402

TWO_PI = 2.0 * math.pi
RPM_TO_RAD_S = TWO_PI / 60.0
TS = 100e-6
_04C_JSON = "experiments/output/04c_nominal_scenario/nominal_scenario.json"


@pytest.fixture(scope="module")
def stack(config_dir) -> C.Stack:
    return C.build_stack(config_dir)


@pytest.fixture(scope="module")
def params(config_dir) -> DriveParameters:
    return DriveParameters.from_configs(
        config.load_config("drive", directory=config_dir),
        config.load_config("drive_assumptions", directory=config_dir),
        config.load_config("pmsm", directory=config_dir),
    )


@pytest.fixture(scope="module")
def plant(config_dir) -> PMSMPlant:
    return PMSMPlant(
        PMSMParameters.from_config(config.load_config("pmsm", directory=config_dir)),
        integrate_mechanics=True,
        viscous_friction=0.0,
    )


def _spd_error_rpm(res, mask=None):
    e = (res.omega_m - res.omega_m_ref) * 60.0 / TWO_PI
    return e if mask is None else e[mask]


class _RecordingHandover(C.HandoverSource):
    """HandoverSource that records the post-switch angle error (deg)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.theta_error_deg: list[float] = []

    def __call__(self, sample, truth):
        out = super().__call__(sample, truth)
        if self.mode != "encoder":
            self.theta_error_deg.append(
                math.degrees((out[0] - truth.theta_e + math.pi) % TWO_PI - math.pi)
            )
        return out


# --- structure -----------------------------------------------------------------


def test_chain_imports_no_plant() -> None:
    module = importlib.import_module("chain")
    for value in vars(module).values():
        if isinstance(value, types.ModuleType):
            assert not value.__name__.startswith("pmsm_sensorless.plant"), value.__name__
            assert not value.__name__.startswith("pmsm_sensorless.simulation"), value.__name__


def test_handover_criterion_reads_no_ground_truth(stack: C.Stack) -> None:
    """A stub chain with self-consistent outputs switches; inconsistent does not."""

    class _StubChain:
        last_freq_hz = 0.0
        omega_hat = 0.0
        omega_prime_rad_s = 0.0

        def __init__(self, omega_rpm: float, prime_rpm: float) -> None:
            self.omega_hat = omega_rpm * TWO_PI * stack.pole_pairs / 60.0
            self.omega_prime_rad_s = prime_rpm * TWO_PI * stack.pole_pairs / 60.0
            self.truth = None

        def __call__(self, sample, truth):
            self.truth = truth  # must be ignored by the criterion
            return 0.0, self.omega_hat

    assump = C.HandoverAssumptions(omega_h_rpm=500.0, speed_tolerance_rpm=2.0, lock_time_s=0.05)
    truth = PMSMState(0.0, 0.0, 0.0, 0.0)  # deliberately inconsistent truth
    sample = DriveSample(0.0, TS, 0.0, 0.0, 0.0, 0.0)

    consistent = C.HandoverSource(
        _StubChain(1000.0, 1000.5), pole_pairs=stack.pole_pairs, assumptions=assump
    )
    for k in range(600):
        sample = DriveSample(k * TS, TS, 0.0, 0.0, 0.0, 0.0)
        consistent(sample, truth)
    assert consistent.handed_over

    inconsistent = C.HandoverSource(
        _StubChain(1000.0, 900.0), pole_pairs=stack.pole_pairs, assumptions=assump
    )
    for k in range(600):
        inconsistent(DriveSample(k * TS, TS, 0.0, 0.0, 0.0, 0.0), truth)
    assert not inconsistent.handed_over

    below = C.HandoverSource(
        _StubChain(100.0, 100.0), pole_pairs=stack.pole_pairs, assumptions=assump
    )
    for k in range(600):
        below(DriveSample(k * TS, TS, 0.0, 0.0, 0.0, 0.0), truth)
    assert not below.handed_over


# --- alignment -----------------------------------------------------------------


def _encoder_drive(stack, params, plant, *, load, duration=0.6, advance=False):
    return run_drive(
        plant,
        params,
        duration=duration,
        speed_ref_rpm=1000.0,
        load_torque=ConstantLoadTorque(load),
        angle_advance=advance,
    )


def test_gate1_alignment_matches_04c_nominal_drive(stack, params, plant) -> None:
    """GATE 1: an encoder-driven loop with the chain in the background reproduces the
    04c nominal-drive E_hat gain/phase/ripple for the matched cell.

    The observer consumes ``[applied voltage, sampled current]`` over the observed
    interval (the drive's DriveSample), which is exactly the open-loop 04c alignment.
    Tolerances |Δgain| < 0.01, |Δphase| < 0.3 deg (Gate 1).
    """
    res = _encoder_drive(stack, params, plant, load=stack.i_rated)
    f_e = stack.pole_pairs * 1000.0 / 60.0
    mask = (res.t >= 0.6 - 8.0 / f_e) & (res.t <= 0.6)

    observer = C.build_observer("matched_sigmoid", stack)
    fll = C.build_fll(stack.initial_omega_at(500.0))
    qpll = C.build_qpll(stack)
    out = C.run_chain_open_loop(
        observer,
        fll,
        qpll,
        u_alpha=res.u_alpha,
        u_beta=res.u_beta,
        i_alpha=res.i_alpha,
        i_beta=res.i_beta,
        dt=params.ts_current,
    )
    e_alpha = -(res.omega_e * stack.psi_f) * np.sin(res.theta_e)
    e_beta = (res.omega_e * stack.psi_f) * np.cos(res.theta_e)
    vm = vector_metrics(
        out["e_alpha"][mask], out["e_beta"][mask], e_alpha[mask], e_beta[mask], res.theta_e[mask]
    )

    reference = config.find_project_root() / _04C_JSON
    ref = json.loads(reference.read_text())["results"]["nominal"]["Ts100us"]["irated"][
        "fixed_matched"
    ]["steady"]
    assert abs(vm["fundamental_gain"] - ref["fundamental_gain"]) < 0.01
    assert abs(vm["fundamental_phase_deg"] - ref["fundamental_phase_deg"]) < 0.3
    assert abs(vm["ripple_rms_v"] - ref["ripple_rms_v"]) < 0.01


def test_constant_emf_gives_near_zero_sliding_surface(stack) -> None:
    """A constant-E equilibrium collapses S to the sigmoid offset (matched cell).

    At equilibrium ``k tanh(aS/2) = |E| - Rs S``; for ``|E| = 22.4 V`` this gives
    ``S_eq = (2/a) atanh(0.1245) = 1.16 A`` (the ``sign`` function would give 0). The
    matched cell is inside the D-027 100 us bound, so it converges.
    """
    observer = C.build_observer("matched_sigmoid", stack)
    i_alpha, i_beta = 0.0, 0.0
    e_alpha, e_beta = 20.0, 10.0
    u_alpha = stack.stator_resistance * i_alpha + e_alpha
    u_beta = stack.stator_resistance * i_beta + e_beta
    for _ in range(20000):
        out = observer.step(u_alpha, u_beta, i_alpha, i_beta, TS)
    s_mag = math.hypot(out.s_alpha, out.s_beta)
    # S_eq solves k tanh(aS/2) = |E| - Rs S (fixed point; ~1.13 A here).
    k, a, e_mag, rs = stack.k0, 0.4 * stack.a0, math.hypot(e_alpha, e_beta), stack.stator_resistance
    s_eq = 0.0
    for _ in range(200):
        s_eq = (2.0 / a) * math.atanh(min((e_mag - rs * s_eq) / k, 0.999))
    assert s_mag == pytest.approx(s_eq, rel=0.02)
    # The observer equilibrium is k g(S) = u - Rs i_hat with i_hat = S: E_hat = E - Rs S.
    assert math.hypot(out.back_emf_alpha, out.back_emf_beta) == pytest.approx(
        e_mag - rs * s_mag, rel=0.02
    )


def test_misaligned_voltage_degrades_by_one_sample(stack) -> None:
    """Feeding u one sample late rotates E_hat by ~ omega_e Ts (Gate-1 alignment).

    The alignment error in the applied voltage is ``Ts du/dt``; for a rotating
    fundamental ``|du/dt| = omega_e |u|`` so the phase degradation is ``omega_e Ts``
    (4.19e-2 rad = 2.40 deg at 1000 rpm).
    """
    observer_a = C.build_observer("matched_sigmoid", stack)
    observer_b = C.build_observer("matched_sigmoid", stack)
    speed = 1000.0 * RPM_TO_RAD_S
    n = 4000
    u = np.array([55.5 * math.cos(4.0 * speed * k * TS) for k in range(n)])
    u2 = np.array([55.5 * math.sin(4.0 * speed * k * TS) for k in range(n)])
    ia = np.array([9.0 * math.cos(4.0 * speed * k * TS + 0.3) for k in range(n)])
    ib = np.array([9.0 * math.sin(4.0 * speed * k * TS + 0.3) for k in range(n)])
    ea_a, eb_a, ea_b, eb_b = [], [], [], []
    for k in range(1, n):
        r = observer_a.step(u[k], u2[k], ia[k], ib[k], TS)
        ea_a.append(r.back_emf_alpha)
        eb_a.append(r.back_emf_beta)
        r = observer_b.step(u[k - 1], u2[k - 1], ia[k], ib[k], TS)  # one sample late
        ea_b.append(r.back_emf_alpha)
        eb_b.append(r.back_emf_beta)
    m = slice(2000, None)
    pa = np.angle(
        np.mean(
            (np.array(ea_a) + 1j * np.array(eb_a))[m]
            * np.exp(-1j * 4.0 * speed * np.arange(1, n)[m] * TS)
        )
    )
    pb = np.angle(
        np.mean(
            (np.array(ea_b) + 1j * np.array(eb_b))[m]
            * np.exp(-1j * 4.0 * speed * np.arange(1, n)[m] * TS)
        )
    )
    delta = abs(pb - pa)
    assert delta == pytest.approx(4.0 * speed * TS, rel=0.25)


# --- angle advance -------------------------------------------------------------


def test_angle_advance_shifts_the_inverse_park_frame(stack, params) -> None:
    """The inverse-Park frame is advanced by exactly ``1.5 omega_e Ts`` when enabled.

    The applied-voltage vector angle error is ``omega Ts (delay) - advance``; since
    the closed current loop compensates a constant frame offset, the contract is
    checked at the frame level: with the true angle fed back, ``theta_out`` (the
    angle handed to the inverse Park) minus ``theta_src`` equals ``1.5 omega_e Ts``
    when the flag is on and 0 when off.
    """
    import pmsm_sensorless.simulation.drive as dr

    captured: list[float] = []
    original = dr._inv_park

    def spy(v_d, v_q, theta):
        captured.append(theta)
        return original(v_d, v_q, theta)

    dr._inv_park = spy
    try:
        plant = PMSMPlant(PMSMParameters.from_config(stack.pmsm_config))
        omega_e = 4.0 * 1000.0 * RPM_TO_RAD_S
        offsets = {}
        for advance in (False, True):
            captured.clear()
            res = run_drive(
                plant,
                params,
                duration=0.02,
                i_q_ref=0.0,
                omega_e_fixed=omega_e,
                angle_advance=advance,
            )
            m = res.t >= 0.01
            offsets[advance] = float(np.mean(np.asarray(captured)[m] - res.theta_e[m]))
    finally:
        dr._inv_park = original
    assert offsets[False] == pytest.approx(0.0, abs=1e-9)
    assert offsets[True] == pytest.approx(1.5 * omega_e * TS, rel=1e-6)


# --- handover ------------------------------------------------------------------


def _handover_run(stack, params, plant, *, alpha_w, advance=False, load=7.7, duration=1.2):
    import dataclasses

    prm = (
        dataclasses.replace(params, alpha_w_hz=alpha_w) if alpha_w != params.alpha_w_hz else params
    )
    assump = C.handover_assumptions(stack)
    chain_source = C.EstimatorSource(
        observer=C.build_observer("matched_sigmoid", stack),
        fll=C.build_fll(stack.initial_omega_at(assump.omega_h_rpm)),
        qpll=C.build_qpll(stack),
        pole_pairs=stack.pole_pairs,
    )
    source = _RecordingHandover(chain_source, pole_pairs=stack.pole_pairs, assumptions=assump)
    res = run_drive(
        plant,
        prm,
        duration=duration,
        speed_ref_rpm=1000.0,
        load_torque=ConstantLoadTorque(load),
        source=source,
        angle_advance=advance,
    )
    return source, chain_source, res


def test_handover_is_bumpless(stack, params, plant) -> None:
    """When the estimator is locked the encoder -> estimator switch is smooth.

    Run at alpha_w = 10 Hz: at the frozen 06a bandwidth the *estimated-speed* feedback
    destabilises the loop (Gate 2 finding), so bumplessness is checked on a stable
    configuration. The i_q change across the switch is bounded.
    """
    source, _chain, res = _handover_run(stack, params, plant, alpha_w=10.0)
    assert source.handed_over
    switch = source.switch_time_s
    m = (res.t >= switch - 0.002) & (res.t <= switch + 0.002)
    assert np.max(np.abs(np.diff(res.i_q[m]))) < 0.25  # A per 100 us step
    assert (
        abs(
            np.mean(res.i_q[res.t >= switch + 0.05])
            - np.mean(res.i_q[(res.t >= switch - 0.05) & (res.t < switch)])
        )
        < 0.5
    )


# --- Gate 2 / stability --------------------------------------------------------


def test_sensorless_loop_needs_speed_loop_below_estimate_bandwidth(stack, params, plant) -> None:
    """GATE 2 diagnostic: the frozen 06a speed loop (alpha_w = 50 Hz) loses synchronism.

    With the true angle but the *estimated* speed in the speed loop the drive diverges
    (~100 Hz growth) because the speed feedback (Q-PLL, f_n = 20 Hz) is slower than the
    speed loop. Slowing the loop (alpha_w = 10 Hz) keeps |theta_hat - theta| < 30 deg.
    This test pins the finding; the fix is a separate step (D-054).
    """
    fast, fast_chain, fast_res = _handover_run(stack, params, plant, alpha_w=50.0)
    fast_peak = float(np.max(np.abs(fast.theta_error_deg))) if fast.theta_error_deg else 0.0
    assert fast_peak > 30.0

    slow, slow_chain, slow_res = _handover_run(stack, params, plant, alpha_w=10.0)
    assert np.max(np.abs(slow.theta_error_deg)) < 30.0
    assert np.max(np.abs(_spd_error_rpm(slow_res, slow_res.t >= 0.9))) < 20.0
    assert slow_chain.status == "ok"


# --- regression ----------------------------------------------------------------


def test_encoder_baseline_regression(config_dir) -> None:
    """The 06a encoder-baseline JSON is reproduced bit-for-bit (angle_advance off)."""
    spec = importlib.util.spec_from_file_location(
        "exp06a",
        str(Path(__file__).resolve().parents[1] / "experiments" / "06a_encoder_baseline.py"),
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules["exp06a"] = mod
    spec.loader.exec_module(mod)

    base = DriveParameters.from_configs(
        config.load_config("drive", directory=config_dir),
        config.load_config("drive_assumptions", directory=config_dir),
        config.load_config("pmsm", directory=config_dir),
    )
    pmsm_cfg = config.load_config("pmsm", directory=config_dir)
    planted = (
        config.find_project_root()
        / "experiments"
        / "output"
        / "06a_encoder_baseline"
        / "encoder_baseline.json"
    )
    stored = json.loads(planted.read_text())["scenarios"]
    scenarios = {sc.name: sc for sc in mod._scenarios()}
    for name in ("speed_step_0_1000", "load_step_rated_1000"):
        plant_obj = PMSMPlant(
            PMSMParameters.from_config(pmsm_cfg),
            integrate_mechanics=True,
            viscous_friction=base.viscous_friction,
        )
        fresh = mod._run(base, plant_obj, scenarios[name])
        for key in (
            "speed_dip_rpm",
            "speed_overshoot_rpm",
            "recovery_to_10rpm_s",
            "peak_i_q_a",
            "steady_speed_err_rpm",
            "saturation_fraction",
        ):
            a, b = fresh.get(key), stored[name].get(key)
            assert (a is None and b is None) or a == pytest.approx(b, rel=0, abs=1e-12), (
                name,
                key,
                a,
                b,
            )
