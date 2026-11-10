# Phase 06b — Sensorless closed loop (estimator in the loop)

Replaces the Phase 06a encoder source with the estimator chain
(**SMO `E_hat` → DSOGI-FLL `v'` → Q-PLL**) as the drive's angle/speed source. No dead
time or PWM (06c); no conventional-SMO baseline (06c).

- Chain composition (experiments only): `experiments/chain.py` (`EstimatorSource`,
  `HandoverSource`). Drive flag: `simulation/drive.py` `angle_advance` (default OFF).
  Assumptions: `configs/drive_assumptions.yaml` (handover + advance keys).
- Scripts: `experiments/06b_step0_analysis.py` (Step 0), `experiments/06b_gate2.py`
  (Gate 2 traces). Tests: `tests/test_chain.py`.
- Outputs: `experiments/output/06b_step0/step0_analysis.json`,
  `experiments/output/06b_gate2/gate2.json`.

**No tuning toward the paper's numbers; every assumption frozen before running.**

## Step 0 — housekeeping

**0a** — Phase 06a committed (`73e706e`), tagged `phase-06a`; the
`simulation/__init__.py` docstring restored (names both scenario generators).

**0b — constant-vs-ramp speed error is a transient-in-window artefact.** The Phase 05b
rows are all pointwise, but the constant-speed runs are only `max(0.25 s, 12 cycles)`
long: at 1000 rpm the "steady" window `[0.13, 0.25] s` still sits in the decaying
FLL/Q-PLL settling envelope (window-start error −1.9 rpm; max 3.4 rpm 5 ms in). Re-run
for 2 s and window after the transient (`[0.75, 1.5]` or `[1.88, 2.0]`) → **0.147 rpm**,
the **same** as the ramp (0.1475 rpm). Corrects the Phase 06a claim that the gap was
physical; `phase05b.md` is amended.

**0c — friction term is consistent (no fix).** `PMSMPlant.speed_derivative` applies
`B·\omega_e` inside `d\omega_e/dt`, which is exactly `p·(B·\omega_m)` — i.e. the
documented mechanical `J d\omega_m/dt = T_e - T_L - B\omega_m` (eq. section 12 writes
`B\omega_e/p = B\omega_m`). Numerically identical to the mechanical form (Δ = 0), and
the same convention the 06a `b>0` test uses. **No inconsistency.**

**0d — start-up / initial position (from `estimator_spec.md`).** *PRINTED:* the method
*needs another strategy for start-up and low speed* (Sections IV-E/V-E, qualitative);
the Q-PLL has an integrator (Fig.1). *UNPRINTED:* any initial position, initial `ω'`,
lock criterion, or handover rule. Our start-up is therefore an ASSUMPTION (D-053).

**0e — the paper's numbers are estimator errors.** In the class-B table the *position*
error can only be `|θ̂−θ|` (the drive is speed-, not position-controlled), and a
*nonzero steady mean speed error* (5 rpm) under a speed PI with integral action can only
be the estimation error `|ω̂−ω|`. So **every** paper "speed/position error" is an
estimator error `(|ω̂−ω|, |θ̂−θ|)`, **not** an actual speed deviation. Comparisons in
06b are made on that basis only.

## Step 1 — the chain as a source

`EstimatorSource(observer, filter, estimator)` implements the 06a `AngleSpeedSource`
interface: it consumes the sampled `i_αβ` and the **voltage actually applied over the
observed interval** (the drive's `DriveSample.u` = the last command under the 1-sample
delay) and returns `(θ̂, ω̂)`; `ω̂` is electrical (TrueSource parity), the mechanical
estimate is `ω̂/p`. It imports no plant.

**GATE 1 (alignment) — PASSED.** Encoder-driven loop at 1000 rpm / rated load with the
chain running in the background (matched cell), steady window (last 8 cycles):

| E_hat fundamental | ours | 04c nominal-drive | Δ | gate |
| --- | --- | --- | --- | --- |
| gain | 0.974077 | 0.974287 | −0.0002 | < 0.01 ✅ |
| phase (deg) | −0.5041 | −0.5167 | +0.0126 | < 0.3 ✅ |
| ripple (V) | 0.041618 | 0.041646 | −2.8e-5 | — |

So the `(applied voltage, sampled current)` pairing is the correct 04c alignment.

## Step 2 — handover and angle advance

Start-up (ASSUMPTION — not from paper; D-053): encoder-driven until an
**estimator-internal** lock criterion holds — `ω̂_m ≥ ω_h` and
`|ω̂_m − ω'_m| < tol` for `t_lock` (all mechanical rpm, no ground truth). FLL initial
`ω' = p·ω_h` (a nonzero design constant, never the true instantaneous frequency;
`ω'=0` is a dead fixed point). Frozen values: `ω_h = 500 rpm`, `tol = 2 rpm`,
`t_lock = 50 ms`. Angle advance: `angle_advance ∈ {off (default), 1.5·ω̂·Ts}`
(1-sample computation delay + ½ ZOH).

## Step 3 — tests

9 tests in `tests/test_chain.py` pass: structure (chain imports no plant; the criterion
reads no ground truth), Gate 1 alignment, constant-E equilibrium (`S →` the sigmoid
offset `(2/a)·atanh((|E|−Rs S)/k) = 1.13 A`), one-sample misalignment (phase degrades by
`ω_e Ts` = 2.40° at 1000 rpm), angle-advance frame contract (exactly `1.5 ω_e Ts`), the
handover bumplessness, the encoder-baseline regression (06a JSON reproduced
bit-for-bit), and the sensorless stability boundary.

**GATE 2 (synchronism) — FAILED.** Sensorless handover then a 0 → rated load step at
1000 rpm, matched cell, advance off and on:

| advance | handover | FLL | peak `|θ̂−θ|` | saturation after step |
| --- | --- | --- | --- | --- |
| off | yes (0.217 s) | **guard-fail** | **86.0°** | 0.000 |
| on | yes (0.217 s) | **guard-fail** | **179.9°** | 0.548 |

The loop **loses synchronism ~48 ms after handover** (not at the load step): the
recorded `|θ̂−θ|` crosses 30° 486 samples after the switch and the FLL `ω'` diverges to
the DSOGI guard (`ω' < 0`). Traces: `experiments/output/06b_gate2/gate2.json`.

**Root cause (isolated).** Replacing one channel of the source with ground truth:

| angle | speed | result |
| --- | --- | --- |
| estimator | **true** | stable (0.52°) |
| **true** | estimator | **unstable (180°)** |
| estimator | estimator | unstable |

So the **estimated angle is fine**; the **estimated speed fed to the speed loop**
destabilises it. The Phase 06a speed loop is far faster than the estimator: crossover
≈ 97 Hz vs the Q-PLL `f_n = 20 Hz`. Raising `f_n` to 200 Hz does **not** fix it; the
loop is stable only for `α_ω ≲ 15 Hz` at **constant** load (5/10/15 Hz stable; ≥20 Hz
collapse). A **load step** destabilises even `α_ω = 10 Hz`.

Because Gate 2 fails, **Step 4 (scenario grid) and its sensitivity are not run**, per
the phase rule (stop on gate failure).

## Proposed fix (separate, approved step — not done here)

The estimated speed must be presented to the speed loop with less gain/phase penalty, or
the speed loop slowed and rate-limited:

1. filter or rate-limit the Q-PLL speed used by the speed loop (the FLL `ω'` helps but
   is not sufficient on its own: `α_ω=10 Hz` step peak 31°);
2. and/or set `α_ω` from the estimator bandwidth (`α_ω ≲ f_n/3`, e.g. 5–10 Hz) with a
   documented sensorless margin, accepting the slower load-step response;
3. re-examine the `ω̂`-based decoupling during transients (`ω̂` peaks of ~400 rpm feed
   `v_d = −ω̂ Ld i_q`).

This is a controller-side change, distinct from the estimator/chain (which is Gate-1
clean), and would be a new approved step.

## Limitations

- No dead time, PWM or current ripple (06c); no conventional-SMO baseline (06c).
- Handover thresholds/`t_lock`, the FLL initial `ω'`, and the angle advance are
  ASSUMPTIONS (D-053).
- The class-B scenario grid and paper comparison are **not reported**: Gate 2 failed
  before them.
