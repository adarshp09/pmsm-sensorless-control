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

---

# Phase 06b-2 — Estimator-aware speed loop + staged gate

Resolves the Gate-2 failure by measuring the estimator response and choosing the speed
loop from it. **No src change**; Q-PLL `f_n` variations are experiment-level overrides.
Scripts: `experiments/06b2_estimator_response.py` (Step 1),
`experiments/06b2_margins.py` (Step 2). Outputs under `experiments/output/06b2_*`.

## Step 0

**0a** — Phase 06b committed `51d35b8`, tagged `phase-06b-gate2-fail`. Repo:
`/Users/adarshpanjwani/Desktop/ACT/project/pmsm-sensorless-control`; branch `master`;
**no git remotes** (`git remote -v` empty).

**0b — paper quantity (rendered pages, PyMuPDF).** The quoted load-step numbers are
**speed *estimation* errors `|ω−ω̂|`**, not actual speed deviations:
- Sim 50 % (Fig.13, p.830): three panels "Speed(r/min)", **"Speed error(r/min)"**,
  "Rotor position error(rad)"; caption *"Speed and position estimation errors …"*.
  The 23 rpm dynamic / 5 rpm steady are the middle-panel error.
- Exp 20 % (Fig.19, p.832): y-axis "Rotor Speed(120 rpm/div)" with traces `N_ref`,
  `N_actual`, `N_err(10 rpm/div)` and an annotated **actual dip ΔN = 45/48 rpm**;
  Section V-D: *"the estimation error of the improved SMO remains around 10 r/min"* —
  the ~10 rpm is `N_err`.
- Load amplitudes confirmed printed: sim **50 %** rated at 1.5 s, exp **20 %** rated.
- **No** speed-loop PI gains/bandwidth (Fig.7 has no numbers), **no minimum speed**
  printed; Table III gives only the 1 kHz speed-loop rate. Start-up is qualitative
  (Section V-E). Table IV's `N_err` = 10 rpm is also an estimation error.
- Since no dynamic figure is an actual dip, `dip ∝ 1/α_ω` does **not** apply to the
  paper; for reference our **encoder** baseline dips 37 rpm at 50 % load / `α_ω = 50 Hz`
  (14.9 rpm at 20 %). Recorded in `estimator_spec.md` §Phase 06b-2 Step 0b.

**0c — process note.** `α_ω = 50 Hz` was frozen in Phase 06a as an **encoder-only**
design; Gate 2 (06b) then used the **rated** step, harsher than the paper's 20 %/50 %.
Neither was tuned against a sensorless loop — the failure is a design-scope mismatch,
not a tuning error.

## Step 1 — measured estimator response T_est

Open-loop nominal drive (04c generator, `I_q = I_rated`), sinusoidal speed
(`ε = 0.5 %`), matched cell, Ts = 100 µs; window = [0.25 s, end], ≥ 8 modulation periods.

| chain (f_n) | speed | −3 dB | phase @25 Hz | @50 Hz |
| --- | --- | --- | --- | --- |
| A (20 Hz) | 1000 | 41.7 Hz | −47.9° | −73.8° |
| A (200 Hz) | 1000 | >200 Hz | −0.4° | −2.2° |
| B (20 Hz) | 1000 | 83.0 Hz | −50.3° | −84.6° |
| B (50 Hz) | 1000 | 108.2 Hz | −11.8° | −47.4° |
| B (200 Hz) | 1000 | 116.3 Hz | −2.8° | −12.6° |
| A (20 Hz) | 500 | 26.2 Hz | −79.4° | −90.3° |
| B (20 Hz) | 500 | 41.4 Hz | −94.8° | −246.1° |
| B (200 Hz) | 500 | 48.9 Hz | −15.9° | −151.7° |

FLL status: **locked** in all runs. The response is **linear** (gain 1.3578/1.3579/1.3580
at f_m = 20 Hz for ε = 0.25/0.5/1 % → <0.02 % variation). Chain B shows a real resonance
(gain ≈ 1.36 near 20 Hz at 1000 rpm).

**H1 (DSOGI envelope caps T_est; raising f_n in B does not widen it) — SUPPORTED.** The
−3 dB saturates with f_n: at 1000 rpm B goes 83→108→116 Hz for f_n 20→50→200; at 500 rpm
41→49→49 Hz — a **speed-dependent cap** (≈2× for 2× speed), matching the envelope
`2/(k_s ω')` corner (≈47 Hz at 1000 rpm).

**H2 (chain A is markedly wider than B at equal f_n) — REFUTED.** At f_n = 20 Hz chain A
is **narrower** than B: −3 dB 41.7 vs 83.0 Hz at 1000 rpm and 26.2 vs 41.4 Hz at 500 rpm.
(The raw-`E_hat` Q-PLL rolls off like a 2nd-order loop; the filtered v′) has a resonance
that inflates its −3 dB even though its phase at 50 Hz is worse, −84.6° vs −73.8°.)

## Step 2 — margins and design rule

`L(jω) = L_enc(jω)` (06a discrete model: ζ = 1 PI, current loop + 1.5 Ts delay, 1 kHz
ZOH) `× T_est(jω)` (measured, log-interpolated, **no extrapolation** outside [2, 200] Hz).

Phase margin (°) for chain B (f_n = 20 Hz):

| α_ω | 500 rpm PM (GM) | 1000 rpm PM (GM) |
| --- | --- | --- |
| 5 | 10.8 (33 dB) | 54.1 (192 dB) |
| 7.5 | −21.1 | 30.0 |
| 10 | −48.5 | 14.6 (57 dB) |
| 15 | −72.6 | −8.2 |
| 20 | −90.7 | −25.7 |
| 30 | −118.4 | −89.3 |
| 50 | −157.7 | −194.6 |

(Chain A is similar: 26.8° at 500 rpm / 56.2° at 1000 rpm for α_ω = 5, falling with α_ω.)

**Design rule (ASSUMPTION, frozen before Step 3):** `α_ω*` = the largest grid value with
PM ≥ 45° and GM ≥ 6 dB at **both** speeds for chain B at f_n = 20 Hz.
**No grid value qualifies** (500 rpm is already 10.8° at α_ω = 5 Hz) ⟹
**`α_ω* = None` ⟹ STOP** (per the phase rule: "if no feasible α_ω, stop and report").

**Predicted vs time-domain boundary.** The margin model (chain B, 1000 rpm) crosses
PM = 0 between α_ω = 10 (14.6°) and 15 Hz (−8.2°) — i.e. it predicts marginal stability
at ~10–12 Hz, consistent with the 06b time-domain finding (constant-load stable ≤ 15 Hz,
unstable ≥ 20 Hz) but **conservative by ~one grid step**. At 500 rpm the model is much
worse (10.8° even at 5 Hz) because T_est is narrower there (envelope ∝ speed); 06b did
not test 500 rpm in the loop.

## Outcome — STOP at Step 2

With the frozen 05b estimator (`f_n = 20 Hz`) and the measured chain-B response, **no
speed-loop bandwidth in the required grid meets PM ≥ 45° / GM ≥ 6 dB at both 500 and
1000 rpm**. Steps 3 (staged gate) and 4 (scenarios/sensitivity) are **not run**.

The exhaustive-but-disallowed alternatives (filtering/rate-limiting `ω̂`, extrapolating
`T_est` below 2 Hz, or lowering PM below 45°) are out of scope per the phase rules. The
next step needs a decision: either accept a lower PM at 500 rpm (state the margin), or
change the estimator (e.g. a faster-phase chain / higher `f_n` with its DSOGI cap) — both
are separate, approved changes.

---

# Phase 06b-3 — operating-point decoupling and low-speed relaxation

Tests the proposed decoupled speed-loop rule (5 Hz at 500 rpm with relaxed PM ≥ 10°;
10 Hz at 1000 rpm) against the margin model, then the staged gate. **No src change**;
config: `configs/drive_assumptions.yaml`. Scripts: `experiments/06b3_margins.py`,
`experiments/06b3_gate.py`.

**Step 0** — Phase 06b-2 committed `c24ce8d`, tagged `phase-06b2-complete`. Justification
for decoupling / low-speed PM relaxation recorded in `estimator_spec.md` (§Phase 06b-3)
and D-059.

## Step 1 — margin verification (`06b3_margins.py`)

`L = L_enc × T_est` (chain B, f_n = 20 Hz, 06b-2 T_est):

| operating point | PM | GM | crossover | target | result |
| --- | --- | --- | --- | --- | --- |
| 500 rpm, α_ω = 5 Hz | 10.8° | 33.3 dB | 16.4 Hz | PM ≥ 10° | **PASS** |
| 1000 rpm, α_ω = 10 Hz | **14.6°** | 56.7 dB | 25.2 Hz | PM ≥ 45° | **FAIL** |

The 1000 rpm point at 10 Hz is **stable but below the 45° target**; the 45°-meeting
bandwidth at 1000 rpm is 5 Hz (PM 54.1°). The margin model is ~one grid step
conservative versus the 06b time domain (stable ≤ 15 Hz), so Step 2 is the arbiter.

## Step 2 — staged gate (`06b3_gate.py`)

Handover threshold **ω_h lowered 500 → 400 rpm**: at a 500 rpm operating point a
critically damped (ζ = 1) loop never overshoots, so `ω̂` cannot exceed a threshold equal
to the operating speed.

| gate | 500 rpm | 1000 rpm |
| --- | --- | --- |
| 2a steady (no-load, 20 %) | **PASS** (peak 0.4°, steady 0.3°, no sat) | **PASS** (peak 0.7°, steady 0.6°, no sat) |
| 2b 20 % load step | **FAIL** — FLL guard-fail, peak 71.5°, dip −223.7 rpm | **PASS** — peak 7.5°, dip −79.6 rpm, recover 64 ms, no sat |
| 2c 50 % (rated) load step | — | **PASS** — peak 21.7°, dip −210 rpm, recover 95 ms, no sat |

Advance off and on behave the same. **GATE 2 = FAIL** (2b at 500 rpm).

**Diagnosis.** Steady state is stable at both speeds (the relaxed 10.8° PM at 500 rpm is
enough for handover and regulation), and the 1000 rpm load steps pass. The **500 rpm
*load step*** excites the lightly damped low-speed loop (PM 10.8°) past synchronism. The
paper's own class-B load steps are at **1000 rpm** (sim 50 % / exp 20 %), which pass; the
500 rpm load step is the extra gate from this phase.

**Per the rules → STOP** (gate failure). Steps 3–4 (scenario suite, sensitivity) are not
run. A fix would need `α_ω < 5 Hz` at 500 rpm (outside the agreed grid) or a faster
low-speed estimator phase — both separate, approved changes; or restrict the 500 rpm
operating point to steady-state / no-load transients only.

## Outcome

- **Decoupling works for steady state:** Gate 2a passes at 500 rpm (5 Hz) and 1000 rpm
  (10 Hz); 2b/2c pass at 1000 rpm.
- **Low-speed load transient fails:** the relaxed 10.8° PM at 500 rpm cannot absorb a
  20 % load step.
- Gate 2 not met → **STOP**; no class-B scenario suite run; Phase 06c not started.

---

# Phase 06b-4 — softened low-speed bandwidth (2.5 Hz)

Softens the 500 rpm bandwidth to `α_ω = 2.5 Hz` (1000 rpm retained at 10 Hz) to raise the
low-speed phase margin. **No src change**; config + experiment only.
Scripts: `experiments/06b4_margins.py`, `experiments/06b4_gate.py`.

**Step 0** — Phase 06b-3 committed `9be88ec`, tagged `phase-06b3-complete`; justification
in `estimator_spec.md` (§Phase 06b-4) and D-061.

## Step 1 — margin verification (`06b4_margins.py`)

`L = L_enc × T_est` (chain B, f_n = 20 Hz, 06b-2 T_est):

| 500 rpm α_ω | PM | GM | fc |
| --- | --- | --- | --- |
| 1.5 Hz | 74.0° | 258 dB | 3.4 Hz |
| 2.0 Hz | 72.8° | 206 dB | 4.7 Hz |
| **2.5 Hz** | **65.5°** | **165 dB** | **6.6 Hz** |
| 3.0 Hz | 57.0° | 131 dB | 9.1 Hz |
| 4.0 Hz | 29.4° | 77 dB | 13.4 Hz |
| 5.0 Hz | 10.8° | 33 dB | 16.4 Hz |

**Target PM ≥ 30° at 2.5 Hz — PASS (65.5°).** 1000 rpm @ 10 Hz unchanged (PM 14.6°).

## Step 2 — staged gate (`06b4_gate.py`)

| gate | 500 rpm | 1000 rpm (regression) |
| --- | --- | --- |
| 2a steady (no-load) | **PASS** (peak/steady 0.3°, no sat) | **PASS** (0.6°) |
| 2a steady (20 % load) | **FAIL** — start-up-under-load trips the FLL at 60 ms | **PASS** (0.6°) |
| 2b 20 % load step | **FAIL** — FLL guard-fail 36 ms after step; peak 35.9°, steady 7.3°, dip −215 rpm | **PASS** (peak 7.5°, dip −79.6 rpm, 64 ms) |
| 2c 50 % step | — | **PASS** (peak 21.7°, dip −210 rpm, 95 ms) |

Advance off/on same. **GATE 2 = FAIL** (500 rpm load transients).

**Diagnosis.** The softened loop is well damped (PM 65.5°), so the 500 rpm failure is
**not** the speed loop: the **unclamped DSOGI-FLL loses lock** (`ω′ < 0`) on the torque
transient. At 500 rpm |E| = 27.8 V (vs 55.5 V at 1000 rpm), so the FLL's amplitude-driven
error/gain is ~4× smaller and the transient drives `ω′` negative. 1000 rpm is clean.

**Per the rules → STOP.** The class-B scenario suite and sensitivity (Step 3 of 06b-3)
are not run; Phase 06c not started. Next lever: `min_omega` clamping on the FLL (a
non-default library option; would be a separate approved change), or a start-up/handover
strategy that applies load only above ~1000 rpm.

---

# Phase 06b-5 — DSOGI-FLL minimum-frequency clamp

Enables the FLL `min_omega` floor to stop `ω′ < 0` collapse on low-speed transients.
**No src change** (the FLL already supports `min_omega`); config + experiment only.
Script: `experiments/06b5_gate.py`.

**Step 0** — Phase 06b-4 committed `4052dea`, tagged `phase-06b4-complete`; justification
in `estimator_spec.md` (§Phase 06b-5) and D-062.

## Step 1 — clamp configuration and static check

`configs/drive_assumptions.yaml`: `fll.min_omega_rpm = 100` (mechanical) →
`ω′ₘᵢₙ = p·ω_m = 41.9 rad/s`; `chain.build_fll(initial_omega, *, min_omega=...)`.
**Static open-loop check — IDENTICAL**: with the floor far below the operating frequency,
the clamped and unclamped chains are bit-identical at 500 and 1000 rpm
(`max|ΔE_hat| = 0`, `max|Δf| = 0`); the clamp never engages normally.

## Step 2 — staged gate (`06b5_gate.py`)

| gate | 500 rpm | 1000 rpm |
| --- | --- | --- |
| 2a steady (no-load, 20 % incl. start-up-under-load) | **PASS** (peak 0.4°, steady 0.3°, no sat, no guard trip) | **PASS** (0.7°, 0.6°) |
| 2b 20 % load step | **FAIL** — no guard trip, but diverges/pins (see sweep) | **PASS** (peak 7.5°, dip −79.6 rpm, 64 ms) |
| 2c 50 % step | — | **PASS** (peak 21.7°, dip −210 rpm, 95 ms) |

Advance off/on same. **GATE 2 = FAIL** (500 rpm load step).

**500 rpm 20 % step — floor sweep** (this is the crux):

| `min_omega` floor | result |
| --- | --- |
| none | guard trip, peak 35.9° |
| 100 rpm (configured) | **diverges** — peak 180°, saturation 0.89 |
| 200 rpm | **diverges** — peak 180° |
| 300 rpm | recovers, no sat, but **persistent 36°** error (FLL pinned at the floor) |
| 400 rpm | **diverges** — peak 180° |

**Diagnosis.** The clamp does what it says — it **removes the guard trip** and fixes
start-up-under-load (2a now passes with a 20 % load from rest, which failed in 06b-4) — but
it does **not** restore low-speed tracking on a torque transient: it changes the failure
mode from "guard trip" to "pinned at the floor / divergence". At 500 rpm the FLL's
amplitude-driven gain is too weak to re-lock after the step; a floor only stops `ω′` going
negative.

**Outcome.** **GATE 2 = FAIL** at 500 rpm loaded → **STOP**; the class-B scenario suite
and sensitivity are not run; Phase 06c not started. All **paper** class-A/B scenarios
(500 rpm no-load; 1000 rpm 20 %/50 % loaded steps) pass. A genuine low-speed loaded
capability needs a stronger estimator (higher Γ, error normalization, or an
amplitude-compensated FLL) — a separate, approved change.
