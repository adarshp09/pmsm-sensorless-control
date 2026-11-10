# Phase 06a — Closed-loop drive with encoder (true-angle) feedback

A closed-loop FOC drive with the **true** angle/speed fed back (an "encoder baseline"),
so the controller and the mechanical loop are validated **before** any estimator enters
the loop (Phase 06b). No observer, filter or estimator is used.

Code: `src/pmsm_sensorless/simulation/drive.py`. Configs: `configs/drive.yaml`
(printed), `configs/drive_assumptions.yaml` (all ASSUMPTION — not from paper).
Scripts: `experiments/06a_step0_analysis.py` (Step 0), `experiments/06a_encoder_baseline.py`
(Step 3). Output: `experiments/output/06a_encoder_baseline/encoder_baseline.json`,
`experiments/output/06a_step0/step0_analysis.json`.

**No tuning toward the paper's numbers.** Every gain comes from the stated design
formulas, frozen before any scenario is run.

## Step 0 (docs/experiments only)

**0a** — `phase-05b` tagged at HEAD (`aa95001`).

**0b — metric basis.** Every angle/speed row in `phase05b.md` is **pointwise**
(`_angle_metrics`/`_speed_metrics`: mean/std/RMS/max over a window mask), constant-speed
over the last 8 cycles and ramp over the settled window [0.75, 1.5] s. The rows
therefore **share one basis**; the earlier premise that the ramp rows were windowed is
**corrected**. The ~20× gap in *speed* max (2.8–9.4 rpm const vs 0.15 rpm ramp) is
physical (constant-speed QPLL speed ripple vs a smooth ramp track), not a metric
artefact. The ramp rows recomputed pointwise (`06a_step0_analysis.py`, which reproduces
`estimator_eval.json` exactly, `reproduces_json = True`):

| input | angle mean / RMS / max (deg) | speed mean / RMS / max (rpm) |
| --- | --- | --- |
| ramp, 100 µs | −1.1025 / 1.1026 / 1.1391 | +0.0025 / 0.0666 / 0.1475 |
| ramp, 50 µs | −1.6780 / 1.6783 / 1.7435 | −0.0057 / 0.0667 / 0.1556 |

The paper-comparison table in `phase05b.md` is re-stated with these (the old ramp
figures are marked superseded).

**0c — angle-bias model.** Measured steady bias (nominal cell) vs the *independent*
model `−atan(ω_e Ld/(Rs+K_eff)) + ω_e Ts/2` (`K_eff = k0 a0 kF aF/2 = 19.36`):

| input | measured | −atan(…) | +ω Ts/2 | predicted | residual |
| --- | --- | --- | --- | --- | --- |
| 100 rpm, 100 µs | +0.014 | −0.178 | +0.120 | −0.058 | +0.072 |
| 500 rpm, 100 µs | −0.203 | −0.888 | +0.600 | −0.288 | +0.085 |
| 1000 rpm, 100 µs | −0.520 | −1.775 | +1.200 | −0.575 | +0.055 |
| 2000 rpm, 100 µs | −1.451 | −3.547 | +2.400 | −1.147 | −0.304 |
| 1000 rpm, 50 µs | −1.168 | −1.775 | +0.600 | −1.175 | +0.008 |
| 2000 rpm, 50 µs | −2.684 | −3.547 | +1.200 | −2.347 | −0.336 |

**Half-sample hypothesis (confirmed):** evaluating the error against
`θ_true(t_k + Ts/2)` — the `+ω Ts/2` term — shrinks the residual from **+1.26°** (no
term, 1000 rpm/100 µs) to **+0.06°**. The model closes to ≲0.09° at 100–1000 rpm for
both Ts; at 2000 rpm −0.30/−0.34° remains (observer discrete lag beyond the model).

**Ramp bias** = constant-speed bias + the type-2 PLL ramp offset
`ω̇_e/ω_n² = 139.63/15791 = 0.507°`: 100 µs predicted −0.939° vs −1.103° measured
(residual −0.16°); 50 µs −1.388° vs −1.678° (−0.29°). Closes to ~0.2–0.3°.

> The Phase 05b "budget closes to ±0.001°" is an **identity at constant speed** (all
> terms evaluated on the same window must balance); it is not a model check. This
> bias-model check (measured vs an *independent* analytical form) is the informative one.

**0d — arctan on raw sign `E_hat` is unusable; no conventional-SMO baseline.** The
raw **sign** observer's `E_hat` is bang-bang, so `atan2(−E_α, E_β)` wrap-glitches (up to
0.5° max at 1000 rpm/100 µs, growing at the ripple frequency; see `phase05b.md`). A
"improved vs conventional" comparison also needs a conventional SMO **with** its LPF —
whose cutoff the paper leaves `null` (`smo.observed_emf_lpf_cutoff`) — so **no
conventional-SMO baseline exists yet**. Both are recorded as required caveats before any
"improved vs conventional" claim.

## Step 1 — the drive

Signal chain (Fig.7, p.828, printed): speed PI → `i_q*`; `i_d* = 0`; two current PIs →
`dq/αβ` → voltage limit → inverter → PMSM. The controller imports **no**
observer/filter/fuzzy/estimation module; it receives the sampled `i_αβ` and an
`(θ_e, ω_e)` pair through a pluggable `AngleSpeedSource`. Only `TrueSource`
(**encoder baseline**) reads the plant state.

- **Plant:** existing mechanical option (`PMSMPlant(integrate_mechanics=True)`), `J`
  printed, viscous friction `b = 0` (ASSUMPTION), load via `plant/load_models.py`, RK4
  with 4 sub-steps per `Ts`, voltage ZOH.
- **Inverter (ASSUMPTION):** average-value, `V_dc = 311 V`, phase-voltage limit
  `V_dc/√3 = 179.56 V`, **one-sample computation delay** (voltage computed at `k` is
  applied over `[k+1, k+2)`). No dead time / PWM (printed 2 µs / 10 kHz → Phase 06c).
- **Current loops (10 kHz, printed):** `Kp_c = α_c Ld = 4.618 V/A`,
  `Ki_c = α_c Rs = 1586.5 V/(A·s)`, `α_c = 2π·500 Hz` (ASSUMPTION). Decoupling **ON**,
  back-EMF feed-forward **OFF** (labelled). Anti-windup: conditional integration with
  the integrator/output clamped at the voltage limit.
- **Speed loop (1 kHz, printed):** `Kp_w = 2ζα_ω J/Kt = 1.208 A/(rad/s)`,
  `Ki_w = α_ω² J/Kt = 189.8 A/rad`, `Kt = ½·3 p ψ_f = 0.7956 N·m/A`, `ζ = 1`,
  `α_ω = 2π·50 Hz` (ASSUMPTION), `i_q*` clamp `±2 I_rated = ±19.36 A` (ASSUMPTION).

### Loop margins actually obtained from the discrete loops

| loop | design | discrete crossover | phase margin |
| --- | --- | --- | --- |
| current (incl. 1-sample delay) | α_c = 500 Hz | **493.4 Hz** | **63.2°** |
| speed (ZOH ½-sample delay) | α_ω = 50 Hz (natural freq.) | **97.0 Hz** | **39.9°** |

The current-loop PM matches the design estimate (90° − 2π·500·1.5·Ts = 63°). The speed
loop's *crossover* (97 Hz) is ~2× its natural frequency — expected for these gains
(`|L|=1` at `ω ≈ 2.24 α_ω` for ζ = 1).

## Step 2 — tests and GATE

14 tests in `tests/test_drive.py` (analytical tolerances justified in-file): the current
step matches the **exact discrete ZOH + PI + 1-sample-delay model** to <1e-6 A (the
continuous `α_c/(s+α_c)` ideal is within ~29 %, the gap being `α_c Ts = 0.31`); the
speed step settles per `α_ω`; steady `i_q = T_L/Kt`, `i_d = 0`, power balance < 1 %;
voltage saturation leaves bounded overshoot; structure, determinism and config-null
`ConfigError` checks.

**GATE — PASSED.** Speed step 0 → 1000 rpm, no load: steady speed error 0 rpm (max
< 1e-9), `|i_d| < 3 % I_rated`, no saturation. Rated-load step at 1000 rpm: steady
speed error 0 rpm, `i_q = 9.6796 A` vs `T_L/Kt = 9.6782 A` (0.015 %), `|i_d| < 3 %
I_rated`, no saturation after the transient.

## Step 3 — scenarios

All class-B scenarios from `docs/estimator_spec.md` plus a speed step, rated-load steps
at 1000/500 rpm and the no-load variable-speed ramp. Each load/ramp scenario is one
continuous rollout with a short settle period before the disturbance; the speed step
starts from rest. **Speed reversal: not run — `estimator_spec.md` lists no reversal
scenario.**

| scenario | speed dip (rpm) | overshoot (rpm) | recovery to ±10 rpm (s) | recovery to ±1 % (s) | peak i_q (A) | settled i_d err (%I_r) | sat. frac |
| --- | --- | --- | --- | --- | --- | --- | --- |
| speed_step_0_1000 | n/a (step) | +12.87 (1.29 %) | — | 0.0212 (settle) | 19.44 | 0.00 | 0.000 |
| **load_step_rated_1000** (headline) | **−74.33** | +0.16 | **0.0118** | 0.0118 | 11.96 | 0.00 | 0.000 |
| load_step_rated_500 | −74.48 | +0.16 | 0.0117 | 0.0140 | 11.97 | 0.00 | 0.000 |
| sim_load_50pct_1000 (paper) | −37.16 | +0.08 | 0.0092 | 0.0092 | 5.98 | 0.00 | 0.000 |
| exp_load_20pct_1000 (paper, sim replica) | −14.86 | +0.03 | 0.0053 | 0.0053 | 2.39 | 0.00 | 0.000 |
| ramp_500_1000_noload | −0.54 | +0.55 | 0.0000 | 0.0000 | 0.08 | 0.00 | 0.000 |

Speed step: 90 % rise = 10.9 ms, 1 % settle = 21.2 ms. The dip scales with load
(−37 rpm at 50 %, −74 rpm at 100 %) and shrinks with speed-loop bandwidth. No scenario
saturates the voltage.

### Sensitivity (headline rated-load step @1000 rpm), α_c × α_ω

| α_c (Hz) | α_ω (Hz) | dip (rpm) | recovery ±10 rpm (s) | peak i_q (A) | sat. frac | stable |
| --- | --- | --- | --- | --- | --- | --- |
| 250 | 25 | −118.75 | 0.0301 | 10.98 | 0.000 | yes |
| 250 | 50 | −81.18 | 0.0119 | 12.94 | 0.000 | yes |
| 250 | 100 | −63.38 | 0.0080 | 18.88 | 0.000 | yes |
| 500 | 25 | −120.24 | 0.0301 | 11.10 | 0.000 | yes |
| **500** | **50** | **−74.33** | **0.0118** | **11.96** | 0.000 | yes |
| 500 | 100 | −57.46 | 0.0034 | 16.61 | 0.000 | yes |
| 1000 | 25 | −121.15 | 0.0301 | 11.18 | 0.000 | yes |
| 1000 | 50 | −70.42 | 0.0118 | 11.88 | 0.000 | yes |
| 1000 | 100 | −54.95 | 0.0038 | 17.88 | 0.000 | yes |

All 9 cells are **stable**; the dip is set almost entirely by the speed-loop bandwidth
(α_ω), and the current-loop bandwidth (α_c) has little effect (10× faster than α_ω).
Higher α_ω lowers the dip but raises peak `i_q`.

### Paper comparison (setup-limited; no reproduction claim)

| Scenario | Paper value | Ours (encoder baseline) | Main setup differences |
| --- | --- | --- | --- |
| sim no-load variable speed (500→1000) | max speed err ≈5 rpm; steady pos err ≈0.07 rad | ramp tracking dip −0.54 rpm, overshoot +0.55 rpm | paper reports *estimator* errors in closed loop; ours is the *control* track of an ideal encoder drive |
| sim 50 % load @1000 rpm | dynamic max speed err 23 rpm; steady mean speed err 5 rpm | speed dip −37.16 rpm, recovery 9.2 ms; steady err 0 | as above; paper adds inverter + estimator |
| exp 20 % load @1000 rpm | speed err ≈10 rpm; position err 0.152 rad | speed dip −14.86 rpm (`DEVIATION: paper is hardware; sim replica`) | as above |
| Table IV @1000 rpm | 0.08 rad, 10 rpm | steady control speed err ≈0, i_d ≈0 | as above |

> The paper's figures are **estimator errors** (position/speed estimation accuracy under
> closed-loop control); ours are **control** dips of a drive that already knows the true
> angle. They are **not the same quantity** and are **not comparable**; we make **no
> reproduction claim**. Our dips are set by the assumed speed-loop bandwidth, not by any
> estimator.

## Limitations

- Encoder baseline only: the true angle/speed is fed back; no observer/filter/estimator
  (that is Phase 06b).
- Average-value inverter: no PWM, no 2 µs dead time, no voltage drop, no current ripple.
- `V_dc`, delay, loop bandwidths, `b`, `i_q*` clamp and the anti-windup scheme are
  ASSUMPTIONS (D-047).
- The current loop's discrete response differs from the continuous `α_c/(s+α_c)` ideal
  at `α_c Ts = 0.31`; the discrete margins above are the honest figures.
