# Phase 05b — Position/speed estimator + open-loop evaluation

Implements the paper's two position estimators (Eq.(8) arctan and the Fig.1 Q-PLL)
and evaluates the chain **observer → filter → estimator** in open loop. **No
closed-loop controller, no tuning toward the paper's numbers.**

Scripts: `experiments/05b_estimator_eval.py`; `src/pmsm_sensorless/estimation/`.
Configs: `configs/estimator.yaml` (printed), `configs/estimator_assumptions.yaml`
(all ASSUMPTION). Output: `experiments/output/05b_estimator_eval/estimator_eval.json`.

## Step 0

**0a — Eqs.(18)–(19) confirmed (rendered page 827).** They are printed as
discrete-time **transfer functions** — numerator/denominator polynomials in
`z^-1`/`z^-2` (biquads), introduced by "The discretized transfer functions of digital
SOGI…". So the biquad is **kept**; the Fig.4 two-integrator realization remains a
documented sensitivity (D-040). No STOP.

**0b — scenario classification** (from `estimator_spec.md`):

| Scenario | Setup | Metric | Paper value | Class |
| --- | --- | --- | --- | --- |
| Sim no-load variable speed | 500→1000 r/min, 1.5 s | max speed err | ≈5 rpm | **A** |
| Sim no-load variable speed | idem | steady position err | ≈0.07 rad | **A** |
| Sim no-load variable speed | idem | dynamic position err | ≈0.2 rad | **A** |
| Sim 50 % load step @1000 | load step at 1.5 s | speed/position err | 5 rpm; 0.08–0.1 rad | B |
| Exp no-load variable speed | 500→1000 | speed/position err | 15 rpm, 0.1256 rad @500; 10 rpm, 0.08 rad @1000 | **A** |
| Exp 20 % load @1000 | 20 % load | speed/position err | ≈10 rpm; 0.152 rad | B |
| Table IV @1000 | steady 1000 rpm | position/speed err | 0.08 rad, 10 rpm | **A** |

Class **A** = kinematic/open-loop feasible (run here); class **B** needs the
closed-loop controller + mechanical/load model (deferred to Phase 06).

**0c** — `phase05a.md` §2b columns relabelled "… vs E_true", filtered−raw separate.
**0d** — `phase-05a` tagged.

## Step 1 — estimators (`estimation/`, no plant/observer/filter/fuzzy imports)

- **`ArctanEstimator`** — Eq.(8) as `atan2(-E_alpha, E_beta)`, wrapped to `(-pi, pi]`
  (D-044); exact for Eq.(2) over a full revolution (tested `|err| < 1e-12`).
- **`QPLL`** — Fig.1 structure; error `delta_e = -E_alpha cos(theta_hat) -
  E_beta sin(theta_hat) = |E| sin(theta - theta_hat)` (restoring). Predict-then-correct
  forward Euler; at lock `theta_hat[k] = theta[k]`.
- **ASSUMPTIONS (D-043):** `zeta = 0.707`, `f_n = 20 Hz` designed at `|E|` for
  1000 rpm (`Kp = 2 zeta w_n/A`, `Ki = w_n^2/A`, `A = 55.543 V`); normalization OFF;
  `theta_hat(0) = theta_true(0)`, `omega_hat(0) = theta_true(0) electric frequency`;
  no phase compensation. `omega_m = omega_hat / p` is DERIVED (`p` printed).
- Analytical tests (29 with the smoke test): arctan exact/wrap; Q-PLL zero steady error
  (<1e-3 rad), ramp lag = `omega_dot/(Ki*A)` (within 5 %), restoring sign, +90° lock-in,
  loop gain ∝ amplitude, normalization removes amplitude dependence; structure and
  determinism/reset.

## Step 2 — chains (nominal drive, `I_q = I_rated`)

192 runs: observers {sign, matched sigmoid (kF=1, aF=0.4), FLC nominal} × filters
{none, v′ (primary), PNS (matched/FLC only)} × estimators {arctan, QPLL} × Ts
{100, 50 µs} × {100/200/500/1000/2000 rpm, ramp 500→1000}. FLL at library defaults,
initial ω′ = ω_true at the window start. **FLL status: 0 guard-fail / 0 collapsed rows
in this cell set.**

### Nominal cell — `matched_sigmoid | v′ | QPLL`

| speed | Ts | angle err mean / max (deg) | speed err max (rpm) | lock-in @5° (s) |
| --- | --- | --- | --- | --- |
| 100 rpm | 100 µs | +0.01 / 5.56 | 9.19 | 0.613 |
| 200 rpm | 100 µs | −0.04 / 2.03 | 6.21 | 0.147 |
| 500 rpm | 100 µs | −0.20 / 1.44 | 9.41 | 0.082 |
| 1000 rpm | 100 µs | −0.52 / 0.73 | 3.40 | 0.037 |
| 2000 rpm | 100 µs | −1.45 / 1.47 | 2.81 | 0.030 |
| 100 rpm | 50 µs | −0.10 / 5.63 | 9.13 | 0.613 |
| 200 rpm | 50 µs | −0.21 / 2.18 | 6.13 | 0.147 |
| 500 rpm | 50 µs | −0.55 / 1.75 | 9.15 | 0.081 |
| 1000 rpm | 50 µs | −1.17 / 1.35 | 2.91 | 0.038 |
| 2000 rpm | 50 µs | −2.68 / 2.70 | 2.77 | 0.032 |
| ramp (settled 0.75–1.5 s) | 100 µs | −1.10 / 1.14 | 0.15 ᵃ | — |
| ramp (settled) | 50 µs | −1.68 / 1.74 | 0.16 ᵃ | — |

Lock-in time scales as 1/|E| (loop gain ∝ amplitude): 0.61 s at 100 rpm → 0.030 s at
2000 rpm. The angle bias grows with speed (observer discrete lag, see the budget).

### Step 0b (Phase 06a) — metric basis and the ramp rows recomputed pointwise

Every angle/speed row in this document is **pointwise**: the same
`_angle_metrics`/`_speed_metrics` helpers (mean/std/RMS/max of the wrapped error over a
window mask) are used for the constant-speed rows (last 8 cycles) and the ramp rows
(settled window [0.75, 1.5] s). No row is a windowed fit, so the constant-speed and
ramp rows **do share one basis**. The ~20× gap in speed max (2.8–9.4 rpm at constant
speed vs 0.15 rpm on the ramp) is therefore **physical, not a metric artefact**: the
constant-speed QPLL speed estimate carries a few-rpm ripple driven by the
filtered-signal ripple, whereas the ramp estimate tracks smoothly. (This corrects the
Phase 06a Step 0b premise that the two could not share a basis.)

The ramp rows recomputed pointwise by `experiments/06a_step0_analysis.py` (which
reproduces `estimator_eval.json` exactly, `reproduces_json = True`):

| input | angle mean / RMS / max (deg) | speed mean / RMS / max (rpm) |
| --- | --- | --- |
| ramp, 100 µs | −1.1025 / 1.1026 / 1.1391 ᵃ | +0.0025 / 0.0666 / 0.1475 ᵇ |
| ramp, 50 µs | −1.6780 / 1.6783 / 1.7435 ᵃ | −0.0057 / 0.0667 / 0.1556 ᵇ |

ᵃ Supersedes the earlier ramp angle "−1.10/1.14" and "−1.68/1.74" (identical to 2 d.p.,
now stated with RMS and as pointwise). ᵇ Supersedes the earlier ramp speed "0.15/0.16
rpm" (identical max, now stated with mean/RMS).

**Angle bias is large at constant speed and small on the ramp?** No: the *angle* rows
are consistent (ramp mean −1.10° vs const-1000 −0.52°). Only the *speed* ripple differs,
for the physical reason above.

### Step 0c (Phase 06a) — angle-bias model check (no new runs)

For the nominal cell the measured steady angle bias is compared with
`−atan(ω_e Ld/(Rs+K_eff)) + ω_e Ts/2` (`K_eff = k0 a0 kF aF/2 = 19.36`; the `+ω_e Ts/2`
is the half-sample term, i.e. evaluating the error against `θ_true(t_k + Ts/2)`):

| input | measured | −atan(…) | +ω Ts/2 | predicted | residual |
| --- | --- | --- | --- | --- | --- |
| 100 rpm, 100 µs | +0.014 | −0.178 | +0.120 | −0.058 | +0.072 |
| 500 rpm, 100 µs | −0.203 | −0.888 | +0.600 | −0.288 | +0.085 |
| 1000 rpm, 100 µs | −0.520 | −1.775 | +1.200 | −0.575 | +0.055 |
| 2000 rpm, 100 µs | −1.451 | −3.547 | +2.400 | −1.147 | −0.304 |
| 1000 rpm, 50 µs | −1.168 | −1.775 | +0.600 | −1.175 | +0.008 |
| 2000 rpm, 50 µs | −2.684 | −3.547 | +1.200 | −2.347 | −0.336 |

The **half-sample hypothesis holds**: without the `+ω Ts/2` term the residual would be
`measured + atan(…)` = +1.26° at 1000 rpm/100 µs; with it the residual is +0.06°. The
model closes to ≲0.09° at 100–1000 rpm for both Ts; at 2000 rpm it leaves −0.30/−0.34°
(the observer's discrete lag beyond the half-sample model).

The ramp bias is the constant-speed bias plus the type-2 PLL ramp offset
`ω̇_e/ω_n² = 139.63/15791 = 0.507°` (`ω̇_e = p·500 rpm/1.5 s`, `ω_n = 2π·20 Hz`):
100 µs: −0.432 − 0.507 = −0.939° predicted vs −1.103° measured (residual −0.16°);
50 µs: −0.882 − 0.507 = −1.388° vs −1.678° (−0.29°). So the ramp bias closes to
~0.2–0.3°.

> **Note (`docs/phase06a.md` §Step 0):** the earlier "bias budget closes to ±0.001°"
> is an **identity at constant speed** — with the whole budget evaluated on the same
> settled window it must balance; it says nothing about the model. This bias-model check
> (measured vs an *independent* analytical model) is the informative one.

**Filter comparison at 1000 rpm, Ts = 100 µs** (QPLL): raw phase error grows with
speed, so the filter helps the sign observer (raw `none` gives wrap glitches);
the matched-sigmoid cell is already clean:

| observer | filter | angle mean / max (deg) | speed max (rpm) |
| --- | --- | --- | --- |
| sign | none | −0.38 / 4.28 | 1949 (raw bang-bang) |
| sign | v′ | −0.26 / 1.32 | 90.8 |
| matched sigmoid | none | −0.52 / 0.52 | 0.32 |
| matched sigmoid | v′ | −0.52 / 0.73 | 3.40 |
| FLC nominal | none | −1.83 / 1.97 | 31.1 |
| FLC nominal | v′ | −1.82 / 1.96 | 5.98 |

`arctan` and `QPLL` agree (matched sigmoid, v′, 1000 rpm: max 0.92° vs 0.73°); the
QPLL is the smoother estimate.

### Angle-error budget (steady runs)

`measured_bias ≈ [raw E_hat phase] + [filter phase vs raw] + [estimator lag]`, all
referred to `E_true`. **The budget closes within ±0.5° for 137/160 steady runs**;
the nominal cells close to ±0.001°. The 23 rows outside come from the **raw sign**
observer (bang-bang `E_hat` → `atan2` wrap glitches and FLL jitter), not the estimator.
Nominal cell (matched|v′|QPLL, 1000 rpm, 100 µs): raw −0.517, filter +0.006,
QPLL lag −0.010, measured −0.520, residual 0.000.

**Stable-sigmoid raw phase vs the linear-observer lag** `-atan(omega Ld/(Rs+K_eff))`
with `K_eff = k a/2 = 19.36`:

| speed | measured raw phase (100/50 µs) | linear lag | diff (100/50 µs) |
| --- | --- | --- | --- |
| 100 rpm | +0.05 / −0.07 | −0.18 | +0.23 / +0.11 |
| 500 rpm | −0.19 / −0.54 | −0.89 | +0.70 / +0.35 |
| 1000 rpm | −0.52 / −1.17 | −1.78 | +1.26 / +0.61 |
| 2000 rpm | −1.45 / −2.68 | −3.55 | +2.10 / +0.86 |

The continuous linear lag over-predicts the observer phase lag; the residual grows
with `Ts` and speed — the forward-Euler observer discretization (D-016/D-027), so the
budget does **not** close against this analytical form at Ts = 100 µs (up to +2.1°).

### Q-PLL `f_n` sensitivity (matched cell, v′, ramp & 1000 rpm)

| multiplier | 1000 rpm mean / max (deg) | ramp mean / max (deg) |
| --- | --- | --- |
| 0.1× | −1.13 / 1.32 | **−15.6 / 180** (loses lock) |
| 1× (nominal) | **−0.52 / 0.73** | **−1.31 / 11.96** |
| 10× | −0.51 / 0.97 | −0.62 / 25.6 (jitter) |

1× is the best compromise; 0.1× is too slow to track the ramp, 10× trades mean lag
for jitter.

### Paper comparison (class A; no tuning)

| Scenario | Paper value | Ours (nominal cell) | Main setup differences |
| --- | --- | --- | --- |
| no-load variable speed (sim/exp) | sim: ≈5 rpm, 0.07 rad, 0.2 rad dyn; exp: 15 rpm/0.1256 rad @500, 10 rpm/0.08 rad @1000 | settled **pointwise** (Step 0b): angle mean/RMS/max −1.10/1.10/1.14° (100 µs), −1.68/1.68/1.74° (50 µs); speed max 0.1475/0.1556 rpm | **open-loop ideal-FOC feedforward**, not closed-loop speed/position control; our `E_hat` = matched-sigmoid SMO (paper: FSMO); **assumed** QPLL gains; ideal voltage, no dead time/PWM |
| steady 1000 rpm (Table IV) | 0.08 rad, 10 rpm | angle mean −0.52/−1.17°, max 0.73/1.35°; speed max 3.40 rpm | same as above |

Our open-loop angle/speed errors are *smaller* than the paper's closed-loop figures,
but the setups are not comparable (open-loop vs closed-loop, ideal voltage vs
inverter, our assumed PLL bandwidth). **No claim of reproducing the paper.**

## Limitations

- No controller, no load, no inverter/dead-time, no mechanical model (class B deferred).
- QPLL gains, discretization, normalization and initialization are ASSUMPTIONS.
- Low-speed (100–200 rpm) angle error is limited by the small back-EMF; the arctan
  estimator on the raw sign `E_hat` is unusable (wrap glitches).
