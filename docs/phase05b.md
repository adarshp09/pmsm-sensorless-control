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
| ramp (settled 0.75–1.5 s) | 100 µs | −1.10 / 1.14 | 0.15 | — |
| ramp (settled) | 50 µs | −1.68 / 1.74 | 0.16 | — |

Lock-in time scales as 1/|E| (loop gain ∝ amplitude): 0.61 s at 100 rpm → 0.030 s at
2000 rpm. The angle bias grows with speed (observer discrete lag, see the budget).

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
| no-load variable speed (sim/exp) | sim: ≈5 rpm, 0.07 rad, 0.2 rad dyn; exp: 15 rpm/0.1256 rad @500, 10 rpm/0.08 rad @1000 | settled: angle mean −1.10/−1.68°, max 1.14/1.74°; speed max 0.15 rpm | **open-loop ideal-FOC feedforward**, not closed-loop speed/position control; our `E_hat` = matched-sigmoid SMO (paper: FSMO); **assumed** QPLL gains; ideal voltage, no dead time/PWM |
| steady 1000 rpm (Table IV) | 0.08 rad, 10 rpm | angle mean −0.52/−1.17°, max 0.73/1.35°; speed max 3.40 rpm | same as above |

Our open-loop angle/speed errors are *smaller* than the paper's closed-loop figures,
but the setups are not comparable (open-loop vs closed-loop, ideal voltage vs
inverter, our assumed PLL bandwidth). **No claim of reproducing the paper.**

## Limitations

- No controller, no load, no inverter/dead-time, no mechanical model (class B deferred).
- QPLL gains, discretization, normalization and initialization are ASSUMPTIONS.
- Low-speed (100–200 rpm) angle error is limited by the small back-EMF; the arctan
  estimator on the raw sign `E_hat` is unusable (wrap glitches).
