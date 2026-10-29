# Phase 05a — Filter prelude (Step 2–3 findings)

Diagnosis of the closed-loop DSOGI-FLL phase offset and frequency bias, plus a SOGI
realization check. **No estimator, no position/speed claims.** Script:
`experiments/05a_filter_prelude.py`; output
`experiments/output/05a_filter_prelude/filter_prelude.json`.

Inputs: raw `E_hat` of the fixed **sign** observer at Ts = 100/50/25 µs and the
fixed **sigmoid** at 50 µs (stable), 03e scenario (short-circuited plant, 500 → 1000
rpm ramp). Steady window = 8 whole electrical cycles at 1000 rpm = [1.880, 2.000] s.
**FLL at library defaults:** `epsilon_filter_tau = OFF`, `min_omega`/`max_omega =
None` (no clamps), `normalize_error = False`, `positive_sequence = True`; initial
`omega'` = true electrical frequency at the window start (500 rpm) = 209.44 rad/s.
ω_e (1000 rpm) = 418.88 rad/s; f_e = 66.67 Hz.

## Step 2a — frozen ω′: the SOGI transfer is `D(jω)` / `½D(1+ω′/ω)`

FLL off; ω′ frozen at ω_true × {0.9, 1.0, 1.135}. Because the SOGI is linear at
frozen ω′, the **transfer** is the output fundamental over the *input* fundamental
(compared against `E_true` it would instead carry the observer's own gain/phase).

| input | max gain error | max phase error |
| --- | --- | --- |
| sign 100 µs | 1.78e-5 | 0.0033° |
| sign 50 µs | 4.4e-6 | 0.0008° |
| sign 25 µs | 1.1e-6 | 0.0002° |
| sigmoid 50 µs | 4.4e-6 | 0.0008° |

The static theory holds to the discrete-SOGI accuracy (~1e-5 / 0.003°; the permanent
`test_sogi_off_resonance` bounds it at ≤2e-3 over ω/ω′ ∈ [0.8, 1.25]). Tolerance
justification: the residual is the prewarped-bilinear discretization error, which is
<1e-5 at these ratios.

## Step 2b — band-split replay of ω′(t)

Closed-loop ω′(t) recorded; replay with ω′(t) replaced by (i) its **steady-window
mean** (the full-run mean is skewed by the ramp), (ii) mean + components below
ω_e/2 = 33.3 Hz, (iii) mean + components in [ω_e/2, 3ω_e] = [33.3, 200] Hz, (iv)
full, (v) mean + components above 3ω_e (diagnostic). **(iv) reproduces the closed
loop bit-for-bit** (checked, `repro=True`). PNS output **fundamental phase vs E_true**
(deg). The closed-loop `filtered − raw` phase is a **separate** quantity (last column)
and is **not** vs E_true.

| input | (i) mean vs E_true | (ii) +low vs E_true | (iii) +mid vs E_true | (v) +high vs E_true | (iv) full vs E_true | closed filtered−raw |
| --- | --- | --- | --- | --- | --- | --- |
| sign 100 µs | +7.45 | +7.11 | +7.27 | **−7.17** | −7.44 | −5.86 |
| sign 50 µs | +4.95 | +4.66 | +5.08 | **−3.90** | −3.92 | −3.06 |
| sign 25 µs | +2.25 | +1.99 | +2.39 | **−1.71** | −1.71 | −1.32 |
| sigmoid 50 µs | −0.38 | −0.61 | −0.24 | −0.38 | −0.38 | +0.00 |

**Finding:** the −5.9° offset (and its 50/25 µs companions) is created by the
**high-frequency band of ω′(t) above 3ω_e ≈ 200 Hz** — the row (v) matches (iv)
almost exactly, while the mean, low and mid bands each give the *static* value
(+7.45 at 100 µs). The high band is the fast ω′ jitter/aliasing driven by the sign
observer's bang-bang ripple. The stable sigmoid's ω′ is clean (all variants agree,
no offset). The v′ output shows the same pattern.

## Step 2c — realization check: biquad vs the paper's two-integrator SOGI

Reference: `v̇′ = ω′[k_s(v − v′) − qv′]`, `q̇v′ = ω′ v′`, each integrator discretized
with Eq.(17) (`T = tan(ω′Ts/2)/ω′`, trapezoidal accumulation), algebraic loop solved
in closed form (experiment-only class `TwoIntegratorSOGI`).
**Paper figure:** Fig.4 (p.826) shows exactly this two-integrator form.

| Ts | constant-ω′ equivalence (max rel gain / phase) | closed-loop ω′ bias, two-int vs biquad |
| --- | --- | --- |
| 100 µs (sign) | 9.6e-15 / 6.7e-12° | **+9.468 vs +7.912 Hz** (Δ = 1.56 Hz) |
| 50 µs (sign) | 4.2e-14 / 1.7e-11° | +5.166 vs +4.974 Hz (Δ = 0.19 Hz) |
| 25 µs (sign) | 9.2e-14 / 3.3e-11° | +2.230 vs +2.201 Hz (Δ = 0.03 Hz) |
| 50 µs (sigmoid) | 4.1e-14 / 1.6e-11° | +0.000 vs +0.000 Hz |

**Finding:** the two-integrator realization is **identical to the biquad at constant
ω′** (≤1e-14, satisfying ≤1e-9), but in the closed loop — where ω′ is time-varying —
it gives a **materially different ω′ bias at Ts = 100 µs** (1.56 Hz ≈ 20 % of the
bias), converging as Ts shrinks. This is a realization-sensitivity to the
time-varying coefficient. Recorded as **D-040**; adopting the paper's Figure-4
structure is proposed as a separate approved step (no src change here).

## Step 3 — ε_f(ω′) sweep and the zero crossing

Frozen ω′ = ω_true × {0.80 … 1.30, step 0.025}; mean ε_f over the steady window for
the real raw `E_hat`, split exactly (the SOGI is linear at frozen ω′) into
fund×fund, fund×ripple, ripple×fund, ripple×ripple (`v = fitted fundamental +
residual`). The four-term split at the operating point: fund×fund is the large
*restoring* term; fund×ripple and ripple×fund are ~0 (the ripple is not correlated
with the fundamental over a whole number of cycles); ripple×ripple is the small
*biasing* term.

| input | ε_f(ω′) zero crossing (bias, Hz) | closed-loop mean bias (Hz) | static explains |
| --- | --- | --- | --- |
| sign 100 µs | +0.220 | +7.912 | **2.8 %** (36× too small) |
| sign 50 µs | +0.057 | +4.974 | **1.1 %** (87× too small) |
| sign 25 µs | +0.014 | +2.201 | **0.6 %** (157× too small) |
| sigmoid 50 µs | +6.7e-7 | +0.000 | **~100 %** (bias ≈ 0) |

**Finding:** the **static ε_f(ω′) balance point explains the stable-sigmoid result**
(bias ≈ 0, crossing at ω_true) but **not the sign-observer bias** — the crossing is
far outside 20 % of the closed-loop mean. Dynamic effects (the ω′ trajectory) remain,
consistent with `phase03.md` §14.5 by an independent method. Plot:
`eps_f_sweep.png` (closed-loop mean marked).

## Verdict per Ts

| input | verdict | reason |
| --- | --- | --- |
| sign 100 µs | **both** | offset created by the high-ω′ band (>3ω_e); realization sensitivity is material (biquad vs two-integrator, 1.56 Hz) |
| sign 50 µs | **ω′ band** | high-ω′ band drives the offset; realization sensitivity small (0.19 Hz) |
| sign 25 µs | **ω′ band** | high-ω′ band; realization sensitivity negligible (0.03 Hz) |
| sigmoid 50 µs | **neither** | clean ω′ → no offset, no bias |

## Limitations

- The band-split is a *replacement* study (the SOGI is nonlinear in ω′), so it
  identifies the responsible band empirically, not by superposition; (iv) full is
  the only bit-for-bit check.
- `TwoIntegratorSOGI` is an experiment-only diagnostic; no estimator is built.
