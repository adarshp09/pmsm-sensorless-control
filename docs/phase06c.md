# Phase 06c — Paper-fidelity items and headline comparisons

Adds the average-value inverter **dead-time model** (default OFF) and runs the
conventional-baseline / headline comparisons. **src changes only** in
`simulation/drive.py` (dead-time flags, default OFF) + `configs/drive_assumptions.yaml`.
With defaults the 06a JSON and 06b-6 Gate P are bit-identical (regression).

## Step 0

**0a** — 06d committed `b9885c8`, tagged `phase-06d`. Repo
`/Users/adarshpanjwani/Desktop/ACT/project/pmsm-sensorless-control`, `master`, **NO REMOTE**.

**0b — paper traces (rendered Fig. 13 / 19 / 20).** Fig. 13 (sim 50 %) top panel
"Speed(r/min)" (700-1000) shows the **actual speed** dipping to ≈**600–650 rpm**
(≈350 rpm) over a ≈0.1–0.2 s transient, recovering in ≈0.2–0.3 s; the middle panel
"Speed error" spikes to ≈30 rpm and the caption says "estimation errors" → the quoted
**23 rpm is the estimation error `ω−ω̂`**, not the dip. Fig. 19 (exp 20 %) plots
`N_ref/N_actual/N_err` with the **actual** dip annotated ΔN = 45/48 rpm and the quoted
~10 rpm = `N_err` (estimation error). **Estimated speed-loop bandwidth** from
dip ∝ 1/α_ω and our encoder 50 % dip 37.2 rpm at 50 Hz: α_ω ≈ 50·37.2/350 ≈ **5 Hz**;
from the recovery τ ≈ 0.25 s → α_ω ≈ 4/τ ≈ **2.5–4 Hz** → **≈3–5 Hz (estimate)**.
Unreadable: no numeric x-ticks at 1.5 s, so the step instant (1.5 s) is from the text.
Printed: no PI gains, no Q-PLL bandwidth, no estimator/filter bandwidth (Fig. 7 has no
numbers; the LPF cutoff is unprinted). Speed-loop rate 1 kHz only (Table III).

**0c** — `phase06d.md` amended: phase @25/50 Hz is now the primary T_est columns, peak
gains (dB) added, and the oracle-vs-nominal and nominal-vs-Q-PLL-only differences
explained (shaping/adaptive-`ω′` lead vs harmonic degradation, not peaking).

**0d** — Arm 1 (raw `E_hat` → Q-PLL) with the **generic** handover criterion: it **does**
hand over (0.06–0.09 s) with peaks e.g. α_ω=10/f_n=20 S2 18.4°/S3 7.6°, α_ω=20 S2 12.2°,
α_ω=40 S2 51.7°. The 06d "arm 1 unstable everywhere (0/20)" was an **artefact** of the
FLL-based handover criterion (no FLL → `ω′ = 0` → the criterion never holds). The generic
criterion (`ω̂ ≥ ω_h` and `|Δe|/|Ê| < sin 5°`, no ground truth) is added to
`experiments/chain.py`, **default OFF** (nominal chain unchanged).

**0e — tracking-error relation.** The load-step dip is critically damped (ζ=1), so a
single spectral "dominant frequency" is ill-defined (the one-sided pulse peaks at DC).
Using the dip's fast-edge band: T_est phase (nominal, 1000 rpm) is −22° at 15 Hz and
−38° at 20 Hz → `2 sin(φ/2)` = **0.39–0.65**, bracketing the measured error/dip ratios
**0.53 / 0.51**. So the estimator's **phase lag at the dip's band explains the dynamic
error** (effective φ ≈ 31° at ≈17 Hz).

## Step 1 — Q-PLL normalisation ablation (normalization ON)

`experiments/06c_comparison.py` (`normalization_ablation`): α_ω ∈ {5,10,20} × f_n ∈
{20,40} × advance {off,on} × {S2,S3,500-20}. **0/36 cells hand over** — the normalized
Q-PLL does **not** meet the 06b-6 FLL-consistency lock criterion (`|ω̂ − ω′/p| < 2 rpm`)
in any tested cell, so the ablation yields no usable closed-loop comparison. Not adopted;
the normalization algebra is an ASSUMPTION (Fig. 5 prints only a labelled block).

## Step 2 — dead time (drive.py flags, default OFF) — GATE D

Model: per phase `ΔV_x = −V_dc t_d f_sw sat(i_x/i_th)`, `V_dc=311`, `t_d=2 µs`,
`f_sw=10 kHz` (printed), `i_th=0.1 A` (ASSUMPTION); observer gets the commanded voltage.
Unit tests (6, `tests/test_dead_time.py`): `|ΔV| = 6.22 V`, sign opposes current, 0 at
`i=0`, fundamental `4/π·6.22 = 7.92 V` with 5th/7th = 1/5, 1/7, defaults bit-identical.

**Gate D** (open-loop nominal drive + dead time, matched cell):

| case | |E| | amp err | (pred) | phase | h5 | h7 |
| --- | --- | --- | --- | --- | --- | --- |
| 500 rpm, I_rated | 27.8 V | **+14.8 %** | 28.5 % | +4.16° | ~0 | ~0 |
| 500 rpm, 20 % | 27.8 V | +0.8 % | 28.5 % | +0.23° | ~0 | ~0 |
| 1000 rpm, I_rated | 55.5 V | **+6.5 %** | 14.3 % | +3.64° | ~0 | ~0 |
| 1000 rpm, 20 % | 55.5 V | −0.9 % | 14.3 % | −0.67° | ~0 | ~0 |

The **ΔV itself matches** (measured α-β vector fundamental **7.76 V** vs 7.92 predicted).
The measured E_hat error is **≈0.6× the prediction** — a >30 % difference, explained:
(a) the SMO transmits only ≈0.6 of the disturbance to its E_hat fundamental
(E_hat 27.07→31.89 V, i.e. +4.83 V, when dead time is added); (b) the residual is a
**quadrature** component (the +4° phase shift), so the *in-phase* amplitude change is
smaller than |ΔV|; (c) at 20 % load the current is small and near the `i_th` crossing, so
the distortion is erratic/negligible (the paper's "erratic at no load"). Gate D is
informational (no STOP), the difference is explained.

**Dead time OFF → ON (headline, matched/FLC, advance off):** with dead time ON the
estimator degrades severely — matched S2 collapses (peak 180°), S1 35°, S3 39°; S4 steady
error 16.5° at no load (erratic sign near `i=0`). FLC is marginally better on S1/S4 but
similar. So dead time is a first-order error source in this model.

## Step 3 — conventional baseline (labeled "conventional — our construction")

Fixed-sign SMO (`k0`) → first-order LPF (cutoff UNPRINTED → ASSUMPTION grid
f_c ∈ {100,200,400} Hz × phase compensation {off,on}) → Q-PLL (f_n=20); plus arctan on the
filtered E. Handover with the generic 5° criterion. **All 8 conventional chains fail to
hand over** in every scenario: the Q-PLL *does* reach ≈1020 rpm, but the LPF-filtered
bang-bang `E_hat` keeps `|Δe|/|Ê|` at ≈0.06–0.09 — i.e. **at/above sin 5°** — so the 50 ms
dwell is never met. This is a genuine limitation of the fixed-sign SMO's ripple on the
filtered E (not a bug); a looser lock or a different LPF would be a separate step.

## Step 4 — headline comparison

Only the **matched nominal** and **FLC nominal** chains hand over, so the headline table
is a two-chain comparison (dead time OFF, advance off):

| chain | S1 steady / dyn `|θ̂−θ|` | S2 dyn `|θ̂−θ|` / `|ω̂−ω|` | S3 dyn `|θ̂−θ|` / `|ω̂−ω|` | S4 steady |
| --- | --- | --- | --- | --- |
| **matched (nominal)** | 0.61° / 2.2° | 21.7° / 111.7 rpm | 7.5° / 40.4 rpm | 0.61° |
| FLC nominal | 1.92° / 3.5° | 21.3° / 116.2 rpm | 6.8° / 42.6 rpm | 1.92° |

≈0.6° steady error, and the dynamic transient error is ~5× the paper's (111.7 vs 23 rpm;
40.4 vs 10 rpm). Paper values are **estimation errors** (Step 0b); ours are on the same
basis. **No reproduction claim** — every setup differs (encoder start-up + handover, ideal
voltage, assumed PLL/FLL gains, our slow α_ω, and now dead time as a separate table).

## Limitations

- The conventional baseline (our construction) cannot lock under the 5° criterion; no
  conventional-vs-improved comparison is possible in this phase. The normalized Q-PLL
  ablation is also non-lockable. Both are reported, not hidden.
- PWM/switching-level inverter is out of scope (average-value only).
- Dead time is a first-order degradation in this model.