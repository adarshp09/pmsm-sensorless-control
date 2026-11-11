# Phase 06e — PLL bandwidth at the paper-matched dips + ablation ladder

Tests whether the estimation-error gap at the paper-matched dips is a **PLL-bandwidth**
artefact (f_n, unprinted ASSUMPTION, had never been varied there), verifies the normalized
Q-PLL lock point, and runs the ablation ladder. **No src changes**; experiments/docs only.

## Step 0

**0a** — 06c-2 committed `eb2872a`, tagged `phase-06c2`. Repo
`/Users/adarshpanjwani/Desktop/ACT/project/pmsm-sensorless-control`, `master`, **NO REMOTE**.

**0b — model corrections (in `phase06c2.md`):**
- The `2 sin(φ/2) ∝ α_ω` prediction **failed** (0.27 predicted at 5 Hz, 0.39 measured; ratio
  flat ≈0.38 for α_ω ≤ 5 Hz). Replaced by the **acceleration-step response of the Q-PLL**:
  peak frequency error `≈0.456·a0/ω_n` at `t ≈ 1.1/ω_n`, **independent of α_ω**, with
  `a0 = ΔT/J` (S2 24 027 rpm/s, S3 9611 rpm/s) and `ω_n = 2π·f_n·√(ω/ω_1000)` at the peak
  speed. Predicted vs measured: S2 f_n=20 **100–110 rpm** vs 121–149 at 5–4 Hz; S3 **36** vs
  40 at 10 Hz (model ~20 % low).
- **Paper-implied f_n**: 23 rpm (S2) → **≈76 Hz**; ~10 rpm (S3) → **≈70 Hz** — **HYPOTHESIS**
  (printed J/rated torque apply to both scenarios, instantaneous step).
- "residual estimator-quality gap" → **"estimator bandwidth f_n (unprinted ASSUMPTION, 20 Hz)
  not yet varied"** (done here).
- Conventional `fc100 raw` steady 35.6° = uncompensated LPF lag `atan(ω_e/ω_c) = 33.7°` at
  1000 rpm + ≈1.6° observer lag; the S3 "not lock-valid (37°)" is the 30° threshold meeting
  that *deterministic lag*, not a tracking failure. The 0.6°-vs-35–40° claim is **restricted
  to the uncompensated cell** (Step 3 shows the compensated cells).
- "DSOGI hurts a clean observer" **scoped** to the matched cell, S1/S3.
- Normalized Q-PLL steady 84–91° marked **"unverified"** until Step 1.
- Gate D′ predictions held within **12–25 %, all low**; the −2.5 % baseline gain only partly
  explains it — **unexplained**.

## Step 1 — normalized Q-PLL lock point (open loop, nominal drive, matched cell, v′)

| case | steady mean | std | final | from +90° |
| --- | --- | --- | --- | --- |
| 500 rpm, OFF | −0.19° | 0.02 | −0.19° | locks (−0.19°) |
| 500 rpm, ON | −1.01° | 0.99 | +0.10° | **fails: final −38.3°** (std 33°) |
| 1000 rpm, OFF | −0.52° | 0.00 | −0.52° | locks (−0.52°) |
| 1000 rpm, ON | −1.78° | 0.36 | −0.89° | **fails: final −39.2°** (std 18°) |

**Verdict:** normalization **OFF locks cleanly**. **ON does not** — from θ0=0 it has a small
bias (−1…−1.8°) with residual ripple; from **+90° it does not converge** (settles ≈−39°
oscillating). This is an **implementation/convention issue in the normalization option**
(reported, **not fixed** — no src change); the closed-loop 84–91° is therefore **not**
evidence about the nominal chain.

## Step 2 — PLL bandwidth sweep (chain B: v′ → unnormalized Q-PLL, ζ=0.707, t_h=0.30 s)

**Predictions (frozen before the runs):** S2 f_n {20,40,80,160} → {100, 46, 22, 11} rpm;
S3 → {37, 18, 9, 4} rpm. **Metric:** `max |ω̂−ω|` over the scenario *dynamic window*
(S2 (0.5, 1.0) s), i.e. a **window max**, not the first peak. **Cell:** α_ω = 4.4 Hz (S2) /
10 Hz (S3). `stable` = peak |θ̂−θ| < 30° **and** no post-transient saturation **and**
lock-valid — none of these exclude being *near* the stability limit.

| f_n | predicted (S2) | measured (S2) | stable (S2) | α_ω/limit | S3 measured | stable (S3) |
| --- | --- | --- | --- | --- | --- | --- |
| 20 | 100 | 135 | yes | 4.4/15 | 42 | yes |
| 40 | 46 | 159 | yes | 4.4/10 | 32 | yes |
| 80 | 22 | 289 | **yes** (1.0× limit) | 4.4/**4.4** | 47 | **yes** (1.0× limit) |
| 160 | 11 | 208 | yes | 4.4/7 | 59 | yes |

**The cells sit within 1.0–3.4× of the f_n-dependent stability limit** (S2 α_ω=4.4 is
*exactly* the f_n=80 limit), so the window max includes loop ringing near the limit; **S3's
α_ω (10 Hz) is above its own limit at f_n ≥ 40** — the sweep's α_ω was **not held at the
matched value** for S3. These caveats are why the 06e verdict is **not established**;
Phase 06f re-measures with a first-peak metric at a fixed stable α_ω.

**Verdicts (as measured, 06e):**
- **(i)** At the α_ω matched to the paper's dip (S2 **4.4 Hz**; S3 **beyond the stable
  range**), no f_n brings the error within 2× of 23 rpm at this metric.
- **(ii)** Max stable α_ω per f_n (S2): 15/10/4.4/7 — **falls with f_n** (unexplained; see
  below).
- **(iii)** Measured error does not ∝ 1/f_n at this metric/cell.
- **(iv)** The S4 "**176.9° at f_n=80**" is **loop instability at α_ω = 10 Hz** (that
  cell's limit is 4.4 Hz), **not** a PLL steady cost: at a stable α_ω = 3 Hz the S4 steady
  error is 0.61° at every f_n (Phase 06f Step 4c). E_hat is clean → no noise test (stated
  limitation).

**Verdict (06e wording): the gap is NOT explained by PLL bandwidth — superceded by
"not established"** (Phase 06f): the window-max metric, the near-limit cells and the
unstated S3 α_ω leave the claim unproven; 06f's first-peak replay decomposition attributes
the factor to the **DSOGI-FLL** instead.

## Step 3 — ablation ladder (fixed t_h; dead time OFF unless stated)

Tap-level statements (each conditional on the assumed LPF cutoff, FLC scaling and f_n):

**f_n = 20 Hz (frozen nominal):**

| rung | S1 (2.5 Hz) steady/dyn | S3 steady/dyn | S4 steady | ripple (A) |
| --- | --- | --- | --- | --- |
| R1 conv fc100 **raw** | 35.6° / 36.5° | 37.0° / 42.8° | 40.3° | 1.32 |
| R1 conv fc100 **comp** (best R1) | 1.77° / 4.2° | 4.09° / 8.2° | 7.90° | 1.29 |
| R1 arctan fc200 | 176° / 176° (dyn 75 024 rpm!) | unstable | 172° | 15.7 |
| R2 sign → DSOGI | 3.87° / 6.7° | 3.69° / 7.4° | 4.87° | 0.30 |
| R3 fixed sigmoid → DSOGI | 180° (sat) | 180° (sat) | 14.3° | 2.42 |
| R4 FLC → DSOGI | 1.98° / 3.5° | 1.96° / 6.8° | 1.97° | 0.071 |
| R5 matched → DSOGI (nominal) | 0.61° / 2.2° | 0.59° / 7.5° | 0.61° | 0.0025 |
| R6 matched → raw Q-PLL (DIAG) | 0.61° / 1.9° | 0.59° / 5.8° | 0.61° | 0.005 |

**f_n = 160 Hz (ASSUMPTION, "paper-level", not reached within 2×):** every rung degrades —
R2 S3/S4 collapse (180°, sat), R5 S4 177° — confirming high f_n is unusable.

**What each step changes:**
- **R1 → R2** (LPF **replaced** by the DSOGI-FLL, observer unchanged): steady **35–40° → 2.5–4.9°**,
  ripple 1.3 → 0.3–0.9 A — but this improvement is dominated by removing the **uncompensated**
  cell's lag; the *best compensated* R1 cell (fc100 comp) is already at 1.8–7.9° steady
  (Step 3 below), so "the LPF lag was the dominant steady error" is an **artefact of the
  uncompensated cell**.
- **R2 → R3** (sign → fixed sigmoid): **unstable** (179.9°, saturation) — the fixed sigmoid's
  limit cycle at 100 µs.
- **R3 → R4** (add **FLC**): **stable** (1.96°) — the FLC fixes the fixed-sigmoid instability
  (R4 is **untuned**, assumed scalings).
- **R4 vs R5** (FLC vs matched sigmoid): 1.96° vs 0.61° — **R5 (kF=1, aF=0.4) was selected
  on these scenarios (tuned)**, while R4 is untuned, so the 1.35° gap is not a fair
  like-for-like FLC penalty.
- **R6** (matched, raw E_hat → Q-PLL, DIAGNOSTIC): 0.59–0.61°, comparable to R5; the FLL adds
  nothing for a clean matched observer (echoes 06c-2).
- **S4 dead ON+comp** (R5): steady 1.12° (vs 0.61° OFF) — compensation restores.

**Claim support (improved R5 vs the *best compensated* conventional R1 fc100 comp):**
| metric | improved (R5) | best conv (R1 fc100 comp) | winner |
| --- | --- | --- | --- |
| current ripple | 0.0025 A | 1.29 A | **improved (515×)** |
| S4 steady | 0.61° | 7.90° | **improved** |
| S3 steady | 0.59° | 4.09° | **improved** |
| S1 (2.5 Hz) steady | 0.61° | 1.77° | conventional (2.9×) |
| S3 dynamic | 7.5° | 8.2° | improved (~9 %) |

So the improved chain wins clearly on **current ripple** and **steady error (S3/S4)**, loses
on **S1 steady**, and its **dynamic gain is only ~15–20 %**. All conditional on the assumed
LPF cutoff, FLC scaling and f_n. **Unexplained observation:** max stable α_ω **falls** with
f_n (15/10/4.4/7 for 20/40/80/160) — see Phase 06f (the ω̂ path).

## Bug
**Normalized Q-PLL option** (Step 1): **KNOWN DEFECT — non-paper, do not use.** It does not
lock at 0° from a +90° start (settles ≈−39°, oscillating) and has a small residual bias from
0°. Reported, **not fixed** (no src change).

## Limitations
- f_n is an unprinted ASSUMPTION; the conventional chain and its LPF cutoff are our
  construction; R6 is DIAGNOSTIC.
- Fixed-time handover t_h = 0.30 s ASSUMPTION; valid only for disturbances after it.
- E_hat is clean, so there is **no noise test**; the dead-time model is average-value.