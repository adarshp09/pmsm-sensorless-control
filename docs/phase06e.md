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
S3 → {37, 18, 9, 4} rpm. Measured dynamic max |ω̂−ω| at α_ω = 4.4 Hz (S2-matched dip):

| f_n | predicted (S2) | measured (S2) | S3 measured |
| --- | --- | --- | --- |
| 20 | 100 | 135 | 42 |
| 40 | 46 | 159 | 32 |
| 80 | 22 | 289 | 47 |
| 160 | 11 | 208 | 59 |

**Verdicts:**
- **(i)** At the α_ω matched to the paper's dip (S2 **4.4 Hz**; S3 **beyond the stable
  range** — the 46 rpm dip needs >15 Hz), **no f_n brings the estimation error within 2× of
  23 rpm — "not reached"** (best S2: 135 rpm at f_n=20, 5.9× the paper's).
- **(ii)** Max stable α_ω per f_n (S2): **15 (f_n=20), 10 (40), 4.4 (80), 7 (160)** — high
  f_n **shrinks** the stable range.
- **(iii)** **Measured error does NOT ∝ 1/f_n** — it is flat-to-worse (135→208 as f_n rises
  20→160). The model is roughly right only at f_n=20 (135 vs 100).
- **(iv)** Cost of high f_n: at S4 (dead time OFF) the **matched scatter's steady error blows
  up at f_n=80 (176.9°)** — unusable; dead ON+comp is then also unstable (180°). High f_n is
  not viable in our loop. E_hat is clean in these cells, so **no measurement-noise test — a
  stated limitation**.

**Verdict: the gap is NOT explained by PLL bandwidth.** The model's f_n scaling fails,
raising f_n does not reduce (and generally increases) the error, and it destabilizes the
speed loop. The f_n=20 level is ~35 % above the model — the bandwidth route is closed, the
gap is elsewhere (estimator/loop interaction, or a different paper setup).

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
  ripple 1.3 → 0.3–0.9 A — the LPF **lag** (33.7°) was the dominant steady error; the
  DSOGI-FLL removes it. (Best R1 = the *compensated* fc100 cell at 1.8–7.9° steady, but its
  ripple stays ~1.3 A.)
- **R2 → R3** (sign → fixed sigmoid): **unstable** (179.9°, saturation) — the fixed sigmoid's
  limit cycle at 100 µs.
- **R3 → R4** (add **FLC**): **stable** (1.96°) — the FLC fixes the fixed-sigmoid instability.
- **R4 vs R5** (FLC vs matched sigmoid): 1.96° vs 0.61° — the matched sigmoid is better
  (~1.3° less steady error); the FLC's value over R3 is *stabilising the observer*, not
  beating a matched sigmoid.
- **R6** (matched, raw E_hat → Q-PLL, DIAGNOSTIC): 0.59–0.61°, comparable to R5; the FLL adds
  nothing for a clean matched observer (echoes 06c-2).
- **S4 dead ON+comp** (R5): steady 1.12° (vs 0.61° OFF) — compensation restores.

**Claim support:** the paper's "conventional LPF vs improved" is supported **on steady error
and current ripple** (R1 raw/comp vs R5: 1.8–40° and 1.3 A vs 0.61° and 0.0025 A), **not**
on dynamic error for the best compensated cell (R1 fc100 comp S3 8.2° ≈ R5 7.5°). The FLC's
benefit is **conditional**: it stabilises a *fixed* sigmoid (R3 unstable), while a matched
sigmoid is better still (R5). All claims assume the unprinted LPF cutoff / FLC scaling and
f_n.

## Bug
**Normalized Q-PLL option** (Step 1): does not lock at 0° from a +90° start (settles ≈−39°,
oscillating) and has a small residual bias from 0°. Reported, **not fixed** (no src change).

## Limitations
- f_n is an unprinted ASSUMPTION; the conventional chain and its LPF cutoff are our
  construction; R6 is DIAGNOSTIC.
- Fixed-time handover t_h = 0.30 s ASSUMPTION; valid only for disturbances after it.
- E_hat is clean, so there is **no noise test**; the dead-time model is average-value.