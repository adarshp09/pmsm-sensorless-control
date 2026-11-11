# Phase 06c-2 — Handover-independent comparison, paper-matched dips, closed-loop dead time

Re-runs the comparison under a **fixed-time handover** (so no result is decided by the lock
criterion), matches the paper's *actual* dips, and closes the dead-time loop with
compensation. **src changes**: `simulation/drive.py` (dead-time compensation flag, default
OFF) + `configs/drive_assumptions.yaml`. With defaults the encoder-baseline JSON, Gate P
and the 06c comparison JSONs re-run byte-identical.

## Step 0

**0a** — 06c committed `789e34d`, tagged `phase-06c`. Repo
`/Users/adarshpanjwani/Desktop/ACT/project/pmsm-sensorless-control`, `master`, **NO REMOTE**.

**0b — paper-implied α_ω:** the 38–43 Hz estimate is **SUPERSEDED/retracted**. The paper's
quoted 23/~10 rpm are *estimation errors*, but its **actual dips** are ≈350 rpm (Fig. 13,
sim S2) and 45–48 rpm (Fig. 19, hardware S3) — **two scenarios, two implied bandwidths**.
From our own sensorless dips (S2 −210 rpm at 10 Hz, −310 at 5 Hz; S3 −79.6 at 10 Hz):
- S2 350 rpm → α_ω ≈ 210·10/350 = 6 Hz (10 Hz point) to 310·5/350 = 4.4 Hz (5 Hz point),
  i.e. **≈4–6 Hz**;
- S3 46 rpm → α_ω ≈ 79.6·10/46 = **17 Hz**, **beyond our stable range** (≈10 Hz), likely a
  different motor/inertia.
So no single paper-implied α_ω exists. (Arithmetic in `phase06d.md` §0g.)

**0c — Gate D corrected.** The "amp err" in Gate D is vs **true E**, not vs the dead-time-free
`E_hat`; the "SMO transmits ~0.6" explanation is **retracted**. In the open-loop FOC
scenario `ΔV` also *drives the plant current* by `≈|ΔV|/|Rs+jωLd| ≈ 13 A at 500 rpm`
(0.59 Ω) and `≈9.7 A at 1000 rpm` (0.80 Ω), so Gate D **mixed drive and observer effects**;
superseded by Gate D′ (Step 4).

**0d — handover-criterion artefacts.** Under fixed-time handover the **conventional** chain
and **arm 1** now lock (see Step 3), so their earlier non-locks were handover-criterion
artefacts. The **normalized Q-PLL**, however, is **genuinely unstable** under fixed-time
handover (Step 5: lock-peak ≈180°, steady ≈90°) — its 0/36 is *not* only a criterion
artefact. The 06b-6 f_n columns and 06d arm-1 are handover artefacts (already noted).

**0e — unnormalized Q-PLL effective ω_n/ζ** (`√(|E|/55.54)` scaling; see `phase06d.md`):

| rpm | ω_eff | ζ_eff | measured |
| --- | --- | --- | --- |
| 250 | 10.0 Hz | 0.354 | — |
| 500 | 14.1 Hz | 0.500 | ω_d = 77.0 rad/s (measured) = 88.9·√(1−0.5²) ✓ |
| 1000 | 20.0 Hz | 0.707 | — |

## Step 1 — fixed-time protocol

`t_h = 0.30 s` (ASSUMPTION), identical for all chains; the chain runs in the background
from the encoder start-up. **Lock-valid** = max |θ̂−θ| over `[t_h−50 ms, t_h] < 30°`
(evaluation only; the controller never uses it). Implemented in `experiments/chain.py`
(`fixed_time_s`, default OFF) and `experiments/c2_protocol.py`.

**Regression:** for the same scenario/α_ω, matched/FLC under `t_h` match the criterion-based
handover (S3 steady 0.59°/dyn 7.5°, S4 0.61° — identical), since both switch in the same
pre-step window (≈0.2–0.3 s); the differences are <0.1°.

## Step 2 — paper-matched dips (matched cell, fixed t_h)

α_ω, true dip, dynamic |ω̂−ω|, error/dip ratio (advance off; advance on similar):

| α_ω | S2 dip / dyn|ω̂−ω| / ratio | S3 dip / dyn|ω̂−ω| / ratio |
| --- | --- | --- |
| 3 | −1527 / 465 / 0.30 | −179 / 42 / 0.24 |
| 4 | −393 / 147 / 0.38 | −137 / 42 / 0.30 |
| 5 | −310 / 122 / 0.39 | −115 / 42 / 0.36 |
| 7 | −245 / 108 / 0.44 | −92 / 41 / 0.45 |
| 10 | −210 / 112 / 0.53 | −79.6 / 40 / 0.51 |
| 15 | −180 / 181 / 1.00 | −69 / 59 / 0.86 |
| 20 | −4545 / 1101 (unstable) | −1898 / 1156 (unstable) |

**Matched-dip verdict.** S2's 350 rpm dip is reached at **α_ω ≈ 4.4 Hz** (interpolating the
4/5 Hz points — the auto-interpolation in `suite.json` is skewed by the marginal α_ω=3 and
unstable α_ω=20 endpoints, so the grid is authoritative), where the ratio ≈0.38 →
**estimation error ≈133 rpm**, ~6× the paper's 23. S3's 46 rpm dip needs **α_ω ≈ 17 Hz**,
**beyond the stable range**; the closest stable point (15 Hz) gives ratio 0.86 → ≈40 rpm.
The predicted `2 sin(φ/2)` (≈0.27 at 5 Hz → 80–95 rpm) is close to the measured 133 rpm at
the S2-matched dip, but **even so the error is 4–6× the paper's**. So **loop bandwidth alone
does not explain the gap; a residual estimator-quality gap remains** (the paper's estimator
is better per unit dip). The S3 matched point is infeasible in our stable range.

## Step 3 — conventional vs improved (fixed t_h, dead time OFF)

Representative cells (full table in `suite.json`); steady/dynamic |θ̂−θ|, current ripple:

| chain | S1 (2.5 Hz) steady / dyn | S3 steady / dyn | S4 steady | ripple (A) | lock-valid |
| --- | --- | --- | --- | --- | --- |
| **matched** (nominal) | 0.61° / 2.2° | 0.59° / 7.5° | 0.61° | 0.0025 | yes |
| FLC nominal | 1.98° / 3.5° | 1.96° / 6.8° | 1.97° | 0.071 | yes |
| arm 1 raw→QPLL (DIAG) | 0.61° / 1.9° | 0.59° / 5.8° | 0.61° | 0.005 | yes |
| conv fc100 raw | 35.6° / 36.5° | 37.0° / 42.8° | 40.3° | 1.32 | S1 yes, S3/S4 **no** |
| conv fc100 comp | **180°** | — | — | 19.4 | unstable |
| qpll normalized | 89.5° | 83.8° | 91.4° | 0.26 | **no** (genuine) |

**Claims the data supports:**
- **Improved vs conventional** — on **steady error** (0.6° vs 35–40°), **dynamic error**
  (2.2°/7.5° vs 36°/43°) and **current ripple** (0.0025 vs 1.3 A) the matched (improved)
  chain is **much better**; several conventional cells are not even lock-valid.
- **DSOGI-FLL when the observer is clean (arm 1 vs nominal)** — the raw→Q-PLL arm is
  **better** on S1 (1.9° vs 2.2°), S3 (5.8° vs 7.5°) and S2, so the DSOGI-FLL **hurts**
  rather than helps for a clean matched observer (its lag dominates); its value is in
  filtering the *chattering* observer, not a clean one.
- **Normalized Q-PLL** — genuinely unstable under fixed-time handover (not a criterion
  artefact); not adopted.

## Step 4 — dead time closed loop

**GATE D′ (encoder-fed, chain in background; comp OFF/ON vs dead-free):**

| case | OFF amp / phase / h6 | ON amp / phase / h6 | ON+comp amp / phase / h6 |
| --- | --- | --- | --- |
| 500, I_rated | −2.6 % / −0.18° / 0.0° | **+25.2 % / −0.05° / 4.19°** | −2.5 % / +0.03° / 0.43° |
| 500, 20 % | −2.6 % / −0.27° / 0.0° | +24.9 % / −0.30° / 2.74° | −2.5 % / +0.01° / 0.53° |
| 1000, I_rated | −2.6 % / −0.50° / 0.0° | **+11.3 % / −0.17° / 2.39°** | −2.6 % / −0.36° / 0.30° |
| 1000, 20 % | −2.6 % / −0.59° / 0.0° | +11.1 % / −0.23° / 1.53° | −2.5 % / −0.21° / 0.68° |

The predictions hold within 30 %: OFF amp +25.2 % (pred 28.5 %) at 500 and +11.3 %
(pred 14.3 %) at 1000, **near-zero phase**, h6 4.19°/2.39° (pred 5.6°/2.8°); **ON+comp
returns to the dead-free value** (−2.5 %) and h6 <0.7°. So Gate D′ **confirms the model**;
Gate D's factor was drive/observer mixing (Step 0c).

**Sensorless (matched, t_h):**

| scenario | OFF steady / dyn | ON steady / dyn | ON+comp steady / dyn |
| --- | --- | --- | --- |
| S1 | 0.61° / 180°* | 176.4° / 176.4° (sat 1.0) | 1.11° / 4.17° |
| S3 | 0.59° / 7.5° | 27.8° / 36.6° | 0.22° / 7.9° |
| S4 | 0.61° / 0.61° | 25.6° / 25.6° | 1.12° / 1.12° |
| S2 (α_ω=3, marginal) | 0.56° / 149.8°* | 2.75° / 179.7° | 178.2° (marginal loop) |

*S1/S2 OFF rows are the loop-margin/α_ω issue, not dead time. Dead time degrades the
estimator severely (steady 25–28°); **compensation restores it**. S4 steady error
0.61° (OFF) / 1.12° (comp) = 0.011 / 0.020 rad, **below** the paper's 0.08 rad (no
reproduction claim).

## Step 5 — normalization spot check

Normalized Q-PLL (α_ω = 10, f_n ∈ {20,40}, S3 and 500/20, t_h): **lock-valid = False** in
every cell (lock-peak ≈180°, steady 84–91°) — a **genuine instability**, not a handover
artefact. Unnormalized nominal: steady 0.59°, dyn |ω̂−ω| 40 rpm. Not adopted.

## Limitations
- Fixed-time handover `t_h = 0.30 s` is an ASSUMPTION; it is only valid for scenarios whose
  disturbance begins after it (all S1–S4 / S2 at t_d = 0.5 s).
- The conventional chain is **our construction**; the LPF cutoff is unprinted.
- Dead time is average-value (no PWM/switching model).