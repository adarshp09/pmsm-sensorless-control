# Phase 06d — Estimation-bandwidth diagnosis (experiments/docs only)

Gate P passes but the dynamic estimation errors under load steps are 4–5× the paper's
(S2 111.7 vs 23 rpm; S3 40.4 vs 10 rpm) and the low-speed/rated steps lose synchronism.
This phase maps the estimator bandwidth and isolates what limits the loop. **No src
change**; all variants are experiment-local subclasses. The 06b-6 nominal cell is the
headline; **arms 1–3 are labeled DIAGNOSTIC — not paper** and never replace it.
Script: `experiments/06d_diagnosis.py`.

## Step 0

**0a** — 06b-6 committed `f053656`, tagged `phase-06b6`. Repo
`/Users/adarshpanjwani/Desktop/ACT/project/pmsm-sensorless-control`, branch `master`, **no
remotes**.

**0b — what the 06b-6 report omitted.** `α_ω* = 2.5 Hz @500 / 10 Hz @1000`; design rule
(06b-2, D-058) "largest grid value with PM≥45° and GM≥6 dB at both speeds, chain B
f_n=20 Hz" — none qualified, so the operating-point decoupling + relaxations (D-059/D-061)
were adopted. `α_c = 500 Hz`. T_est (06b-2, chain A/B, f_n=20): see the Step-1 table
below. **H1 supported** (DSOGI envelope caps T_est, speed-dependent), **H2 refuted**
(raw-`E_hat` chain is narrower). Predicted vs observed boundary: the margin model puts
PM=0 between α_ω = 10 and 15 Hz at 1000 rpm; the time domain was stable ≤ 15 and unstable
≥ 20 Hz — the model is conservative by ~one grid step.

**0c — S1 nominal-α_ω row.** At the nominal `α_ω* = 10 Hz`, S1 (500→1000 ramp) gives peak
**180° advance-off / 18.5° advance-on** — advance-off fails. The 2.5 Hz S1 row in
`phase06b.md` is therefore a **scenario-specific bandwidth chosen after a failure**.

**0d — sensorless cost (two numbers).** At equal `α_ω = 10 Hz`: encoder dip −140.3 /
−56.1 rpm vs sensorless −210.0 / −79.6 rpm (S2/S3) → **cost −69.6 / −23.4 rpm**. At each
config's own design `α_ω`: encoder (50 Hz) −37.2 / −14.9 vs sensorless (10 Hz) −210 /
−79.6 → cost **−172.8 / −64.7 rpm**. The earlier "≈0" was wrong.

**0e — f_n sensitivity corrected.** The f_n×0.5 (10 Hz) row ("peak 0.6°") is **invalid**:
the slow Q-PLL delays the lock criterion, so handover at **0.70 s** falls *after* the
0.5 s load step — the step was encoder-driven (the recorded errors are exactly 0 through
it; 0.6° is the steady error). f_n = 5 Hz **never hands over**. Only f_n ≥ 20 Hz (handover
≈ 0.20 s) measures a sensorless transient. The `phase06b.md` sensitivity table is corrected
accordingly (and its `dyn |ω̂−ω|` peak angle error is *not* the PLL ramp offset
`ω̇_e/ω_n²`: a step is not a sustained ramp, and the ω̂ proportional path dominates).

**0f — FLC rows.** The FLC steady speed error (4.7–4.9 rpm vs 0.14 rpm matched) follows the
04b/04c steady ripple of the FLC-adapted observer; its similarity to the paper's ≈5 rpm is
**not** a reproduction signal.

**0g — paper-implied bandwidth (estimate) — SUPERSEDED by Phase 06c-2 Step 0b.** The
38–43 Hz figure transferred our *error/dip ratio* to the paper's 23/10 rpm, which is
wrong: those are *estimation errors*, while the paper's **actual dips** are ≈350 rpm
(Fig. 13, sim S2) and 45–48 rpm (Fig. 19, hardware S3) — **two scenarios, two implied
bandwidths**. Using our own sensorless dips (S2 −210 rpm at 10 Hz, −310 at 5 Hz; S3
−79.6 at 10 Hz) with dip ∝ 1/α_ω: S2 350 rpm → α_ω ≈ 210·10/350 = **6 Hz** (from the
10 Hz point) to **4.4 Hz** (from the 5 Hz point), i.e. **≈4–6 Hz**; S3 46 rpm →
α_ω ≈ 10·79.6/46 = **17 Hz**, which is **beyond our stable range** (≈10 Hz). The paper's
S3 point may be a different motor/inertia. So **no single paper-implied α_ω exists**;
38–43 Hz is **retracted**.

**0e (06c-2) — unnormalized Q-PLL effective ω_n/ζ.** The Q-PLL gains are designed at
`A_d = |E|(1000 rpm) = 55.54 V`; at another amplitude both scale as `√(A/A_d) = √(|E|/55.54)`:

| rpm | |E| | ω_eff | ζ_eff |
| --- | --- | --- | --- |
| 250 | 13.89 V | 10.0 Hz (62.8 rad/s) | 0.354 |
| 500 | 27.77 V | 14.1 Hz (88.9 rad/s) | 0.500 |
| 1000 | 55.54 V | 20.0 Hz (125.7 rad/s) | 0.707 |

Measured at 500 rpm (initial +20° angle error): damped period 81.6 ms → **ω_d = 77.0
rad/s**, exactly the analytic `ω_eff√(1−ζ²) = 88.9·0.866 = 77.0` — verified.

## Step 1 — open-loop T_est (06b-2 modulation, ε = 0.5 %, 250/500/1000 rpm)

Primary columns are the **phase at 25/50 Hz**; peak gain (dB) and −3 dB are secondary.

| arm | 250 rpm ph25 / ph50 (peak) | 500 rpm ph25 / ph50 (peak) | 1000 rpm ph25 / ph50 (peak) | −3 dB (250/500/1000) |
| --- | --- | --- | --- | --- |
| **0 nominal B** | −239° / −190° (9.8 dB) | −95° / −253° (4.6 dB) | **−50° / −85° (2.7 dB)** | 26/42/84 Hz |
| 1 raw A (DIAG) | −103° / −110° (4.8) | −79° / −90° (3.1) | −48° / −74° (2.1) | 17/26/42 Hz |
| 2 oracle ω′ (DIAG) | −152° / −259° (5.9) | −81° / −134° (3.5) | −45° / −74° (2.2) | 27/44/69 Hz |
| 3 normalised (DIAG) | −197° / −307° (6.2) | −130° / −165° (4.3) | −78° / −120° (2.7) | 15/24/37 Hz |

All locked. Amplitude dependence is strong (250 rpm ~3× narrower and more peaked than
1000 rpm).

**Why the nominal (84 Hz) is *wider* than the Q-PLL alone (raw `E_hat`, 42 Hz)** — not
peaking (2.7 vs 2.1 dB is similar): the DSOGI's in-phase `v'` is a clean fundamental,
whereas the Q-PLL on the raw bang-bang `E_hat` sees strong switching harmonics that
perturb its phase detector, degrading and narrowing the measured ω̂ response. The SOGI
therefore *helps* the Q-PLL.

**Why the oracle (69 Hz) is *narrower* than the nominal (84 Hz)** — the −3 dB is a shaping
difference, not peaking (2.7 vs 2.2 dB). The nominal's **adaptive FLL** lets `ω'` follow
the frequency modulation, which retunes the SOGI within the cycle and keeps its gain up
into the 50–80 Hz band (nominal gain at 80 Hz 0.77 vs oracle lower); the oracle pins `ω'`
to the true value and gets no such anticipation. Note the nominal's *phase* at 50 Hz is
actually slightly worse (−85° vs −74°), so the higher −3 dB is a gain-shaping effect, not
a better phase.

## Step 2 — closed-loop map (S2/S3; α_ω × f_n; matched cell, advance off)

**Methodological flag:** the f_n ∈ {5, 10} columns are **invalid** — at f_n = 5 the chain
never locks; at f_n = 10 the handover (0.70 s) is after the 0.5 s step. Only **f_n ≥ 20 Hz**
(handover ≈ 0.20 s) measures a sensorless transient. Valid columns:

| arm | max stable α_ω (both S2&S3) | best valid dyn `|ω̂−ω|` S2 / S3 | stable cells (fn20/40) |
| --- | --- | --- | --- |
| **0 nominal** | **10 Hz** | 112 / 40 rpm | 10/20 |
| 1 raw A | none | — | 0/20 |
| 2 oracle ω′ | **20 Hz** | **56 / 22 rpm** | 17/20 |
| 3 normalised FLL | 10 Hz (fn40) | 107 / 35 rpm | 9/20 |

- **None of the valid cells reaches the paper's 23 / ~10 rpm**, except the oracle on S3
  (22 rpm) — closest.
- The **oracle roughly doubles the stable α_ω (20 vs 10 Hz) and halves the transient
  estimation error** → the **DSOGI-FLL is the limiting element** between nominal and
  oracle, with the Q-PLL (f_n=20) and the sample delay behind it.
- The **normalised FLL does not help** (narrower: 36.6 Hz vs 83.8 at 1000 rpm; stable count
  9/20 vs 10/20). Its algebra is **ASSUMPTION** — Fig. 5 prints only a "gain
  normalization" block (two 0.5-gain paths summed), no formula.

**Stress (informational).** At the nominal cell (α_ω=10, f_n=20): 1000 rpm rated step
fails on arms 0/3 (minimum speed −1444 / −3474 rpm — the collapse goes through zero into
the weak-FLL region); 500 rpm 20 %/50 % fail. At each arm's best valid cell the 1000 rpm
rated step passes (peak 0.6°) on arms 0/2/3, but the **500 rpm steps still fail**
(minimum speed −400…−650 rpm). So the stress failures are **speed dips into the weak-FLL
region** (supported by the large negative minimum speeds), not a loop-margin problem.

## Step 3 — verdict

| arm | T_est −3 dB (250/500/1000) | max stable α_ω (1000) | dyn `|ω̂−ω|` S2/S3 (best valid) | stress 1000 rated (best) |
| --- | --- | --- | --- | --- |
| 0 nominal | 26/42/84 Hz | 10 Hz | 112 / 40 rpm | PASS |
| 1 raw A | 17/26/42 Hz | none | — | — |
| 2 oracle ω′ | 27/44/69 Hz | 20 Hz | 56 / 22 rpm | PASS |
| 3 normalised FLL | 15/24/37 Hz | 10 Hz (fn40) | 107 / 35 rpm | PASS |

**Limiting element:** the **DSOGI-FLL** (oracle-vs-nominal: 2× the stable α_ω, half the
transient error), then the **Q-PLL bandwidth** and the **sample delay / observer lag**.
The normalisation is **ASSUMPTION**, and as implemented it narrows the loop — not adopted.

**Recommendation (separate approved step; nothing applied):** a faster/stronger FLL
(e.g. higher Γ, or a normalization *plus* a gain that preserves the loop bandwidth, or a
PI/PLL on the SOGI output rather than the raw integrator) is the lever — it is the element
the oracle improves. The paper's implied α_ω (38–43 Hz) is out of reach of the current
chain at any valid cell.

## Acceptance / reproducibility

`experiments/06d_diagnosis.py` writes `experiments/output/06d_diagnosis/diagnosis.json`.
The f_n ∈ {5,10} grid columns are retained in the JSON but flagged invalid; no floor sweep
was done (per D-062).