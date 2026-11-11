# Phase 06f — Replay decomposition of the dynamic estimation error

**Verdict (one paragraph).** The dynamic estimation error decomposes cleanly once the
recorded `(u_αβ, i_αβ)` is replayed offline (Gate R: bit-exact). With an **ideal** back-EMF
(V0), the Q-PLL follows the frozen 06e acceleration-step model **within 20 % at every f_n**
(S2 87/42/20/10 vs 101/44/21/11; S3 33/19/8/4 vs 37/18/9/4) — the model is **validated**. The
**observer** (V0→V1) adds only 3–5 % — **not** the limit. The **DSOGI-FLL** (V1→V2) accounts
for the **whole factor, 1.5× at f_n=20 rising to 9× at f_n=160**, and it **breaks the
∝1/f_n scaling** (its bandwidth Γ=50 is fixed and amplitude-dependent; its ≈85 rpm lag is
f_n-independent). V4 (oracle ω′) restores V0's scaling, and V3 (`E_true → SOGI+FLL`) equals
V2 — so the **FLL alone** is responsible, not the SOGI band-pass and not the observer. In the
closed loop the instability is carried by the **ω̂ path** (channel isolation: with ω from the
encoder the loop stays stable at f_n=80, peak 5.7°; with ω from the chain it reaches 180°).
The 06e "flat/no-1/f_n" verdict is therefore a **stability-limit artefact** — at a stable
α_ω = 2–3 Hz the error *does* fall with f_n — but even then the nominal chain never reaches
within 2× of the paper's errors; only the ideal V0 does (at f_n ≥ 80). **No further bandwidth
experiments** unless a **printed** parameter is found to differ from ours.

## Step 0
**0a** — 06e committed `11423e0`, tagged `phase-06e`. `master`, **NO REMOTE**.
**0b** — `phase06e.md` corrected: verdict → **"not established"**; Step 2 table now carries
α_ω, the stability flag and the metric definition (window max); the S4 "176.9° at f_n=80" is
**loop instability at α_ω=10** (limit 4.4), not a PLL cost; ladder claims re-based on the
**best compensated** conventional cell (table above), "LPF lag dominant" restated as an
uncompensated-cell artefact; R5 (tuned) vs R4 (untuned) noted; max-stable-α_ω-falls-with-f_n
recorded as unexplained.
**0c** — the normalized Q-PLL option is marked **"KNOWN DEFECT — non-paper, do not use"** in
`estimator_spec.md` and `README.md` (no src fix).

## Step 1 — record + GATE R
Nominal chain (matched → DSOGI-FLL v′ → unnormalized Q-PLL f_n=20, t_h=0.30 s, advance off),
S2 at α_ω=4.4 Hz and S3 at α_ω=10 Hz; recorded arrays + encoder baselines.

**GATE R — PASS, exact:** replaying the identical chain on the recorded `(u,i)` from t=0
reproduces `θ̂, ω̂` with max |Δ| = **0.0** (both scenarios; tolerance 1e-9). The chain has
**no hidden dependence** on the controller or ground truth.

## Step 2 — replay decomposition (open loop, recorded trajectories, dead time OFF)

First-peak `|ω̂−ω|` [mechanical rpm] in `[t_step, t_step+50 ms]`; model = frozen 06e value;
lag = peak `ω_e − ω′` over 200 ms.

**S2 (paper 23 rpm; model {100,46,22,11}):**

| variant | f_n=20 | 40 | 80 | 160 | notes |
| --- | --- | --- | --- | --- | --- |
| V0 E_true → Q-PLL | **87** | **42** | **20** | **10** | model 101/44/21/11 → within 20 % ✓ |
| V1 E_hat → Q-PLL | 90 | 44 | 23 | 12 | observer adds 3–5 % |
| V2 E_hat → SOGI+FLL (nominal) | 135 | 131 | 130 | 108 | **flat**, lag 85 rpm (f_n-independent) |
| V3 E_true → SOGI+FLL | 141 | 145 | 149 | 130 | ≈ V2 → observer irrelevant |
| V4 E_hat → SOGI, oracle ω′ | 82 | 58 | 45 | 36 | **restores the model trend** |

**S3 (paper 10; model {37,18,9,4}):** V0 33/19/8/4 (✓ within 20 %); V1 34/20/9/7; V2
40/42/36/29 (lag 24); V3 40/41/37/30; V4 33/22/17/15.

**Attribution (S2, f_n = 20 → 160):** V0→V1 (**observer**) 87→90 / 10→12 (**×1.03–1.2**);
V1→V2 (**SOGI+FLL**) 90→135 / 12→108 (**×1.5–9**). V3≈V2 and V4≈V0 fix the attribution: the
**FLL** is the element. **Model check: V0 follows it within 20 % at every f_n ✓.**
**Within 2× of the paper?** Only **V0 and V4** (V0 f_n≥80: S2 20/10 ≤ 46, S3 8/4 ≤ 20);
the **nominal V2 never** (min 108 rpm at f_n=160).

**Verdict:** the factor between the paper-level errors and V2 is the **DSOGI-FLL**, not the
observer and not the PLL (whose model is validated). The residual between V2 and V0 is
**PLL bandwidth *as seen through the FLL*** — the FLL's fixed Γ caps the effective bandwidth,
so raising f_n cannot help.

## Step 3 — FLL-speed diagnostic (DIAGNOSTIC — not adopted)

V2, first-peak rpm / `(ω_e−ω′)` lag:

| Γ | f_n=20 | f_n=80 |
| --- | --- | --- |
| 50 (nominal) | 135 / 85 rpm, ok | 130 / 85 rpm, ok |
| 500 | 90 / 1014 rpm, **guard-fail** | 23 / 1014, **guard-fail** |
| 5000 | 90 / 1024, **guard-fail** | 23 / 1024, **guard-fail** |

Higher Γ **destabilises the un-normalized FLL** (the error signal is quadratic in amplitude,
so Γ=50 already sits near the edge; 500/5000 trip the `ω′≥0` guard). No usable Γ > 50.
Not adopted.

## Step 4 — closed-loop cross-checks (S2, matched, advance off, t_h=0.30 s)

**(a) low α_ω** (first-peak |ω̂−ω| rpm / peak |θ̂−θ|):

| α_ω | f_n=20 | 80 | 160 | stable |
| --- | --- | --- | --- | --- |
| 2 Hz | 399 / 62° | 343 / 87° | 325 / 88° | yes |
| 3 Hz | 379 / 43° | 267 / 40° | 254 / 29° | yes |
| 4.4 Hz (Step 2 V2) | 135 | 130 | 108 | near limit at f_n≥80 |

At a **stable** α_ω = 2–3 Hz the error **falls** with f_n (379→254 at α_ω=3) — the 06e
"flat" row was a **stability-limit artefact** (α_ω=4.4 is exactly the f_n=80 limit). The
*level* is larger at low α_ω (deeper dip) but the trend is model-like.

**(b) channel isolation** (S2, α_ω=10):

| channels | f_n=20 fp / θ | f_n=80 fp / θ |
| --- | --- | --- |
| θ_enc, ω_chain | 111 / 20.9° | **667 / 60.8°** |
| θ_chain, ω_enc | 74.6 / 13.6° | **72 / 5.7°** (stable) |
| chain, chain | 112 / 21.7° | **844 / 179.8°** (unstable) |

The **ω̂ (speed) path** carries the f_n-dependent instability: with ω from the encoder the
loop is stable at f_n=80; with θ from the encoder it is not. (Matches 06b's finding.)

**(c) cost of high f_n at a stable α_ω = 3 Hz, S4** (steady |θ̂−θ| mean / θ̂ ripple rms, deg):

| f_n | dead OFF | dead ON+comp |
| --- | --- | --- |
| 20 | 0.610 / 0.0015 | 0.569 / 0.626 |
| 40 | 0.610 / 0.0030 | 0.918 / 1.011 |
| 80 | 0.610 / 0.0062 | 1.378 / 1.518 |
| 160 | 0.610 / 0.013 | 0.596 / 0.682 |

At a stable α_ω the high-f_n steady **cost is negligible** (0.61° everywhere; ripple grows
slightly with f_n). The 06e "176.9° at f_n=80" was indeed α_ω=10 instability, not a PLL
steady cost.

## Bug
None in `src`. The 06e metric/cell caveats are corrections, not bugs. (The normalized Q-PLL
option remains a KNOWN DEFECT, Phase 06e.)

## Limitations
- Replay is open loop on the recorded `(u,i)`; the closed-loop interaction is captured only
  through Step 4.
- V0 uses the plant's own back-EMF convention (ideal sensor).
- Γ=50 is a PRINTED value; f_n is an unprinted ASSUMPTION; α_ω, t_h and the FLC scalings are
  ASSUMPTIONS.
- **No further bandwidth experiments** unless a printed parameter is found to differ.