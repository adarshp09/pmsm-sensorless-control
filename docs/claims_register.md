# Claims register — pmsm-sensorless-control

**Purpose.** One row per claim the project makes, each carrying a **status**, a **scope**
(scenario / Ts / observer / assumed values) and an **evidence path** (doc § or JSON), plus
its **decision IDs**. No claim here asserts that the paper is reproduced or validated.

**Status values**
- **SUPPORTED** — a stated test was run and passed; the evidence path is given.
- **HYPOTHESIS** — consistent with the evidence but not isolated by a discriminating test.
- **UNEXPLAINED** — observed, cause not established.
- **RETRACTED** — withdrawn (see the list at the end).
- **UNTESTED** — not executed in this project.

**Standing scope qualifiers (apply to every row unless the row overrides them):**
`Ts = 100 µs` unless stated; observer = matched fixed sigmoid (`kF = 1`, `aF = 0.4`, **selected
on these scenarios**), FLC scalings assumed (`configs/fuzzy_assumptions.yaml`); nominal chain =
matched → DSOGI-FLL (`k_s = √2`, `Γ = 50`, `min_ω = 100 rpm`) → **unnormalized** Q-PLL
(`ζ = 0.707`, `f_n = 20 Hz`); `α_ω = 2.5 Hz @ 500 rpm / 10 Hz @ 1000 rpm`; `t_h = 0.30 s`;
dead time OFF; paper numbers are **estimation errors**, not true dips.

## Observer

| Claim | Status | Scope | Evidence | Decisions |
| --- | --- | --- | --- | --- |
| The SMO reported `g(S_post)` instead of the applied `g(S_pre)`; fixed | SUPPORTED | all observers, all Ts | `docs/phase03.md` §10; `docs/phase04c.md` | **D-025**, D-028 |
| Discrete stability boundary `Ts* = 2·Ld/(Rs + k·a/2) = 60.1 µs` | SUPPORTED | sigmoid SMO, `i_d = 0`, rated op. point | `docs/phase03.md` §13; `experiments/output/03*/` | D-027, D-029 |
| The fixed sigmoid is a **period-2 limit cycle** at `Ts = 100 µs` | SUPPORTED | fixed sigmoid, 100 µs | `docs/phase03.md`; `06e_ladder.json` (R3 = 176–180°, sat) | D-027, D-029, D-078 |
| Sign vs sigmoid comparison is scoped to the **stable regime (`Ts = 50 µs`)** | SUPPORTED | sign + sigmoid, 50 µs | `docs/phase02.md` §6; `docs/phase03.md` §13 | D-027, D-029 |
| Angle-bias model = observer lag + `ω·Ts/2` (half-sample) | SUPPORTED | matched cell, 500–1000 rpm, 100/200 µs | `docs/phase05b.md`; `docs/equations_and_conventions.md` | D-040, D-043 |
| Static `ε_f(ω′)` balance explains the sigmoid bias but **not** the sign bias | SUPPORTED (negative) | 1000 rpm, static `ε_f` sweep | `docs/phase03.md` §15 | D-042 |

## FLC

| Claim | Status | Scope | Evidence | Decisions |
| --- | --- | --- | --- | --- |
| The FLC **stabilises** the sigmoid at `Ts = 100 µs` (R3 unstable → R4 stable) | SUPPORTED | nominal scenarios, 100 µs, assumed scalings | `06e_ladder.json` R3/R4; `docs/phase04c.md` | D-030/D-031, D-078 |
| The **matched fixed gain** beats the **untuned FLC** on nominal metrics | SUPPORTED, **not like-for-like** | same scenarios; R5 tuned (kF=1, aF=0.4), R4 untuned | `06e_ladder.json` R4 1.96° vs R5 0.61°; `docs/phase06e.md` | D-078 |
| "FLC strongly helps at 100 µs" (broad framing) | **RETRACTED** | — | `docs/phase04.md:120` | D-033 |
| FLC robustness to parameter/measurement error | **UNTESTED** | — | `docs/future_work.md` | — |
| "FLC reduces chatter by 17.5 % (MA)" | **RETRACTED** | — | `docs/phase02.md:121` | D-035 |

## DSOGI-FLL

| Claim | Status | Scope | Evidence | Decisions |
| --- | --- | --- | --- | --- |
| The DSOGI-FLL is **transparent on a clean observer** (no steady bias) | SUPPORTED | matched cell, constant speed, 1× amplitude | `docs/phase05a.md`; `docs/phase03.md` §13 | D-020/D-022, D-028 |
| The FLL shows **frequency/phase bias with the sign observer's ripple** | UNEXPLAINED | sign SMO, 1000/500 rpm, 100 µs | `docs/phase03.md` §12.1 | D-028 |
| Dynamic-error contribution: **observer 3–5 %** at `f_n ≤ 40 Hz` | SUPPORTED | S2/S3, replay, `f_n ∈ {20,40}` | `06f_replay/replay.json`; `docs/phase06f.md` §Step 2 | **D-081**, D-083 |
| At `f_n ≥ 80 Hz` **both the SOGI band-pass and the FLL** contribute | SUPPORTED | S2/S3, `f_n ∈ {80,160}` | `06f_replay/replay.json` (V4/V1 0.91→2.97; V2/V4 1.64→2.98) | D-083 |
| V4 (oracle ω′) does **not** restore V0's scaling (`V4/V0` → 3.7) | SUPPORTED (negative) | `f_n = 160`, S2/S3 | `06f_replay/replay.json` | D-083 |
| `τ_FLL = k_s·ω′/(Γ·A²)`, `A = ψ_f·ω_e` | HYPOTHESIS (empirical form) | 1000/800/500 rpm, frozen amplitude | `docs/phase06g.md` §Step 0; measured 3.82 ms @1000 rpm | **D-085**, D-084 |
| `a0·τ` explains the FLL lag (85 rpm S2 / 24 rpm S3) | HYPOTHESIS | S2 @1000 rpm, S3 @1000 rpm | `docs/phase06g.md` §Step 0; `06f_replay/replay.json` | D-085 |
| The factor-2 form of the FLL `τ` | **RETRACTED** | — | `docs/phase06g.md` §Step 0 | D-085 |

## Estimator (position/speed)

| Claim | Status | Scope | Evidence | Decisions |
| --- | --- | --- | --- | --- |
| Eq.(8) arctan on raw `E_hat` is unusable (ripple); needs a PLL | SUPPORTED | fixed-frequency, all Ts | `docs/phase05b.md`; `06e_ladder.json` R1 arctan | D-044 |
| Q-PLL **acceleration-step model** (`e_peak ≈ 0.456·a0/ω_n`) | SUPPORTED (V0 within 20 %) | V0 ideal E, `f_n ∈ {20..160}`, S2/S3 | `06f_replay/replay.json` (V0 87/42/20/10 vs 101/44/21/11) | **D-081**, D-075 |
| The earlier `2·sin(φ/2)` model and "ratio ∝ α_ω" | **RETRACTED** | — | `docs/phase06c2.md` §Step 2 | D-072 |
| `normalize=True` Q-PLL option | **RETRACTED (defect)** — see known defects | non-paper option only | `06e_step1/lockpoint.json`; `docs/estimator_spec.md` register | **D-076** |

## Closed loop (sensorless)

| Claim | Status | Scope | Evidence | Decisions |
| --- | --- | --- | --- | --- |
| **Gate P PASS on S1–S4** (paper-shaped scenarios) | SUPPORTED | nominal chain, P-set, `α_ω` per scenario | `06b6_gatep/gatep.json`; `docs/phase06b.md` | **D-063/D-064** |
| 500 rpm loaded steps (20–50 %) lose synchronism | SUPPORTED (negative) | 500 rpm, loaded steps, `min_ω = 100 rpm` | `06b6_scenarios.json`; `docs/phase06b.md` | D-062, D-064 |
| The 1000 rpm **rated** step loses synchronism; 50 % passes | SUPPORTED | S2 at 1000 rpm | `06c_comparison.json`; `docs/phase06c.md` | D-069 |
| Speed-loop bandwidth limit ≈ **10 Hz @ 1000 rpm**, **2.5 Hz @ 500 rpm** | **ASSUMPTION** (empirical) | measured `T_est`, nominal chain | `06b2_estimator_response.json`; `docs/phase06b.md` | D-056–D-061 |
| Max stable `α_ω` **falls** with `f_n` (15/10/4.4/7) | UNEXPLAINED (observed) | S2, `f_n ∈ {20,40,80,160}` | `06e_sweep/sweep.json`; `docs/phase06e.md` | D-077, D-080 |
| The **ω̂ path** carries the `f_n`-dependent instability | SUPPORTED | S2, `α_ω = 10 Hz`, `f_n ∈ {20,80}` | `06f_closed/closed.json`; `docs/phase06f.md` §Step 4b | D-081 |
| Handover protocols (`ω_h = 400 rpm`, `t_h = 0.30 s`, generic 5°) | **ASSUMPTION** | comparison layer only | `docs/closed_loop_assumptions.md` | D-053, D-070, D-071 |
| Stable `α_ω = 2–3 Hz` recovers the `f_n` trend; the 06e "flat" row was a stability-limit artefact | SUPPORTED | S2, `α_ω ∈ {2,3}`, fixed `t_h` | `06f_closed/closed.json`; `docs/phase06f.md` §Step 4a | D-081, D-080 |

## Dead time / inverter

| Claim | Status | Scope | Evidence | Decisions |
| --- | --- | --- | --- | --- |
| Average-value dead-time model (`ΔV = −V_dc·t_d·f_sw·sat(i/i_th)`), default OFF | SUPPORTED (model) | `t_d = 2 µs`, `f_sw = 10 kHz`, `i_th = 0.1 A` (assumed) | `tests/test_dead_time.py`; `docs/phase06c.md` | **D-069** |
| **Gate D′** predictions held within **12–25 %**, all on the low side | SUPPORTED | encoder-fed loop, 500/1000 rpm, I_rated & 20 % | `06c2_deadtime/deadtime.json`; `docs/phase06c2.md` §Step 4 | **D-074** |
| Dead-time **compensation** restores accuracy (S4 0.61° → 1.12°) | SUPPORTED | matched chain, S4, ON+comp | `06c2_deadtime/deadtime.json`; `06c2_suite/suite.json` | D-074 |
| Compensation is **not printed** in the paper (our construction) | SUPPORTED (provenance) | — | `docs/parameter_extraction.md` | D-074 |
| Gate D "SMO transmits ≈0.6 of ΔV" | **RETRACTED** | — | `docs/phase06c.md` §Step 2 | D-069, D-073 |

## Paper comparison

| Claim | Status | Scope | Evidence | Decisions |
| --- | --- | --- | --- | --- |
| **Steady** estimation errors are **below** the paper's reported values | SUPPORTED | S1/S2/S3/S4, nominal chain, dead time OFF | `06b6_gatep/gatep.json`; `docs/paper_comparison.md` | D-064 |
| **Dynamic** estimation errors are **4–5× above** the paper's (S2 111.7 vs 23; S3 40.4 vs 10 rpm) | SUPPORTED (as measured) | S2/S3 at the nominal cells | `06b6_gatep/gatep.json`; `docs/paper_comparison.md` | D-064, D-081 |
| The dynamic-error **cause**: the paper's load transition is **gentler** than an instantaneous step on `J = 0.00153` | **HYPOTHESIS (supported, scoped)** | open-loop replay, prescribed profile; `a0 ≈ 5 000` rpm/s (S2) / `≈1 400` rpm/s (S3) inferred from the printed errors, vs printed `ΔT/J = 24 027 / 9 611` | `06g2_profiles/profiles.json`; `docs/phase06g.md` §Step 2/3 | **D-085, D-086, D-087** |
| Cause of the **remaining** S3 gap (hardware non-idealities, offsets, parameter mismatch) | **UNEXPLAINED** | S3 hardware | `docs/phase06g.md` §Step 2 verdict 3 | D-086 |
| Fig. 13 dip: **190 / 235 / 350 rpm** all carried | UNEXPLAINED (reading) | 300 dpi render, truncated axis | `docs/figures/`; `docs/phase06g.md` §Step 0 | D-083 |
| Fig. 19 ΔN = 45 / 48 rpm | SUPPORTED (printed) | exp 20 % | `docs/figures/fig19_*` | D-083 |
| "paper-implied α_ω 38–43 Hz" | **RETRACTED** | — | `docs/phase06d.md` §0g | D-066, D-080 |
| "paper-implied `f_n` 76/70 Hz" | **RETRACTED** | — | `docs/phase06g.md` §Step 0 | D-075, D-085 |
| "the FLL alone is responsible" | **RETRACTED** | — | `docs/phase06f.md`; `docs/paper_comparison.md` | D-083 |
| "LPF lag dominant steady error" | **RETRACTED (artefact)** | uncompensated conventional cell only | `docs/phase06e.md` §Step 3 | D-080 |

## RETRACTED list (consolidated)

| # | Retracted claim | Decision |
| --- | --- | --- |
| 1 | `epsilon_filter_tau = 10 ms` results (all filtered 03/03d numbers) | D-028 |
| 2 | Pre-D-025 observer numbers (`g(S_post)`) | D-025, D-028 |
| 3 | Stationary-fit ramp metrics; old `current_rmse_a`/`mean|S|` pair | D-035 |
| 4 | Gate D "SMO transmits ≈0.6 of ΔV" | D-069 |
| 5 | "paper-implied α_ω 38–43 Hz" | D-066/D-080 |
| 6 | "paper-implied `f_n` 76 Hz" | D-075/D-085 |
| 7 | "the FLL alone is responsible" | D-083 |
| 8 | "error/dip ratio ∝ α_ω" | D-072 |
| 9 | "`2·sin(φ/2)`" model | D-072 |
| 10 | "LPF lag dominant" (uncompensated-cell artefact) | D-080 |
| 11 | Factor-2 FLL `τ` form | D-085 |
| 12 | "FLC reduces chatter by 17.5 % (MA)"; "FLC strongly helps at 100 µs" | D-033, D-035 |

## Inconsistencies found and fixed in this wrap-up

| Where | Contradiction | Resolution |
| --- | --- | --- |
| `phase06f.md` §Step 0c | claimed the verbatim "KNOWN DEFECT" label was in `README.md` | corrected (label added; claim annotated) — **D-083** |
| `phase06f.md`, `phase06.md`, `paper_comparison.md` | "the FLL alone is responsible" | re-attributed to SOGI+FLL — **D-083** |
| `phase06.md` limitations | "at 500 rpm … the FLL lag is about 85 rpm" | corrected: 85 rpm = **S2 at 1000 rpm** (S3: 24 rpm) — D-083 |
| `README.md` | phase-06f-complete / phase-06-complete presented as final | marked **PROVISIONAL/superseded** — D-083 |
| `docs/phase06g.md` | Step 0c quoted a `τ` "10–20 %" agreement | replaced by the speed-specific comparison; factor-2 form retracted — D-085 |
