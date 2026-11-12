# Final report — pmsm-sensorless-control (v1.0)

## 1. Purpose

A modular, test-driven Python implementation and experimental study of *"An Improved Sliding
Mode Observer Sensorless Control for PMSM Based on Fuzzy Logic Controller and DSOGI-FLL"*
(`docs/act.pdf`). The goal is a **fidelity-audited** comparison: every value is labelled
PRINTED, DERIVED or ASSUMPTION, and every claim carries a status (see
[claims_register.md](claims_register.md)). **The paper is not reproduced or validated**; the
results are comparative simulation findings under stated assumptions.

## 2. What was built (module map)

- `plant/` — PMSM (Eqs. 1–2) + load models (constant, step, callable).
- `transforms/` — amplitude-invariant Clarke (D-010).
- `observers/` — conventional sign SMO, fixed sigmoid SMO, fuzzy-adaptive SMO (Eqs. 3–7).
- `filters/` — SOGI-QSG + DSOGI-FLL (Eqs. 9–19, Fig. 5).
- `estimation/` — arctan position (Eq. 8) + Q-PLL (`normalize` option = defect).
- `fuzzy/` — FLC engine (assumed Ke/Kec/Ku, MFs, operators — D-030/031).
- `simulation/` — closed-loop drive (FOC, 10 kHz/1 kHz), nominal drive, dead-time model.
- `configs/` — all parameters, each tagged PRINTED/DERIVED/ASSUMPTION.
- `experiments/` — phase scripts + chain composition (no src coupling).
- `tests/` — **255 tests**.

## 3. Headline results (scoped)

- **Gate P PASS on S1–S4** (paper-shaped scenarios, nominal chain, `Ts = 100 µs`, dead time
  OFF) — `experiments/output/06b6_gatep/gatep.json`; D-063/D-064.
- **Steady** estimation errors are **below** the paper's; **dynamic** errors are **4–5×
  above** (S2 111.7 vs 23 rpm; S3 40.4 vs 10 rpm) — `gatep.json`; D-064/D-081.
- The dynamic gap is **consistent with a gentler paper load-transition** than an
  instantaneous step on `J = 0.00153` (**HYPOTHESIS**, `a0 ≈ 5 000 rpm/s` S2 / `≈1 400` S3
  inferred vs printed `ΔT/J = 24 027 / 9 611`) — `06g2_profiles/profiles.json`; D-086/D-087.
- The **observer adds only 3–5 %** to the dynamic error at `f_n ≤ 40 Hz`; the **SOGI+FLL**
  contributes the rest (V2/V1 1.5× → 8.9×) — `06f_replay/replay.json`; D-083.
- **Stress limits:** 500 rpm loaded steps (20–50 %) and the 1000 rpm **rated** step lose
  synchronism — `06b6_scenarios.json`, `06c_comparison.json`; D-062/D-069.
- **Dead time:** average-value model, default OFF; Gate D′ held within 12–25 %; compensation
  restores accuracy (not a printed paper item) — `06c2_deadtime/deadtime.json`; D-069/D-074.

## 4. Paper-vs-ours (metric bases: **estimation errors**, not true-speed dips)

Nominal chain, `α_ω` per scenario, `Ts = 100 µs`, dead time OFF. Source:
`docs/paper_comparison.md`; `06b6_gatep/gatep.json`.

| Scenario | Paper | Sensorless (ours) | Assessment |
| --- | --- | --- | --- |
| S1 (no-load 500→1000 rpm ramp) | max speed err ~5 rpm; pos 0.07/0.2 rad | steady 0.61° / 0.15 rpm; dyn 2.24° | matches directionally; setup differs |
| S2 (sim 50 % step @1000) | dyn speed 23 rpm; pos 0.22 rad | steady 0.56°; dyn 21.7° / **111.7 rpm** | steady below; **dynamic 4.9×** |
| S3 (exp 20 % step @1000) | ~10 rpm; pos 0.152 rad | steady 0.59°; dyn 7.5° / **40.4 rpm** | steady below; **dynamic 4.0×** |
| S4 (steady 1000) | 10 rpm; 0.08 rad | steady 0.61° / 0.1 rpm | below the paper |

Fig. 13's true dip is **unresolved**: 190 / 235 / 350 rpm across three reads
(`docs/figures/`); Fig. 19's printed ΔN = 45/48 rpm.

## 5. What is NOT reproduced, and why

- **No exact reproduction claim.** Unprinted: Q-PLL gains/normalization algebra, speed PI
  gains, DC-bus voltage, inverter details, load-transition profile, position-error units
  (mechanical vs electrical), several timing/profile details (`docs/parameter_extraction.md`).
- **Dynamic errors are larger.** Cause is a **HYPOTHESIS** (gentler transient), not a
  reproduction; the residual S3 gap is **UNEXPLAINED**.
- **Low-speed/high-load transients fail** in our model (D-062/D-064) while the paper reports
  operation; the paper prints no start-up/low-speed method (Section V-E, qualitative).

## 6. Top-10 unprinted parameters (assumed values)

| # | Parameter | Assumed value | Origin |
| --- | --- | --- | --- |
| 1 | Q-PLL bandwidth `f_n` | 20 Hz | ASSUMPTION (D-077) |
| 2 | Speed-loop bandwidth `α_ω` | 2.5 Hz @500 / 10 Hz @1000 rpm | ASSUMPTION/empirical (D-060/D-061) |
| 3 | FLC scalings `Ke/Kec/Ku` (+ MF/operators) | as in `configs/fuzzy_assumptions.yaml` | ASSUMPTION (D-030/031) |
| 4 | Handover (`ω_h`, `t_h`, generic 5°) | 400 rpm; 0.30 s; `sin 5°` | ASSUMPTION (D-053/D-070/D-071) |
| 5 | Per-unit current base `I_base`; `k`/`a` SI scaling | `configs/observer.yaml` | DERIVED (D-015) |
| 6 | Conventional LPF cutoff | grid 100–400 Hz | ASSUMPTION (D-070) |
| 7 | DC-bus voltage `V_dc` | 311 V | ASSUMPTION (D-047) |
| 8 | Dead-time current threshold `i_th` | 0.1 A | ASSUMPTION (D-069) |
| 9 | Dead-time compensation | our construction (flag, default OFF) | ASSUMPTION (D-074) |
| 10 | FLL `ω′_0` / clamps / `ε_f` filter | 500 rpm init; `min_ω = 100 rpm` | ASSUMPTION (D-053/D-062) |

## 7. Known defects

- **Normalized Q-PLL option (`normalize=True`)** — **KNOWN DEFECT, non-paper, do not use**:
  does not lock at 0° from a +90° start (settles ≈−39°, oscillating) — `06e_step1/lockpoint.json`;
  D-076. The nominal chain always runs `normalize=False`.
- **FLL `ω′ ≥ 0` guard** — trips on divergence; experiments freeze `ω′` and flag
  `guard-fail` (`experiments/chain.py`). Γ = 500/5000 are diagnostic-only (D-082).

## 8. Reading order

`README.md` → `docs/FINAL_REPORT.md` → `docs/claims_register.md` →
`docs/paper_comparison.md` → `docs/phase06.md` → `docs/phase06{a..g}.md` →
`docs/implementation_decisions.md` (D-001…D-087) → `docs/parameter_extraction.md`,
`docs/estimator_spec.md`, `docs/equations_and_conventions.md`, `docs/closed_loop_assumptions.md`,
`docs/flc_spec.md` → `docs/phase02–05*` → `docs/lint_status.md`.

## 9. Decision-log index by theme

| Theme | Decisions |
| --- | --- |
| Plant / transforms / units | D-001–D-015 |
| Conventional SMO | D-016–D-027 |
| DSOGI-FLL | D-020–D-029, D-082, D-085 |
| Fuzzy logic controller | D-030–D-035 |
| Nominal drive / metrics | D-036–D-039, D-047–D-051 |
| Position/speed estimator | D-040–D-046, D-075 |
| Closed-loop drive | D-047–D-055 |
| Handover protocols | D-053, D-070, D-071 |
| Speed-loop bandwidth | D-056–D-062 |
| Paper-scenario re-scope | D-063–D-065 |
| Bandwidth diagnosis | D-066–D-068, D-077 |
| Dead time | D-069, D-073, D-074 |
| Comparison protocol / dips | D-071, D-072 |
| Ablation ladder | D-078 |
| Replay decomposition | D-079–D-083 |
| Transient profile | D-084–D-087 |
