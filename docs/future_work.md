# Future work — Phase 07 and beyond (NOT STARTED)

Status of every item here: **UNTESTED / NOT STARTED**. Nothing in this file is a claim; it is
a specification for the next approved phase.

## Phase 07 — Robustness of the fuzzy-adaptive SMO under non-idealities

**The question to answer:** *does the FLC beat the matched fixed gain under non-idealities?*
The Phase 04c/06e comparison is a **nominal, like-for-like-uneven** one: the matched fixed
gain (`kF = 1`, `aF = 0.4`) was **selected on these scenarios**, while the FLC ran with
**assumed** scalings (`configs/fuzzy_assumptions.yaml`). No robustness test exists
(`docs/claims_register.md`, FLC rows).

### 07a — Offline replay with parameter error and measurement non-idealities

Replay the **recorded** `(u_αβ, i_αβ)` (Phase 06f `06f_record/record.npz`; Gate R exact, D-079)
through the observer/estimator chain with:

| Perturbation | Range | Placement |
| --- | --- | --- |
| `Rs` | ±20 % | observer only (plant untouched) |
| `Ld` | ±20 % | observer only |
| `ψ_f` | ±10 % | observer only (back-EMF amplitude) |
| Current noise | white, `σ` swept | measurement |
| Current offset | DC bias swept | measurement |
| Quantization | e.g. 12-bit ADC | measurement |

Metrics: steady/dynamic `|θ̂−θ|`, `|ω̂−ω|`, `E_hat` ripple, FLL status.

### 07b — Closed-loop confirmation of the best cells only

Only the cells that survived 07a enter the closed loop (`α_ω` per operating point, `t_h` =
0.30 s, dead time OFF, then ON+comp), on S1–S4 and the paper-shaped set.

### 07c — Decision rule to freeze

Whether the FLC's advantage over the matched fixed gain is **established under
non-idealities**; if not, the fixed gain stays the nominal choice and the FLC is reported as
**UNTESTED beyond nominal**.

## Additional items (separate approvals)

1. **Paper-literal FLL algebra (Fig. 5).** Implement the DSOGI-FLL exactly as printed where
   the algebra is currently **derived** (positive-sequence combination D-021; `ε_f` algebra
   D-020/D-022); re-run the Phase 03/06 comparisons (D-082 bars further bandwidth work unless
   a **printed** parameter differs — this is that case).
2. **Repair the normalized Q-PLL option.** Currently a **KNOWN DEFECT** (D-076): does not
   lock at 0° from a +90° start. Repair as a **separate approved src step**, then re-test
   `06e_step1` and the 06c/06c-2 normalization ablation.
3. **Switching-level PWM model.** Replace the average-value inverter; add dead time at the
   switching level and verify Gate D′ against it (currently average-value only, D-069).
4. **Speed-profile generator (reusable).** The Phase 06g-2 profile test used an
   **experiment-side** generator (`experiments/profile_drive.py`); no reusable `src` API was
   added (D-084/D-085). Promote it to `src/` only if other phases need it.
5. **Transient-profile exhaustion.** Extend the 06g-2 grid (more `a0`/`τ_d`, S3 hardware
   non-idealities) and test whether the inferred `a0 ≈ 5 000 / 1 400 rpm/s` holds under
   offsets and mismatch.
6. **Remote hosting + license.** There is **NO REMOTE** (see `README.md`). Nothing is pushed.
   The repository has **no LICENSE** — an **open decision for the owner**; do not add one
   without that decision.
7. **CI.** Wire `python -m pytest`, `python -m compileall -q src`, and
   `python scripts/reproduce.py --fast` into CI; hold the `ruff check --statistics` baseline
   (`docs/lint_status.md`, 694 findings) as a non-blocking report until reviewed.
