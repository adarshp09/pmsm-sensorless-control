# Implementation Decisions

A short, dated decision log. Each entry records *what* was decided, *why*, and
whether it is paper-mandated or a local implementation choice. Paper-specific
numbers are **not** decided here — they are extracted (or left unresolved).

## D-001 — Source paper is `docs/act.pdf`

- **Decision:** Use the repository PDF as the sole Phase 00 source.
- **Why:** It is a readable 12-page copy of the cited IEEE article.
- **Type:** source-control decision.
- **Consequence:** Values not stated in the PDF remain null/unresolved.

## D-002 — Repository layout: `src/` layout, installed as a package

- **Decision:** Use a `src/pmsm_sensorless/` package with setuptools
  (`packages.find` where `src`), editable install.
- **Why:** Standard, importable, avoids accidental imports of the working
  directory, and keeps tests honest about what is actually installed.
- **Type:** implementation choice.

## D-003 — Configuration as validated YAML

- **Decision:** Motor/observer/experiment parameters live in `configs/*.yaml`
  and are loaded and **structurally validated** by `pmsm_sensorless.config`.
- **Why:** Keeps paper numbers out of source; makes missing values explicit
  (`value: null`) instead of silently defaulting; a bad parameter type fails
  loudly in a test rather than deep inside a simulation.
- **Type:** implementation choice.
- **Format:** each numeric parameter is `{value, unit, source}`; `value: null`
  means unresolved. Non-scalar specs (rule tables, transfer-function forms) are
  kept under a separate `specs:` mapping, also unresolved.

## D-004 — Fuzzy controller: hand-written first, scikit-fuzzy optional

- **Decision:** `scikit-fuzzy` is an **optional** extra (`.[fuzzy]`), not a hard
  dependency; the FLC is expected to be implemented directly.
- **Why:** The paper's exact membership functions, inference method and
  defuzzification must be reproducible; a library's own defaults can silently
  differ. Direct implementation keeps those choices explicit and testable.
- **Type:** implementation choice.
- **Status:** the FLC is **not** implemented in Phase 00.

## D-005 — No algorithms in Phase 00

- **Decision:** Plant, Clarke/Park, SMO, FLC, SOGI/DSOGI and FLL sub-packages
  contain only docstrings.
- **Why:** The specification must be fixed before code (Master Prompt 01,
  acceptance criterion: "No observer algorithm has been prematurely
  implemented").
- **Type:** process constraint.

## D-006 — No git repository initialisation in this step

- **Decision:** Do not run `git init` or create commits.
- **Why:** The parent directory (`~/Desktop`) is already a Git repository; a
  nested repo would be a footgun, and committing was not requested.
- **Type:** implementation choice.

## D-007 — Placeholder values are forbidden

- **Decision:** A `null` parameter is never replaced by a "reasonable" number
  without an explicit `assumed` status and an entry here.
- **Why:** Silent defaults are the main way a reproduction drifts from the
  paper while still appearing to work.
- **Type:** process constraint.

## Open decisions (blocked on the paper)

- Clarke transform scaling (changes all gains).
- Discretization method and Tustin prewarping.
- Whether the FLC output adapts the switching gain `k`, the sliding gain, or a
  boundary layer, and in what units.
- Which baselines to reproduce for comparison.
- Clarke scaling, axis signs, Q-PLL equations, FLL error algebra, FLL limits and
  initialization, FLC scaling/breakpoints, DSOGI channel combination, speed
  conversion, solver/timestep, and noise injection remain unspecified by the
  paper and are not filled in Phase 00.

---

## Phase 01 — plant and transforms (2026-10-14)

## D-008 — Plant implements the paper's electrical model exactly; kinematics are
inputs by default

- **Decision:** `pmsm_sensorless.plant.pmsm.PMSMPlant` integrates only the
  stationary `alpha-beta` current states of Eq.(1), p.2, with the back-EMF of
  Eq.(2), p.2, and with `theta_e`/`omega_e` supplied by the caller (prescribed
  kinematics). The paper's printed model has no mechanical equation and treats
  speed/angle as inputs, so nothing beyond the printed model is assumed.
- **Why:** Reproduces Eq.(1)-(2) without inventing a mechanical law, friction
  value, or dq torque convention that the paper does not print.
- **Type:** paper-faithful scope decision.
- **Consequence:** the default plant reports currents only; speed/angle are
  reported as *prescribed inputs*.

## D-009 — Plant integration method is classical RK4

- **Decision:** state updates use a classical fourth-order Runge-Kutta step
  (`plant.pmsm.rk4_step`), applied to the electrical states (and the mechanical
  states when enabled).
- **Why:** the paper does not specify a plant solver (only the SOGI Tustin
  prewarping is given). RK4 is deterministic, has no stability/tuning choices,
  and matches closed-form solutions to <1e-6 relative on the validating cases.
- **Type:** implementation choice.
- **Validation:** `tests/test_plant.py` checks the exponential decay, the
  rotating short-circuit current, and `rk4_step` against closed forms.

## D-010 — Clarke scaling is configurable; default amplitude-invariant

- **Decision:** `transforms.clarke` provides both amplitude-invariant and
  power-invariant Clarke matrices, selected by an explicit `scaling` argument
  (default `"amplitude_invariant"`).
- **Why:** the paper uses `alpha-beta` quantities but prints no Clarke matrix,
  so the scaling cannot be taken from the source. Amplitude-invariant is chosen
  as the default because it is the convention consistent with the paper's
  back-EMF amplitude, `|E| = omega_e * psi_f`, in Eq.(2).
- **Type:** PROVISIONAL / documented implementation convention.
- **Consequence:** **this is an assumption, not a paper value.** It must not be
  claimed to reproduce the paper. If the paper's source matrix is later found,
  the default changes in one place (the `SCALING_CONVENTIONS` table).

## D-011 — Optional mechanical extension (derived; explicitly not the paper's model)

- **Decision:** `PMSMPlant(..., integrate_mechanics=True)` optionally integrates
  `J d(omega_m)/dt = T_e - T_L - B omega_m`, `d(theta_m)/dt = omega_m`, with
  `T_e = (3/2) p psi_f i_q`, `i_q = -i_alpha sin(theta_e) + i_beta cos(theta_e)`
  (valid because `Ld = Lq`). It is off by default.
- **Why:** `J`, `p` and a load torque are reported, so a torque balance is the
  natural derived extension and lets speed/angle behaviour be demonstrated. The
  paper prints none of these equations.
- **Type:** derived / implementation choice, **not** a reproduction of the paper.
- **Friction:** `B` is unresolved in the paper and is never defaulted silently.
  Enabling mechanical mode requires an explicit `viscous_friction` when the
  config value is `null`; otherwise construction raises `ValueError`.

## D-012 — Ground-truth separation

- **Decision:** the plant never consumes observer estimates; it exposes
  ground-truth states only. Observer, estimator and evaluation code may read the
  plant state, but the plant may not read their outputs.
- **Why:** required so that Phase 02+ estimators cannot accidentally leak truth.
- **Type:** architectural constraint.

## D-013 — Unresolved bonus items remain out of scope

- **Decision:** the Clarke zero-sequence treatment, angle-wrap boundary, and the
  plant solver were resolved only as documented local conventions (D-009,
  D-010). The FLC/DSOGI/FLL/Q-PLL equations, speed conversion and noise injection
  remain unresolved and unimplemented.
- **Type:** scope constraint.

---

## Phase 02 — conventional and sigmoid sliding-mode observers (2026-10-14)

## D-014 — Observer implements Eqs.(3)-(4) and (7) verbatim; two variants

- **Decision:** `observers/conventional_smo.py` and `observers/sigmoid_smo.py`
  implement the paper's current observer with a shared base
  (`observers/smo.py`). Sliding surface `S = i_hat - i` (Eq.(3));
  current dynamics `Ld d(i_hat)/dt = u - Rs*i_hat - k*g(S)` (Eq.(4)); back-EMF
  estimate `E_hat = k*g(S)` (Eq.(7)). The injection polarity is confirmed against
  the error dynamics of Eq.(6).
- **Why:** these are the only observer equations the paper prints for this stage.
- **Type:** paper-faithful.
- **Scope:** no FLC, SOGI/DSOGI, FLL, Q-PLL or position/speed estimation (later
  phases). The observer consumes only `(u_alpha, u_beta, i_alpha, i_beta)` and a
  timestep; it never reads the plant's true angle, speed or back-EMF.

## D-015 — Paper `k`/`a` are per-unit; converted to SI via peak bases

- **Decision:** the config values `k = 1` and `a = 4` (p.5) are per-unit. The
  paper states the back-EMF reference is `311/sqrt(3)` V (p.5), i.e. the peak
  phase voltage `V_base = sqrt(2)*V_ll_rms/sqrt(3)`. `from_config` therefore maps
  `k_si = k_pu * V_base` (a voltage gain) and `a_si = a_pu / I_base` (the sliding
  surface is a current). Pass `per_unit_scaling=False` to keep the raw values.
- **Why:** with `k = 1` interpreted as 1 V in SI the sliding reachability
  condition `k > |E|` fails (at 500 r/min `|E| ~ 27.8 V`), so the paper's number
  cannot be a physical SI volt value.
- **Type:** documented unit interpretation.
- **Open item:** the current base `I_base = sqrt(2)*P/(sqrt(3)*V_ll_rms)` is
  **derived** from Table III (rated 220 V line, 2 kW), not printed by the paper.
  It sets the sigmoid boundary layer and is the single scaling assumption here.

## D-016 — Observer discretization is explicit forward Euler

- **Decision:** the observer uses a forward-Euler update at the control period
  `Ts = 100 us`; the measurement is zero-order held over the step. The plant
  (D-009) remains RK4.
- **Why:** the standard digital SMO implementation; the paper specifies only the
  SOGI prewarping, not a solver for the observer.
- **Type:** implementation choice.
- **Consequence:** the discrete switching produces a finite chattering
  amplitude `~ Ts*k/Ld`; this is physical and is the effect the paper reduces.

## D-017 — No back-EMF low-pass filter in this phase

- **Decision:** the reported back-EMF estimate is the raw Eq.(7) switching signal
  `k*g(S)`. No LPF is applied.
- **Why:** the paper's LPF cutoff is unresolved (`smo.observed_emf_lpf_cutoff` is
  `null`) and the paper replaces the LPF with DSOGI-FLL, which is a later phase.
- **Type:** scope/fidelity decision.

## D-018 — One Phase-00 stub test intentionally updated

- **Decision:** `tests/test_package_smoke.py::test_algorithm_subpackages_are_still_stubs`
  no longer lists `observers`. Importing observer submodules binds them as
  attributes of the `observers` package, so the "package must be empty" invariant
  cannot hold once the package is implemented; the test now guards only the
  still-unimplemented `filters`/`fuzzy`/`estimation`.
- **Type:** deliberate test update (review signal), as anticipated in Phase 00.

---

## Phase 03 — DSOGI-FLL (2026-10-14)

## D-019 — SOGI-QSG discretization is the paper's Tustin-with-prewarping

- **Decision:** `filters/sogi.py` discretizes Eqs.(9)-(10) with the prewarped
  bilinear substitution of Eq.(17) (integrator `tan(w'Ts/2)/w' (1+z^-1)/(1-z^-1)`),
  giving the biquad `D(z)`/`Q(z)` with `numerators` `k_s sin(w'Ts)(1-z^-2)` and
  `k_s(1-cos w'Ts)(1+z^-1)^2`, shared denominator
  `(2+k_s sin w'Ts) - 4cos(w'Ts) z^-1 + (2-k_s sin w'Ts) z^-2`.
- **Why:** this realizes the paper's stated method and satisfies the resonance
  property exactly: at `z=exp(j w'Ts)`, `|D|=1`, `angle(D)=0`, `|Q|=1`,
  `angle(Q)=-90 deg`, and `Q(1)=k_s`.
- **Flagged inconsistency:** the coefficients printed in the paper's Eqs.(18)-(19)
  (as extracted) carry an extra `k_s` in the `z^-1` denominator term and a factor
  of 2 in the `D` numerator; with those, `|D| = 0.0135` and `Q(1) = -0.0068` at
  resonance, contradicting Eq.(17) and Eq.(9)-(10). The correct prewarped-bilinear
  coefficients above are used. (Recorded as a paper inconsistency.)
- **Type:** paper-faithful (with a flagged source inconsistency).
- **Phase 03b verification:** the printed coefficients were re-checked against a
  **rendered image of the PDF page** (not just the text layer). The discrepancy
  **survives** — the page really prints the extra `k_s` (in the `z^-1` term) and the
  factor 2 (in the `D` numerator). The prewarped-bilinear coefficients remain the
  implementation.

## D-020 — FLL error signal and integrator are derived, not printed

- **Decision:** `epsilon_f = qv'_a(v_a - v'_a) + qv'_b(v_b - v'_b)` and
  `d w'/dt = -Gamma epsilon_f`, matching the Fig.5 structure ("integrator with
  negative gain -Gamma").
- **Why:** the paper shows the FLL block and `Gamma=50` (p.5) but never prints the
  algebraic error signal (paper_method.md item 9). The standard SOGI-FLL form is a
  documented derived equation; the sign is verified against the measured
  discriminator gain (corrects in the right direction for `w'` too high/low).
- **Type:** derived / documented.
- **Consequence:** no normalization, clamp or extra filter is applied by default.

## D-021 — DSOGI positive-sequence combination is derived

- **Decision:** `v_a+ = (v'_a - qv'_b)/2`, `v_b+ = (qv'_a + v'_b)/2`
  (`positive_sequence=True`, default). `False` uses the per-axis band-pass of
  Eq.(12).
- **Why:** the paper uses the DSOGI structure but does not print the combination
  equations (item 8); this standard form gives unity gain for a balanced positive
  sequence and rejects the negative sequence (verified in tests).
- **Type:** derived / documented.

## D-022 — `Gamma = 50` used verbatim in SI; loop bandwidth is amplitude-dependent

- **Decision:** `Gamma = 50` (config, p.5) is applied directly to SI signals.
  Normalization is available but **off by default**.
- **Why:** the error signal is quadratic in amplitude, so the un-normalized loop
  bandwidth scales with the squared signal amplitude; the paper's normalization
  unit (Fig.5) is what removes that dependence, but its algebra is not printed.
  With clean SI-scale back-EMF (~50 V) and `Gamma=50` the loop locks in ~0.05 s.
- **Consequence:** the paper's `Gamma` frame is unresolved; treat `Gamma` as
  calibrated for the signal scale in use.

## D-023 — Unresolved FLL values are explicit

- **Decision:** initial `w'` is unresolved (`fll.initial_frequency` and
  `dsogi.nominal_center_frequency` are `null`); `from_config` raises
  `ConfigError` unless `initial_omega` is supplied. Frequency limits
  (`fll.min_frequency`/`max_frequency`) are applied only if resolved. The error
  low-pass (`epsilon_filter_tau`) and `normalize_error` are labeled options,
  off by default.
- **Validation usage:** the Phase 03 experiment enables `epsilon_filter_tau=10 ms`
  and `min_omega=2*pi*2 rad/s` explicitly (documented), because the Phase 02
  back-EMF estimate is a near-bang-bang switching signal whose ripple otherwise
  drives the un-normalized loop unstable. These are **not** paper values.

## D-024 — Filters are plant/observer-independent

- **Decision:** `filters/` imports neither the plant nor the observers; the
  inputs are `alpha-beta` signals, `dt` and config. The SMO -> DSOGI-FLL
  composition lives only in `experiments/`.
- **Type:** architectural constraint (matches D-012).

---

## Phase 03b — verification and diagnosis (2026-10-14)

## D-025 — SMO reported `g(S_post)` instead of the applied `g(S_pre)` — **FIXED** (Phase 03c)

- **Finding (Phase 03b):** `SlidingModeObserver.step` returned
  `back_emf = k*g(i_hat_new - i)`, i.e. the switching function recomputed from the
  **post-update** estimate, whereas the injection actually applied in Eq.(4) uses the
  **sample-instant** error. For the discontinuous `sign` function the one-step
  overshoot flips the sign of `S`, corrupting the *reported* signal; at 1000 rpm its
  fundamental was gain 0.61 / phase -49 deg (100 us) vs the applied 0.957 / -1.6 deg.
- **Correction to the earlier claim:** "the sigmoid is nearly unaffected" is
  **REFUTED**. Phase 03c Step 1 (both observers, 100/50/25 us, 1000 rpm) shows the
  sigmoid reported signal is also badly corrupted (100 us: reported 0.479 / -59.4 deg
  vs applied 0.962 / -1.3 deg; 25 us: 0.653 / -37.9 deg vs applied 0.987 / -0.6 deg).
  Both variants were affected.
- **Fix (Phase 03c):** `step` now computes the switching at the sample instant and
  reports `s_* = i_hat - i` and `back_emf_* = k*g(S_pre)` -- the injection actually
  applied. The current update, parameters and defaults are **unchanged**; only the
  reported sliding-surface/back-EMF convention changed.
- **Tests:** `tests/test_smo_reported_injection.py` (sinusoidal back-EMF; sign
  reported gain/phase vs truth at 1000 rpm, tolerances from the measured
  0.996 / -0.2 deg at 10 us and 0.957 / -1.6 deg at 100 us); the two hand-calculated
  single-step tests updated to expect `S_pre`.
- **Type:** bug fix.

## D-027 — Discrete small-signal stability bound for the sigmoid (forward Euler)

- **Result:** with `g(S) ~ (a/2)S`, the linearised error dynamics are
  `S[k+1] = (1 - Ts (Rs + k a/2)/Ld) S[k]`, stable iff
  `Ts (Rs + k a/2)/Ld < 2`, i.e. `Ts* = 2 Ld/(Rs + k a/2)`.
- **Numbers (paper-derived):** `k = 179.63 V`, `a = 0.5389 1/A`, `Ld = 1.47 mH`,
  `Rs = 0.505 ohm` -> `Ts* = 60.1 us`. Sweep (1000 rpm) confirms the period-2 limit
  cycle for `Ts >= 65 us` and a clean signal for `Ts <= 60 us`. At the paper's
  `Ts = 100 us` the sigmoid observer is therefore **unstable** (period-2 limit cycle)
  with these SI gains. Largest stable slope at `Ts = 100 us`: `a <= 0.322 1/A`,
  implying a per-unit current base `I_base >= 12.4 A` (we use 7.42 A, D-015).
- **D-016 unchanged:** forward Euler is kept. An implicit / exact discretization of
  the injection is a possible *option* (it would remove the `Ts` bound), but is
  **not implemented** and is out of scope here.
- **Type:** analysis + documented limitation.

## D-028 — FOUND: the error low-pass (`epsilon_filter_tau = 10 ms`) destabilises the FLL

- **Finding (Phase 03d isolation):** the Phase 03 experiment enabled
  `epsilon_filter_tau = 10 ms`. With a clean input the loop then shows a ~−7.5 Hz
  bias and 16 Hz jitter at 1000 rpm and fails at 500 rpm (gain 0.075). With the
  **library default** (no error LPF) the same clean input locks to −0.001 Hz / gain
  1.000 at both speeds, and the post-fix stable SMO locks to ~0 Hz / gain 0.98.
- **Consequence:** the Phase 03 post-fix ~−5 Hz bias and ~0.85 gain ratio are caused
  by the **error-LPF option**, not by the input ripple or the observer. The LPF is
  off by default in `DSOGIFLLParameters`; only the experiment turned it on. Its
  purpose was to tame the pre-fix chattering signal, which D-025 has since fixed.
- **Status:** reported only; no filter/default change. **Phase 03e retracts** every
  filtered number computed with this option (see `phase03.md` §0 and §13) and
  re-bases them with `epsilon_filter_tau=OFF` in `experiments/03e_filter_rebase.py`.
  The 03/03d JSONs are tagged `"retracted": true` / list the option via
  `config_diff`. Removing the LPF from the 03/03d experiments (or choosing a much
  smaller time constant) remains a separate approved step; the filter default is
  already OFF.
- **Type:** verification finding / bug report.

## D-029 — Regime labels for the sign-vs-sigmoid comparison

- **Decision:** the 100 µs sign-vs-sigmoid results are labelled **"limit-cycle vs
  bang-bang"** (sigmoid is a period-2 limit cycle for `Ts >= 65 µs`, D-027); the fair
  comparison is the **stable regime at `Ts = 50 µs`** (sign RMSE 5.10 A / ripple
  248.1 V vs sigmoid RMSE 0.922 A / ripple 0.02 V). Recorded in `phase02.md` and
  `phase03.md`.
- **Type:** reporting convention.

## D-026 — Metric redefinition and re-based claims

- **Decision:** replace `std(E - one-period moving average)` with the Phase 03b
  definitions (fundamental gain/phase by least-squares fit at the true electrical
  angle, ripple = RMS about the fitted fundamental, total error = RMS vs truth).
  See `experiments/validation_metrics.py` and `docs/phase03.md`.
- **Consequence:** the phase02 "17.5 %" claim is re-based to **17.7 %** (steady-window
  raw ripple, sign vs sigmoid); the phase03 "87-89 %" claim is re-based/strengthened
  to **97.1 % (sign)** / **99.4 % (sigmoid)** steady-window raw -> filtered ripple
  reduction at `Ts = 100 us` (ramp: 88.8 % / 97.6 %).
- **Type:** verification.
