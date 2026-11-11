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
- **Option disclosure (Phase 03f):** `03` used **both** `epsilon_filter_tau = 0.01 s`
  **and** `min_omega = 12.566 rad/s`; `03d` used both on **all** rows, so its
  "LPF off" rows are really "LPF off, min_omega=12.566 rad/s ON". The floor-pinned
  03d rows are listed in `phase03.md` §14.1 (D-028 covers why the LPF numbers are
  retracted; the clamp is a separate non-default option).

## D-029 — Regime labels for the sign-vs-sigmoid comparison

- **Decision:** the 100 µs sign-vs-sigmoid results are labelled **"limit-cycle vs
  bang-bang"** (sigmoid is a period-2 limit cycle for `Ts >= 65 µs`, D-027); the fair
  comparison is the **stable regime at `Ts = 50 µs`** (sign RMSE 5.10 A / ripple
  248.1 V vs sigmoid RMSE 0.922 A / ripple 0.02 V). Recorded in `phase02.md` and
  `phase03.md`.
- **Phase 03f additions:** the paper-literal filter output is the in-phase `v'`
  (Eq.(12) `Ê_f = Ê·D`, re-checked on the rendered page); the positive-sequence
  output is the D-020 non-paper option, and the API does not expose both at once.
  **Corrected (Phase 04 Step 0b/0c):** the independent `scipy.signal.freqs` check
  proves `∠D = +atan((r²−1)/(k_s r))` with `r = ω′/ω`; the earlier "−atan /
  inverted formula" claim is **retracted**. The closed-loop filtered−raw phase
  differs because `ω′(t)` is time-varying, not because of a filter bug.
- **Type:** reporting convention.

## Phase 04 — FLC adaptation (D-030 to D-034)

## D-030 — FLC engine: printed structure, assumed numerics

- **Printed:** input sets {NB,NS,Z,PS,PB} over {-1,-0.5,0,0.5,1}; output sets
  {Z,PS,PM,PB} over [0,1]; triangular MFs; Mamdani; centre of gravity; the two 5x5
  rule bases; `k=kF*k0`, `a=aF*a0` (Tables I-II, p.4; p.3).
- **ASSUMPTION — not from paper:** Ruspini triangular partition with half-width =
  centre spacing; output centres equally spaced {0,1/3,2/3,1}; AND=min,
  aggregation=max; CoG on a 1001-point grid on [0,1]; one FLC per axis; `s_dot` by
  backward difference (zero first step); non-finite normalised input -> 0.
- All assumptions live in `configs/fuzzy_assumptions.yaml`, each key commented
  "ASSUMPTION — not from paper"; null/missing raises `ConfigError`. Printed items
  live in `configs/fuzzy.yaml`.
- **Type:** implementation choices (assumed).

## D-031 — Nominal FLC scaling assumptions

- **ASSUMPTION — not from paper:** `Ke = 1/I_base = 0.13472 1/A`,
  `Kec = Ts/I_base = 1.3472e-5 s/A`, `Ku = 1` (fixed by the printed [0,1] output
  universe). Only a 3x3 sensitivity grid (0.1x/1x/10x) was explored.
- **Type:** assumed values.

## D-032 — Adaptive observer is a subclass; base observers unchanged

- **Decision:** `observers/fuzzy_smo.py: FuzzySMO(SlidingModeObserver)` overrides
  `step` to use per-step/per-axis `k_t=kF*k0`, `a_t=aF*a0`; `smo.py` is unchanged.
- **Why:** keeps `ConventionalSMO`/`SigmoidSMO` bit-identical (verified: forced
  `kF=aF=1` reproduces `SigmoidSMO` exactly; constant `kF=aF=c` reproduces a fixed
  observer with `k=c*k0`, `a=c*a0`).
- **FLC inputs:** only the observer's own `S` and its backward difference; no plant,
  angle, speed or `E_true` (structure test).
- **Type:** implementation choice.

## D-033 — Phase 04 evaluates RAW E_hat only

- **Decision:** no DSOGI-FLL in Phase 04; no rotor position/speed. All results are
  filter-independent.
- **Type:** scope constraint.

## D-034 — FLC results and conditional conclusion

- **Result (nominal assumptions):** at `Ts=100 us` the FLC reduces the fixed-sigmoid
  ripple from 194.7 V (limit cycle) to **1.23 V** and current RMSE from 7.06 to
  **3.46 A**; at `Ts=50 us` it is worse than the stable fixed sigmoid (1.22 V vs
  0.02 V; 4.08 vs 0.92 A). kF*aF stays below the stability bound 100% of the time;
  `kF*k0 < |E_true|` ~5.3% of the time.
- **Type:** experimental result, **conditional on the assumed Ke/Kec/MF/operator/grid
  values**; not a paper reproduction.
- **Superseded (Phase 04b/04c):** the "reduces the limit-cycle default" framing is
  retracted — compare against matched fixed gain only (D-036, D-037). See
  `docs/phase04.md` §6 and `docs/phase04c.md`.

## Phase 04b — fixed-gain control and metric cleanup

## D-035 — Unified sliding-surface metrics; old RMSE/mean|S| pair retracted

- **Decision:** report `rms_s_alpha`, `rms_s_beta` (per-axis RMS), `mean|S|`, and
  `RMS|S| = sqrt(mean(Sα²+Sβ²))`, all from the **sample-instant** `S_pre` over the
  same window.
- **Why:** the old `current_rmse_a` used the **post-update** estimate while
  `mean|S|` used `S_pre`; one Euler step reduces the error, which is why the old
  "RMSE" appeared smaller than `mean|S|` in 5 of 6 rows. With `S_pre` for both,
  `RMS|S| >= mean|S|` always.
- **Consequence:** the old columns are marked/retracted; the Phase 04 tables are
  rewritten with the unified definitions.
- **Type:** metric definition (correction).

## D-036 — Matched fixed-gain control; FLC claims re-based

- **Decision:** compare the FLC against constant-gain `SigmoidSMO` cells
  (kF ∈ {0.35,0.5,0.75,1.0} × aF ∈ {0.1,0.2,0.4,0.6,1.0}, plus the FLC time-mean
  `0.355/0.591`) and against the Pareto front in `(t_conv, ripple)` and
  `(t_conv, |gain−1|)`.
- **Result:** 7 fixed-gain cells at `Ts=100 us` (16 across both Ts) **dominate the
  FLC nominal on ripple, |gain−1|, |phase| and RMS|S| simultaneously**; the matched
  fixed-gain cell has lower ripple (0.96 vs 1.23 V) and lower `max|S|` (2.78 vs
  3.36 A). **No FLC cell lies beyond the fixed-gain Pareto front.**
- **Retraction:** the Phase 04 claim "the FLC strongly helps at 100 us" held only
  against the **limit-cycle default** (kF=aF=1); it does **not** hold against matched
  fixed-gain control. The stable fixed sigmoid at `Ts=50 us` remains best.
- **Type:** experimental result (conditional on the assumed FLC values).

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

---

## Phase 04c — nominal drive (2026-10-29)

## D-037 — Open-loop nominal-drive scenario; matched fixed-gain cell selection

- **Decision:** add `simulation/nominal_drive.py`, a **plant-side** generator that
  drives the plant with the ideal-FOC feedforward
  `u = Rs i_ref + Ld d(i_ref)/dt + E_true`, `i_ref = I_q[-sin θ_e, cos θ_e]`, with `u`
  held over each `Ts`. It may use ground truth; the observer never sees it (no
  controller, no feedback). `I_rated` is **derived** as `T_rated / ((3/2) p ψ_f)`
  (= 9.678 A) under the amplitude-invariant Clarke convention with `i_d = 0`.
- **Matched-cell rule:** the comparison cell is the constant-gain grid cell with the
  smallest steady ripple among Ts = 100 µs cells inside the D-027 bound
  (`kF·aF < 0.597`) -> **kF = 1.0, aF = 0.4**. The 04b reference cell (0.355/0.591) is
  kept alongside.
- **Type:** new scenario + experiment (no existing src behaviour changed).

## D-038 — GATE passed; 04b conclusions hold under the nominal drive

- **Gate:** fixed sigmoid, `Ts = 10 us`, `I_q = I_rated`: E_hat gain 0.9894 (within
  2 %) and phase −0.608° (within 1°) — validates the `u` sign, units and ZOH handling
  in Eq.(4) (no observer code change was needed).
- **Result:** every Phase 04b conclusion is **CONFIRMED** under the nominal drive at
  identical observers and Ts: the matched fixed gain (1.0, 0.4) beats the FLC nominal
  on steady ripple (0.042 vs 1.242 V); 7 fixed-gain cells dominate the FLC nominal at
  Ts = 100 µs (9 at 50 µs); no FLC grid cell dominates the matched cell on all four
  metrics; no FLC cell extends the fixed-gain Pareto front; the FLC sits at
  `kF·aF ≈ 0.212` and avoids the D-027 limit cycle. Versus `u = 0` the nominal drive
  shifts gain by +0.005…+0.011 and steady ripple by ~1 %.
- **Type:** experimental result (conditional on the assumed FLC values).

## D-039 — I_q independence and the limits of the nominal scenario

- **Finding:** steady ripple is essentially independent of `I_q ∈ {2, 5, 9.678} A`
  (matched 0.042 V, FLC ~1.24 V throughout): the observer is driven by `S` and its own
  gains, not the current magnitude. The fixed sigmoid at Ts = 100 µs remains a period-2
  limit cycle (`kF·aF = 1.0 ≥ 0.597`) regardless of `u` or `I_q`.
- **Scope:** open-loop ideal-FOC replica only — no inverter non-idealities, no load
  steps, no field weakening, no DSOGI-FLL, no rotor position/speed estimation.
- **Type:** scope / caveat.

---

## Phase 05a — filter prelude (2026-10-29)

## D-040 — SOGI realization: biquad vs the paper's two-integrator form

- **Decision/finding:** `filters/sogi.py` uses the prewarped-bilinear biquad `D(z)`/
  `Q(z)` (D-019). The paper's Fig.4 (p.826) shows the **two-integrator** SOGI-QSG
  `v̇′ = ω′[k_s(v−v′) − qv′]`, `q̇v′ = ω′ v′`, each integrator discretized with
  Eq.(17). An experiment-only implementation of the latter matches the biquad at
  **constant ω′ to ≤1e-14 gain / ≤1e-11 deg**, but in the **closed loop** (ω′
  time-varying) it gives a **materially different ω′ bias at Ts = 100 µs: +9.468 vs
  +7.912 Hz (Δ = 1.56 Hz, ≈20 %)**, converging with Ts (50 µs Δ 0.19 Hz, 25 µs Δ
  0.03 Hz, stable sigmoid Δ 0).
- **Consequence:** the SOGI *realization* matters when ω′ varies. Adopting the paper's
  Figure-4 structure is **proposed as a separate, approved step**; **no src change**
  is made here. If adopted, D-019's biquad stays as the constant-ω′ equivalent.
- **Phase 05b Step 0a confirmation (rendered page 827):** Eqs.(18)–(19) are printed as
  **discrete-time transfer functions** — ratios of numerator/denominator polynomials in
  `z^-1`/`z^-2` (biquads), introduced by "The discretized transfer functions of digital
  SOGI are given as follows". They are *not* a block diagram or difference equation. So
  the biquad implementation is **kept**; the Fig.4 two-integrator realization remains a
  documented sensitivity (biquad vs two-integrator closed-loop ω′ bias Δ = +1.56 Hz at
  sign 100 µs, 0 for the stable sigmoid).
- **Type:** analysis + proposal.

## D-041 — The −5.9° DSOGI-FLL phase offset is the high-frequency ω′ band

- **Finding (band-split replay, Step 2b):** replaying the SOGI open-loop with the
  recorded closed-loop ω′(t) replaced by its steady-window mean, or mean + the bands
  below ω_e/2 or in [ω_e/2, 3ω_e], all give the *static* filtered phase; only the band
  **above 3ω_e ≈ 200 Hz** reproduces the closed-loop value (sign 100 µs: high band
  −7.17° vs mean +7.45° vs full −7.44°). The full replay reproduces the closed loop
  bit-for-bit.
- **Consequence:** the −5.9° (and its 50/25 µs companions) is caused by the fast
  ω′ jitter/aliasing driven by the sign observer's ripple, not by the low/mid
  frequency content. This **tests and confirms** the Phase 03 relabel of §14.2
  (previously HYPOTHESIS — untested). The stable sigmoid's clean ω′ shows no offset.
- **Type:** verification finding.

## D-042 — Static ε_f(ω′) balance explains the sigmoid but not the sign bias

- **Finding (Step 3):** the zero crossing of `mean ε_f(ω′)` over the window is at
  +0.220 / +0.057 / +0.014 Hz for sign @100/50/25 µs, versus the closed-loop mean
  biases +7.912 / +4.974 / +2.201 Hz (2.8 / 1.1 / 0.6 % of the bias). For the stable
  sigmoid the crossing is at ω_true (bias ≈ 0).
- **Consequence:** the static balance point **explains the sigmoid** (no bias) but
  **not the sign bias** (<20 %); dynamic effects remain, independently confirming
  `phase03.md` §14.5 (also previously HYPOTHESIS — untested). The four-term split
  shows fund×fund is the restoring term and ripple×ripple the small biasing term.
- **Type:** verification finding.

---

## Phase 05b — position/speed estimator (2026-10-29)

## D-043 — Estimator assumptions and discretization

- **Printed (kept):** Eq.(8) `theta_hat_e = -arctan(E_alpha/E_beta)` (implemented as
  `atan2(-E_alpha, E_beta)`) and the Fig.1 Q-PLL block structure. Eqs.(18)-(19) are
  confirmed as discrete-time transfer functions (D-040) → the biquad stays.
- **ASSUMPTIONS — not from paper** (`configs/estimator_assumptions.yaml`): `zeta =
  0.707`, `f_n = 20 Hz` designed at `A = |E|` for 1000 rpm (`Kp = 2 zeta w_n/A`,
  `Ki = w_n^2/A`); normalization OFF; predict-then-correct **forward Euler**; Q-PLL
  error `delta_e = -E_alpha cos(theta_hat) - E_beta sin(theta_hat)`; `theta_hat(0) =
  theta_true(0)`, `omega_hat(0) = theta_true(0) electric frequency`; no phase
  compensation. `omega_m = omega_hat/p` is DERIVED (`p` printed).
- **Type:** implementation choices (assumed).

## D-044 — Estimator angle convention `(-pi, pi]`

- **Decision:** estimator outputs are wrapped to `(-pi, pi]`
  (`wrap_angle_pi_closed`, `-pi` mapped to `+pi`), superseding the earlier `[-pi, pi)`
  note. The plant's `wrap_angle` keeps `[-pi, pi)`; the two differ only on the
  measure-zero boundary. **PROVISIONAL** (the paper prints no wrapping rule).
- **Type:** convention (documented).

## D-045 — Open-loop estimator evaluation (no tuning)

- **Result:** 192 chain runs (3 observers × 3 filters × 2 estimators × 2 Ts × 6
  scenarios) under the nominal drive. FLL status: **0 guard-fail / 0 collapsed** rows
  in this cell set. Nominal cell (matched sigmoid | v′ | QPLL): steady angle error
  mean −0.52° / max 0.73° at 1000 rpm/100 µs (speed max 3.40 rpm), ramp-settled max
  1.14°; lock-in from +90° scales as 1/|E| (0.61 s @100 rpm → 0.030 s @2000 rpm).
- **Angle-error budget** `measured ≈ raw + filter + QPLL-lag` closes within ±0.5° for
  **137/160** steady runs (nominal cells to ±0.001°); the exceptions are all the raw
  **sign** observer (bang-bang wrap glitches). Against the **continuous** linear-lag
  form `-atan(omega Ld/(Rs+K_eff))` the budget does **not** close at Ts = 100 µs
  (up to +2.1°), because of the forward-Euler observer discretization.
- **Type:** experimental result (conditional on the assumed QPLL gains).

## D-046 — No paper reproduction; class A/B split

- **Decision:** run only class-A (kinematic/open-loop) scenarios; class B (load
  disturbance) needs the closed-loop controller + mechanical model and is deferred to
  Phase 06. The paper comparison is setup-limited (open-loop vs closed loop, ideal
  voltage vs inverter, our `E_hat`/assumed PLL bandwidth) and **makes no reproduction
  claim** despite our errors being numerically smaller.
- **Type:** scope / reporting.

---

## Phase 06a — closed-loop drive with encoder feedback (2026-10-29)

## D-047 — Drive gains from the stated design formulas (ASSUMPTION)

- **Decision:** `simulation/drive.py` closes dq current PIs and a mechanical-speed PI on
  the **true** angle/speed (`TrueSource`, encoder baseline). Gains are fixed by design
  formulas before any scenario runs, and every non-printed value is a labelled
  ASSUMPTION in `configs/drive_assumptions.yaml` (null → `ConfigError`):
  `Kp_c = α_c Ld`, `Ki_c = α_c Rs`, `α_c = 2π·500 Hz`; `Kp_w = 2ζα_ω J/Kt`,
  `Ki_w = α_ω² J/Kt`, `Kt = ½·3 p ψ_f`, `ζ = 1`, `α_ω = 2π·50 Hz`; `i_q*` clamp
  `±2 I_rated`; decoupling ON, back-EMF feed-forward OFF; anti-windup = conditional
  integration with the integrator/output clamped at the voltage limit.
- **Printed (kept):** current loop 10 kHz, speed loop 1 kHz, switching 10 kHz, dead
  time 2 µs, `i_d* = 0` (Table III / Fig.7 / Section V).
- **Type:** implementation choices (assumed). **No tuning toward the paper's numbers.**

## D-048 — Average-value inverter with one-sample delay (ASSUMPTION)

- **Decision:** the inverter is an average-value model with `V_dc = 311 V` (hinted by the
  printed back-EMF base 311/√3), phase-voltage limit `V_dc/√3 = 179.56 V`, and a
  **one-sample computation delay** (voltage computed at `k` applied over `[k+1, k+2)`).
  No PWM and no 2 µs dead time (deferred to Phase 06c). The plant is advanced with RK4
  over ≥4 sub-steps per `Ts` and a zero-order-held voltage.
- **Type:** implementation choices (assumed).

## D-049 — Pluggable angle/speed source; `TrueSource` is the only ground-truth path

- **Decision:** the controller receives `(i_αβ, θ_src, ω_src, references)`; the
  `(θ, ω)` pair comes from an `AngleSpeedSource`. The only source implemented is
  `TrueSource`, which reads the plant's true `θ_e/ω_e` (labelled *encoder baseline*). A
  structure test asserts the drive imports no observer/filter/fuzzy/estimation module,
  so Phase 06b can swap in an estimator source without touching the controller.
- **Type:** architectural constraint (extends D-012/D-024).

## D-050 — Metrics from the true state; no conventional-SMO baseline

- **Decision:** the drive metrics are computed from the **true** state (evaluation
  layer); the current loop is checked against the exact discrete ZOH+PI+delay model and
  the speed loop against the designed second-order response.
- **Caveat:** `arctan` on the raw **sign** observer `E_hat` is unusable (bang-bang wrap
  glitches) and **no conventional-SMO baseline exists** (its LPF cutoff is `null`), so no
  "improved vs conventional" claim can be made yet.
- **Type:** metric definition + scope caveat.

## D-051 — GATE passed; open-loop angle-bias model check

- **GATE:** with `TrueSource`, a 0 → 1000 rpm no-load step and a rated-load step at
  1000 rpm give steady speed error < 0.1 rpm, `|i_d| < 3 % I_rated`, `i_q` within 1 % of
  `T_L/Kt`, and no voltage-limit saturation after the transient. **PASSED** (see
  `docs/phase06a.md`).
- **Bias model (Step 0c):** the constant-speed steady angle bias closes to ≲0.09°
  against `−atan(ωLd/(Rs+K_eff)) + ω Ts/2` (half-sample hypothesis **confirmed**: the
  `+ω Ts/2` term cuts the 1000 rpm/100 µs residual from +1.26° to +0.06°). The ramp bias
  closes to ~0.2–0.3° as constant bias + `ω̇_e/ω_n²`. The earlier "budget ±0.001°" is an
  identity at constant speed, not a model check.
- **Loop margins (discrete):** current 493.4 Hz / 63.2° PM; speed 97.0 Hz / 39.9° PM.
- **Type:** experimental result (conditional on the assumed drive values).

---

## Phase 06b — sensorless closed loop (2026-10-30)

## D-052 — Estimator chain composed only in `experiments/chain.py`

- **Decision:** the SMO → DSOGI-FLL `v'` → Q-PLL chain is composed in
  `experiments/chain.py` (`EstimatorSource`) and plugs into the 06a `AngleSpeedSource`
  interface. `src/` modules stay decoupled: the drive imports no observer/filter/
  estimation module, and the chain imports no plant. `drive.py` gains one optional,
  backward-compatible flag (`angle_advance`, default OFF); default behaviour is
  bit-identical to Phase 06a (regression test).
- **Voltage alignment:** the observer consumes the voltage **actually applied over the
  observed interval** (the drive's `DriveSample.u`, i.e. the last command under the
  1-sample delay) with the current sample. **GATE 1 passed**: E_hat gain/phase/ripple in
  the encoder-driven loop match the 04c nominal-drive values for the matched cell
  (Δgain −0.0002, Δphase +0.013°, Δripple −2.8e-5).
- **Type:** architecture (extends D-024/D-049) + verification.

## D-053 — Start-up / handover and angle advance (ASSUMPTION)

- **Decision (ASSUMPTION — not from paper):** encoder-driven start-up with the chain in
  the background; handover when an **estimator-internal** criterion holds
  (`ω̂_m ≥ ω_h`, `|ω̂_m − ω'_m| < tol` for `t_lock`; mechanical rpm; no ground truth).
  FLL initial `ω' = p·ω_h` (nonzero design constant). Frozen values: `ω_h = 500 rpm`,
  `tol = 2 rpm`, `t_lock = 50 ms`. Angle advance `1.5·ω̂·Ts` (computation delay + ½ ZOH),
  default OFF.
- **Printed (kept):** the paper states only that start-up/low speed need another
  strategy (qualitative); no initial position, `ω'`, lock rule or handover is printed.
- **Type:** implementation choices (assumed).

## D-054 — GATE 2 FAILED: the estimated speed destabilises the speed loop

- **Finding:** with the frozen 06a speed loop (`α_ω = 50 Hz`) the sensorless loop loses
  synchronism ~48 ms after handover (peak `|θ̂−θ|` 86°/180°, FLL guard-fail) on the
  rated-load step at 1000 rpm, matched cell, advance off/on. Isolation: the estimated
  **angle** alone is stable (0.52°); the estimated **speed** in the speed loop is the
  cause (true-speed feedback is stable). The loop is stable only for `α_ω ≲ 15 Hz` at
  constant load; a load **step** destabilises even `α_ω = 10 Hz`. Raising the Q-PLL
  `f_n` to 200 Hz does not fix it.
- **Consequence:** per the phase rule the class-B scenario grid and its sensitivity are
  **not run**. A fix (filter/rate-limit `ω̂`; set `α_ω` from the estimator bandwidth;
  review `ω̂`-based decoupling) is a **separate approved step** — the estimator/chain is
  Gate-1 clean, so this is a controller-side problem.
- **Type:** experimental result / blocker.

## D-055 — Phase 05b/06a metric-basis correction (`|ω̂−ω|` is a transient in the window)

- **Finding:** the ~20× constant-vs-ramp speed-error gap in `phase05b.md` is a
  **transient-in-window artefact**: the short constant-speed runs put the "steady"
  window inside the decaying FLL/Q-PLL envelope. A post-transient window gives 0.147 rpm,
  matching the ramp. `docs/phase05b.md` and `docs/phase06a.md` are amended.
- **Also:** the paper's class-B speed/position numbers are **estimator errors**
  `(|ω̂−ω|, |θ̂−θ|)` (a steady 5 rpm cannot be an actual error under an integral speed
  loop), so 06b compares on that basis only. The plant friction `B·\omega_e` is
  consistent with the documented `B·\omega_m` (no fix).
- **Type:** verification / reporting correction.

---

## Phase 06b-2 — estimator-aware speed loop (2026-10-30)

## D-056 — Paper load-step quantities are estimation errors; 06a's α_ω was encoder-only

- **Finding (rendered pages, D-056):** the quoted load-step figures are speed
  **estimation** errors `|ω−ω̂|`: Fig.13 (sim 50 %) has a "Speed error(r/min)" panel and
  the caption says "… estimation errors"; Fig.19 (exp 20 %) plots `N_ref/N_actual/N_err`
  and the text calls ~10 rpm the "estimation error" (the actual dip is a separate
  annotated ΔN ≈ 45 rpm). Load amplitudes confirmed 50 % (sim) / 20 % (exp). No PI
  gains/bandwidth or minimum speed are printed; start-up is qualitative.
- **Process note:** Phase 06a's `α_ω = 50 Hz` was frozen as an encoder-only design, and
  Gate 2 used the **rated** step (harsher than the paper's 20 %/50 %) — a scope mismatch,
  not a tuning error.
- **Type:** paper verification + process note.

## D-057 — Measured estimator response T_est; H1 supported, H2 refuted

- **Decision:** measure `T_est(jω)` on the open-loop nominal drive with a 0.5 % sinusoidal
  speed modulation (f_m 2–200 Hz, 500/1000 rpm, matched cell), chains A (raw E_hat) and B
  (DSOGI v'), Q-PLL `f_n` as experiment overrides. Linear (gain < 0.02 % across
  ε = 0.25/0.5/1 % at 20 Hz).
- **Result:** chain B −3 dB ≈ 83 Hz (1000 rpm) / 41 Hz (500 rpm) at `f_n = 20`; the
  `f_n` widening **saturates** at a speed-dependent cap (≈116 Hz / ≈49 Hz at f_n = 200),
  supporting **H1** (DSOGI envelope). Chain A is **narrower** than B (−3 dB 42 vs 83 Hz),
  **refuting H2**.
- **Type:** measurement.

## D-058 — No feasible α_ω* in the frozen grid → STOP before Step 3

- **Rule (frozen):** `α_ω*` = largest grid value with PM ≥ 45° and GM ≥ 6 dB at both
  speeds, chain B `f_n = 20 Hz`.
- **Result:** **none** qualifies. With `L = L_enc × T_est`, chain B PM is 10.8° at
  500 rpm even for `α_ω = 5 Hz` (1000 rpm: 54.1°); PM falls monotonically with `α_ω`
  (1000 rpm: 14.6° at 10 Hz, −8.2° at 15 Hz). The margin model is **conservative by ~one
  grid step** vs the 06b time-domain boundary (stable ≤ 15 Hz, unstable ≥ 20 Hz) at
  1000 rpm, and much worse at 500 rpm (T_est narrower, envelope ∝ speed).
- **Consequence:** `α_ω* = None`; per the phase rule, **STOP** and report. Steps 3–4 not
  run. No src or config change was made (`git diff --stat phase-06b-gate2-fail -- src
  configs` is empty).
- **Type:** analysis result / blocker.

---

## Phase 06b-3 — operating-point decoupling and low-speed relaxation (2026-10-30)

> Numbering note: the phase brief asked for D-056/D-057, but Phase 06b-2 already used
> D-056–D-058, so the log continues at **D-059/D-060**.

## D-059 — Operating-point decoupled speed-loop bandwidths + low-speed PM relaxation (ASSUMPTION)

- **Decision (ASSUMPTION — not from paper):** the sensorless speed loop uses
  `α_ω = 5 Hz` at 500 rpm and `α_ω = 10 Hz` at 1000 rpm
  (`configs/drive_assumptions.yaml` `speed_loop.alpha_w_hz_at_{500,1000}rpm`, applied
  per scenario as an experiment-level override). The low-speed point accepts the relaxed
  **PM ≥ 10°** because `T_est` is speed-dependent (DSOGI envelope corner ≈ 41 Hz at
  500 rpm vs ≈ 83 Hz at 1000 rpm), the loop remains stable (PM > 0, GM 33 dB), and the
  paper prints no low-speed requirement. The encoder-only 06a value (50 Hz) is unchanged
  for the encoder baseline.
- **Handover threshold lowered 500 → 400 rpm:** at the 500 rpm operating point a
  ζ = 1 loop never overshoots, so `ω̂` cannot exceed a threshold equal to the operating
  speed; ω_h must sit below the lowest operating speed.
- **Step-1 verification:** 500 rpm @ 5 Hz PM 10.8° / GM 33.3 dB (meets the relaxed
  target); 1000 rpm @ 10 Hz PM 14.6° / GM 56.7 dB (stable but **below the 45° target**;
  45° would need 5 Hz → PM 54.1°).
- **Type:** implementation choices (assumed).

## D-060 — Gate 2 passes at steady state and 1000 rpm transients; fails the 500 rpm load step

- **Result (decoupled rule, chain B, f_n = 20 Hz, advance off/on):**
  - **2a** (steady, no-load & 20 % load): **PASS** at 500 rpm (peak 0.4°, steady 0.3°)
    and 1000 rpm (peak 0.7°, steady 0.6°), no saturation.
  - **2b** (20 % load step): **1000 rpm PASS** (peak 7.5°, dip −79.6 rpm, recover 64 ms,
    no sat); **500 rpm FAIL** — FLL guard-fail, peak 71.5°, dip −223.7 rpm.
  - **2c** (50 % / rated step at 1000 rpm): **PASS** (peak 21.7°, dip −210 rpm, recover
    95 ms, no sat).
- **Diagnosis:** the relaxed 10.8° PM at 500 rpm is sufficient for handover/steady state
  but a 20 % load step excites the lightly damped low-speed loop past synchronism. The
  paper's own class-B steps are at **1000 rpm** and pass; the 500 rpm step is this
  phase's extra gate.
- **Consequence:** **GATE 2 = FAIL** → **STOP**; class-B scenario suite / sensitivity not
  run; Phase 06c not started. A fix needs `α_ω < 5 Hz` at 500 rpm (outside the agreed
  grid) or a faster low-speed estimator phase — separate approved changes — or restricting
  the 500 rpm point to steady/no-load operation.
- **Type:** experimental result / blocker.

---

## Phase 06b-4 — softened low-speed bandwidth (2026-10-30)

## D-061 — 500 rpm bandwidth softened 5 → 2.5 Hz; low-speed failure is the FLL, not the loop

- **Decision (ASSUMPTION — not from paper):** `speed_loop.alpha_w_hz_at_500rpm = 2.5`
  (1000 rpm retained at 10 Hz), frozen before the time-domain run. Against the measured
  `L = L_enc × T_est` (chain B, `f_n = 20 Hz`), 500 rpm @ 2.5 Hz gives **PM 65.5°,
  GM 165 dB, crossover 6.6 Hz** (target PM ≥ 30°, up from 10.8° at 5 Hz).
- **Gate result:** the softened loop passes the **steady** 500 rpm gate (no-load peak/steady
  0.3°) and all 1000 rpm regression gates (2a 0.6°, 2b 7.5° / dip −79.6 rpm, 2c 21.7° /
  dip −210 rpm), but the 500 rpm **load transient** still fails: the **unclamped
  DSOGI-FLL loses lock** (`ω′ < 0`) 36 ms after a 20 % step (peak 35.9°, steady 7.3°); a
  start-up-under-load trips it at 60 ms.
- **Diagnosis:** at 500 rpm `|E| = 27.8 V` (vs 55.5 V at 1000 rpm), so the FLL's
  amplitude-driven gain is ~4× smaller and the torque transient drives `ω′` negative —
  an **estimator-amplitude** limit, **not** the speed-loop bandwidth. `min_omega` clamping
  is the next lever (a non-default FLL option, separate approval).
- **Consequence:** **GATE 2 = FAIL** → **STOP**; no scenario suite; Phase 06c not started.
- **Type:** implementation choice (assumed) + experimental result / blocker.

---

## Phase 06b-5 — DSOGI-FLL minimum-frequency clamp (2026-10-30)

## D-062 — `min_omega` clamp removes the guard trip but not the low-speed load failure

- **Decision (ASSUMPTION — not from paper):** enable the FLL lower bound
  `fll.min_omega_rpm = 100` (mechanical → `ω′ₘᵢₙ = p·ω_m = 41.9 rad/s`) via config +
  `chain.build_fll(..., min_omega=...)`. No src change (the FLL already applies
  `ω′ ← max(ω′, min_omega)`).
- **Static check:** clamped ≡ unclamped open-loop chain at 500/1000 rpm (bit-identical);
  the floor never engages in normal operation.
- **Result:** the clamp **removes the `ω′ < 0` guard trip** and fixes **start-up-under-load**
  (Gate 2a passes at both speeds, no-load and 20 % load, peak/steady ≤ 0.7°); 1000 rpm
  2b/2c pass. The **500 rpm 20 % load step still fails** — floor sweep: none → guard trip
  (35.9°); 100 rpm → diverges (180°, sat 0.89); 200 rpm → diverges (180°); 300 rpm →
  recovers without sat but persistent 36° (FLL pinned); 400 rpm → diverges. The clamp
  changes the failure mode, it does not restore low-speed tracking.
- **Consequence:** **GATE 2 = FAIL** → **STOP**; scenario suite / sensitivity not run;
  Phase 06c not started. *(Correction, Phase 06b-6: the earlier claim that all paper
  class-A/B scenarios pass was wrong — those were gate sub-cases, not the scenario suite,
  which had not been run.)* A low-speed *loaded* capability needs a stronger estimator
  (higher Γ / error normalization / amplitude-compensated FLL) — a separate approved
  change.
- **Type:** implementation choice (assumed) + experimental result / blocker.

---

## Phase 06b-6 — re-scope to the paper's scenarios; sensorless suite (2026-10-30)

## D-063 — Gate P PASS on the paper scenarios; suite run; no reproduction claim

- **Re-scope:** the paper prints values only for sim 50 % @1000, exp 20 % @1000, the
  no-load 500→1000 ramp and Table IV steady @1000 (`estimator_spec.md` §06b-6 Step 0d).
  The 500 rpm **loaded** steps were our own gates; they move to a stress table, not gates.
- **Gate P (chain B, matched cell, clamp ON, advance off; on reported): PASS** on all four
  — S1 ramp peak 2.2° / steady 0.61°; S2 sim 50 % peak 21.7° / steady 0.56°; S3 exp 20 %
  peak 7.5° / steady 0.59°; S4 steady 0.7° / 0.61°. No saturation, no guard trip, advance
  on/off equivalent. **Clamp ablation:** identical with clamp OFF (the floor never engages
  on paper scenarios).
- **S1 bandwidth:** uses `α_ω = 2.5 Hz` (the 500 rpm value) — a fixed bandwidth for a
  500→1000 ramp must be safe at the low end (10 Hz is marginal at 500 rpm and diverges
  with advance off).
- **Suite:** all paper scenarios pass; sensorless cost ≈ 0 dip/recovery vs the encoder
  baseline at the same `α_ω` (the dip is set by `α_ω`). Steady estimation errors are far
  below the paper's (0.6° vs 4°, 0.14 vs 5 rpm), but **dynamic** transient estimation
  errors during load steps are **larger** (111.7 vs 23 rpm; 40.4 vs 10 rpm) because our
  speed loop is slow. **No reproduction claim** — every setup differs (encoder start-up,
  ideal voltage, assumed gains, slow loop).
- **Type:** experimental result (conditional on the frozen assumptions).

## D-064 — Documented limitation: low-speed / high-load steps lose synchronism

- **Stress table (beyond paper, informational):** 500 rpm 20 % and 50 % steps → synchronism
  loss (peak 180°, saturation 1.0); 1000 rpm **rated (100 %)** step → synchronism loss
  (peak 180°); 500 rpm start-up under 20 % load → **PASS** (0.4°). The paper's 1000 rpm
  **50 %** step passes; 100 % does not, so the load-step limit is between 50 % and 100 %
  at 1000 rpm.
- **Root cause:** the DSOGI-FLL's amplitude-driven gain is too weak at low `|E|` (and the
  transient is too large at 100 % load); neither a bandwidth change (06b-4) nor a
  frequency floor (06b-5) fixes it.
- **Consequence:** accepted as a documented limitation of this composition; a fix needs a
  stronger estimator (higher Γ / error normalization / amplitude-compensated FLL) or a
  load-rate limit — separate approved changes. Gates are not re-opened.
- **Type:** experimental result / documented limitation.

## D-065 — 06b-5 "all paper scenarios pass" correction

- **Correction:** the Phase 06b-5 claim that all paper class-A/B scenarios passed was wrong
  — those were **gate sub-cases**, not the paper's scenario suite, which had not run. The
  suite runs in 06b-6 (D-063). Corrected in `estimator_spec.md`, `phase06b.md`, and D-062.
- **Type:** reporting correction.

---

## Phase 06d — estimation-bandwidth diagnosis (2026-10-30)

## D-066 — The DSOGI-FLL limits the closed-loop bandwidth (oracle vs nominal)

- **Method:** experiment-local arms — 0 nominal (v′→Q-PLL), 1 raw `E_hat` (DIAGNOSTIC),
  2 oracle `ω′` (DSOGI with prescribed true `ω′`, FLL bypassed; DIAGNOSTIC), 3 normalised
  FLL (DIAGNOSTIC; ASSUMPTION form). Open-loop T_est at 250/500/1000 rpm and a closed-loop
  `α_ω × f_n` grid on S2/S3. No src change.
- **T_est −3 dB (250/500/1000):** arm0 26/42/84 Hz; arm1 17/26/42; arm2 27/44/69;
  arm3 15/24/37. The nominal chain is the widest; amplitude dependence is strong.
- **Grid (valid f_n ≥ 20):** max stable `α_ω` = **10 Hz (arm0)** vs **20 Hz (oracle arm2)**;
  best valid dynamic `|ω̂−ω|` = **112/40 rpm (arm0)** vs **56/22 rpm (oracle)**. Raw-`E_hat`
  arm is unstable everywhere; the normalised FLL narrows the loop (9/20 stable cells).
- **Verdict:** the **DSOGI-FLL** is the limiting element (oracle doubles the stable `α_ω`
  and halves the transient error), then the Q-PLL (`f_n=20`) and the sample delay. The
  normalisation is **ASSUMPTION** (Fig.5 prints only a labelled block) and as implemented
  narrows the loop — not adopted.
- **Paper-implied `α_ω` ≈ 38–43 Hz** (estimate from our error/dip ratios 0.53/0.51 × the
  06a encoder dips), out of reach of the current chain at any valid cell.
- **Type:** diagnostic result; recommendation only (a stronger FLL is a separate step).

## D-067 — f_n × 0.5 sensitivity row invalid (handover after the step); cost corrected

- **Finding:** at Q-PLL `f_n = 10 Hz` the lock criterion is satisfied later (handover
  0.70 s), after the 0.5 s load step, so the step is encoder-driven (recorded errors exactly
  0; the 0.6° peak is the steady error). `f_n = 5 Hz` never hands over. Only `f_n ≥ 20 Hz`
  (handover ≈ 0.20 s) measures a sensorless transient. The `phase06b.md` sensitivity row is
  corrected; the grid's `f_n ∈ {5,10}` columns are flagged invalid.
- **Sensorless cost** corrected from "≈0" to two numbers: at equal `α_ω=10` Hz the dip cost
  is −69.6/−23.4 rpm (S2/S3); at each config's own design `α_ω` it is −172.8/−64.7 rpm.
- **Type:** reporting correction (+ methodological caveat).

## D-068 — Stress failures are speed dips into the weak-FLL region

- **Finding:** the failing 1000 rpm rated step and 500 rpm loaded steps reach negative
  minimum speeds (−1444 / −3474 rpm; −400…−650 rpm), i.e. the true speed collapses through
  zero into the weak-FLL region before synchronism is lost — a control/estimator collapse,
  not a loop-margin limit. At each arm's best valid cell the 1000 rpm rated step passes
  (peak 0.6°); the 500 rpm steps still fail.
- **Type:** diagnostic result / documented limitation (extends D-064).

---

## Phase 06c — paper-fidelity items and headline comparisons (2026-10-30)

## D-069 — Average-value dead-time model (default OFF); Gate D explained

- **Decision:** add optional dead-time flags to `run_drive` (default OFF):
  per phase `ΔV = −V_dc t_d f_sw sat(i_x/i_th)`, `V_dc=311 V`, `t_d=2 µs`, `f_sw=10 kHz`
  (printed), `i_th=0.1 A` (ASSUMPTION, `configs/drive_assumptions.yaml`); the observer
  receives the **commanded** voltage. Six unit tests (`tests/test_dead_time.py`) and a
  bit-identical default regression.
- **Gate D:** ΔV magnitude matches (α-β vector fundamental **7.76 V** vs 7.92 predicted),
  but the E_hat fundamental error is **≈0.6×** the prediction (+14.8 % vs 28.5 % at
  500 rpm/I_rated; +6.5 % vs 14.3 % at 1000 rpm). Explained: the SMO transmits ≈0.6 of
  the disturbance to its E_hat fundamental, the residual is partly **quadrature** (+4°),
  and at 20 % load the distortion is erratic near `i_th`. **Dead time degrades the loop
  severely** when ON (S2 collapses, S4 steady error 16.5° at no load).
- **Type:** implementation (assumed threshold) + verification.

## D-070 — Conventional baseline cannot lock; normalization ablation not lockable

- **Conventional chain ("our construction"):** fixed-sign SMO → LPF (cutoff UNPRINTED;
  grid 100/200/400 Hz × phase comp off/on) → Q-PLL (f_n=20), arctan variant. **All 8 chains
  fail the generic 5° handover criterion** in every scenario: the Q-PLL reaches ≈1020 rpm
  but the filtered bang-bang `E_hat` keeps `|Δe|/|Ê|` at ≈0.06–0.09 ≈ sin 5°, so the dwell
  is never met — a genuine ripple limitation, not a bug.
- **Normalization ablation:** Q-PLL normalization ON → **0/36** cells meet the 06b-6
  FLL-consistency lock criterion; no usable comparison. Not adopted (algebra ASSUMPTION).
- **Headline:** only the matched and FLC nominal chains hand over; steadier errors ~0.6° /
  ~1.9°; dynamic transients 111.7 / 40.4 rpm vs the paper's 23 / 10 — **no reproduction
  claim**.
- **Arm 1 (raw `E_hat` → Q-PLL):** with the generic criterion it *does* hand over
  (0.06–0.09 s, peaks 1.6–52°); the 06d "0/20 unstable" was a **handover artefact**
  (FLL-based criterion with no FLL). Corrected.
- **Type:** experimental result / documented limitation.
