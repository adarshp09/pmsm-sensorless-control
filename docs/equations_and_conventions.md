# Equations and Conventions

This file fixes the notation and sign/angle conventions used throughout the
implementation so that plant, observer, filter and estimator code speak the same
language.

> The paper uses alpha-beta quantities but does not specify a Clarke matrix,
> zero sequence, axis alignment, wrapping, or electrical/mechanical conversion.
> Paper equations are recorded below; local conventions remain provisional.

Each convention carries a status: `confirmed (paper)` or `PROVISIONAL`.

## 1. Angles and speeds

| Quantity | Symbol | Convention | Status |
| --- | --- | --- | --- |
| Mechanical angle | `theta_m` | radians, continuous (unwrapped internally) | PROVISIONAL |
| Electrical angle | `theta_e` | radians | PROVISIONAL |
| Mechanical speed | `omega_m` | rad/s | PROVISIONAL |
| Electrical speed | `omega_e` | rad/s | PROVISIONAL |

- The configuration reports 4 pole pairs (Table III, p.6), but the paper does
  not explicitly print the electrical/mechanical conversion equations.
- The paper may report speed in `rpm` or `rad/s`, and electrical or mechanical.
  All internal code uses **electrical rad/s** unless a name ends in `_m`.
  Conversions live in one place (planned `pmsm_sensorless.estimation`), not
  scattered through call sites.

## 2. Alpha-beta axis orientation

- `alpha` is aligned with phase-`a` (`x_alpha = x_a` at `theta_e = 0`):
  PROVISIONAL.
- `beta` leads `alpha` by +90 degrees for a positive-sequence, counter-clockwise
  field: PROVISIONAL.
- Clarke transform convention (amplitude-invariant vs. power-invariant) is
  **unresolved** — this changes every downstream gain and must be taken from the
  paper, not assumed. See `paper_method.md` item 3.

## 3. Signs

| Quantity | Convention | Status |
| --- | --- | --- |
| Stator current | positive into the machine terminals (motor convention) | PROVISIONAL |
| Stator voltage | measured phase-to-neutral (confirm vs. line/phase) | PROVISIONAL |
| Load torque | positive opposing positive rotation | PROVISIONAL |
| Back-EMF polarity | sign relation between `e_alpha/beta` and `theta_e` | **unresolved** |
| Rotation direction | positive = counter-clockwise, electrical | PROVISIONAL |

Back-EMF polarity is the single most error-prone sign in this project: a flipped
sign gives a position estimate that is mirror-imaged and looks "almost right".
It must be pinned down from the paper's equations (item 4) and validated with a
unit test on a known synthetic signal.

## 4. Angle wrapping and error computation

- All angles are wrapped to `(-pi, pi]` before use in error terms:
  `wrap(x) = ((x + pi) mod 2*pi) - pi`.
- Angular error is always computed as a wrapped difference,
  `err = wrap(theta_hat - theta)`, never as a raw subtraction.
- `atan2` (four-quadrant) is used for position extraction, not `atan`.
- Status: PROVISIONAL — confirm the paper's convention (some use `[0, 2*pi)`).

## 5. Continuous-time vs. discrete-time

- Continuous-time equations are written in Markdown/LaTeX and in docstrings.
- Discrete-time implementation is separated from the continuous formulation;
  the paper specifies Tustin with prewarping (Eqs. 17-19, p.5).
- `Ts=0.0001 s` is derived from the paper's 10 kHz sampling frequency and
  confirmed by the `Ts=100 us` example in Eq.(20), p.5.

## 6. Signal naming conventions

| Suffix / pattern | Meaning |
| --- | --- |
| `x_abc`, `x_alpha`, `x_beta`, `x_d`, `x_q` | frame |
| `x_hat` | *estimated* quantity (observer output) |
| `x_ref` | reference / commanded |
| `x_meas` | directly measured |
| `x_filt` | filtered (e.g. DSOGI output) |
| `x_err` | error signal |
| `x_m`, `x_e` | mechanical / electrical |

Prefixes for the two DSOGI channels and the FLL internal signals must follow the
paper's notation; the mapping from the paper's symbols to these Python names
will be added here once extraction is complete.

## 7. Paper-verified equations and remaining open items

- PMSM model: Eq.(1), p.2.
- Back EMF: `E = omega_e*psi_f[-sin(theta_e), cos(theta_e)]^T`, Eq.(2), p.2.
- Position: `theta_hat_e=-arctan(E_hat_alpha/E_hat_beta)`, Eq.(8), p.3.
- Do not silently replace the paper's one-argument `arctan` with `atan2`.

- [ ] Clarke transform scaling (amplitude vs. power invariant)
- [ ] Back-EMF sign convention and its relation to `theta_e`
- [ ] Angle range convention `(-pi, pi]` vs `[0, 2*pi)`
- [ ] Voltage reference (phase-to-neutral vs. line-to-line)
- [x] Discretization method and whether Tustin prewarping is used
- [ ] Symbol-to-variable name map for SMO / FLC / DSOGI / FLL

## 8. Phase 01 implementation notes (2026-10-14)

These record what Phase 01 actually implemented; where the paper is silent the
choice is named PROVISIONAL and cross-referenced to
`docs/implementation_decisions.md`.

- **Back-EMF** is the paper's Eq.(2) verbatim:
  `[E_alpha, E_beta] = omega_e*psi_f*[-sin(theta_e), cos(theta_e)]`
  (`plant.pmsm.PMSMPlant.back_emf`). No atan2/sign reinterpretation is added.
- **Electrical dynamics** are the paper's Eq.(1) with `Ld = Lq`; the plant uses
  `Ld` on both axes exactly as printed (`plant.pmsm.PMSMPlant.current_derivative`).
- **Angle wrap** is implemented as the documented formula
  `((x + pi) mod 2*pi) - pi`, which returns values in `[-pi, pi)` (the half-open
  boundary is a PROVISIONAL local choice; it differs from the stated `(-pi, pi]`
  only on a measure-zero set).
- **Clarke scaling** is an explicit, configurable argument with default
  amplitude-invariant in `transforms.clarke` (D-010). The `alpha` axis is aligned
  with phase `a` and `beta` leads by `+90 deg`; this is consistent with Eq.(2)
  but is itself PROVISIONAL.
- **Plant solver** is classical RK4 for both the electrical and the optional
  mechanical states (D-009).
- **Mechanical model** is an opt-in, derived torque balance, *not* printed in the
  paper (D-011); it is off by default and requires an explicit friction value.

## 9. Phase 02 observer conventions (2026-10-14)

- **Signals:** the observer consumes only measured `u_alpha/beta` [V] and
  `i_alpha/beta` [A]; it never reads the plant's true `theta_e`, `omega_e` or
  back-EMF (D-014).
- **Sliding surface:** `S = i_hat - i` (Eq.(3), p.2). `S` has the units of
  current; a positive `S` yields a negative injection (`-k*g(S)` in Eq.(4)).
- **Switching:** `g = sign` (conventional, Eq.(4)) or `g = 2/(1+exp(-a*S)) - 1 =
  tanh(a*S/2)` (Eq.(5)). `sign(0)=0` and `g(0)=0`.
- **Back-EMF estimate:** `E_hat = k*g(S)` (Eq.(7), p.2), with `S` the *sample-instant*
  error `i_hat - i` -- i.e. the injection actually applied in Eq.(4), reported with no
  low-pass filter (D-017, D-025). It is **not** recomputed from the post-update
  estimate.
- **Discretization:** explicit forward Euler at `Ts = 100 us`, zero-order-held
  measurement (D-016).
- **Units:** SI (`k` [V], `a` [1/A]); the paper's per-unit `k`/`a` are converted
  in `SMOObserverParameters.from_config` (D-015).

## 10. Phase 04 FLC conventions (2026-10-14)

- **Inputs:** normalised `s_n = Ke·S`, `s_dot_n = Kec·(S[k]-S[k-1])/dt` (first step
  `s_dot_n = 0`), clipped to `[-1, 1]`. `S = i_hat - i` is the observer's own current
  error; no plant/ground-truth signal enters the FLC (D-032).
- **Membership:** triangular Ruspini partition on the printed centres
  `{-1,-0.5,0,0.5,1}` (half-width 0.5); output centres `{0,1/3,2/3,1}` on `[0,1]`
  (half-width 1/3). The feet/centres are **ASSUMPTION — not from paper**.
- **Inference/defuzzification:** Mamdani with AND = min, aggregation = max, centre
  of gravity on a 1001-point grid (print: Mamdani + CoG; operators/grid are assumed).
- **Rule bases:** the printed 5x5 tables (Tables I-II, p.4); rows = `s`, columns =
  `s_dot`; rule strength = `min(mu_s, mu_sdot)`.
- **Adaptation:** `k_t = kF·k0`, `a_t = aF·a0` (printed mapping; `k0`, `a0` from
  D-015). The reported back-EMF is `k_t·g(S_pre; a_t)` (D-025).
- **Scope:** raw `E_hat` only; no DSOGI-FLL, no position/speed (D-033).
