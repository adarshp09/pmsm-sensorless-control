# Paper Method — Specification Extraction

Source: `docs/act.pdf`, X. Zhu et al., “An Improved Sliding Model Observer
Sensorless Control for PMSM Based on Fuzzy Logic Controller and DSOGI-FLL,”
IEEE TTE, vol. 11, no. 1, pp. 823–834, 2025, DOI 10.1109/TTE.2024.3396732.
The PDF is readable and all 12 pages were text-extracted and rendered for
equation/table checks.

This document records the specification extracted *from the paper*.
Phase 00. Each row is an extraction target with the source location that must
be cited (page, equation number, figure or table). A row is only marked
`verified` when the value/equation has been copied from the source and the
citation recorded.

## Extraction status summary

| Item | Description | Status |
| --- | --- | --- |
| 1 | PMSM mathematical model and assumptions | verified |
| 2 | Reported motor parameters | verified; some omitted |
| 3 | Clarke transform / alpha-beta conventions | unresolved |
| 4 | Back-EMF equations and relation to rotor position/speed | partly verified |
| 5 | Conventional sliding-mode observer equations | verified |
| 6 | Sliding surface, switching term, sigmoid function | verified |
| 7 | Fuzzy controller (inputs, MFs, rules, inference, defuzz, output) | partly verified |
| 8 | DSOGI transfer functions and signal combinations | partly verified |
| 9 | FLL error signal, adaptation law, gains, limits | partly verified |
| 10 | Discretization method, sampling period, Tustin prewarping | verified |
| 11 | Rotor position and speed estimation equations | partly verified |
| 12 | Simulation/experimental conditions, baselines, reported results | partly verified |

## 1. PMSM mathematical model and assumptions

The stationary two-axis model is Eq. (1), p.2:
`d[i_alpha,i_beta]^T/dt = (-Rs/Ld) i + (1/Ld) u - (1/Ld) E`.
The paper ignores cross-coupling magnetic saturation, iron loss, and magnet
eddy-current loss (p.2). It reports `Ld=Lq=1.47 mH` but does not identify a
surface/interior topology. Parameters are transcribed in
[`parameter_extraction.md`](parameter_extraction.md).

## 2. Reported motor parameters

Do not insert generic values. Each parameter must cite the paper's parameter
table. Full table in [`parameter_extraction.md`](parameter_extraction.md).

- **Source (page/table):** _unresolved_

## 3. Clarke transform and alpha-beta axis conventions

- Exact Clarke matrix used (amplitude-invariant vs. power-invariant):
- Whether the zero sequence is included:
- alpha aligned with phase-a? rotation direction? sign of beta?
- The paper uses alpha-beta quantities but gives no Clarke matrix, zero-sequence
   treatment, axis alignment, or rotation/sign convention. These remain
   unresolved; do not infer amplitude-invariant or power-invariant scaling.
- **Source:** Eq.(1), p.2; no transform equation reported.

## 4. Back-EMF equations and relationship to rotor position/speed

- Back-EMF expressions in alpha-beta:
- Mapping from `(e_alpha, e_beta)` to electrical angle and speed (atan2 form,
  sign, division by flux linkage):
- Relationship between electrical and mechanical angle/speed (pole pairs):
- Eq. (2), p.2, gives `[E_alpha,E_beta]^T = omega_e*psi_f[-sin(theta_e), cos(theta_e)]^T`.
   Eq. (8), p.3, estimates `theta_hat_e = -arctan(E_hat_alpha/E_hat_beta)`.
   The paper does not state an atan2 replacement, angle wrapping rule, or a
   speed-estimation equation. It also does not explicitly state the
   electrical/mechanical pole-pair conversion.

## 5. Conventional sliding-mode observer equations

- Current-observer dynamics:
- Estimated back-EMF extraction (filter/equation):
- Gain definitions and how they appear:
- Eq. (4), p.2, copies Eq. (1) with estimated current and adds
   `-(k/Ld)[sign(S_alpha), sign(S_beta)]^T`. When the error converges, Eq. (7)
   estimates back EMF as `E_hat = k[ sigmoid(S_alpha), sigmoid(S_beta) ]^T`.

## 6. Sliding surface, switching term and sigmoid function

- Sliding surface `s` definition:
- Switching function: sign / saturation / sigmoid — exact form written in the
  paper:
- Sigmoid parameters (slope, boundary layer) and their values:
- `S(x)=i_hat_s-i_s` (Eq. 3, p.2). The sigmoid is
   `2/(1+exp(-a*S))-1` (Eq. 5, p.2), with positive slope parameter `a`.
   Simulation values are `k=1`, `a=4` (p.5). No boundary-layer parameter is
   reported.

## 7. Fuzzy controller

- Inputs (e.g. error and error-derivative) and their definitions:
- Normalization / scaling gains (Ke, Kec, Ku) and input/output universes:
- Membership functions: type (triangular/trapezoidal/gaussian), ranges,
  breakpoints, number of sets per variable:
- Rule table (verbatim matrix): 
- Inference method (Mamdani/Larsen, min/max) and defuzzification (centroid/…):
- Output variable and its range; how the output acts on the observer:
- Inputs are `s` and `s_dot`; input labels are NB, NS, Z, PS, PB over
   `{-1,-0.5,0,0.5,1}`. Output labels are Z, PS, PM, PB over `[0,1]`.
   Membership functions are triangular (Fig.2, p.3), but exact breakpoints and
   scaling gains are not reported. The complete `aF` and `kF` rule matrices are
   recorded in `configs/observer.yaml` from Tables I-II, p.4. Inference is
   Mamdani and defuzzification is center of gravity (p.3). `kF` and `aF`
   multiply the fixed `k` and `a`, respectively (Fig.3, p.4).

## 8. DSOGI transfer functions and signal combination

- SOGI direct/quadrature transfer functions (continuous- and discrete-time):
- DSOGI structure (two SOGIs, positive/negative sequence extraction?):
- Signal-combination equations producing the filtered back-EMF:
- `D(s)=k_s omega' s/(s^2+k_s omega' s+omega'^2)` and
   `Q(s)=k_s omega'^2/(s^2+k_s omega' s+omega'^2)` (Eqs. 9-10, p.4).
   `D` is band-pass and `Q` is low-pass; `qv_o` lags `v_o` by 90 degrees.
   The paper describes two SOGIs sharing one FLL for the alpha/beta EMFs, but
   does not print explicit alpha/beta sequence-combination equations.

## 9. FLL

- Frequency-error signal equation:
- Adaptation law (integral/controller form) and gains:
- Initialization and frequency saturation limits:
- The block diagram (Fig.5, p.4) shows gain normalization and an integrator
   with negative gain `-Gamma`; text sets `Gamma=50` and `k_s=sqrt(2)` (p.5).
   No standalone algebraic FLL error equation, initialization, or saturation
   limits are reported. `Eqs. (11)-(16)` analyze DC, fundamental, and harmonic
   filtering but are not an implementation law for the FLL.

## 10. Discretization

- Method (forward Euler / Tustin / trapezoidal) and justification:
- Sampling period / switching period:
- Any prewarping of the Tustin substitution (and how `omega` is injected):
- Tustin with prewarping is used. Eq. (17), p.5, substitutes
   `tan(omega' Ts/2)/omega' * (1+z^-1)/(1-z^-1)`. The digital `D(z)` and
   `Q(z)` are Eqs. (18)-(19). `Ts=100 us` and `omega'=100*pi` give `f'=50 Hz`
   in Eq. (20); the 10 kHz sampling frequency in Table III independently gives
   the same derived period.

## 11. Rotor position and speed estimation equations

- Position estimate equation (incl. any compensation/offset term):
- Speed estimate equation (frequency to speed conversion, pole pairs):
- Angle wrapping / normalization convention:
- Position is Eq. (8), p.3, through the negative one-argument arctangent.
   The block diagram includes a normalized Q-PLL (Fig.1, p.3), but its equations
   and the speed conversion are not printed. Electrical versus mechanical speed
   and pole-pair conversion therefore remain unresolved in the source.

## 12. Simulation and experimental conditions

- Simulation tool, timestep, duration, solver:
- Speed profiles, load-torque steps, injected noise:
- Baseline methods compared:
- Reported quantitative results (figures/tables) — recorded for reference only,
  never fabricated:
- Table III, p.6, reports: 2 kW, 220 V line, 2000 rpm, `J=0.00153 kg.m^2`,
  7.7 N.m load, 4 pole pairs, `Ld=Lq=1.47 mH`, `Rs=0.505 ohm`,
  `psi_f=0.1326 Wb`, 10 kHz sampling/current-loop frequency, and 1 kHz
  speed-loop frequency. The paper compares STASMO-CVGI, SMO+LPF and
  FSMO+DSOGI-FLL. Scenarios and reported numerical errors are in
  `configs/experiments.yaml`, with citations to pp.6-10. No solver, injected
  noise, or complete simulation duration is specified.

## Traceability rule

Preserve the PDF page and equation, figure, or table citation when refining any
unresolved item. Where the paper is incomplete, record the gap rather than
resolving it silently.

## Flagged ambiguities and inconsistencies

- Eq. (8) uses a one-argument `arctan`, not an explicit `atan2`, and the paper
   provides no angle wrapping or speed-conversion equation.
- FLC label universes are given, but numeric triangular breakpoints and scaling
   gains are not reported.
- The FLL block and `-Gamma` integrator are shown, but error algebra,
   initialization, and frequency limits are not printed.
- The Clarke matrix, DSOGI alpha/beta combination equations, solver/timestep,
   and injected noise are not reported.
