# Parameter Extraction Table

> **Source available:** `docs/act.pdf` is the 12-page source paper. Numeric
> values are copied or explicitly derived from it; omitted values remain null.
>
> Every row below is a required parameter. The `value`, `unit` and `source`
> columns stay empty until the value is read from the paper. **Do not fill
> these with textbook defaults.** A blank row is a truthful statement that the
> value is unknown; a plausible-looking guess is a silent correctness bug.

## Status legend

- `unresolved` — not yet extracted; value must come from the paper.
- `derived` — computed from other (resolved) parameters; record the formula and
  the reference it is derived from.
- `assumed` — not stated by the paper; an explicit implementation choice, which
  must also be logged in `implementation_decisions.md`.
- `verified` — copied from the paper with a recorded citation.

The verified motor values are in the YAML and come from Table III, p.6.

## Motor parameters (`configs/pmsm.yaml`)

Verified Table III values: `p=4`, `Rs=0.505 ohm`, `Ld=Lq=1.47 mH`,
`psi_f=0.1326 Wb`, `J=0.00153 kg.m^2`, rated line voltage `220 V`, rated speed
`2000 rpm`, load `7.7 N.m`, power `2 kW`, and sampling/switching frequency
`10 kHz` (PDF p.6, printed p.828). Rated current, DC bus voltage, viscous and
static friction, and rated electrical frequency are not reported.

| Parameter | Symbol | Value | Unit | Source (page / eq / table) | Status |
| --- | --- | --- | --- | --- | --- |
| pole pairs | p | 4 | - | Table III, p.6 | verified |
| stator resistance | Rs | 0.505 | ohm | Table III, p.6 | verified |
| d-axis inductance | Ld | 0.00147 | H | Table III, p.6; derived from 1.47 mH | derived |
| q-axis inductance | Lq | 0.00147 | H | Table III, p.6; derived from 1.47 mH | derived |
| PM flux linkage | psi_f | 0.1326 | Wb | Table III, p.6 | verified |
| rotor inertia | J | 0.00153 | kg.m^2 | Table III, p.6 | verified |
| viscous friction | B |  | N.m.s/rad |  | unresolved |
| DC bus voltage | Udc |  | V |  | unresolved |
| rated current | Ir |  | A |  | unresolved |
| rated speed | nr | 2000 | rpm | Table III, p.6 | verified |
| switching frequency | fsw | 10000 | Hz | Table III, p.6; derived from 10 kHz | derived |

## Observer / FLC / DSOGI-FLL parameters (`configs/observer.yaml`)

Paper-printed values are resolved and cited below; the FLC scaling gains are
ASSUMPTION values in `configs/fuzzy_assumptions.yaml`. The full printed-vs-unprinted
breakdown (SI ranges, stability/tracking constraints under D-015) is in
[`flc_spec.md`](flc_spec.md).

| Parameter | Symbol | Value | Unit | Source (page / eq / table) | Status |
| --- | --- | --- | --- | --- | --- |
| SMO sliding-surface gain |  |  | - |  | unresolved |
| SMO switching gain | k | 1 (per-unit; 179.629 V in SI, D-015) | - | p.5 | verified |
| sigmoid slope | a | 4 (per-unit; 0.5389 1/A in SI, D-015) | - | p.5 | verified |
| boundary layer | delta |  | - |  | unresolved |
| FLC input scale (error) | Ke | 0.13472 (= 1/I_base) | 1/A | `configs/fuzzy_assumptions.yaml` | **assumed** |
| FLC input scale (delta error) | Kec | 1.3472e-5 (= Ts/I_base) | s/A | `configs/fuzzy_assumptions.yaml` | **assumed** |
| FLC output scale | Ku | 1.0 | - | `configs/fuzzy_assumptions.yaml` | **assumed** |
| DSOGI damping gain | k_s | sqrt(2) | - | p.5 | verified |
| FLL gain | Gamma | 50 | - | p.5 | verified |
| FLL min/max frequency |  |  | Hz |  | unresolved |
| sampling period | Ts | 0.0001 | s | Eq.(20), p.5; Table III | verified |

## Simulation conditions (`configs/experiments.yaml`)

| Parameter | Symbol | Value | Unit | Source (page / fig) | Status |
| --- | --- | --- | --- | --- | --- |
| simulation timestep | Ts | 0.0001 | s | Eq.(20), p.5; SOGI sample period | derived |
| simulation duration | T |  | s |  | unresolved |
| load torque step |  |  | N.m |  | unresolved |
| speed reference profile |  |  | rpm |  | unresolved |
| current/voltage noise sigma |  |  | A / V |  | unresolved |

## Verified structural specifications

These are not single numbers and cannot be captured as table cells; they live in
`configs/observer.yaml` under `specs:` and in `docs/paper_method.md`.

- [x] sliding surface equation, Eq.(3), p.2
- [x] switching function / sigmoid form, Eqs.(4)-(5), p.2
- [x] fuzzy inputs, outputs, rule tables, inference and defuzzification, pp.3-4
- [x] DSOGI transfer functions, Eqs.(9)-(10), p.4
- [ ] DSOGI signal-combination equations (not printed)
- [ ] FLL error signal (only Fig.5 block notation is shown)
- [x] FLL adaptation description and Gamma=50, p.5
- [x] Tustin prewarping, Eqs.(17)-(20), p.5

## Unit conversions and derived values

- `1.47 mH = 0.00147 H`; `2 kW = 2000 W`; `10 kHz = 10000 Hz`.
- `Ts=1/10000=0.0001 s`; `k_s=sqrt(2)` is stored numerically in YAML.

## Rule for filling this table

1. Read the value directly from the paper; never estimate.
2. Record the exact page and equation/table/figure number.
3. If the paper omits the value, leave the row `unresolved` and add an entry to
   the "Flagged ambiguities" section of `docs/paper_method.md`. If an
   implementation subsequently chooses a value, mark it `assumed` and log the
   choice and rationale in `docs/implementation_decisions.md`.

## DSOGI-FLL status (Phase 03)

| Item | Symbol | Value | Unit | Source | Status |
| --- | --- | --- | --- | --- | --- |
| SOGI damping gain | `k_s` | `sqrt(2)` | - | p.5 | verified |
| FLL gain | `Gamma` | 50 | - | p.5 | verified |
| SOGI/sample period | `Ts` | 0.0001 | s | Eq.(20), p.5; Table III | verified |
| FLL initial `w'` | `w'_0` |  | rad/s |  | unresolved |
| FLL frequency limits |  |  | rad/s |  | unresolved |
| FLL error signal algebra | `epsilon_f` |  |  | only Fig.5 | unresolved (derived in code) |
| Gain-normalization algebra |  |  |  | only Fig.5 | unresolved (option, off by default) |
| DSOGI signal combination |  |  |  | not printed | unresolved (derived in code) |

Notes:

- The paper **does** state a step/sample time: `Ts = 100 us` (Eq.(20), p.5) and
  10 kHz (Table III, p.6).
- `k_s = sqrt(2)` and `Gamma = 50` are resolved and cited; they stay in
  `configs/observer.yaml` as `dsogi.damping_gain_k` and `fll.gamma_gain`.
- `fll.initial_frequency`, `dsogi.nominal_center_frequency`, `fll.min_frequency`
  and `fll.max_frequency` remain `null`; `DSOGIFLLParameters.from_config` raises a
  clear error if `w'_0` is needed and unresolved.
- The printed `D(z)`/`Q(z)` coefficients in Eqs.(18)-(19) are inconsistent with
  Eq.(17); the implementation uses the prewarped-bilinear result (D-019).

## FLC status (Phase 03e spec sheet, `docs/flc_spec.md`)

| Item | Value | Unit | Source | Status |
| --- | --- | --- | --- | --- |
| FLC input scale (error) `Ke` | 0.13472 | 1/A | `configs/fuzzy_assumptions.yaml` | **assumed** (not printed) |
| FLC input scale (error derivative) `Kec` | 1.3472e-5 | s/A | `configs/fuzzy_assumptions.yaml` | **assumed** (not printed) |
| Input per-unit base for `s` | 7.4227 | A | D-015 derivation | derived |
| Sample time for `s_dot` | 0.0001 | s | control `Ts` (assumed for FLC) | **assumed** (not printed) |
| Input MF feet | half-width 0.5 | - | `fuzzy_assumptions.yaml` Ruspini | **assumed** (Fig.2 only) |
| Output MF centres / feet | {0,1/3,2/3,1} / half-width 1/3 | - | `fuzzy_assumptions.yaml` | **assumed** (Fig.2 only) |
| Output scale `Ku` | 1.0 | - | `fuzzy_assumptions.yaml` | **assumed** (not printed) |
| Mamdani AND / aggregation | min / max | - | `fuzzy_assumptions.yaml` | **assumed** (only "Mamdani" printed) |
| CoG grid | 1001 points on [0,1] | - | `fuzzy_assumptions.yaml` | **assumed** (not printed) |
| Per-axis structure | one FLC per axis | - | `fuzzy_assumptions.yaml` | **assumed** (not printed) |
| `s_dot` scheme | backward difference, first step 0 | - | `fuzzy_assumptions.yaml` | **assumed** (not printed) |
| `k0` (fixed sliding gain) | 1 (per-unit) | - | p.5 | verified |
| `a0` (fixed sigmoid slope) | 4 (per-unit) | - | p.5 | verified |
| Rule tables `aF`, `kF` | see `specs.fuzzy_rule_table` | - | Tables I-II, p.4 | verified |
| Inference / defuzzification | Mamdani / centre of gravity | - | p.3 | verified |
| Output mapping | `k = kF*k0`, `a = aF*a0` | - | pp.3-4 | verified |
| FLL normalization algebra |  | - | Fig.5 only | unresolved |
| Positive-sequence combination |  | - | not printed | unresolved |

All unresolved rows must stay `null`; the Phase 03e derived SI ranges (under D-015)
are in `docs/flc_spec.md` §(c). The FLC is not implemented.

## Estimator / controller status (Phase 05a Step 4, `docs/estimator_spec.md`)

Rendered-page extraction (pp.825–829).

| Item | Value | Unit | Source | Status |
| --- | --- | --- | --- | --- |
| Position estimate form | `θ̂_e = −arctan(Ê_α/Ê_β)` | rad | Eq.(8), p.825 | verified (direct form; the paper's block diagram uses the Q-PLL instead) |
| Q-PLL structure | two multipliers (`cos θ̂`, `sin θ̂`) → `Δe` → `PI` → `ω̂` → `1/s` → `θ̂` | - | Fig.1, p.825 | verified (block only) |
| Q-PLL error `Δe` algebra |  | - | Fig.1 only | unresolved |
| Q-PLL PI gains |  | - | not printed | unresolved |
| Q-PLL normalization algebra |  | - | label only | unresolved |
| Speed equation | `ω̂` = Q-PLL PI output | rad/s | Fig.1 | partial (no equation) |
| Electrical↔mechanical conversion |  | - | not printed | unresolved |
| Angle wrapping convention |  | - | not printed | unresolved |
| Estimator input | filtered `Ê_f` from DSOGI-FLL | V | Fig.7, p.828; Eq.(12) | verified |
| Speed-loop frequency | 1000 | Hz | Table III, p.828 | verified |
| Current-loop / sampling frequency | 10000 | Hz | Table III, p.828 | verified |
| Switching frequency | 10000 | Hz | Section V, p.829 | verified |
| Inverter dead time | 2 | µs | Section V, p.829 | verified |
| `i_d` reference | 0 | A | Fig.7, p.828 | verified |
| DC-bus voltage `V_DC` |  | V | Fig.7 (symbol only) | unresolved |
| Controller PI gains (speed/current) |  | - | not printed | unresolved |
| Load steps | 50 % rated (sim) / 20 % rated (exp) | - | Sections IV-D, V-D | verified |
| Phase 05b targets (position/speed errors) | see `docs/estimator_spec.md` §(b) | rad / rpm | Sections IV-V, Table IV | verified |

The position/speed estimator is **implemented** (Phase 05b); the closed-loop controller
is implemented as the Phase 06a **encoder baseline** (`simulation/drive.py`), not yet in
the estimator loop (Phase 06b).

## Drive status (Phase 06a, `configs/drive.yaml` + `configs/drive_assumptions.yaml`)

| Parameter | Symbol | Value | Unit | Source | Status |
| --- | --- | --- | --- | --- | --- |
| current-loop frequency | — | 10000 | Hz | Table III, p.828 | verified |
| speed-loop frequency | — | 1000 | Hz | Table III, p.828 | verified |
| switching frequency | — | 10000 | Hz | Section V, p.829 | verified |
| inverter dead time | — | 2 | µs | Section V, p.829 | verified (not modeled in 06a) |
| `i_d` reference | `i_d*` | 0 | A | Fig.7, p.828 | verified |
| DC-bus voltage | `V_DC` | 311 | V | symbol only (Fig.7); hinted by the 311/√3 base | **assumed** |
| inverter delay | — | 1 | sample | `drive_assumptions.yaml` | **assumed** |
| current-loop bandwidth | `α_c` | 500 | Hz | `drive_assumptions.yaml` | **assumed** |
| decoupling / back-EMF FF | — | ON / OFF | - | `drive_assumptions.yaml` | **assumed** |
| speed-loop bandwidth | `α_ω` | 50 | Hz | `drive_assumptions.yaml` | **assumed** |
| speed-loop damping | `ζ` | 1 | - | `drive_assumptions.yaml` | **assumed** |
| `i_q*` clamp | — | 2 | ×I_rated | `drive_assumptions.yaml` | **assumed** |
| viscous friction | `B` | 0 | N.m.s/rad | `drive_assumptions.yaml` | **assumed** |
| RK4 sub-steps per `Ts` | — | 4 | - | `drive_assumptions.yaml` | **assumed** |

Controller PI gains are DERIVED from `α_c/α_ω` (`Kp_c = α_c Ld`, `Ki_c = α_c Rs`,
`Kp_w = 2ζα_ω J/Kt`, `Ki_w = α_ω² J/Kt`, `Kt = ½·3 p ψ_f`); the paper prints none.
`I_rated = 9.678 A` is derived (D-037). No tuning toward the paper's numbers.
