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

| Parameter | Symbol | Value | Unit | Source (page / eq / table) | Status |
| --- | --- | --- | --- | --- | --- |
| SMO sliding-surface gain |  |  | - |  | unresolved |
| SMO switching gain | k |  | - |  | unresolved |
| sigmoid slope | a |  | - |  | unresolved |
| boundary layer | delta |  | - |  | unresolved |
| FLC input scale (error) | Ke |  | - |  | unresolved |
| FLC input scale (delta error) | Kec |  | - |  | unresolved |
| FLC output scale | Ku |  | - |  | unresolved |
| DSOGI damping gain | k |  | - |  | unresolved |
| FLL gain | gamma |  | - |  | unresolved |
| FLL min/max frequency |  |  | Hz |  | unresolved |
| sampling period | Ts | 0.0001 | s | Eq.(20), p.5; derived from 10 kHz | derived |

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
