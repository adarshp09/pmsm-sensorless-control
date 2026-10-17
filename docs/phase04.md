# Phase 04 — FLC adaptation of k and a

Scope: **RAW `E_hat` only** — no DSOGI-FLL, no rotor position/speed claims. The FLC
adapts the sigmoid SMO's switching gain and slope per step and per axis:
`k_t = kF·k0`, `a_t = aF·a0` (D-015), reporting `k_t·g(S_pre; a_t)` (D-025).

## Implementation

- `src/pmsm_sensorless/fuzzy/engine.py` — Mamdani FLC: triangular Ruspini
  membership, the printed 5×5 rule bases (Tables I–II), AND=min, aggregation=max,
  centre of gravity on a 1001-point grid over the printed [0,1] output universe.
- `src/pmsm_sensorless/observers/fuzzy_smo.py` — `FuzzySMO`, a subclass of
  `SlidingModeObserver`; the existing `ConventionalSMO`/`SigmoidSMO` code is
  untouched. The FLC sees only the observer's own `S = i_hat - i` and its backward
  difference; no plant/angle/speed/`E_true`.
- Printed items: `configs/fuzzy.yaml`. Every unprinted item:
  `configs/fuzzy_assumptions.yaml` (keys commented "ASSUMPTION — not from paper";
  null/missing raises `ConfigError`).

## Assumptions (each is `ASSUMPTION — not from paper`)

| Item | Value | Where |
| --- | --- | --- |
| Ke | `1/I_base = 0.13472 1/A` | fuzzy_assumptions |
| Kec | `Ts/I_base = 1.3472e-5 s/A` | fuzzy_assumptions |
| Ku | 1 (fixed by the printed [0,1] output universe) | fuzzy_assumptions |
| MF feet | Ruspini, half-width = centre spacing (inputs 0.5, outputs 1/3) | fuzzy_assumptions |
| Output centres | equally spaced {0, 1/3, 2/3, 1} | fuzzy_assumptions |
| AND / aggregation | min / max | fuzzy_assumptions |
| CoG grid | 1001 points on [0,1] | fuzzy_assumptions |
| per-axis structure | one independent FLC per α/β axis | fuzzy_assumptions |
| `s_dot` | backward difference, zero on the first step | fuzzy_assumptions |
| non-finite input | normalised to neutral 0 | fuzzy_assumptions |

## Experiment (`experiments/04_flc_adaptation.py`)

Identical scenario to 02/03e: short-circuited plant (`u=0`), 500→1000 rpm ramp,
`Ts ∈ {100, 50} µs`. Steady window = 8 whole electrical cycles at 1000 rpm =
[1.880, 2.000] s; ramp = 03f sliding-window fit over [0, 1.500] s.

### Nominal cell (ASSUMPTION Ke=0.13472 1/A, Kec=1.3472e-5 s/A, Ku=1, grid=1001)

| Ts | observer | gain | phase (deg) | ripple (V) | current RMSE (A) | mean\|S\| (A) | period-2 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 100 µs | fixed sign | 0.958 | −1.58 | 248.40 | 10.225 | 9.825 | no |
| 100 µs | fixed sigmoid | 0.962 | −1.26 | 194.69 | 7.058 | 7.126 | **yes (limit cycle)** |
| 100 µs | **FLC-sigmoid (nominal)** | 0.943 | −2.42 | **1.23** | **3.456** | 5.003 | no |
| 50 µs | fixed sign | 0.981 | −0.86 | 248.13 | 5.101 | 4.976 | no |
| 50 µs | fixed sigmoid | 0.984 | −0.38 | **0.02** | **0.922** | 1.157 | no |
| 50 µs | FLC-sigmoid (nominal) | 0.947 | −2.78 | 1.22 | 4.081 | 5.017 | no |

Nominal FLC gains (both Ts): kF mean **0.355** (0.279–0.374), aF mean **0.591**
(0.418–0.663), kF·aF mean **0.211**. Fraction with kF·aF below the stability bound
`B(Ts)` (0.597 at 100 µs, 1.204 at 50 µs): **1.00**. Fraction with `kF·k0 < |E_true|`:
**0.053**.

### Sensitivity grid (Ke, Kec ∈ {0.1, 1, 10} × nominal; steady ripple, V)

| Ke \ Kec | 0.1 | 1 | 10 |
| --- | --- | --- | --- |
| 0.1 | 1.78 / 1.75 | 10.07 / 8.94 | 14.41 / 14.00 |
| 1 | 1.36 / 1.34 | **1.23 / 1.22** | 44.49 / 36.91 |
| 10 | 73.33 / 5.11 | 133.77 / 11.69 | 138.10 / 51.10 |

(values are Ts=100 µs / Ts=50 µs). Best cell = nominal (1×,1×) at both Ts; worst =
(10×,10×). Cells beating the fixed sigmoid on **ripple AND gain error**: **3/9 at
100 µs** (`Ke=10×` cells) and **0/9 at 50 µs**.

## Findings

- **At `Ts = 100 µs` the FLC strongly helps**: the fixed sigmoid is a period-2 limit
  cycle (ripple 194.7 V), while the FLC keeps `kF·aF` below the stability bound and
  reduces ripple to **1.23 V** and current RMSE from 7.06 to **3.46 A**.
- **At `Ts = 50 µs` the FLC is worse** than the stable fixed sigmoid (ripple 1.22 V
  vs 0.02 V, RMSE 4.08 vs 0.92 A): the fixed observer is already stable, and the
  FLC's reduced gain (`kF·aF ≈ 0.21`) leaves a larger sliding-surface band.
- Grid: `Ke = 0.1×` under-drives (gain ~0.85, RMSE ~12–13 A); `Ke = 10×` over-drives
  (higher gain but higher ripple; period-2 at 100 µs). The nominal (1×,1×) is the
  best ripple cell at both Ts.
- The FLC's fundamental gain error (0.943/0.947 → 0.057/0.053) is larger than the
  fixed sigmoid's (0.038/0.016).
- **FLC conclusions are conditional on the assumed Ke/Kec/MF/operator/grid values**;
  none of these is printed by the paper.

## Limitations

- No DSOGI-FLL, no position/speed estimation.
- The FLC's step change means the observer is no longer bit-identical to the fixed
  version by construction; the forced `kF=aF=1` case is verified bit-identical in
  tests.
- The nominal Ke/Kec are single-point assumptions; only a 3×3 grid was explored.
