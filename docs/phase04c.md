# Phase 04c — Nominal-drive scenario

Scope: **RAW `E_hat` only** — no DSOGI-FLL, no rotor position/speed claims. Phase
04c adds an open-loop *nominal drive* so the observer's **voltage input** is
exercised (every earlier study ran the short-circuited plant, ``u = 0``), then
re-runs the Phase 04b observer comparison under it.

**All FLC scaling / MF / operator / grid values are ASSUMPTIONS — not from paper:**
Ke = 0.13472 1/A, Kec = 1.3472e-5 s/A, Ku = 1, CoG grid = 1001, Ruspini MF feet,
AND = min, aggregation = max, per-axis FLC, backward-difference `s_dot`.

Scripts: `experiments/04c_nominal_scenario.py`; generator
`src/pmsm_sensorless/simulation/nominal_drive.py`. Steady window = 8 whole
electrical cycles at 1000 rpm = [1.880, 2.000] s; ramp = 03f sliding-window (60 ms)
fit over [0, 1.500] s. Scenario: prescribed 500 → 1000 rpm ramp (0 → 1.5 s, held),
identical to Phases 02–04.

## 1. The nominal drive (Step 1)

An **open-loop replica of ideal FOC**, plant-side only (it may use ground truth;
the observer never sees it — there is no controller and no feedback). Reference
current on the q-axis (`i_d = 0`), in phase with the true back-EMF of Eq.(2):

```
i_ref(t) = I_q [ -sin θ_e(t),  cos θ_e(t) ]
u_αβ     = Rs i_ref + Ld d(i_ref)/dt + E_true,   d(i_ref)/dt = I_q ω_e [ -cos θ_e, -sin θ_e ]
```

`u` is held constant over each `Ts` (inverter-like ZOH); the plant starts at
`i = i_ref(0)`. Under the amplitude-invariant Clarke convention this gives
`i_q = I_q`.

Rated current (derived, `i_d = 0`, `Ld = Lq`): **`I_rated = T_rated / ((3/2) p ψ_f)`
= 7.7 / (1.5·4·0.1326) = 9.678 A`** (≈ 1.30 × `I_base` = 7.423 A).

**Tracking (criterion i).** At `Ts = 10 us`, `I_q = I_rated`, the plant current
tracks `i_ref` to **max |i − i_ref| = 0.160 A = 1.65 % of `I_q`** (bound 2 %). The
feedforward is the exact inverse of Eq.(1) at the plant's own angle, so the only
residual is the ZOH of `u` over one step (`ω_e Ts = 0.24°`) plus RK4 truncation —
second order in `ω_e Ts`, making 2 % a generous bound. Structure and determinism
tests: `tests/test_nominal_drive.py`.

**GATE (Step 1).** Stable fixed sigmoid, `Ts = 10 us`, `I_q = I_rated`:

| metric | value | bound | result |
| --- | --- | --- | --- |
| E_hat fundamental gain | 0.9894 | within 2 % | **PASS** |
| E_hat fundamental phase | −0.608° | within 1° | **PASS** |
| steady ripple | 0.018 V | — | — |

The gate passing validates the `u` sign, units and ZOH handling in Eq.(4) (no
`smo.py`/`sigmoid_smo.py` change was needed).

## 2. Comparison under the nominal drive (Step 2)

**Matched fixed-gain cell selection** (from `experiments/output/04b_fixed_gain/fixed_gain.json`):
*among the constant-gain grid cells at Ts = 100 µs inside the D-027 stability bound
(`kF·aF < B(100 µs) = 0.597`), take the smallest steady-window ripple.* → **kF = 1.0,
aF = 0.4** (`Ts100_fixed_kf1_af0.4`, u=0 ripple 0.040 V). It is applied at both Ts.
The Phase 04b cell (FLC time-mean **0.355 / 0.591**) is kept as a reference.

### 2.1 Headline (nominal drive, `I_q = I_rated`)

| Ts | observer | gain | phase (°) | ripple (V) | RMS\|S\| (A) | peak\|S\| (A) | period-2 | t_conv (ms) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 100 µs | fixed sign | 0.969 | −0.82 | 248.27 | 10.55 | 19.09 | no | — |
| 100 µs | fixed sigmoid (k0, a0) | 0.972 | −0.66 | 194.18 | 7.42 | 9.98 | **yes** | 0 |
| 100 µs | **matched fixed gain (1.0, 0.4)** | 0.974 | −0.52 | **0.042** | **2.86** | **2.88** | no | 0.1 |
| 100 µs | reference cell (0.355, 0.591) | 0.940 | −2.82 | 1.00 | 6.52 | 7.08 | no | 0 |
| 100 µs | FLC-sigmoid (nominal) | 0.953 | −1.83 | 1.24 | 5.04 | 5.44 | no | 0.1 |
| 50 µs | fixed sign | 0.986 | −0.55 | 248.07 | 5.28 | 9.43 | no | 2000 |
| 50 µs | fixed sigmoid | 0.990 | −0.09 | **0.018** | **1.16** | **1.17** | no | 0.2 |
| 50 µs | matched fixed gain | 0.974 | −1.17 | 0.042 | 2.86 | 2.88 | no | 0.1 |
| 50 µs | reference cell | 0.939 | −3.46 | 1.00 | 6.51 | 7.06 | no | 0 |
| 50 µs | FLC-sigmoid (nominal) | 0.952 | −2.48 | 1.22 | 5.04 | 5.42 | no | 0.1 |

`t_conv` rule identical to 04b (first time after which `|S|` stays below 1.5× its
steady RMS; `—` = never within the run). `peak|S|` is the steady-window maximum.

### 2.2 `u = 0` vs nominal at identical observers and Ts

| Ts | observer | u=0 gain / phase / ripple | nominal gain / phase / ripple |
| --- | --- | --- | --- |
| 100 µs | fixed sign | 0.958 / −1.58 / 248.40 | 0.969 / −0.82 / 248.27 |
| 100 µs | fixed sigmoid | 0.962 / −1.26 / 194.69 (p2) | 0.972 / −0.66 / 194.18 (p2) |
| 100 µs | matched (1.0, 0.4) | 0.964 / −1.10 / 0.040 | 0.974 / −0.52 / 0.042 |
| 100 µs | reference (0.355, 0.591) | 0.930 / −3.37 / 0.96 | 0.940 / −2.82 / 1.00 |
| 100 µs | FLC-sigmoid (nominal) | 0.943 / −2.42 / 1.23 | 0.953 / −1.83 / 1.24 |
| 50 µs | fixed sign | 0.981 / −0.86 / 248.13 | 0.986 / −0.55 / 248.07 |
| 50 µs | fixed sigmoid | 0.984 / −0.38 / 0.018 | 0.990 / −0.09 / 0.018 |
| 50 µs | matched (1.0, 0.4) | 0.969 / −1.46 / 0.040 | 0.974 / −1.17 / 0.042 |
| 50 µs | reference (0.355, 0.591) | 0.934 / −3.74 / 0.97 | 0.939 / −3.46 / 1.00 |
| 50 µs | FLC-sigmoid (nominal) | 0.947 / −2.78 / 1.22 | 0.952 / −2.48 / 1.22 |

The nominal drive shifts every result only slightly (gain +0.005…+0.011, ~0.3–0.6°
less phase lag, ripple within ~1 %) — confirming the Phase 04b conclusions did
**not** depend on the short-circuit.

### 2.3 Dependence on `I_q` (nominal)

Steady ripple is essentially **independent of `I_q`** across {2, 5, 9.678} A (the
observer is driven by `S` and its own gains, not the current magnitude):

| Ts | fixed sign | fixed sigmoid | matched | FLC nominal |
| --- | --- | --- | --- | --- |
| 100 µs | 248.28 / 248.30 / 248.27 | 194.18 (all) | 0.042 (all) | 1.241 / 1.242 / 1.242 |
| 50 µs | 248.07 / 248.06 / 248.07 | 0.018 (all) | 0.042 (all) | 1.221 (all) |

### 2.4 FLC gain statistics (nominal, `I_q = I_rated`)

| Ts | quantity | FLC nominal (100 µs) |
| --- | --- | --- |
| 100 µs | kF mean / min / max | 0.357 / 0.271 / 0.376 |
| 100 µs | aF mean / min / max | 0.592 / 0.395 / 0.663 |
| 100 µs | kF·aF mean | 0.212 |
| 100 µs | fraction `kF·aF < B(100 µs)=0.597` | 1.00 |
| 100 µs | fraction `kF·k0 < |E_true| = 55.54 V` (eval only) | 0.04 |
| 100 µs | input-saturation fraction (`|s_n|=1` or `|ṡ_n|=1`) | 0.00 |

(The FLC's time-mean gains, `kF ≈ 0.357`, `aF ≈ 0.592`, reproduce the Phase 04b
reference cell `0.355 / 0.591`.)

(`u = 0` values are essentially identical: `kF·aF` = 0.211, `kF` ∈ [0.279, 0.374],
`aF` ∈ [0.418, 0.662], fraction `kF·k0 < |E_true|` = 0.053.)

### 2.5 FLC grid vs the matched fixed-gain cell (`I_q = I_rated`)

Ke, Kec ∈ {0.1, 1, 10} × nominal (9 cells), both Ts, both scenarios:

- **No FLC cell dominates the matched fixed-gain cell** on
  (ripple, |gain−1|, |phase|, RMS|S|) simultaneously — `any_flc_cell_dominates_matched`
  = **false** in all four (scenario, Ts) combinations.
- **No FLC cell extends** the fixed-gain `(t_conv, ripple)` Pareto front —
  `any_flc_beyond_front` = **false**.
- Fixed-gain cells **dominating the FLC nominal**: **7** at Ts = 100 µs, **9** at
  Ts = 50 µs (identical counts for u=0 and nominal).

### 2.6 Ramp window (03f windowed fit, [0, 1.5] s)

Mean over the 25 windows (worst-window ripple in brackets), nominal drive,
`I_q = I_rated`:

| Ts | observer | mean gain | mean phase (°) | mean ripple (V) |
| --- | --- | --- | --- | --- |
| 100 µs | fixed sign | 0.966 | −0.91 | 250.69 |
| 100 µs | fixed sigmoid | 0.966 | −0.73 | 205.96 |
| 100 µs | matched (1.0, 0.4) | 0.974 | −0.35 | **0.29** |
| 100 µs | reference (0.355, 0.591) | 0.946 | −1.81 | 0.49 |
| 100 µs | FLC-sigmoid | 0.947 | −1.60 | 1.03 |
| 50 µs | fixed sign | 0.984 | −0.52 | 250.57 |
| 50 µs | fixed sigmoid | 0.989 | −0.05 | 0.30 |
| 50 µs | matched (1.0, 0.4) | 0.974 | −0.85 | **0.24** |
| 50 µs | reference (0.355, 0.591) | 0.945 | −2.31 | 0.49 |
| 50 µs | FLC-sigmoid | 0.947 | −2.10 | 0.97 |

During the ramp the matched cell's ripple is larger than in the `u = 0` case
(0.29 vs 0.06 V at 100 µs) because the reference accelerates (`d i_ref/dt ≠ 0`)
and `u` is held over each step; it is still ~3.5× below the FLC and far below the
sign/sigmoid chatter.

## 3. Phase 04b conclusions under the nominal drive

| # | 04b conclusion | u = 0 | nominal | status |
| --- | --- | --- | --- | --- |
| 1 | matched fixed gain beats FLC nominal on steady ripple (Ts=100 µs) | matched 0.040 vs FLC 1.232 V | matched **0.042** vs FLC **1.242** V | **CONFIRMED** |
| 2 | several fixed-gain cells dominate FLC nominal on all four metrics (Ts=100 µs) | 7 | **7** | **CONFIRMED** |
| 3 | no FLC cell extends the fixed-gain Pareto front (Ts=100 µs) | 0 beyond | **0 beyond** | **CONFIRMED** |
| 4 | stable fixed sigmoid is the best cell at Ts=50 µs | 0.0176 V | **0.0179 V** | **CONFIRMED** |
| 5 | FLC moves to low gain (`kF·aF ≈ 0.21`) and avoids the D-027 limit cycle at Ts=100 µs | 0.211, no p2 | **0.212, no p2** | **CONFIRMED** |

No 04b conclusion is changed by the nominal drive. Holding everything else fixed,
the fixed sigmoid at Ts = 100 µs remains a **period-2 limit cycle** (`kF·aF = 1.0
≥ 0.597`; note also `fraction_in_stability_bound` = 0.0) exactly as D-027 predicts —
this is a gain/`Ts` property, independent of the operating current.

## 4. Limitations

- All FLC statements remain **conditional on the assumed Ke/Kec/MF/operator/grid**
  values; the FLC grid explores only a 3×3 scale grid at `I_q = I_rated`.
- No DSOGI-FLL and no rotor position/speed estimation; the "nominal drive" is an
  open-loop ideal-FOC replica, **not** a controller and **not** a closed loop.
- `I_q` is swept at three values; load-torque steps, field weakening and inverter
  non-idealities (dead time, DC-bus limit) are not modelled.
