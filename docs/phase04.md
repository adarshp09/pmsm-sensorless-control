# Phase 04 — FLC adaptation of k and a (with Phase 04b re-base)

Scope: **RAW `E_hat` only** — no DSOGI-FLL, no rotor position/speed claims. The FLC
adapts `k_t = kF·k0`, `a_t = aF·a0` (D-015) and reports `k_t·g(S_pre; a_t)` (D-025).

**All FLC scaling / MF / operator / grid values are ASSUMPTIONS — not from paper:**
Ke = 0.13472 1/A, Kec = 1.3472e-5 s/A, Ku = 1, CoG grid = 1001, Ruspini MF feet,
AND = min, aggregation = max, per-axis FLC, backward-difference `s_dot`.

Scripts: `experiments/04_flc_adaptation.py`, `experiments/04b_fixed_gain.py`.
Steady window = 8 whole electrical cycles at 1000 rpm = [1.880, 2.000] s; ramp =
03f sliding-window (60 ms) fit over [0, 1.500] s.

## 1. Unified S metrics (Phase 04b correction)

All sliding-surface metrics are now computed **from `S_pre = i_hat[k] - i[k]`** over
the same window:

- `rms_s_alpha_a`, `rms_s_beta_a` — per-axis RMS: `sqrt(mean(S_axis²))`.
- `mean_abs_s_a` — mean of the vector magnitude `|S| = sqrt(Sα² + Sβ²)`.
- `rms_abs_s_a` — RMS of the vector magnitude, `sqrt(mean(Sα² + Sβ²))`.

**Why "RMSE" < "mean|S|" appeared in 5 of 6 Phase 04 rows:** the old
`current_rmse_a` was computed from the **post-update** estimate `i_hat[k+1]`, while
`mean_abs_surface_a` used the **sample-instant** `S_pre`. One explicit Euler step
reduces the error, so the post-update RMS is systematically smaller than the
pre-update mean. It was comparing two different quantities at two different times.
With the unified definition, `RMS|S| ≥ mean|S|` always (Jensen), which the new tables
satisfy. **The old `current_rmse_a` / `mean|S|` pair is retracted as a metric pair**;
the new columns below are the reference.

## 2. Headline comparison (unified metrics)

| Ts | observer | gain | phase (deg) | ripple (V) | RMS\|S\| (A) | mean\|S\| (A) | RMS Sα / Sβ (A) | period-2 | t_conv (s) | max\|S\| first 20 ms (A) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 100 µs | fixed sign | 0.958 | −1.58 | 248.40 | 10.589 | 9.825 | 7.446 / 7.529 | no | 1.999 | 16.83 |
| 100 µs | fixed sigmoid (default, kF=aF=1) | 0.962 | −1.26 | 194.69 | 7.427 | 7.126 | 5.251 / 5.251 | **yes** | 0.0 | 9.90 |
| 100 µs | **matched fixed gain (kF=0.355, aF=0.591)** | 0.930 | −3.37 | **0.96** | 6.411 | 6.400 | 4.533 / 4.533 | no | 0.0 | **2.78** |
| 100 µs | FLC-sigmoid (nominal) | 0.943 | −2.42 | 1.23 | **5.009** | 5.003 | 3.542 / 3.542 | no | 0.0 | 3.36 |
| 50 µs | fixed sign | 0.981 | −0.86 | 248.13 | 5.311 | 4.976 | 3.755 / 3.755 | no | 2.000 | 9.36 |
| 50 µs | fixed sigmoid (default) | 0.984 | −0.38 | **0.02** | **1.157** | 1.157 | 0.818 / 0.818 | no | 0.0 | **0.94** |
| 50 µs | matched fixed gain | 0.934 | −3.74 | 0.97 | 6.457 | 6.445 | 4.566 / 4.566 | no | 0.0 | 2.79 |
| 50 µs | FLC-sigmoid (nominal) | 0.947 | −2.78 | 1.22 | 5.023 | 5.017 | 3.552 / 3.552 | no | 0.0 | 3.36 |

`t_conv` rule: first time after which `|S|` stays below **1.5× its own steady-window
RMS** (reported values 0.0 mean "already below from the start" — the threshold is
generous for stable observers; `max|S|` in the first 20 ms is the discriminating
transient metric).

## 3. Matched fixed-gain control (SigmoidSMO with constant gains)

`k = kF·k0`, `a = aF·a0`, Ts = 100 µs (Ts = 50 µs in the JSON). Bound
`B(100 µs) = 0.597`.

| kF | aF | gain | phase | ripple (V) | RMS\|S\| | period-2 | in bound | max\|S\| 20 ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.35 | 0.1 | 0.713 | −16.34 | 1.090 | 26.238 | no | yes | 12.97 |
| 0.35 | 0.2 | 0.830 | −9.65 | 1.363 | 16.129 | no | yes | 7.48 |
| 0.35 | 0.4 | 0.903 | −5.16 | 1.209 | 9.203 | no | yes | 4.06 |
| 0.35 | 0.6 | 0.930 | −3.41 | 1.006 | 6.464 | no | yes | 2.78 |
| 0.35 | 1.0 | 0.953 | −1.87 | 0.734 | 4.062 | no | yes | 1.86 |
| 0.5 | 0.1 | 0.793 | −11.86 | 0.570 | 19.451 | no | yes | 9.68 |
| 0.5 | 0.2 | 0.884 | −6.38 | 0.517 | 11.042 | no | yes | 5.35 |
| 0.5 | 0.4 | 0.935 | −3.07 | 0.351 | 5.911 | no | yes | 2.82 |
| 0.5 | 0.6 | 0.953 | −1.87 | 0.258 | 4.034 | no | yes | 1.92 |
| 0.5 | 1.0 | 0.967 | −0.86 | 0.167 | 2.467 | no | yes | 1.86 |
| 0.75 | 0.1 | 0.858 | −8.02 | 0.229 | 13.565 | no | yes | 6.81 |
| 0.75 | 0.2 | 0.921 | −3.99 | 0.164 | 7.324 | no | yes | 3.64 |
| 0.75 | 0.4 | 0.955 | −1.73 | 0.097 | 3.807 | no | yes | 1.88 |
| 0.75 | 0.6 | 0.966 | −0.93 | 0.068 | 2.571 | no | yes | 1.86 |
| 0.75 | 1.0 | 0.973 | −0.55 | 97.081 | 3.889 | **yes** | **no** (0.75) | 5.62 |
| 1.0 | 0.1 | 0.891 | −5.99 | 0.112 | 10.421 | no | yes | 5.25 |
| 1.0 | 0.2 | 0.939 | −2.82 | 0.072 | 5.506 | no | yes | 2.76 |
| 1.0 | 0.4 | 0.964 | −1.10 | 0.040 | 2.830 | no | yes | 1.86 |
| 1.0 | 0.6 | 0.973 | −0.51 | 0.028 | 1.904 | no | **no** (0.60) | 1.86 |
| 1.0 | 1.0 | 0.962 | −1.26 | 194.690 | 7.427 | **yes** | **no** (1.0) | 9.90 |

The two cells just above `B` (`kF·aF = 0.75` and `0.60`) are **outside the small-signal
bound**; `0.60` is still clean, `0.75` is a limit cycle — the bound is sufficient, not
necessary.

**Dominance.** A fixed-gain cell **dominates the FLC nominal** if it is no worse on
ripple, `|gain−1|`, `|phase|` and `RMS|S|` simultaneously. **7 cells at Ts = 100 µs**
do (`0.35/1.0`, `0.5/0.6`, `0.5/1.0`, `0.75/0.4`, `0.75/0.6`, `1.0/0.4`, `1.0/0.6`);
**16 cells across both Ts** dominate. The matched fixed-gain cell (`0.355/0.591`) is
better than the FLC on ripple (0.96 vs 1.23 V) and `max|S|` (2.78 vs 3.36 A) but worse
on `|gain−1|` (0.070 vs 0.057) and `RMS|S|` (6.41 vs 5.01 A).

## 4. Transient metric and Pareto

`t_conv` (rule above) is 0.0 for every stable cell and 1.999 s for the fixed sign
observer; it does not discriminate the stable cells. `max|S|` in the first 20 ms
does: fixed sign 16.83 A, fixed sigmoid 9.90 A, matched fixed gain 2.78 A, FLC 3.36 A
at 100 µs.

Pareto: comparing `(t_conv, steady ripple)` and `(t_conv, |gain−1|)`, **every FLC cell
is dominated by at least one fixed-gain cell**; **no FLC cell lies beyond the
fixed-gain Pareto front**. (The FLC grid points are shown in
`experiments/output/04b_fixed_gain/transient_tradeoff.png`.)

## 5. FLC diagnostics (nominal, 100 µs; surfaces in `flc_surfaces.png`)

- **Achievable output range** over a dense [−1,1]² grid: kF and aF both
  **[0.111, 0.889]** (the ±0.111 offsets are centre-of-gravity compression of the
  edge sets `Z`/`PB` against the [0,1] universe).
- **Observed range**: kF **[0.279, 0.374]**, aF **[0.418, 0.663]**.
- **Input saturation**: 0.0 % of samples have `|s_n|=1` or `|s_dot_n|=1`; visited
  `s_n ∈ [−0.621, 0.620]`, `s_dot_n ∈ [−0.064, 0.066]`.
- So the narrow observed range is **not** input saturation and **not** CoG
  compression of the extremes; it comes from the **rule table acting on the visited
  central input band** (the trajectory stays near `s_dot_n ≈ 0`, where the rules
  return mostly `PS`/`PM`). CoG compression only sets the two extreme values
  (0.111 / 0.889), it does not narrow the central range.
- **Minimum achievable `kF·k0` = 19.9 V**, which is **below** `|E|max = 55.5 V` — the
  FLC cannot guarantee the tracking condition `kF·k0 ≥ |E|` over its whole output
  range.

## 6. Re-based findings (Phase 04b)

- **Retracted:** "the FLC strongly helps at 100 µs". That was only true against the
  **limit-cycle default** `kF=aF=1` (ripple 194.7 → 1.23 V). Against **matched
  fixed-gain control**, the FLC nominal (ripple 1.23 V) is beaten on steady ripple by
  the matched cell (0.96 V) and by several grid cells; **7 fixed-gain cells dominate
  it on all four metrics**, and **no FLC cell extends the fixed-gain Pareto front**.
- At `Ts = 50 µs` the stable fixed sigmoid (ripple 0.02 V) remains the best cell; the
  FLC (1.22 V) and the matched fixed gain (0.97 V) are both worse.
- The FLC's main effect is to move the operating point to a lower-gain regime
  (`kF·aF ≈ 0.21`), which avoids the D-027 limit cycle but leaves a larger surface
  band than the best fixed gains.
- All statements remain **conditional on the assumed Ke/Kec/MF/operator/grid values**.

## 7. Assumptions (unchanged, repeated for every table)

Ke = 0.13472 1/A, Kec = 1.3472e-5 s/A, Ku = 1, CoG grid = 1001, Ruspini triangular
feet (input half-width 0.5, output half-width 1/3, output centres {0,1/3,2/3,1}),
AND = min, aggregation = max, one FLC per axis, `s_dot` backward difference (zero
first step), non-finite normalised input → 0. Every value is
**ASSUMPTION — not from paper**.

## 8. Limitations

- No DSOGI-FLL, no position/speed.
- `t_conv` is degenerate for stable cells; use `max|S|` in the first 20 ms.
- Only a 3×3 Ke/Kec grid and a 4×5 fixed-gain grid were explored.
