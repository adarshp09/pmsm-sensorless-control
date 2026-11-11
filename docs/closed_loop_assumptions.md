# Closed-loop assumptions register (Phase 06b-6, Step 0b)

Every non-paper element active in the sensorless loop: **value | origin | decision |
what it affects**. Origin: **PRINTED** (paper), **DERIVED** (computed locally from
printed values or a stated local rule), **ASSUMPTION** (local choice).

| Element | Value (as configured) | Origin | Decision | Affects |
| --- | --- | --- | --- | --- |
| Sampling / current-loop `Ts` | 100 µs (10 kHz) | PRINTED (Table III) | — | all discrete timing |
| Current-loop bandwidth `α_c` | 500 Hz (`Kp=α_c Ld`, `Ki=α_c Rs`) | ASSUMPTION | D-047 | current-loop transient, voltage command |
| Speed-loop damping `ζ` | 1.0 | ASSUMPTION | D-047 | speed-loop shape |
| Speed-loop bandwidth `α_ω*` | **2.5 Hz @ 500 rpm, 10 Hz @ 1000 rpm** (encoder baseline uses 50 Hz) | ASSUMPTION | D-059, D-061 | sensorless stability/margins, load-step dip |
| `i_q*` clamp | ±2·I_rated = ±19.36 A | ASSUMPTION | D-047 | spin-up/transient current limit |
| Viscous friction `b` | 0 | ASSUMPTION | D-047 | mechanical balance |
| RK4 sub-steps per `Ts` | 4 | ASSUMPTION | D-047 | plant integration accuracy |
| DC-bus `V_dc` / phase limit | 311 V / `V_dc/√3` = 179.6 V | ASSUMPTION (hinted by 311/√3) | D-048 | voltage saturation |
| Computation delay | 1 sample + ½ ZOH (1.5 `Ts`) | ASSUMPTION | D-048 | effective loop delay |
| Decoupling / back-EMF comp. | ON / OFF | ASSUMPTION | D-047 | dq voltage cross-coupling |
| Observer cell `kF` / `aF` | 1.0 / 0.4 (matched fixed sigmoid) | DERIVED (matched-cell rule) | D-037 | observer gain/ripple |
| Observer `k` / `a` (per-unit) | 1 / 4 → SI via `V_base`,`I_base` | PRINTED (p.5) / DERIVED (SI) | D-015 | observer switching gain/slope |
| DSOGI `k_s` / FLL `Γ` | √2 / 50 | PRINTED (p.5) | — | filter/FLL dynamics |
| Q-PLL `f_n` / `ζ` | 20 Hz / 0.707, normalization OFF | ASSUMPTION | D-043 | speed-estimate bandwidth |
| FLL lower clamp `min_omega` | 100 rpm mechanical (41.9 rad/s) | ASSUMPTION | D-062 | prevents `ω′ < 0` collapse |
| Handover threshold `ω_h` | 400 rpm (was 500) | ASSUMPTION | D-053, D-059 | when sensorless takes over |
| Handover tolerance / dwell | 2 rpm / 50 ms | ASSUMPTION | D-053 | lock criterion |
| Initial `ω′` | `p·ω_h` (design constant) | ASSUMPTION | D-053 | FLL start |
| Angle-advance flag | OFF by default; `1.5·ω̂·Ts` when ON | ASSUMPTION | D-053 | commanded-voltage frame |
| FLC scaling `Ke`/`Kec`/`Ku` | 0.13472 1/A, 1.3472e-5 s/A, 1.0 | ASSUMPTION | D-031 | FLC cell only (FLC nominal) |
| `I_rated` | 9.678 A = T_rated/1.5pψ_f | DERIVED | D-037 | operating point |
| Dead-time model | OFF by default; when ON `ΔV=-V_dc t_d f_sw sat(i_x/i_th)`, `i_th=0.1 A` | ASSUMPTION (i_th); dead time/f_sw PRINTED | D-069 | plant voltage distortion |
| Conventional baseline (DIAG) | sign SMO → LPF (cutoff unprinted; grid 100–400 Hz) → Q-PLL | ASSUMPTION | D-070 | comparison only |
| Generic handover criterion | `ω̂≥ω_h` and `|Δe|/|Ê|<sin5°`; default OFF | ASSUMPTION | D-070 | chains without an FLL |
| Fixed-time handover | `t_h = 0.30 s`; default OFF | ASSUMPTION | D-071 | comparison protocol |
| Dead-time compensation | OFF by default; `+V_dc t_d f_sw sat(i_meas/i_th)` added to plant voltage | ASSUMPTION | D-074 | plant voltage |

**Handover / `f_n` interaction (Phase 06d):** the lock criterion (`|ω̂−ω′|<2 rpm` for 50 ms)
is satisfied later as the Q-PLL slows — at `f_n = 10 Hz` handover is at 0.70 s, after a
0.5 s load step (so the step is encoder-driven), and at `f_n = 5 Hz` it never hands over.
Only `f_n ≥ 20 Hz` gives handover ≈ 0.20 s. Diagnostic arms (raw `E_hat`, oracle `ω′`,
normalised FLL) are **DIAGNOSTIC — not paper** and never replace the nominal cell.

**What 06b-2/3/4 changed and concluded (≤10 lines):**
- **06b-2** measured `T_est(jω)`; H1 (DSOGI envelope caps `T_est`, speed-dependent) **supported**; H2 (raw-E_hat chain wider) **refuted**. No global `α_ω` met PM≥45° / GM≥6 dB at both speeds → `α_ω* = None` (D-057/D-058).
- **06b-3** introduced operating-point decoupling (5 Hz @500 / 10 Hz @1000) + low-speed PM relaxation (≥10°). 500 rpm @5 Hz PM 10.8°. Gate 2a all pass; 2b@1000, 2c pass; **2b@500 fails** (lightly damped) (D-059/D-060).
- **06b-4** softened 500 rpm to 2.5 Hz → PM 65.5°; 500 rpm steady + all 1000 rpm gates pass; **2b@500 still fails** — the *unclamped FLL* loses lock on the step (D-061).
- Net: the speed loop is no longer the limiting factor at 1000 rpm; the low-speed *loaded* transient is an estimator (FLL) amplitude problem.