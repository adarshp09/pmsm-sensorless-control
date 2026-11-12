# Phase 06g-2 — Paper-implied transient and transient-profile test

**Status: COMPLETE, no `src/` changes.** Step 0 corrections and the prescribed
profile experiment were completed. The provisional Phase 06 tags were not
replaced or moved, and Phase 07 remains unopened.

## Step 0 — corrections and frozen interpretation

### FLL time constant

The small-signal frozen-amplitude test measured a 63% rise time of **3.82 ms at
1000 rpm** and **5.08 ms at 800 rpm**. The earlier formula was low by a
spurious factor of two. The empirical relation used here is:

`tau = k_s * omega' / (Gamma * A^2)`,

with `k_s = sqrt(2)`, `Gamma = 50`, and `A = psi_f * omega_e`. It gives
**3.84 ms at 1000 rpm, 4.80 ms at 800 rpm, and 7.68 ms at 500 rpm**. For S2,
using the falling speed at the transient peak, approximately **765 rpm**,
gives `tau ≈ 5.02 ms`; `a0 * tau ≈ 121 rpm` for `a0 = 24,027 rpm/s`,
compared with the measured first peak of **135 rpm**. The prior “10–20% at
500 rpm” statement is replaced by this speed-specific comparison.

The Phase 03d default settling time of **5.1 ms** used a settling band rather
than a 63% time constant: it is consistent with the approximately 3.84 ms
1000-rpm time constant when the reported band is reached after roughly
1.3 time constants. The 0.1-amplitude result scales approximately as `1/A^2`,
as expected.

The FLL algebra is not constrained by the paper. Depending on the assumed
normalization, the observed time-scale spans approximately **0.25 ms
(A-normalized)** to **3–12 s (A^2-normalized without an omega' factor)**; the
standard omega'-factor form is approximately `k_s/Gamma = 28 ms`. This
ambiguity does not change the conclusion: ideal V0 at `f_n = 20 Hz` already
has an S2 first peak of 87 rpm.

### Paper-implied acceleration estimate

The paper's figures label speed error and position error, but do not state
whether the position error is mechanical or electrical. The estimator
convention in this repository is electrical angle; the following is therefore
an **upper estimate**, assuming the printed position error is electrical.
With `p = 4`, `e_omega` is converted to electrical rad/s. For a sustained
deceleration and a second-order PLL:

`r = e_theta / e_omega = 2.29 / omega_n` for `zeta = 0.707`,

with the corresponding standard second-order factors also evaluated at
`zeta = 0.5` and `zeta = 1.0`. The `zeta = 0.707` estimates are:

| Case | Printed errors | Estimated `omega_n` | Estimated `a0` | Printed `DeltaT/J` |
| --- | --- | ---: | ---: | ---: |
| S2 | 23 rpm, 0.22 rad | about 100 rad/s (16 Hz) | about 5,000 rpm/s | 24,027 rpm/s |
| S3 | 10 rpm, 0.152 rad | about 63 rad/s (10 Hz) | about 1,400 rpm/s | 9,611 rpm/s |

Thus the inferred accelerations are about **4.8x** and **7x** below the
instantaneous printed `DeltaT/J`. A pulse-like transient has a smaller
`r` than a sustained deceleration, so these are upper estimates. Other
assumptions are electrical position, `p = 4`, `zeta = 0.707`, the
acceleration-step model, and the printed torque/inertia applying to the
respective scenarios. The earlier paper-implied `f_n ≈ 70–76 Hz` claim is
**retracted**. The chain's measured own `r` values are **8.1 ms (S2)** and
**7.7 ms (S3)**.

### Ruff baseline

`python -m ruff check src tests experiments --statistics` reports **694
pre-existing errors**. Counts are recorded without fixes in
[lint_status.md](lint_status.md).

## Step 1 — prescribed-profile generator

`PMSMPlant.step` already accepts caller-prescribed `state.omega_e` and advances
the electrical angle in prescribed-kinematics mode. No `src/` change or
`nominal_drive_rollout` extension was needed. The experiment-side generator is
[profile_drive.py](../experiments/profile_drive.py).

Its ideal feedforward is the nominal-drive equation:

`u = Rs*i_ref + Ld*d(i_ref)/dt + E_true`, with `i_q = I_rated` and ZOH over
`Ts = 100 us`.

The linear-ramp regression reproduces `nominal_drive_rollout` exactly:
maximum absolute difference is **0.0** for voltage, current, angle, and speed
arrays.

## Step 2 — open-loop transient profiles

Predictions and criteria were written to
`experiments/output/06g2_profiles/profiles.json` before the profiles ran:

- S2 drops `D = {190, 235, 350}` rpm and `a0 = {1400, 2500, 5000,
  10000, 24027}` rpm/s.
- S3 drop `D = 46` rpm and `a0 = {700, 1400, 2800, 5600, 9611}` rpm/s.
- V0, V1, V2, V4 at `f_n = 20 Hz`; V0 and V2 also at 80 Hz for the
  paper-estimate and instantaneous-step accelerations.
- Frozen effective time constants: V2 `5.63/4.20 ms` and V0 `3.62/3.44 ms`
  for S2/S3; tolerance **40%**.
- Joint match means speed and position are each within 2x the paper value.

At `f_n = 20 Hz`, nominal V2 results for the 235-rpm S2 reading and S3 are:

| Case | `a0` rpm/s | `tau_d` ms | V2 first peak rpm | V2 peak position rad | `r` ms | `e/a0` ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| S2, D=235 | 1,400 | 456.3 | 9.0 | 0.033 | 8.79 | 6.43 |
| S2, D=235 | 2,500 | 255.5 | 14.1 | 0.064 | 10.82 | 5.64 |
| S2, D=235 | 5,000 | 127.8 | 25.3 | 0.127 | 11.96 | 5.05 |
| S2, D=235 | 10,000 | 63.9 | 45.7 | 0.222 | 11.59 | 4.57 |
| S2, D=235 | 24,027 | 26.6 | 92.2 | 0.327 | 8.46 | 3.84 |
| S3, D=46 | 700 | 178.6 | 5.5 | 0.010 | 4.16 | 7.90 |
| S3, D=46 | 1,400 | 89.3 | 8.4 | 0.023 | 6.57 | 6.03 |
| S3, D=46 | 2,800 | 44.7 | 13.5 | 0.040 | 7.08 | 4.81 |
| S3, D=46 | 5,600 | 22.3 | 21.3 | 0.058 | 6.47 | 3.80 |
| S3, D=46 | 9,611 | 13.0 | 29.1 | 0.073 | 5.97 | 3.03 |

V0 and V4 remain diagnostic chains. Full first-peak/window-max results,
including all D values and the `f_n = 80 Hz` rows, are in the JSON output.

### Frozen verdicts

1. **Joint match: supported.** There is one free profile parameter against two
   printed metrics. S2 has joint matches through `a0 = 10,000 rpm/s` for the
   235-rpm reading; S3 has joint matches through `a0 = 2,800 rpm/s`.
   The closest grid points to the Step 0d acceleration estimates are S2
   `5,000 rpm/s` and S3 `1,400 rpm/s`.
2. **Step 0d comparison: supported.** The selected estimates are within the
   frozen 50% criterion of the inferred values. They are respectively 4.8x
   and 6.9x below the printed instantaneous `DeltaT/J`, giving
   `J_eff/J ≈ 4.8` and `6.9` if interpreted as an instantaneous torque step.
3. **Gentler-transient hypothesis: supported, with scope.** The paper's
   printed errors are compatible with gentler acceleration profiles than an
   instantaneous 50%/20% step on the printed inertia. This does not explain
   all paper/ours differences: S3 hardware non-idealities and the unprinted
   profile/estimator settings remain untested.

## Step 3 — closed-loop confirmation

`run_drive` accepts a callable load torque, so this conditional step ran with
the nominal chain, `alpha_omega = 10 Hz`, `t_h = 0.30 s`, and dead time OFF.
The load ramp starts at 0.5 s and reaches 50% rated torque:

| Ramp time | True dip rpm | Peak deceleration rpm/s | Peak position error | Peak speed error | FLL |
| ---: | ---: | ---: | ---: | ---: | --- |
| 0.1 s | -60.4 | -2,000 | 3.22 deg | 9.05 rpm | ok |
| 0.2 s | -30.5 | -1,001 | 1.86 deg | 4.15 rpm | ok |

Both runs hand over at 0.2036 s and remain stable. These closed-loop ramps
are gentler than the Step 0d S2 acceleration estimate, so they are a
confirmation of direction, not a new paper reproduction.

## Limitations

- The paper's load-transition profile is unprinted.
- Fig. 13 has figure-reading uncertainty: candidate speed drops are 190, 235,
  and 350 rpm; all were tested.
- The paper does not state whether the position error is electrical or
  mechanical; the acceleration inference assumes electrical angle.
- V0/V4 are oracle diagnostics; V2 is the nominal sensorless chain.
- The profile generator is experiment-side only; no reusable `src` API was
  added.
