# Phase 06 paper-vs-ours comparison

This table is the final comparison of the paper's reported estimation errors with
the Phase 06 simulation results. Values are estimation errors, not the plant's
true-speed dip. The nominal sensorless chain is the matched fixed-sigmoid SMO ->
DSOGI-FLL -> unnormalized Q-PLL (`f_n = 20 Hz`); dead time is OFF unless noted.
The paper does not print enough controller and estimator parameters to support an
exact reproduction claim.

## Headline comparison

| Scenario | Paper result | Encoder baseline (ours) | Sensorless (ours) | Assessment |
| --- | --- | --- | --- | --- |
| S1: no-load 500 -> 1000 rpm ramp | max speed error ~5 rpm; steady position ~0.07 rad; dynamic position ~0.2 rad | steady `|theta_err|` 0.61 deg; dynamic 2.2 deg; true dip -12.4 rpm | steady 0.61 deg / 0.15 rpm; dynamic 2.24 deg / 2.2 rpm; `alpha_w = 2.5 Hz` | **Matches directionally**; ours is lower on the reported position-error scale, but setup differs |
| S2: simulated 50% load step at 1000 rpm | dynamic speed error 23 rpm; steady 5 rpm; dynamic position 0.22 rad (12.6 deg); steady 0.08-0.1 rad | at the design encoder cell (`alpha_w = 50 Hz`): true dip -37.2 rpm; at equal `alpha_w = 10 Hz`: -140.3 rpm | steady 0.56 deg / 0.14 rpm; dynamic 21.7 deg / 111.7 rpm; true dip -210 rpm; `alpha_w = 10 Hz` | **Steady exceeds** paper; **dynamic diverges** (4.9x speed error, 1.7x position error) |
| S3: experimental 20% load step at 1000 rpm | ~10 rpm; position 0.152 rad (8.7 deg) | at `alpha_w = 50 Hz`: true dip -14.9 rpm; at equal `alpha_w = 10 Hz`: -56.1 rpm | steady 0.59 deg / 0.14 rpm; dynamic 7.5 deg / 40.4 rpm; true dip -79.6 rpm; `alpha_w = 10 Hz` | **Steady exceeds** paper; **dynamic diverges** (4.0x speed error, 0.86x position error) |
| S4: steady 1000 rpm | Table IV: 10 rpm; 0.08 rad (4.6 deg) | steady position 0.61 deg; speed 0.14 rpm | steady position 0.61 deg; speed 0.1 rpm | **Exceeds** paper on steady error |

The encoder values are a control baseline, not a paper baseline: the encoder
removes estimator feedback from the speed/angle source but retains the local
controller, plant, voltage limit, integration and delay assumptions. The two
encoder dip values at equal `alpha_w` show why a plant-speed dip must not be
reported as an estimation error.

## Estimator-bandwidth decomposition

The recorded S2/S3 trajectories were replayed open-loop in Phase 06f. The
first-peak metric is `max |omega_hat - omega|` during the first 50 ms after the
load step.

| Scenario | Paper | V0: ideal `E -> Q-PLL` | V1: observer `E_hat -> Q-PLL` | V2: nominal `E_hat -> SOGI+FLL -> Q-PLL` | V4: oracle FLL frequency |
| --- | ---: | ---:|---:|---:|---:|
| S2, `f_n = 20/40/80/160 Hz` | 23 rpm | 87/42/20/10 rpm | 90/44/23/12 rpm | 135/131/130/108 rpm | 82/58/45/36 rpm |
| S3, `f_n = 20/40/80/160 Hz` | 10 rpm | 33/19/8/4 rpm | 34/20/9/7 rpm | 40/42/36/29 rpm | 33/22/17/15 rpm |

V0 follows the frozen acceleration-step model within 20% at every tested
`f_n`. The observer adds only about 3-5% in S2. V2 is approximately flat with
`f_n`, while V4 restores the decreasing trend; V3 (`E_true -> SOGI+FLL`) agrees
with V2. Therefore the discrepancy is attributed to the fixed-gain,
amplitude-dependent FLL, not to the observer or the Q-PLL.

## Interpretation and scope

- The nominal V2 chain never reaches within 2x of the paper's dynamic speed
  errors (best S2 value 108 rpm; best S3 value 29 rpm).
- The ideal V0 reaches within 2x at `f_n >= 80 Hz`, but V0 is an oracle
  diagnostic and is not an implemented sensorless result.
- At stable closed-loop `alpha_w = 2-3 Hz`, the dynamic error decreases with
  `f_n`; the 06e flat result and the 176.9 deg S4 result were near/over
  speed-loop stability limits, not steady Q-PLL costs.
- The normalized Q-PLL is excluded from all headline results: it is a known
  non-paper defect and does not reliably lock.

Source reports: [phase06b.md](phase06b.md), [phase06c2.md](phase06c2.md),
[phase06e.md](phase06e.md), [phase06f.md](phase06f.md), and JSON outputs under
`experiments/output/06{b6,c2,e,f}_*/`.
