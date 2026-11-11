# Phase 06 wrap-up — comparison, register, and boundaries

**Status: COMPLETE.** Phases 06a-06f are complete. This document is the release
index and the authoritative summary for the estimator-in-the-loop work. The
source implementation remains frozen; Phase 06 deliverables are documentation,
experiment scripts, and report outputs only.

## Executive verdict

Gate P passes the four paper-shaped scenarios with the nominal chain, and Gate R
is exact: replaying recorded `(u_alpha_beta, i_alpha_beta)` reproduces the
estimator with a maximum difference of zero (within a `1e-9` tolerance). The
steady-state estimates are better than the paper's reported errors, but the
nominal closed-loop load-step errors are larger. Phase 06f isolates the cause:
the observer contributes only 3-5%; the DSOGI-FLL's fixed `Gamma = 50` produces
an approximately `f_n`-independent lag and accounts for the residual factor.

The earlier 06e “flat/no-`1/f_n`” conclusion is superseded. At stable
`alpha_omega = 2-3 Hz`, error falls with `f_n`; the prior flat row and the
176.9 deg S4 result were stability-limit artifacts. No additional bandwidth
experiments are justified unless a printed paper parameter is found to differ
from the configured assumptions.

See the complete metric table in [paper_comparison.md](paper_comparison.md).

## Operational boundaries and limitations

| Boundary | Evidence and consequence |
| --- | --- |
| DSOGI-FLL amplitude-dependent lag | At 500 rpm, back-EMF amplitude is lower and the FLL lag is about 85 rpm in S2 (about 24 rpm in S3); the lag is approximately independent of `f_n`. Raising Q-PLL bandwidth does not remove it. |
| Speed-loop bandwidth stability | The maximum stable `alpha_omega` depends on observer frequency and falls in the tested 06e sweep (S2: 15/10/4.4/7 Hz for `f_n = 20/40/80/160 Hz`). At stable 2-3 Hz, the trend with `f_n` is recovered; near the limit, ringing or synchronism loss dominates. |
| Low-speed loaded operation | 500 rpm 20-50% load steps lose synchronism despite the 100 rpm `min_omega` clamp. The clamp prevents the negative-frequency collapse but does not make the transient controllable. |
| Severe load steps | The 1000 rpm 50% step (S2) passes; the 1000 rpm rated step loses synchronism. The exact boundary lies between 50% and 100% rated load in this model. |
| Voltage saturation | Saturation and FLL guard trips occur during severe transients. Increasing `Gamma` to 500 or 5000 is not a remedy: the `omega' >= 0` guard trips at both tested `f_n` values. |
| Dead time / inverter fidelity | The model is average-value. Dead time causes first-order degradation when enabled; the measured dead-time compensation case returns close to the dead-time-free result, but PWM/switching behavior is not validated. |
| Replay scope | Replay is open-loop on recorded plant trajectories. Closed-loop interaction is covered only by the 06f cross-checks, not by the decomposition itself. |
| Oracle diagnostics | V0 uses the plant's own back-EMF convention and V4 uses true `omega'`; neither is an implementable sensorless result. |
| Normalized Q-PLL | `normalize=True` is a known non-paper defect: it does not reliably lock from a +90 deg initial angle and is excluded from the nominal chain. |
| Paper comparability | Q-PLL gains, normalization algebra, speed PI gains, inverter details and several timing/profile details are unprinted. Results are therefore comparative simulation findings, not an exact reproduction. |

## Release evidence

| Gate | Result | Report/output |
| --- | --- | --- |
| Gate P | PASS for S1-S4 | `experiments/output/06b6_gatep/gatep.json` |
| Gate R | PASS, max replay delta 0.0 | `experiments/output/06f_record/record.json` |
| FLL attribution | V0-V4 decomposition; nominal FLL is limiting element | `experiments/output/06f_replay/replay.json` |
| Closed-loop stability attribution | speed (`omega_hat`) path carries the instability | `experiments/output/06f_closed/closed.json` |

Regenerate or audit these outputs with
[`scripts/regenerate_phase06_reports.py`](../scripts/regenerate_phase06_reports.py).
