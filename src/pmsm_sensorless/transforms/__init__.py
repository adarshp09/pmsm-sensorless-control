"""Clarke and Park (and inverse) transforms.

Phase 01 implements the Clarke transform in :mod:`pmsm_sensorless.transforms.clarke`.
The scaling convention (amplitude- vs. power-invariant) is not fixed by the paper,
so it is exposed as an explicit, configurable argument (default amplitude-invariant).
"""

from __future__ import annotations
