"""A linkage posed by its servo: the sweep the figures sample, and the
checked pairs' gaps along it."""

from __future__ import annotations

import math

import numpy as np

from core.morphology.linkage.packaging import pad_margins

SERVO_STEP_DEG = 0.05


def servo_sweep(linkage, step_deg: float = SERVO_STEP_DEG):
    """The servo angles from extension (turning the way the finger flexes, as
    the evaluator does) and the linkage's sweep over them."""

    step = math.radians(step_deg)
    sense = 1.0 if linkage.sweep_servo([0.0, step]).q_pip_rad[-1] >= 0.0 else -1.0
    servo = sense * np.arange(0.0, math.pi, step)
    return servo, linkage.sweep_servo(servo)


def index_at(sweep, phi_rad):
    """The first sample of ``sweep`` whose distal flexion reaches ``phi_rad``
    (the last, past its reach)."""

    reached = np.maximum.accumulate(sweep.total_flexion_rad)
    index = np.searchsorted(reached, np.asarray(phi_rad, dtype=float) - 1e-9)
    return np.minimum(index, len(reached) - 1)


def collisions(margins: dict, index: int) -> set[str]:
    """The parts of every pair whose gap is negative at ``index``."""

    return {name for pair, margin in margins.items() if margin[index] < 0.0 for name in pair}


def linkage_margins(linkage, packaging, step_deg: float):
    """Every checked pair's gap from 0 to 180 deg of flexion in ``step_deg``
    (to the linkage's reach): the flexions reached, the gaps, the sweep and
    the index of each flexion in it."""

    _, sweep = servo_sweep(linkage)
    index = index_at(sweep, np.radians(np.arange(0.0, 180.0 + 1e-9, step_deg)))
    phi = np.degrees(sweep.total_flexion_rad[index])
    margins = pad_margins(linkage, packaging, sweep.b[index], sweep.c[index], sweep.bp[index], (0.0,))
    return phi, margins, sweep, index
