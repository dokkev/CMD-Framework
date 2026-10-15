"""Dense post-optimization verification of a Stage-0 candidate.

The optimizer samples the MCP sweep coarsely. Before a candidate is selected,
its safe operating range ``[0, grasp + minimum_post_grasp_travel_rad]`` is
re-swept on a fine grid so a singularity or joint reversal between samples
cannot slip through. Past that range, the first near-singular pose (margin
shortfall or no closure) bounds where a mechanical stopper may go; a q2/q3
reversal there is reported separately because it is not dangerous.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from core.experimental.linkage import FingerLinkageParameters, LinkageMotionEvaluator

from core.experimental.stage0.config import Stage0Config


@dataclass(frozen=True)
class Verification:
    """Fine-grid safety summary of one candidate (angles in radians)."""

    safe_limit_rad: float
    """Grasp MCP plus the required post-grasp travel."""
    closes_to_rad: float
    """Last MCP angle at which both loops still close (up to the horizon)."""
    min_margin_rad: float
    """Smallest transmission margin of either loop over the safe range."""
    max_backoff_rad: float
    """Largest amount q2 or q3 falls back from its running peak over the safe range."""
    first_singular_rad: float
    """First MCP angle past the safe limit with a transmission margin shortfall
    or no closure (the horizon if none). A stopper belongs before it."""
    first_reversal_rad: float
    """First MCP angle past the safe limit where q2 or q3 falls back by more
    than the tolerance (the horizon if none)."""
    passed: bool


def verify_candidate(
    parameters: FingerLinkageParameters,
    config: Stage0Config,
    grasp_mcp_rad: float,
    *,
    step_rad: float = math.radians(0.05),
    backoff_tolerance_rad: float = math.radians(0.1),
) -> Verification:
    """Re-sweep ``[0, horizon]`` finely and check the safe range of one candidate."""

    samples = np.arange(0.0, config.search_horizon_rad + 0.5 * step_rad, step_rad)
    motion = LinkageMotionEvaluator(tuple(float(s) for s in samples)).evaluate(parameters)
    solved = motion.solved_sample_count
    x = samples[:solved]
    margin = np.minimum(
        _margins(motion.transmission_angle_loop_1_rad),
        _margins(motion.transmission_angle_loop_2_rad),
    )
    q2 = np.asarray(motion.q2_flexion_rad)
    q3 = np.asarray(motion.q3_flexion_rad)
    backoff = np.maximum(
        np.maximum.accumulate(q2) - q2,
        np.maximum.accumulate(q3) - q3,
    )

    safe_limit = grasp_mcp_rad + config.minimum_post_grasp_travel_rad
    closes_to = float(x[-1]) if solved else 0.0
    in_safe = x <= safe_limit + 1e-12
    min_margin = float(margin[in_safe].min()) if in_safe.any() else 0.0
    max_backoff = float(backoff[in_safe].max()) if in_safe.any() else math.inf

    threshold = config.criteria.minimum_transmission_margin_rad
    beyond = x > safe_limit

    def first(condition: np.ndarray) -> float:
        hits = np.flatnonzero(condition & beyond)
        return float(x[hits[0]]) if hits.size else closes_to

    return Verification(
        safe_limit_rad=safe_limit,
        closes_to_rad=closes_to,
        min_margin_rad=min_margin,
        max_backoff_rad=max_backoff,
        first_singular_rad=first(margin < threshold),
        first_reversal_rad=first(backoff > backoff_tolerance_rad),
        passed=(
            closes_to >= safe_limit
            and min_margin >= threshold
            and max_backoff <= backoff_tolerance_rad
        ),
    )


def _margins(angles: tuple[float, ...]) -> np.ndarray:
    values = np.asarray(angles)
    return np.minimum(values, math.pi - values)


__all__ = ["Verification", "verify_candidate"]
