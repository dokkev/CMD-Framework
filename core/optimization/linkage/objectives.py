"""The linkage optimization objectives.

A study optimizes the ones its config lists, or else the default ones:
``transmission`` and ``curl`` (named alike in every mechanism, so their fronts
compare). The pads are requirements (minimum or fixed lengths), not an
objective; ``support_length`` (a constant when every pad's length is fixed)
and ``flatness`` are there to opt into.
"""

from __future__ import annotations

from types import MappingProxyType

from core.optimization.mechanism import Objective


OBJECTIVES = MappingProxyType({
    "transmission": Objective(
        value=lambda e: e.torque_ratio_max, column="torque_ratio_max",
        label="worst torque ratio k_max over the usable range", role="best_transmission",
        larger_is_better=False),
    "flatness": Objective(
        value=lambda e: e.torque_ratio_spread, column="torque_ratio_spread",
        label="torque ratio spread k_max / k_min over the usable range", role="best_flatness",
        larger_is_better=False, default=False),
    "support_length": Objective(
        value=lambda e: -e.support_length_mm, column="support_length_mm",
        label="pad length, proximal + distal a-b [mm]", role="best_length",
        larger_is_better=True, default=False),
    "curl": Objective(
        value=lambda e: e.curl_shortfall_deg, column="phi_safe_deg",
        label="phi_safe_max [deg]", role="best_curl", larger_is_better=True),
})
"""Every objective, by the name configs use."""


__all__ = ["OBJECTIVES"]
