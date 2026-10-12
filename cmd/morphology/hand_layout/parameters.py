"""Independent design variables for CLEAR Hand layout and whole-hand design."""

from __future__ import annotations

from dataclasses import dataclass

from ..pad import FingertipFamilyParameters, FingertipParameters
from ..linkage import FingerLinkageParameters
from ..._validation import require_finite, require_positive


@dataclass(frozen=True)
class HandPlanePoint:
    """Independent hand-plane XY position, in millimetres."""

    x_mm: float
    y_mm: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x_mm", require_finite("x_mm", self.x_mm))
        object.__setattr__(self, "y_mm", require_finite("y_mm", self.y_mm))


@dataclass(frozen=True)
class FingertipLengths:
    """Independent total fingertip lengths for each digit, in millimetres."""

    index_mm: float
    middle_mm: float
    ring_mm: float
    little_mm: float
    thumb_mm: float

    def __post_init__(self) -> None:
        for name in ("index_mm", "middle_mm", "ring_mm", "little_mm", "thumb_mm"):
            object.__setattr__(self, name, require_positive(name, getattr(self, name)))


@dataclass(frozen=True)
class HandLayoutParameters:
    """Independent placement variables for the hand reference-frame graph.

    The index MCP-AA is the fixed hand-frame origin. Ordinary MCP-FE positions
    and all downstream thumb positions are derived by transform composition.
    ``theta_rad`` is the positive CMC-RO rotation about ``+Z_H`` from the
    aligned reference axes; ``phi_rad`` is the positive MCP-FE rotation about
    the thumb-frame ``+Z`` axis after the second scalar inter-axis translation.
    """

    middle_mcp_aa: HandPlanePoint
    ring_mcp_aa: HandPlanePoint
    little_mcp_aa: HandPlanePoint
    thumb_cmc_ro: HandPlanePoint
    theta_rad: float
    ro_raa_distance_mm: float
    phi_rad: float

    def __post_init__(self) -> None:
        for name in (
            "middle_mcp_aa",
            "ring_mcp_aa",
            "little_mcp_aa",
            "thumb_cmc_ro",
        ):
            if not isinstance(getattr(self, name), HandPlanePoint):
                raise TypeError(f"{name} must be a HandPlanePoint")
        object.__setattr__(self, "theta_rad", require_finite("theta_rad", self.theta_rad))
        object.__setattr__(
            self,
            "ro_raa_distance_mm",
            require_positive("ro_raa_distance_mm", self.ro_raa_distance_mm),
        )
        object.__setattr__(self, "phi_rad", require_finite("phi_rad", self.phi_rad))


@dataclass(frozen=True)
class HandMorphologyParameters:
    """Optimizer-facing aggregate of shared fingertip, linkage, and layout.

    Each ordinary finger shares one linkage definition and every digit shares
    one fingertip family; only the five total fingertip lengths differ.
    """

    fingertip: FingertipFamilyParameters
    fingertip_lengths: FingertipLengths
    linkage: FingerLinkageParameters
    layout: HandLayoutParameters

    def __post_init__(self) -> None:
        if not isinstance(self.fingertip, FingertipFamilyParameters):
            raise TypeError("fingertip must be a FingertipFamilyParameters instance")
        if not isinstance(self.fingertip_lengths, FingertipLengths):
            raise TypeError("fingertip_lengths must be a FingertipLengths instance")
        if not isinstance(self.linkage, FingerLinkageParameters):
            raise TypeError("linkage must be a FingerLinkageParameters instance")
        if not isinstance(self.layout, HandLayoutParameters):
            raise TypeError("layout must be a HandLayoutParameters instance")

        # The fingertip rotation axis sits bc_length_mm from the tip, so it
        # must fall within every carrier.
        bc_mm = self.linkage.bc_length_mm
        if bc_mm <= self.fingertip.carrier_distal_inset_mm:
            raise ValueError(
                "linkage.bc_length_mm must exceed fingertip.carrier_distal_inset_mm "
                "so the rotation axis lies within the carrier"
            )
        for name in ("index", "middle", "ring", "little", "thumb"):
            length_mm = getattr(self.fingertip_lengths, f"{name}_mm")
            FingertipParameters(family=self.fingertip, total_length_mm=length_mm)
            if length_mm <= bc_mm:
                raise ValueError(
                    f"fingertip_lengths.{name}_mm must exceed linkage.bc_length_mm"
                )


__all__ = [
    "FingertipLengths",
    "HandLayoutParameters",
    "HandMorphologyParameters",
    "HandPlanePoint",
]
