"""Named initial designs for CLEAR linkage studies."""

from .linkage import FingerLinkage
from .linkage_parameter import (
    AnchorYZ,
    CouplingAnchorParameters,
    FingerLinkageParameters,
    MainChainParameters,
)


# Body proportions from the MotionGen prototype. Coupler lengths are derived
# by FingerLinkage from this design's fully extended q1=q2=q3=0 reference.
BASELINE_LINKAGE_V0 = FingerLinkage(
    FingerLinkageParameters(
        main_chain=MainChainParameters(
            proximal_length_mm=35.36,
            middle_length_mm=25.50,
            distal_length_mm=20.00,
        ),
        coupling=CouplingAnchorParameters(
            base_anchor=AnchorYZ(10.00, 10.00),
            proximal_anchor=AnchorYZ(29.70, -4.24),
            middle_anchor=AnchorYZ(5.88, -3.92),
            distal_anchor=AnchorYZ(-10.00, 5.00),
        ),
    )
)


__all__ = ["BASELINE_LINKAGE_V0"]
