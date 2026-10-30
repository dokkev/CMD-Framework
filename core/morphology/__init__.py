"""Concrete analytic morphology components: what each finger mechanism is
and does (design vector, kinematics, solids, their gaps along the motion),
apart from the requirements a study holds it to (``core.optimization``).

- ``linkage``: the four-bar linkage finger (CoDe)
- ``revolute``, ``revolute_parallel``, ``rcj``: the serial 2R (servo on the
  joint, or through a belt from ``A'``) and the rolling contact joint
- ``open_chain``, ``rolling_contact_joint``: their retention-comparison models
- ``contact``: what of a contact face is lost (``loss``) and the retention modules (``retention``)
- ``icr``: where each mechanism bends when it starts to flex
- ``pad``, ``hand_layout``: the fingertip pads and the hand's layout
"""

from core.morphology.pad import (
    Fingertip,
    FingertipFamilyParameters,
    FingertipParameters,
    MeshPad,
    Pad,
    RectangularPad,
)
from core.morphology.hand_layout import (
    HandLayout,
    HandLayoutParameters,
    HandMorphology,
    HandMorphologyParameters,
    HandPlanePoint,
)

__all__ = [
    "Fingertip",
    "FingertipFamilyParameters",
    "FingertipParameters",
    "HandLayout",
    "HandLayoutParameters",
    "HandMorphology",
    "HandMorphologyParameters",
    "HandPlanePoint",
    "MeshPad",
    "Pad",
    "RectangularPad",
]
