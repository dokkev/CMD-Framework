"""Concrete analytic morphology components: what each finger mechanism is
and does (design vector, kinematics, solids, their gaps along the motion),
apart from the requirements a study holds it to (``core.optimization``).

- ``finger``: a finger as its segments, the joint between them and its servo's mount
- ``mechanism``: the finger mechanisms (``linkage``, the CoDe; ``revolute``,
  ``revolute_parallel``, ``rcj``; ``open_chain``, ``rolling_contact_joint``)
- ``packaging``: the common hardware, its rigid bodies in Y layers and their gaps
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
