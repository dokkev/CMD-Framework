"""Fingertip contact pads: the generic :class:`Pad` interface and its kinds.

- :class:`Fingertip`: the parametric pad and carrier;
- :class:`RectangularPad`: a plain block;
- :class:`MeshPad`: a triangle mesh, for example from CAD.
"""

from core.morphology.pad.base import Pad
from core.morphology.pad.parameters import (
    FingertipFamilyParameters,
    FingertipParameters,
    PadParameters,
    PhalanxParameters,
    stem_corner_pad_thickness_mm,
)
from core.morphology.pad.model import Fingertip
from core.morphology.pad.rectangular import RectangularPad
from core.morphology.pad.mesh import MeshPad

__all__ = [
    "Fingertip",
    "FingertipFamilyParameters",
    "FingertipParameters",
    "MeshPad",
    "Pad",
    "PadParameters",
    "PhalanxParameters",
    "RectangularPad",
    "stem_corner_pad_thickness_mm",
]
