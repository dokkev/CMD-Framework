"""How any finger mechanism is packaged: the hardware (servos, pin housings,
contact pads, clearances), the planar solids it makes and the gaps between
them, and what of a contact face is hidden. A mechanism places these on its
own links (``core.morphology.linkage.packaging``, ``core.morphology.revolute``, ...).

- ``hardware``: :class:`Packaging` and :class:`ContactPad`
- ``shapes``: polygons, disks, segments, the servo's section (:class:`ServoFootprint`), a pad's outline
- ``gaps``: the gaps as a packaging sets them (overlap depth, the MCP servo over its flexion)
- ``faces``: a contact face hidden in the fold or behind a solid within a probe's depth
- ``assembly``: a finger as rigid bodies of parts in Y layers, the pairs checked
  between them and their margins at the bodies' poses
"""

from core.morphology.packaging.assembly import (
    PALM,
    Assembly,
    Bar,
    Disk,
    Footprint,
    Joint,
    KeepOut,
    Layout,
    Pair,
    Part,
    Polygon,
    Poses,
    margins,
)
from core.morphology.packaging.hardware import BODY_CLEARANCE_MM, PIVOT_CLEARANCE_DIAMETER_MM, ContactPad, Packaging
from core.morphology.packaging.shapes import PadSpan, ServoFootprint, carrier_outline, pad_outline

__all__ = [
    "BODY_CLEARANCE_MM",
    "PALM",
    "PIVOT_CLEARANCE_DIAMETER_MM",
    "Assembly",
    "Bar",
    "ContactPad",
    "Disk",
    "Footprint",
    "Joint",
    "KeepOut",
    "Layout",
    "Packaging",
    "PadSpan",
    "Pair",
    "Part",
    "Polygon",
    "Poses",
    "ServoFootprint",
    "carrier_outline",
    "margins",
    "pad_outline",
]
