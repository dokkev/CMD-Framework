"""Concrete analytic morphology components: what each finger mechanism is
and does (design vector, kinematics, solids, their gaps along the motion),
apart from the requirements a study holds it to (``core.optimization``).

- ``finger``: a finger as its segments, the joint between them and its servo's mount
- ``code_hand``: the five-digit 3D CoDe Hand assembly, frames and forward kinematics
- ``mechanism``: the finger mechanisms (``linkage``, the CoDe; ``revolute``,
  ``revolute_parallel``, ``rcj``)
- ``packaging``: the common hardware, its rigid bodies in Y layers and their gaps
- ``contact``: what of a contact face is lost (``loss``)
- ``icr``: where each mechanism bends when it starts to flex
"""

from core.morphology.finger import (
    Finger,
    Joint,
    Kinematics,
    PointXZ,
    Segment,
    ServoMount,
)

__all__ = [
    "Finger",
    "Joint",
    "Kinematics",
    "PointXZ",
    "Segment",
    "ServoMount",
]
