"""Each mechanism's joint view (``visualize.finger.view.JointView``), as
``core.morphology.mechanism`` holds each mechanism's model:

- ``linkage``: the CoDe's crossed four-bar, the PIP servo on ``A'``
- ``revolute``: the serial 2R's pin, its servo on it (or, idealized, none)
- ``revolute_parallel``: the 2R's pin driven through a belt from a servo on ``A'``
- ``rcj``: the rolling contact joint's two rollers and their carrier (``rcj_idealized``: without its servo)
"""

from visualize.finger.mechanism.linkage import FourBarView
from visualize.finger.mechanism.rcj import RollerView
from visualize.finger.mechanism.revolute import PinView
from visualize.finger.mechanism.revolute_parallel import BeltPinView

JOINTS = {"linkage": FourBarView, "revolute": PinView, "revolute_idealized": PinView,
          "revolute_parallel": BeltPinView, "rcj": RollerView, "rcj_idealized": RollerView}
"""The joint view of each study mechanism (``core.optimization.mechanism`` names)."""

__all__ = ["JOINTS", "BeltPinView", "FourBarView", "PinView", "RollerView"]
