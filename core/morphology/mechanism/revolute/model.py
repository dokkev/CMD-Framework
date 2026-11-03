"""The serial 2R's joint: a pin at ``A`` between the proximal and distal
segments, and the finger it makes from a design vector.

The distal segment's frame is on the pin and turns about it by the flexion
(positive turns ``+X`` toward ``-Z``), so its centre of rotation is ``A`` at
every pose; the servo on the pin (or, the belt 2R, through a belt) turns it
directly, its angle the flexion's. The finger
(``core.morphology.finger.Finger``) places the fingertip and its segments'
ends on the frames the pin poses, as it does on any joint.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from core._validation import require_finite, require_positive
from core.morphology.finger import Finger, PointXZ, Segment, ServoMount


@dataclass(frozen=True)
class PinPose:
    """The pin at one flexion, in the proximal frame (the MCP at ``q_mcp_rad``)."""

    q_mcp_rad: float
    q_pip_rad: float
    a: PointXZ
    distal_rad: float
    """The distal segment's rotation relative to the proximal one (``-q_pip``)."""
    tip: PointXZ | None = None
    proximal_end: PointXZ | None = None
    distal_start: PointXZ | None = None
    """(``tip``, ``proximal_end``, ``distal_start``: the finger's points, placed by the finger.)"""

    @property
    def distal_origin(self) -> PointXZ:
        """The distal segment's frame origin: the pin ``A``."""

        return self.a

    @property
    def icr(self) -> PointXZ:
        """The distal segment's centre of rotation relative to the proximal one: the pin."""

        return self.a

    def turned(self, q_mcp_rad: float) -> "PinPose":
        """This pose with the whole finger turned about ``O`` by the MCP flexion."""

        def turn(point: PointXZ | None) -> PointXZ | None:
            return None if point is None else point.rotated(-q_mcp_rad)

        return dataclasses.replace(self, q_mcp_rad=q_mcp_rad, **{
            name: turn(getattr(self, name)) for name in ("a", "tip", "proximal_end", "distal_start")})


@dataclass(frozen=True)
class PinSweep:
    """The pin over a sequence of flexions, in the proximal frame; points ``(n, 2)``."""

    q_pip_rad: np.ndarray
    distal_origin: np.ndarray
    """The pin ``A`` at each sample."""
    distal_rad: np.ndarray
    servo_rad: np.ndarray
    """The servo's rotation: the flexion's."""
    tip: np.ndarray | None = None
    """The fingertip, placed by the finger."""

    @property
    def icr(self) -> np.ndarray:
        """The centre of rotation at each sample: the pin."""

        return self.distal_origin

    @property
    def total_flexion_rad(self) -> np.ndarray:
        """The distal segment's flexion relative to the proximal one."""

        return self.q_pip_rad


@dataclass(frozen=True)
class Pin:
    """The 2R's joint: a pin at ``A`` on the proximal segment's line."""

    a: PointXZ

    def __post_init__(self) -> None:
        if not isinstance(self.a, PointXZ):
            raise TypeError("a must be a PointXZ instance")
        require_positive("a.x_mm", self.a.x_mm)

    @property
    def distal_origin(self) -> PointXZ:
        """The distal segment's frame origin in the extended pose: the pin."""

        return self.a

    @property
    def icr(self) -> PointXZ:
        """The extended pose's centre of rotation: the pin."""

        return self.a

    def check(self) -> None:
        """A pin poses at every flexion."""

    def solve(self, q_pip_rad: float) -> PinPose:
        """The pin at a flexion, in the proximal frame (the MCP at 0)."""

        q_pip_rad = require_finite("q_pip_rad", q_pip_rad)
        return PinPose(q_mcp_rad=0.0, q_pip_rad=q_pip_rad, a=self.a, distal_rad=-q_pip_rad)

    def sweep_pip(self, q_pip_rad: Sequence[float]) -> PinSweep:
        """The pin at a sequence of flexions from extension."""

        q = np.asarray(q_pip_rad, dtype=float)
        return PinSweep(q_pip_rad=q, distal_origin=np.broadcast_to(self.a.array, (len(q), 2)).copy(),
                        distal_rad=-q, servo_rad=q)

    def sweep_servo(self, servo_rad: Sequence[float]) -> PinSweep:
        """The pin at a sequence of its servo's angles: the flexions themselves."""

        return self.sweep_pip(servo_rad)


def make_finger(a_x_mm: float, ac_length_mm: float, app_x_mm: float, bpp_from_a_mm: float,
                tilt_rad: float = 0.0) -> Finger:
    """The 2R finger: its proximal segment ``O`` to ``A''``, the pin at
    ``A = (a_x_mm, 0)``, its distal segment ``B''`` (``bpp_from_a_mm`` past
    ``A``) to ``C`` (``ac_length_mm``), the servo tilted dorsally by ``tilt_rad``."""

    require_positive("ac_length_mm", ac_length_mm)
    return Finger(
        proximal=Segment(0.0, app_x_mm),
        distal=Segment(bpp_from_a_mm, ac_length_mm),
        joint=Pin(PointXZ(a_x_mm, 0.0)),
        servo=ServoMount(tilt_rad),
    )


def decode(x: Sequence[float]) -> Finger:
    """The finger of a 2R design vector (``solids.VARIABLE_NAMES``; its pads'
    drops, the 2R's own packaging, are not the finger's)."""

    a_x, ac, app_x, bpp_from_a, _, _, tilt_deg = (float(value) for value in x)
    return make_finger(a_x, ac, app_x, bpp_from_a, math.radians(tilt_deg))


__all__ = ["Pin", "PinPose", "PinSweep", "decode", "make_finger"]
