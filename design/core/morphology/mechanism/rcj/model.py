"""The symmetric 1:1 rolling-contact joint (RCJ) as a :class:`~core.morphology.finger.Joint`.

Two equal-radius rollers (``r > 0``) roll without slip: the proximal roller is
fixed on the proximal segment at ``A = (a_x, a_z)``, the distal roller is fixed
on the distal segment at ``B``, and the rigid carrier ``A-B`` has length ``2r``.
When the carrier turns by ``theta_c = phi / 2`` toward ``-Z``, the distal
segment turns by ``phi`` relative to the proximal one (constant transmission
ratio ``k = d(phi)/d(theta_c) = 2``), and the instantaneous rolling contact
point lies at ``A + r (cos(phi/2), -sin(phi/2))``.
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
class RollingPose:
    """The symmetric RCJ solved at one distal flexion angle."""

    q_mcp_rad: float
    q_pip_rad: float
    a: PointXZ
    b: PointXZ
    icr: PointXZ
    distal_rad: float
    """Active rotation of the distal frame relative to the proximal one (``-q_pip_rad``)."""
    carrier_rad: float
    """Flexion rotation of the carrier ``A-B`` (``q_pip_rad / 2``)."""
    tip: PointXZ | None = None
    proximal_end: PointXZ | None = None
    distal_start: PointXZ | None = None

    @property
    def distal_origin(self) -> PointXZ:
        """The distal segment's frame origin: the distal roller centre ``B``."""

        return self.b

    def turned(self, q_mcp_rad: float) -> RollingPose:
        """This pose with the whole finger turned about ``O`` by ``q_mcp_rad``."""

        def turn(point: PointXZ | None) -> PointXZ | None:
            return None if point is None else point.rotated(-q_mcp_rad)

        return dataclasses.replace(
            self,
            q_mcp_rad=q_mcp_rad,
            **{name: turn(getattr(self, name)) for name in ("a", "b", "icr", "tip", "proximal_end", "distal_start")},
        )


@dataclass(frozen=True)
class RollingSweep:
    """The symmetric RCJ solved over a sequence of flexion samples."""

    q_pip_rad: np.ndarray
    distal_origin: np.ndarray
    """The distal roller centre ``B`` at each sample ``(n, 2)``."""
    icr: np.ndarray
    """The rolling contact point at each sample ``(n, 2)``."""
    distal_rad: np.ndarray
    servo_rad: np.ndarray
    """The carrier / actuator shaft angle ``q_pip_rad / 2``."""
    tip: np.ndarray | None = None

    @property
    def b(self) -> np.ndarray:
        return self.distal_origin

    @property
    def total_flexion_rad(self) -> np.ndarray:
        return self.q_pip_rad


@dataclass(frozen=True)
class RollingJoint:
    """Symmetric 1:1 no-slip rolling contact joint with proximal roller at ``a`` and radius ``radius_mm``."""

    a: PointXZ
    radius_mm: float

    def __post_init__(self) -> None:
        if not isinstance(self.a, PointXZ):
            raise TypeError("a must be a PointXZ instance")
        require_positive("a.x_mm", self.a.x_mm)
        require_positive("radius_mm", self.radius_mm)

    @property
    def distal_origin(self) -> PointXZ:
        """Extended-pose distal roller centre ``B(0) = A + (2r, 0)``."""

        return PointXZ(self.a.x_mm + 2.0 * self.radius_mm, self.a.z_mm)

    @property
    def icr(self) -> PointXZ:
        """Extended-pose rolling contact point ``ICR(0) = A + (r, 0)``."""

        return PointXZ(self.a.x_mm + self.radius_mm, self.a.z_mm)

    def check(self) -> None:
        """Ideal no-slip equal rollers close at every flexion angle."""

    def solve(self, q_pip_rad: float) -> RollingPose:
        phi = require_finite("q_pip_rad", q_pip_rad)
        half = 0.5 * phi
        c, s = math.cos(half), -math.sin(half)
        r = self.radius_mm
        b = PointXZ(self.a.x_mm + 2.0 * r * c, self.a.z_mm + 2.0 * r * s)
        icr = PointXZ(self.a.x_mm + r * c, self.a.z_mm + r * s)
        return RollingPose(
            q_mcp_rad=0.0,
            q_pip_rad=phi,
            a=self.a,
            b=b,
            icr=icr,
            distal_rad=-phi,
            carrier_rad=half,
        )

    def sweep_pip(self, q_pip_rad: Sequence[float]) -> RollingSweep:
        phi = np.asarray(q_pip_rad, dtype=float)
        half = 0.5 * phi
        direction = np.stack((np.cos(half), -np.sin(half)), axis=1)
        a = self.a.array
        r = self.radius_mm
        return RollingSweep(
            q_pip_rad=phi,
            distal_origin=a + 2.0 * r * direction,
            icr=a + r * direction,
            distal_rad=-phi,
            servo_rad=half,
        )

    def sweep_servo(self, servo_rad: Sequence[float]) -> RollingSweep:
        theta = np.asarray(servo_rad, dtype=float)
        return self.sweep_pip(2.0 * theta)


def make_finger(
    a_x_mm: float,
    roller_radius_mm: float,
    bc_length_mm: float,
    app_x_mm: float,
    bpp_from_b_mm: float,
    tilt_rad: float = 0.0,
    pad_offsets_mm: tuple[float, float] = (0.0, 0.0),
) -> Finger:
    """Construct an RCJ :class:`~core.morphology.finger.Finger`."""

    require_positive("bc_length_mm", bc_length_mm)
    return Finger(
        proximal=Segment(0.0, app_x_mm),
        distal=Segment(bpp_from_b_mm, bc_length_mm),
        joint=RollingJoint(PointXZ(a_x_mm, 0.0), roller_radius_mm),
        servo=ServoMount(tilt_rad),
        pad_offsets_mm=(float(pad_offsets_mm[0]), float(pad_offsets_mm[1])),
    )


def decode(x: Sequence[float]) -> Finger:
    """Decode an RCJ design vector (``rcj.solids.VARIABLE_NAMES``) to a :class:`Finger`."""

    values = [float(v) for v in x]
    if len(values) == 7:
        values.append(0.0)
    if len(values) != 8:
        raise ValueError(f"RCJ design vector has {len(values)} values; expected 8")
    a_x, r, bc, app_x, bpp_from_b, distal_drop, proximal_drop, tilt_deg = values
    return make_finger(
        a_x,
        r,
        bc,
        app_x,
        bpp_from_b,
        math.radians(tilt_deg),
        (proximal_drop, distal_drop),
    )


__all__ = ["RollingJoint", "RollingPose", "RollingSweep", "decode", "make_finger"]
