"""A finger as a chain of rigid segments joined by joints, whatever the joints are.

Each segment has its own frame: its origin on the joint at its proximal end
(the proximal segment's: the MCP, :attr:`Finger.origin`), ``+X`` along the
segment, ``+Z`` dorsal. A segment spans ``start_mm`` to ``end_mm`` along its
axis: its body as far as it reaches, whatever its joint's parts (where its
contact interface sits is the packaging's). A joint (:class:`Joint`, e.g. the
CoDe's four-bar ``core.morphology.mechanism.linkage.FourBar`` or the 2R's
pin ``core.morphology.mechanism.revolute.Pin``) poses the distal segment's
frame on the proximal one; a servo mount (:class:`ServoMount`) says how the
actuator driving it sits on its segment. A two-joint finger (MCP and PIP)
has no middle segment.

The finger owns its joint and poses itself through it (:meth:`Finger.solve`,
:meth:`Finger.sweep_pip`, :meth:`Finger.sweep_servo`): the joint closes its
mechanism and poses the distal segment's frame (its origin and rotation);
the finger places its segments' points on that frame (the fingertip
:attr:`Finger.tip`, the distal segment's start :attr:`Finger.distal_start`)
and turns the whole finger about the MCP. The PIP angle is the distal
segment's flexion relative to the proximal one; extended, every segment's
axis is the proximal one's ``+X``. :meth:`Finger.kinematics` gives the motion
(:class:`Kinematics`: flexion, servo angle, frame origin, centre of
rotation, fingertip, the segments' ends, transmission ratio).
"""
from __future__ import annotations

import math
import dataclasses
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from core._validation import require_finite


@dataclass(frozen=True)
class PointXZ:
    """A point in the finger's plane (+X distal, +Z dorsal), in mm."""

    x_mm: float
    z_mm: float

    def __post_init__(self) -> None:
        require_finite("x_mm", self.x_mm)
        require_finite("z_mm", self.z_mm)

    def distance_to(self, other: "PointXZ") -> float:
        """Return the planar Euclidean distance to another point."""

        return math.hypot(self.x_mm - other.x_mm, self.z_mm - other.z_mm)

    def __add__(self, other: "PointXZ") -> "PointXZ":
        return PointXZ(self.x_mm + other.x_mm, self.z_mm + other.z_mm)

    def __sub__(self, other: "PointXZ") -> "PointXZ":
        return PointXZ(self.x_mm - other.x_mm, self.z_mm - other.z_mm)

    def scaled(self, factor: float) -> "PointXZ":
        return PointXZ(self.x_mm * factor, self.z_mm * factor)

    def rotated(self, angle_rad: float) -> "PointXZ":
        """Turned by ``angle_rad`` about the origin (counter-clockwise in XZ)."""

        cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
        return PointXZ(cosine * self.x_mm - sine * self.z_mm, sine * self.x_mm + cosine * self.z_mm)

    @property
    def array(self) -> np.ndarray:
        return np.array((self.x_mm, self.z_mm))


def turned(vectors: np.ndarray, angle_rad) -> np.ndarray:
    """``(2,)`` or ``(n, 2)`` vectors turned by angles (counter-clockwise in XZ)."""

    angle = np.asarray(angle_rad, dtype=float)
    cosine, sine = np.cos(angle), np.sin(angle)
    x, z = vectors[..., 0], vectors[..., 1]
    return np.stack((cosine * x - sine * z, sine * x + cosine * z), axis=-1)


@dataclass(frozen=True)
class Segment:
    """A rigid segment along its own axis, from its proximal joint's pin."""

    start_mm: float
    """Where the body starts along the axis."""
    end_mm: float
    """Where it ends (the distal segment's end: the fingertip)."""

    def __post_init__(self) -> None:
        require_finite("start_mm", self.start_mm)
        require_finite("end_mm", self.end_mm)

    @property
    def length_mm(self) -> float:
        """The segment's length: as far as its body reaches (``start`` to ``end``)."""

        return self.end_mm - self.start_mm


@dataclass(frozen=True)
class ServoMount:
    """How the servo driving a joint sits on its segment: its body laid back
    from the shaft along ``-X``, tilted dorsally by ``tilt_rad``."""

    tilt_rad: float = 0.0

    def __post_init__(self) -> None:
        require_finite("tilt_rad", self.tilt_rad)

    @property
    def tilt_deg(self) -> float:
        return math.degrees(self.tilt_rad)


class Joint(Protocol):
    """A mechanism joining the proximal segment to the distal one. It knows
    its own parts, not the segments': it closes itself and poses the distal
    segment's frame, in the proximal frame (the MCP at 0)."""

    @property
    def distal_origin(self) -> PointXZ:
        """The distal segment's frame origin in the extended pose (its axis there: ``+X``)."""

    @property
    def icr(self) -> PointXZ | None:
        """The extended pose's instantaneous centre of rotation of the distal
        segment relative to the proximal one (``None``: none finite)."""

    def check(self) -> None:
        """``ValueError`` when the joint cannot be posed from its reference."""

    def solve(self, q_pip_rad: float) -> Any:
        """The joint at a PIP angle (the distal segment's flexion relative to the proximal one): a pose with ``distal_origin``, ``distal_rad``,
        its centre of rotation ``icr``, the finger's points ``tip``,
        ``proximal_end``, ``distal_start`` left to fill, and ``turned``."""

    def sweep_pip(self, q_pip_rad: Sequence[float]) -> Any:
        """The joint at a sequence of PIP angles from extension: a sweep with
        ``distal_origin``, ``distal_rad``, ``icr`` (``(n, 2)``, NaN: none
        finite), ``servo_rad`` and ``q_pip_rad`` per sample (up to the
        first it cannot pose) and ``tip`` left to fill."""

    def sweep_servo(self, servo_rad: Sequence[float]) -> Any:
        """As :meth:`sweep_pip`, at a sequence of the joint's servo angles."""


@dataclass(frozen=True)
class Kinematics:
    """The finger over a motion, whatever its joint: per sample, ``(n,)`` or
    ``(n, 2)`` in the proximal frame turned by the MCP (``O`` at the origin)."""

    q_mcp_rad: float
    flexion_rad: np.ndarray
    """The distal segment's flexion relative to the proximal one."""
    servo_rad: np.ndarray
    """The joint's servo angle from extension."""
    distal_origin: np.ndarray
    """The distal segment's frame origin (the joint's)."""
    icr: np.ndarray
    """The distal segment's instantaneous centre of rotation relative to the
    proximal one (NaN: none finite)."""
    tip: np.ndarray
    """The fingertip: where the distal segment ends."""
    distal_start: np.ndarray
    """Where the distal segment's body starts."""
    proximal_end: np.ndarray
    """Where the proximal segment's body ends (``(2,)``: fixed on it)."""

    @property
    def transmission_ratio(self) -> np.ndarray:
        """``d(flexion)/d(servo)`` at each sample (``k``: the servo torque per
        unit of distal moment held), by finite differences (NaN: one sample)."""

        if len(self.flexion_rad) < 2:
            return np.full(len(self.flexion_rad), math.nan)
        return np.gradient(self.flexion_rad, self.servo_rad)


@dataclass(frozen=True)
class Finger:
    """The segments, the joint between them and its servo's mount."""

    proximal: Segment
    distal: Segment
    joint: Joint
    """The mechanism joining the proximal segment to the next."""
    servo: ServoMount = field(default_factory=ServoMount)
    middle: Segment | None = None
    """Between the proximal and distal segments, for a three-joint finger (``None``: two joints)."""
    pad_offsets_mm: tuple[float, float] = (0.0, 0.0)
    """How far palmar of the proximal and the distal segment's line each
    contact interface's mounting surface lies (``d``, 0 or more; the module's
    thickness runs palmar of it): the segment's line the one datum, the
    offset a design choice (2026-11-04)."""

    def __post_init__(self) -> None:
        for name in ("proximal", "distal"):
            if not isinstance(getattr(self, name), Segment):
                raise TypeError(f"{name} must be a Segment")
        if self.middle is not None:
            raise NotImplementedError("a three-joint finger (a middle segment) is not modelled yet")
        if not isinstance(self.servo, ServoMount):
            raise TypeError("servo must be a ServoMount")
        if len(self.pad_offsets_mm) != 2 or not all(math.isfinite(d) and d >= 0.0 for d in self.pad_offsets_mm):
            raise ValueError("pad_offsets_mm must be two offsets, 0 or more")

    # The segments' points in the extended pose, in the proximal frame.

    @property
    def origin(self) -> PointXZ:
        """The MCP: the proximal frame's origin."""

        return PointXZ(0.0, 0.0)

    @property
    def tip(self) -> PointXZ:
        """The fingertip: where the distal segment ends."""

        origin = self.joint.distal_origin
        return PointXZ(origin.x_mm + self.distal.end_mm, origin.z_mm)

    @property
    def proximal_end(self) -> PointXZ:
        """Where the proximal segment's body ends, on its line (the ``x`` axis)."""

        return PointXZ(self.proximal.end_mm, 0.0)

    @property
    def distal_start(self) -> PointXZ:
        """Where the distal segment's body starts, on its line."""

        origin = self.joint.distal_origin
        return PointXZ(origin.x_mm + self.distal.start_mm, origin.z_mm)

    @property
    def icr(self) -> PointXZ | None:
        """The distal segment's centre of rotation when it starts to flex (the
        joint's; posed, each pose and sweep sample carries its own ``icr``)."""

        return self.joint.icr

    # Posed through the joint.

    def solve(self, q_mcp_rad: float, q_pip_rad: float) -> Any:
        """The pose at the MCP and PIP angles (flexion positive, from extension;
        the PIP: the distal segment's flexion relative to the proximal one)."""

        pose = self.joint.solve(q_pip_rad)
        origin = pose.distal_origin
        tip = origin + (self.tip - self.joint.distal_origin).rotated(pose.distal_rad)
        # The distal start rides on the distal segment: the same share of the posed origin -> tip.
        start = origin + (tip - origin).scaled(self.distal.start_mm / self.distal.end_mm)
        return dataclasses.replace(pose, tip=tip, proximal_end=self.proximal_end, distal_start=start).turned(q_mcp_rad)

    def sweep_pip(self, q_pip_rad: Sequence[float]) -> Any:
        """The finger at a sequence of PIP angles from extension (MCP at 0): the
        distal segment's flexion relative to the proximal one, whatever the joint."""

        return self.place(self.joint.sweep_pip(q_pip_rad))

    def sweep_servo(self, servo_rad: Sequence[float]) -> Any:
        """The finger at a sequence of the joint's servo angles from extension (MCP at 0)."""

        return self.place(self.joint.sweep_servo(servo_rad))

    def kinematics(self, q_pip_rad: Sequence[float], q_mcp_rad: float = 0.0) -> Kinematics:
        """The finger at a sequence of PIP angles from extension (as far as
        the joint poses), the MCP at ``q_mcp_rad``."""

        sweep = self.sweep_pip(q_pip_rad)
        origin = np.asarray(sweep.distal_origin, dtype=float)
        start = origin + (sweep.tip - origin) * (self.distal.start_mm / self.distal.end_mm)

        def turn(points: np.ndarray) -> np.ndarray:
            return turned(np.asarray(points, dtype=float), -q_mcp_rad)

        return Kinematics(q_mcp_rad=q_mcp_rad, flexion_rad=np.asarray(sweep.q_pip_rad, dtype=float),
                          servo_rad=np.asarray(sweep.servo_rad, dtype=float), distal_origin=turn(origin),
                          icr=turn(sweep.icr), tip=turn(sweep.tip), distal_start=turn(start),
                          proximal_end=turn(self.proximal_end.array))

    def place(self, sweep: Any) -> Any:
        """A sweep of the joint (by whatever it is driven) with the fingertip on its posed distal frame."""

        tip = self.tip.array - self.joint.distal_origin.array
        return dataclasses.replace(sweep, tip=sweep.distal_origin + turned(tip, sweep.distal_rad))


__all__ = ["Finger", "Joint", "Kinematics", "PointXZ", "Segment", "ServoMount", "turned"]
