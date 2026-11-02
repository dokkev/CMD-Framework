"""Geometry and closed-form kinematics of the 2-DoF polycentric finger.

The MCP servo (``O``) turns the whole finger; the PIP servo (``A'``) turns the
driven link ``A'-B`` of the four-bar ``A-B'-B-A'`` on the proximal link, which
turns the driver link ``A-B'`` about ``A`` and the distal segment about ``B'``
together. Flexion angles, servo angles included, are positive and turn
``+X`` toward ``-Z``.

The finger's PIP angle ``q_pip`` is the distal segment's flexion relative to
the proximal one (``O-A`` to ``B-C``, ``q_driver + q_dip``); the four-bar's
own coordinates are the driver link's rotation about ``A``, ``q_driver``, and
the distal segment's relative to it about ``B'``, ``q_dip``. The joint is
solved by ``q_pip`` (:meth:`FourBar.solve`, :meth:`FourBar.sweep_pip`), by
the driver (:meth:`FourBar.solve_driver`, :meth:`FourBar.sweep_driver`) or
by the servo (:meth:`FourBar.sweep_servo`).

The finger (``core.morphology.finger.Finger``) owns its joint, the four-bar
(:class:`FourBar`), and poses itself through it: the proximal segment from
``O`` to ``A''``, the distal one from ``B''`` to the fingertip ``C``, its
frame on ``B``. ``A''`` and ``B''`` are where the segments' bodies end, not
joints.
"""

from __future__ import annotations

import dataclasses
import functools
import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from core._validation import require_finite, require_positive
from core.morphology.finger import PointXZ, turned


_TOLERANCE = 1e-10


class KinematicClosureError(ValueError):
    """Raised when the four-bar cannot close at a requested pose."""


@dataclass(frozen=True)
class FingerPose:
    """One solved pose; joint angles are flexion from the reference pose."""

    q_mcp_rad: float
    q_pip_rad: float
    """The distal segment's flexion relative to the proximal one (``O-A`` to ``B-C``)."""
    q_driver_rad: float
    """The driver link ``A-B'``'s rotation about ``A``."""
    q_dip_rad: float
    """The distal segment's rotation about ``B'`` relative to the driver link."""
    o: PointXZ
    a: PointXZ
    ap: PointXZ
    bp: PointXZ
    b: PointXZ
    distal_rad: float
    """The distal segment's rotation relative to the proximal one (``q_pip + q_dip``)."""
    icr: PointXZ | None
    """The distal segment's instantaneous centre of rotation relative to the
    proximal one: where the bars ``A-B'`` and ``A'-B`` cross, the remote
    centre the joint turns it about (``None``: the bars parallel)."""
    tip: PointXZ | None = None
    """``C``, the fingertip."""
    proximal_end: PointXZ | None = None
    """``A''``, where the proximal segment's body (its support) ends."""
    distal_start: PointXZ | None = None
    """``B''``, where the distal segment's body (its support) starts.
    (``tip``, ``proximal_end``, ``distal_start``: the finger's points, placed by the finger.)"""

    @property
    def distal_origin(self) -> PointXZ:
        """The distal segment's frame origin: the pin ``B``."""

        return self.b

    def turned(self, q_mcp_rad: float) -> "FingerPose":
        """This pose with the whole finger turned about ``O`` by the MCP flexion."""

        def turn(point: PointXZ | None) -> PointXZ | None:
            return None if point is None else point.rotated(-q_mcp_rad)

        return dataclasses.replace(self, q_mcp_rad=q_mcp_rad, **{
            name: turn(getattr(self, name)) for name in ("a", "ap", "bp", "b", "icr", "tip", "proximal_end", "distal_start")})

    @property
    def is_crossed(self) -> bool:
        """Whether the driver link ``A-B'`` and the coupler ``A'-B`` intersect."""

        return _segments_intersect(self.a, self.bp, self.ap, self.b)

    @property
    def margin_at_b_rad(self) -> float:
        """Margin from 0 or pi of the angle at ``B`` between ``B->A'`` and ``B->B'``.

        It is the transmission margin when the PIP servo drives ``A-B'``.
        """

        return _margin(_angle_between(self.ap - self.b, self.bp - self.b))

    @property
    def margin_at_bp_rad(self) -> float:
        """Margin from 0 or pi of the angle at ``B'`` between ``B'->A`` and ``B'->B``.

        It is the transmission margin when the PIP servo drives ``A'-B``.
        """

        return _margin(_angle_between(self.a - self.bp, self.b - self.bp))


@dataclass(frozen=True)
class PipSweep:
    """The four-bar solved over a sequence of poses, in the proximal-link frame.

    Arrays cover the samples up to the first one that fails to close; angles
    are unwrapped along the sweep. ``servo_rad`` is the driven link's rotation
    from the reference pose. Points are ``(n, 2)`` XZ arrays.
    """

    q_pip_rad: np.ndarray
    """The finger's PIP: the distal segment's flexion relative to the proximal one."""
    q_driver_rad: np.ndarray
    q_dip_rad: np.ndarray
    bp: np.ndarray
    b: np.ndarray
    distal_rad: np.ndarray
    """The distal segment's rotation relative to the proximal one at each sample."""
    margin_at_b_rad: np.ndarray
    margin_at_bp_rad: np.ndarray
    servo_rad: np.ndarray
    icr: np.ndarray
    """The distal segment's instantaneous centre of rotation at each sample
    (see :attr:`FingerPose.icr`; NaN: the bars parallel)."""
    tip: np.ndarray | None = None
    """The fingertip ``C``, placed by the finger."""

    @property
    def distal_origin(self) -> np.ndarray:
        """The distal segment's frame origin at each sample: the pin ``B``."""

        return self.b

    @property
    def total_flexion_rad(self) -> np.ndarray:
        """Distal-link rotation relative to the proximal link: the finger's ``q_pip``."""

        return self.q_pip_rad


@dataclass(frozen=True)
class FourBar:
    """The CoDe's joint: the crossed four-bar ``A-B'-B-A'`` between the
    proximal and distal segments, and its kinematics.

    The four points in the extended (reference) pose, in the proximal
    segment's frame (``O`` at the origin, ``+X`` along it): ``A`` (PIP) and
    ``A'`` on the proximal segment, ``B`` and ``B'`` (DIP) on the distal one,
    ``B`` its frame's origin (:attr:`distal_origin`). The bodies are the
    proximal segment ``O-A-A'``, the driver link ``A-B'``, the distal segment
    ``B'-B-C`` and the coupler ``A'-B``; the PIP servo's shaft is on ``A'``.

    The joint knows its own parts, not the segments': posing it gives the
    distal segment's frame (``B`` and the distal rotation); the finger
    (``core.morphology.finger.Finger``) places its points on it.

    The assembly branch of ``B`` is fixed by the reference pose, which must
    not be a dead point (:meth:`check`; solving fails there too). The
    four-bar may be open or crossed there (the driver link and the coupler
    intersecting); like a polycentric knee it may start open and cross as it
    flexes, without changing branch. Whether the DIP flexes with the PIP is a
    property of the motion, checked by the caller.
    """

    a: PointXZ
    ap: PointXZ
    b: PointXZ
    bp: PointXZ

    def __post_init__(self) -> None:
        for name in ("a", "ap", "b", "bp"):
            if not isinstance(getattr(self, name), PointXZ):
                raise TypeError(f"{name} must be a PointXZ instance")
        require_positive("a.x_mm", self.a.x_mm)
        if self.b.x_mm <= self.a.x_mm:
            raise ValueError("B must lie past A along the finger (b.x_mm > a.x_mm)")

    @property
    def distal_origin(self) -> PointXZ:
        """The distal segment's frame origin in the extended pose: the pin ``B``."""

        return self.b

    @functools.cached_property
    def _branches(self) -> tuple[int, int]:
        """The side of ``B'->A'`` that ``B`` lies on (a PIP-driven solve) and of
        ``A->B`` that ``B'`` lies on (a servo-driven one)."""

        side = _cross(self.ap - self.bp, self.b - self.bp)
        servo_side = _cross(self.b - self.a, self.bp - self.a)
        if abs(side) <= _TOLERANCE or abs(servo_side) <= _TOLERANCE:
            raise ValueError("the reference four-bar is at a dead point")
        return (1 if side > 0.0 else -1), (1 if servo_side > 0.0 else -1)

    def check(self) -> None:
        """``ValueError`` when the reference pose is a dead point (no assembly branch)."""

        self._branches

    @property
    def is_crossed(self) -> bool:
        """Whether the driver link ``A-B'`` and the coupler ``A'-B`` intersect."""

        return _segments_intersect(self.a, self.bp, self.ap, self.b)

    @property
    def icr(self) -> PointXZ | None:
        """The extended pose's centre of rotation (see :attr:`FingerPose.icr`)."""

        return _icr_point(self.a, self.ap, self.b, self.bp)

    def solve(self, q_pip_rad: float) -> FingerPose:
        """The four-bar at the finger's PIP angle (the distal segment's flexion),
        in the proximal frame (the MCP at 0)."""

        q_pip_rad = require_finite("q_pip_rad", q_pip_rad)
        sweep = self.sweep_pip([q_pip_rad])
        if not len(sweep.q_pip_rad):
            raise KinematicClosureError("no closure on the reference assembly branch")
        bp, b = (PointXZ(float(point[0][0]), float(point[0][1])) for point in (sweep.bp, sweep.b))
        return self._pose(q_pip_rad, float(sweep.q_driver_rad[0]), bp, b, float(sweep.distal_rad[0]))

    def solve_driver(self, q_driver_rad: float) -> FingerPose:
        """The four-bar at a rotation of the driver link ``A-B'`` about ``A``."""

        q_driver_rad = require_finite("q_driver_rad", q_driver_rad)
        bp = self.a + (self.bp - self.a).rotated(-q_driver_rad)
        b = self._close(bp)
        distal_rad = _signed_rotation(self.b - self.bp, b - bp)
        return self._pose(-distal_rad, q_driver_rad, bp, b, distal_rad)

    def _pose(self, q_pip_rad: float, q_driver_rad: float, bp: PointXZ, b: PointXZ, distal_rad: float) -> FingerPose:
        return FingerPose(
            q_mcp_rad=0.0,
            q_pip_rad=q_pip_rad,
            q_driver_rad=q_driver_rad,
            q_dip_rad=math.remainder(-(distal_rad + q_driver_rad), 2.0 * math.pi),
            o=PointXZ(0.0, 0.0),
            a=self.a,
            ap=self.ap,
            bp=bp,
            b=b,
            distal_rad=distal_rad,
            icr=_icr_point(self.a, self.ap, b, bp),
        )

    @functools.cached_property
    def _flexion_branch(self) -> int:
        """The side of ``(A' - (B - B')) -> A`` that ``B'`` lies on (a solve by the PIP)."""

        moving = self.ap - (self.b - self.bp)
        side = _cross(self.a - moving, self.bp - moving)
        if abs(side) <= _TOLERANCE:
            raise ValueError("the reference four-bar is at a dead point of the distal segment")
        return 1 if side > 0.0 else -1

    def sweep_pip(self, q_pip_rad: Sequence[float]) -> PipSweep:
        """Solve the four-bar at a sequence of the finger's PIP angles (the
        distal segment's flexion) starting from extension.

        The distal segment turned by ``q_pip`` keeps ``B - B'``; ``B'`` lies
        on its circle about ``A`` and ``B`` on its circle about ``A'``.
        """

        q = np.asarray(q_pip_rad, dtype=float)
        a, ap = _xz(self.a), _xz(self.ap)
        bar = turned(_xz(self.b) - _xz(self.bp), -q)
        bp, count = _close_circles(ap - bar, a, np.linalg.norm(_xz(self.b) - ap),
                                   np.linalg.norm(_xz(self.bp) - a), self._flexion_branch)
        return self._sweep(bp, bp + bar[:count])

    def sweep_driver(self, q_driver_rad: Sequence[float]) -> PipSweep:
        """Solve the four-bar at a sequence of driver-link rotations starting from extension.

        The vectorized counterpart of :meth:`solve_driver` for the proximal-link frame.
        """

        q = np.asarray(q_driver_rad, dtype=float)
        a, ap = _xz(self.a), _xz(self.ap)
        bp = a + turned(_xz(self.bp) - a, -q)
        b, count = _close_circles(bp, ap, np.linalg.norm(_xz(self.b) - _xz(self.bp)),
                                  np.linalg.norm(_xz(self.b) - ap), self._branches[0])
        return self._sweep(bp[:count], b)

    def sweep_servo(self, servo_rad: Sequence[float]) -> PipSweep:
        """Solve the four-bar at a sequence of PIP servo angles starting from extension.

        The servo turns the driven link ``A'-B`` by ``servo_rad`` (positive turns
        ``+X`` toward ``-Z``); ``B'`` closes on the driver link ``A-B'``.
        """

        s = np.asarray(servo_rad, dtype=float)
        a, ap = _xz(self.a), _xz(self.ap)
        b = ap + turned(_xz(self.b) - ap, -s)
        bp, count = _close_circles(b, a, np.linalg.norm(_xz(self.bp) - _xz(self.b)),
                                   np.linalg.norm(_xz(self.bp) - a), -self._branches[1])
        return self._sweep(bp, b[:count])

    def _sweep(self, bp: np.ndarray, b: np.ndarray) -> PipSweep:
        """Joint angles, the distal rotation and the transmission margins of solved ``B'`` and ``B``."""

        a, ap = _xz(self.a), _xz(self.ap)
        bp_ref, b_ref = _xz(self.bp), _xz(self.b)
        q_driver = -_rotation(bp_ref - a, bp - a)
        distal_rad = _rotation(b_ref - bp_ref, b - bp)
        servo = -_rotation(b_ref - ap, b - ap)
        q_dip = -(distal_rad + q_driver)

        def margin(first: np.ndarray, second: np.ndarray) -> np.ndarray:
            angle = np.abs(np.arctan2(first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0],
                                      np.sum(first * second, axis=1)))
            return np.minimum(angle, math.pi - angle)

        return PipSweep(
            q_pip_rad=q_driver + q_dip,
            q_driver_rad=q_driver,
            q_dip_rad=q_dip,
            bp=bp,
            b=b,
            distal_rad=distal_rad,
            margin_at_b_rad=margin(ap - b, bp - b),
            margin_at_bp_rad=margin(a - bp, b - bp),
            servo_rad=servo,
            icr=bar_crossings(a, ap, b, bp),
        )

    def _close(self, bp: PointXZ) -> PointXZ:
        """Return ``B`` on the reference branch, given the posed ``B'``."""

        candidates = _circle_intersections(
            bp, self.ap, self.bp.distance_to(self.b), self.ap.distance_to(self.b)
        )
        for candidate in candidates:
            side = _cross(self.ap - bp, candidate - bp)
            if len(candidates) == 1 or side * self._branches[0] > 0.0:
                return candidate
        raise KinematicClosureError("no closure on the reference assembly branch")


def four_bar(finger) -> FourBar:
    """The finger's joint, which the linkage's checks and measures read: ``TypeError`` if not a :class:`FourBar`."""

    if not isinstance(finger.joint, FourBar):
        raise TypeError(f"a CoDe finger's joint is a FourBar, not {type(finger.joint).__name__}")
    return finger.joint


def bar_crossings(a: np.ndarray, ap: np.ndarray, b: np.ndarray, bp: np.ndarray) -> np.ndarray:
    """Where the lines ``A-B'`` and ``A'-B`` cross, ``(n, 2)`` for ``(n, 2)``
    ``B`` and ``B'`` (``(2,)`` for ``(2,)``): the distal segment's
    instantaneous centre of rotation; NaN where the bars are parallel."""

    first, second = bp - a, b - ap
    det = first[..., 0] * -second[..., 1] - first[..., 1] * -second[..., 0]
    gap = ap - a
    parallel = np.abs(det) < 1e-9
    t = (gap[..., 0] * -second[..., 1] - gap[..., 1] * -second[..., 0]) / np.where(parallel, 1.0, det)
    return np.where(parallel[..., None], np.nan, a + t[..., None] * first)


def _icr_point(a: PointXZ, ap: PointXZ, b: PointXZ, bp: PointXZ) -> PointXZ | None:
    x, z = bar_crossings(_xz(a), _xz(ap), _xz(b), _xz(bp))
    return None if math.isnan(x) else PointXZ(float(x), float(z))


def _rotation(reference: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Unwrapped signed angle (counter-clockwise) from ``reference`` to each ``current``."""

    return np.unwrap(np.arctan2(
        reference[0] * current[:, 1] - reference[1] * current[:, 0],
        reference[0] * current[:, 0] + reference[1] * current[:, 1],
    ))


def _close_circles(
    moving: np.ndarray, fixed: np.ndarray, moving_radius: float, fixed_radius: float, branch: int
) -> tuple[np.ndarray, int]:
    """Points at ``moving_radius`` from each ``moving`` and ``fixed_radius`` from ``fixed``.

    ``branch`` picks the side of ``moving->fixed``. Returns the solved points
    up to the first sample that fails to close, and their count.
    """

    delta = fixed - moving
    distance = np.linalg.norm(delta, axis=1)
    along = (moving_radius ** 2 - fixed_radius ** 2 + distance ** 2) / (2.0 * distance)
    height_squared = moving_radius ** 2 - along ** 2
    tolerance = _TOLERANCE * max(1.0, moving_radius ** 2)
    failed = np.flatnonzero(height_squared < -tolerance)
    count = int(failed[0]) if failed.size else len(moving)
    unit = delta[:count] / distance[:count, None]
    height = np.sqrt(np.maximum(height_squared[:count], 0.0))
    perpendicular = np.stack((-unit[:, 1], unit[:, 0]), axis=1)
    points = moving[:count] + along[:count, None] * unit + branch * height[:, None] * perpendicular
    return points, count


def _circle_intersections(
    center_0: PointXZ, center_1: PointXZ, radius_0: float, radius_1: float
) -> tuple[PointXZ, ...]:
    delta = center_1 - center_0
    distance = _length(delta)
    tolerance = _TOLERANCE * max(1.0, distance, radius_0, radius_1)
    if distance <= tolerance:
        raise KinematicClosureError("the four-bar closure circles are concentric")
    if distance > radius_0 + radius_1 + tolerance or distance < abs(radius_0 - radius_1) - tolerance:
        raise KinematicClosureError("the four-bar cannot close")
    along = (radius_0 ** 2 - radius_1 ** 2 + distance ** 2) / (2.0 * distance)
    height = math.sqrt(max(0.0, radius_0 ** 2 - along ** 2))
    unit = (delta.x_mm / distance, delta.z_mm / distance)
    middle = PointXZ(center_0.x_mm + along * unit[0], center_0.z_mm + along * unit[1])
    if height <= tolerance:
        return (middle,)
    offset = PointXZ(-unit[1] * height, unit[0] * height)
    return (middle + offset, middle - offset)


def _xz(point: PointXZ) -> np.ndarray:
    return point.array


def _segments_intersect(p0: PointXZ, p1: PointXZ, q0: PointXZ, q1: PointXZ) -> bool:
    """Whether the open segments ``p0-p1`` and ``q0-q1`` properly cross."""

    d1 = _cross(p1 - p0, q0 - p0)
    d2 = _cross(p1 - p0, q1 - p0)
    d3 = _cross(q1 - q0, p0 - q0)
    d4 = _cross(q1 - q0, p1 - q0)
    return d1 * d2 < 0.0 and d3 * d4 < 0.0


def _signed_rotation(reference: PointXZ, current: PointXZ) -> float:
    return math.atan2(_cross(reference, current), _dot(reference, current))


def _margin(angle_rad: float) -> float:
    return min(angle_rad, math.pi - angle_rad)


def _angle_between(first: PointXZ, second: PointXZ) -> float:
    return abs(_signed_rotation(first, second))


def _length(point: PointXZ) -> float:
    return math.hypot(point.x_mm, point.z_mm)


def _dot(first: PointXZ, second: PointXZ) -> float:
    return first.x_mm * second.x_mm + first.z_mm * second.z_mm


def _cross(first: PointXZ, second: PointXZ) -> float:
    return first.x_mm * second.z_mm - first.z_mm * second.x_mm


__all__ = ["FourBar", "FingerPose", "KinematicClosureError", "PipSweep", "bar_crossings", "four_bar"]
