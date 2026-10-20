"""Geometry and closed-form kinematics of the 2-DoF polycentric finger.

The MCP servo (``O``) turns the whole finger; the PIP servo (``A'``) turns the
driven link ``A'-B`` of the four-bar ``A-B'-B-A'`` on the proximal link, which
sets the PIP (``A``) and the passive DIP (``B'``) together. Flexion angles,
servo angles included, are positive and turn ``+X`` toward ``-Z``.

The contact-supporting structures end at ``A''`` and ``B''``, points of the
proximal and distal bodies (not joints): ``A''`` is ``A'`` projected on the
proximal mounting line (the ``x`` axis), ``B''`` is ``B'`` projected on the
distal one (the line ``BC``), both in the reference pose.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from core._validation import require_finite
from core.morphology.linkage import FingerLinkageParameters, PointXZ


_TOLERANCE = 1e-10


class KinematicClosureError(ValueError):
    """Raised when the four-bar cannot close at a requested pose."""


@dataclass(frozen=True)
class FingerPose:
    """One solved pose; joint angles are flexion from the reference pose."""

    q_mcp_rad: float
    q_pip_rad: float
    q_dip_rad: float
    o: PointXZ
    a: PointXZ
    ap: PointXZ
    bp: PointXZ
    b: PointXZ
    c: PointXZ
    app: PointXZ
    """``A''``, end of the proximal support structure, on the proximal body."""
    bpp: PointXZ
    """``B''``, end of the distal support structure, on the distal body."""

    @property
    def is_crossed(self) -> bool:
        """Whether the middle link ``A-B'`` and the coupler ``A'-B`` intersect."""

        return _segments_intersect(self.a, self.bp, self.ap, self.b)

    @property
    def margin_at_b_rad(self) -> float:
        """Margin from 0 or pi of the angle at ``B`` between ``B->A'`` and ``B->B'``.

        It is the transmission margin when the PIP servo drives ``A-B'``.
        """

        return _margin(_angle_between(_subtract(self.ap, self.b), _subtract(self.bp, self.b)))

    @property
    def margin_at_bp_rad(self) -> float:
        """Margin from 0 or pi of the angle at ``B'`` between ``B'->A`` and ``B'->B``.

        It is the transmission margin when the PIP servo drives ``A'-B``.
        """

        return _margin(_angle_between(_subtract(self.a, self.bp), _subtract(self.b, self.bp)))


@dataclass(frozen=True)
class PipSweep:
    """The four-bar solved over a sequence of poses, in the proximal-link frame.

    Arrays cover the samples up to the first one that fails to close; angles
    are unwrapped along the sweep. ``servo_rad`` is the driven link's rotation
    from the reference pose. Points are ``(n, 2)`` XZ arrays.
    """

    q_pip_rad: np.ndarray
    q_dip_rad: np.ndarray
    bp: np.ndarray
    b: np.ndarray
    c: np.ndarray
    margin_at_b_rad: np.ndarray
    margin_at_bp_rad: np.ndarray
    servo_rad: np.ndarray

    @property
    def total_flexion_rad(self) -> np.ndarray:
        """Distal-link rotation relative to the proximal link, ``q_pip + q_dip``."""

        return self.q_pip_rad + self.q_dip_rad


@dataclass(frozen=True)
class FingerLinkage:
    """Reference geometry and pose solver for one finger linkage.

    The assembly branch of ``B`` is fixed by the reference pose, which must
    not be a dead point. The four-bar may be open or crossed there (the middle
    link ``A-B'`` and the coupler ``A'-B`` intersecting); like a polycentric
    knee it may start open and cross as it flexes, without changing branch.
    Whether the DIP flexes with the PIP is a property of the motion, checked
    by the caller.
    """

    parameters: FingerLinkageParameters
    _branch: int = field(init=False, repr=False)
    """Side of ``B'->A'`` that ``B`` lies on, for a PIP-driven solve."""
    _servo_branch: int = field(init=False, repr=False)
    """Side of ``A->B`` that ``B'`` lies on, for a servo-driven solve."""

    def __post_init__(self) -> None:
        if not isinstance(self.parameters, FingerLinkageParameters):
            raise TypeError("parameters must be a FingerLinkageParameters instance")
        side = _cross(_subtract(self.ap, self.bp), _subtract(self.b, self.bp))
        servo_side = _cross(_subtract(self.b, self.a), _subtract(self.bp, self.a))
        if abs(side) <= _TOLERANCE or abs(servo_side) <= _TOLERANCE:
            raise ValueError("the reference four-bar is at a dead point")
        object.__setattr__(self, "_branch", 1 if side > 0.0 else -1)
        object.__setattr__(self, "_servo_branch", 1 if servo_side > 0.0 else -1)

    @property
    def o(self) -> PointXZ:
        return PointXZ(0.0, 0.0)

    @property
    def a(self) -> PointXZ:
        return PointXZ(self.parameters.a_x_mm, self.parameters.a_z_mm)

    @property
    def ap(self) -> PointXZ:
        return self.parameters.ap

    @property
    def bp(self) -> PointXZ:
        return self.parameters.bp

    @property
    def b(self) -> PointXZ:
        return PointXZ(self.parameters.b_x_mm, self.parameters.b_z_mm)

    @property
    def c(self) -> PointXZ:
        return PointXZ(self.parameters.b_x_mm + self.parameters.bc_length_mm, self.parameters.b_z_mm)

    @property
    def app(self) -> PointXZ:
        """``A'' = (A'_x, 0)``, ``A'`` projected on the proximal mounting line."""

        return PointXZ(self.parameters.ap.x_mm, 0.0)

    @property
    def bpp(self) -> PointXZ:
        """``B'' = (B'_x, B_z)``, ``B'`` projected on the distal mounting line ``BC``."""

        return PointXZ(self.parameters.bp.x_mm, self.parameters.b_z_mm)

    @property
    def bpp_along_bc(self) -> float:
        """Signed position of ``B''`` along ``BC`` from ``B``, as a fraction of ``BC``."""

        return (self.parameters.bp.x_mm - self.parameters.b_x_mm) / self.parameters.bc_length_mm

    @property
    def is_crossed(self) -> bool:
        """Whether the middle link ``A-B'`` and the coupler ``A'-B`` intersect."""

        return _segments_intersect(self.a, self.bp, self.ap, self.b)

    def solve(self, q_mcp_rad: float, q_pip_rad: float) -> FingerPose:
        """Return the pose for the two actuated flexion angles."""

        q_mcp_rad = require_finite("q_mcp_rad", q_mcp_rad)
        q_pip_rad = require_finite("q_pip_rad", q_pip_rad)

        # Close the four-bar in the proximal-link frame, which is the
        # reference frame, then turn the whole finger about O.
        bp = _add(self.a, _rotate(_subtract(self.bp, self.a), -q_pip_rad))
        b = self._close(bp)
        distal_rad = _signed_rotation(_subtract(self.b, self.bp), _subtract(b, bp))
        c = _add(b, _rotate(_subtract(self.c, self.b), distal_rad))

        # B'' rides on the distal link: the same fraction of the posed B->C.
        bpp = _add(b, _scale(_subtract(c, b), self.bpp_along_bc))

        def turn(point: PointXZ) -> PointXZ:
            return _rotate(point, -q_mcp_rad)

        return FingerPose(
            q_mcp_rad=q_mcp_rad,
            q_pip_rad=q_pip_rad,
            q_dip_rad=math.remainder(-(distal_rad + q_pip_rad), 2.0 * math.pi),
            o=self.o,
            a=turn(self.a),
            ap=turn(self.ap),
            bp=turn(bp),
            b=turn(b),
            c=turn(c),
            app=turn(self.app),
            bpp=turn(bpp),
        )

    def sweep_pip(self, q_pip_rad: Sequence[float]) -> PipSweep:
        """Solve the four-bar at a sequence of PIP angles starting from extension.

        The vectorized counterpart of :meth:`solve` for the proximal-link frame.
        """

        q = np.asarray(q_pip_rad, dtype=float)
        a, ap = _array(self.a), _array(self.ap)
        bp = a + _turn(_array(self.bp) - a, -q)
        b, count = _close_circles(bp, ap, np.linalg.norm(_array(self.b) - _array(self.bp)),
                                  np.linalg.norm(_array(self.b) - ap), self._branch)
        return self._sweep(bp[:count], b)

    def sweep_servo(self, servo_rad: Sequence[float]) -> PipSweep:
        """Solve the four-bar at a sequence of PIP servo angles starting from extension.

        The servo turns the driven link ``A'-B`` by ``servo_rad`` (positive turns
        ``+X`` toward ``-Z``); ``B'`` closes on the middle link ``A-B'``.
        """

        s = np.asarray(servo_rad, dtype=float)
        a, ap = _array(self.a), _array(self.ap)
        b = ap + _turn(_array(self.b) - ap, -s)
        bp, count = _close_circles(b, a, np.linalg.norm(_array(self.bp) - _array(self.b)),
                                   np.linalg.norm(_array(self.bp) - a), -self._servo_branch)
        return self._sweep(bp, b[:count])

    def _sweep(self, bp: np.ndarray, b: np.ndarray) -> PipSweep:
        """Joint angles, ``C`` and the transmission margins of solved ``B'`` and ``B``."""

        a, ap = _array(self.a), _array(self.ap)
        bp_ref, b_ref = _array(self.bp), _array(self.b)
        q_pip = -_rotation(bp_ref - a, bp - a)
        distal_rad = _rotation(b_ref - bp_ref, b - bp)
        servo = -_rotation(b_ref - ap, b - ap)
        c = b + _turn(_array(self.c) - b_ref, distal_rad)

        def margin(first: np.ndarray, second: np.ndarray) -> np.ndarray:
            angle = np.abs(np.arctan2(first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0],
                                      np.sum(first * second, axis=1)))
            return np.minimum(angle, math.pi - angle)

        return PipSweep(
            q_pip_rad=q_pip,
            q_dip_rad=-(distal_rad + q_pip),
            bp=bp,
            b=b,
            c=c,
            margin_at_b_rad=margin(ap - b, bp - b),
            margin_at_bp_rad=margin(a - bp, b - bp),
            servo_rad=servo,
        )

    def _close(self, bp: PointXZ) -> PointXZ:
        """Return ``B`` on the reference branch, given the posed ``B'``."""

        candidates = _circle_intersections(
            bp, self.ap, self.bp.distance_to(self.b), self.ap.distance_to(self.b)
        )
        for candidate in candidates:
            side = _cross(_subtract(self.ap, bp), _subtract(candidate, bp))
            if len(candidates) == 1 or side * self._branch > 0.0:
                return candidate
        raise KinematicClosureError("no closure on the reference assembly branch")


def _turn(vectors: np.ndarray, angle_rad) -> np.ndarray:
    """Rotate ``(2,)`` or ``(n, 2)`` vectors by angles (counter-clockwise in XZ)."""

    angle = np.asarray(angle_rad, dtype=float)
    cosine, sine = np.cos(angle), np.sin(angle)
    x, z = vectors[..., 0], vectors[..., 1]
    return np.stack((cosine * x - sine * z, sine * x + cosine * z), axis=-1)


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
    delta = _subtract(center_1, center_0)
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
    return (_add(middle, offset), _subtract(middle, offset))


def _array(point: PointXZ) -> np.ndarray:
    return np.array((point.x_mm, point.z_mm))


def _segments_intersect(p0: PointXZ, p1: PointXZ, q0: PointXZ, q1: PointXZ) -> bool:
    """Whether the open segments ``p0-p1`` and ``q0-q1`` properly cross."""

    d1 = _cross(_subtract(p1, p0), _subtract(q0, p0))
    d2 = _cross(_subtract(p1, p0), _subtract(q1, p0))
    d3 = _cross(_subtract(q1, q0), _subtract(p0, q0))
    d4 = _cross(_subtract(q1, q0), _subtract(p1, q0))
    return d1 * d2 < 0.0 and d3 * d4 < 0.0


def _signed_rotation(reference: PointXZ, current: PointXZ) -> float:
    return math.atan2(_cross(reference, current), _dot(reference, current))


def _margin(angle_rad: float) -> float:
    return min(angle_rad, math.pi - angle_rad)


def _angle_between(first: PointXZ, second: PointXZ) -> float:
    return abs(_signed_rotation(first, second))


def _rotate(point: PointXZ, angle_rad: float) -> PointXZ:
    cosine, sine = math.cos(angle_rad), math.sin(angle_rad)
    return PointXZ(cosine * point.x_mm - sine * point.z_mm, sine * point.x_mm + cosine * point.z_mm)


def _add(first: PointXZ, second: PointXZ) -> PointXZ:
    return PointXZ(first.x_mm + second.x_mm, first.z_mm + second.z_mm)


def _subtract(first: PointXZ, second: PointXZ) -> PointXZ:
    return PointXZ(first.x_mm - second.x_mm, first.z_mm - second.z_mm)


def _scale(point: PointXZ, factor: float) -> PointXZ:
    return PointXZ(point.x_mm * factor, point.z_mm * factor)


def _length(point: PointXZ) -> float:
    return math.hypot(point.x_mm, point.z_mm)


def _dot(first: PointXZ, second: PointXZ) -> float:
    return first.x_mm * second.x_mm + first.z_mm * second.z_mm


def _cross(first: PointXZ, second: PointXZ) -> float:
    return first.x_mm * second.z_mm - first.z_mm * second.x_mm


__all__ = ["FingerLinkage", "FingerPose", "KinematicClosureError", "PipSweep"]
