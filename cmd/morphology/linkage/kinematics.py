"""Closed-form kinematics for the CLEAR planar finger linkage."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from numbers import Real

from . import FingerLinkage, PointXZ


_GEOMETRIC_TOLERANCE = 1e-10


class KinematicClosureError(ValueError):
    """Raised when a requested linkage pose cannot satisfy a fixed coupler."""


@dataclass(frozen=True)
class LinkagePose:
    """One posed XZ configuration; q angles are changes from reference."""

    q1_rad: float
    q2_rad: float
    q3_rad: float
    o: PointXZ
    a: PointXZ
    b: PointXZ
    c: PointXZ
    op: PointXZ
    ap: PointXZ
    bp: PointXZ
    cp: PointXZ


@dataclass(frozen=True)
class LinkageKinematics:
    """Solve the two coupler loops from the fully extended reference pose.

    ``q1``, ``q2``, and ``q3`` are angular changes from the stored reference
    configuration. Circle-intersection branches are fixed by the reference
    assembly, so solutions do not depend on call order or motion history.
    """

    linkage: FingerLinkage
    _loop_1_branch: int = field(init=False, repr=False)
    _loop_2_branch: int = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.linkage, FingerLinkage):
            raise TypeError("linkage must be a FingerLinkage instance")

        reference = self.linkage.reference_positions
        object.__setattr__(
            self,
            "_loop_1_branch",
            _reference_branch_sign(reference.a, reference.op, reference.bp, "O'-B'"),
        )
        object.__setattr__(
            self,
            "_loop_2_branch",
            _reference_branch_sign(reference.b, reference.ap, reference.cp, "A'-C'"),
        )

    def solve(self, q1_rad: float) -> LinkagePose:
        """Return the deterministic closed-chain pose for ``q1_rad``."""

        _require_finite_angle(q1_rad)
        reference = self.linkage.reference_positions
        parameters = self.linkage.parameters

        o = reference.o
        a = _add(o, _rotate(parameters.a_from_o, q1_rad))
        ap = _add(o, _rotate(parameters.ap_from_o, q1_rad))

        bp, delta_middle = _solve_dyad(
            a,
            parameters.bp_from_a,
            reference.op,
            self.linkage.coupler_1_length_mm,
            self._loop_1_branch,
            "O'-B'",
        )
        q2_rad = _wrap_angle(delta_middle - q1_rad)
        b = _add(a, _rotate(parameters.b_from_a, delta_middle))

        cp, delta_distal = _solve_dyad(
            b,
            parameters.cp_from_b,
            ap,
            self.linkage.coupler_2_length_mm,
            self._loop_2_branch,
            "A'-C'",
        )
        q3_rad = _wrap_angle(delta_distal - delta_middle)
        c = _add(
            b,
            _rotate(PointXZ(parameters.bc_length_mm, 0.0), delta_distal),
        )

        return LinkagePose(
            q1_rad=q1_rad,
            q2_rad=q2_rad,
            q3_rad=q3_rad,
            o=o,
            a=a,
            b=b,
            c=c,
            op=reference.op,
            ap=ap,
            bp=bp,
            cp=cp,
        )


def _solve_dyad(
    pivot: PointXZ,
    reference_arm: PointXZ,
    coupler_anchor: PointXZ,
    coupler_length_mm: float,
    branch_sign: int,
    loop_name: str,
) -> tuple[PointXZ, float]:
    """Close one rotating-arm/coupler loop on its reference branch.

    Returns the coupler joint on the arm and the arm's rotation from its
    reference direction.
    """

    candidates = _circle_intersections(
        pivot,
        coupler_anchor,
        _length(reference_arm),
        coupler_length_mm,
        loop_name,
    )
    joint = _select_branch(pivot, coupler_anchor, candidates, branch_sign, loop_name)
    return joint, _signed_rotation(reference_arm, _subtract(joint, pivot))


def _circle_intersections(
    center_0: PointXZ,
    center_1: PointXZ,
    radius_0: float,
    radius_1: float,
    loop_name: str,
) -> tuple[PointXZ, ...]:
    """Return zero, one, or two intersections of two planar circles."""

    center_delta = _subtract(center_1, center_0)
    center_distance = _length(center_delta)
    scale = max(1.0, center_distance, radius_0, radius_1)
    length_tolerance = _GEOMETRIC_TOLERANCE * scale

    if center_distance <= length_tolerance:
        if abs(radius_0 - radius_1) <= length_tolerance:
            raise KinematicClosureError(
                f"{loop_name} closure has coincident centers and no unique intersection"
            )
        raise KinematicClosureError(
            f"{loop_name} closure has coincident centers with different radii"
        )

    if center_distance > radius_0 + radius_1 + length_tolerance:
        raise KinematicClosureError(f"{loop_name} closure circles are separated")
    if center_distance < abs(radius_0 - radius_1) - length_tolerance:
        raise KinematicClosureError(
            f"{loop_name} closure has one circle inside the other"
        )

    along = (
        radius_0 * radius_0
        - radius_1 * radius_1
        + center_distance * center_distance
    ) / (2.0 * center_distance)
    height_squared = radius_0 * radius_0 - along * along
    squared_tolerance = _GEOMETRIC_TOLERANCE * max(
        1.0,
        radius_0 * radius_0,
        along * along,
    )
    if height_squared < -squared_tolerance:
        raise KinematicClosureError(f"{loop_name} closure has no real intersection")
    height = math.sqrt(max(0.0, height_squared))

    unit = (
        center_delta.x_mm / center_distance,
        center_delta.z_mm / center_distance,
    )
    midpoint = PointXZ(
        center_0.x_mm + along * unit[0],
        center_0.z_mm + along * unit[1],
    )
    if height <= math.sqrt(squared_tolerance):
        return (midpoint,)

    perpendicular = (-unit[1], unit[0])
    offset = PointXZ(height * perpendicular[0], height * perpendicular[1])
    return (_add(midpoint, offset), _subtract(midpoint, offset))


def _reference_branch_sign(
    center_0: PointXZ,
    center_1: PointXZ,
    reference_point: PointXZ,
    loop_name: str,
) -> int:
    """Return the reference cross-product sign used to select a branch."""

    center_delta = _subtract(center_1, center_0)
    point_delta = _subtract(reference_point, center_0)
    cross = _cross(center_delta, point_delta)
    scale = max(1.0, _length(center_delta) * _length(point_delta))
    if abs(cross) <= _GEOMETRIC_TOLERANCE * scale:
        raise KinematicClosureError(
            f"{loop_name} reference assembly branch is degenerate"
        )
    return 1 if cross > 0.0 else -1


def _select_branch(
    center_0: PointXZ,
    center_1: PointXZ,
    candidates: tuple[PointXZ, ...],
    reference_sign: int,
    loop_name: str,
) -> PointXZ:
    if len(candidates) == 1:
        return candidates[0]

    center_delta = _subtract(center_1, center_0)
    for candidate in candidates:
        if _cross(center_delta, _subtract(candidate, center_0)) * reference_sign > 0.0:
            return candidate
    raise KinematicClosureError(
        f"{loop_name} closure has no candidate on the reference assembly branch"
    )


def _signed_rotation(reference: PointXZ, current: PointXZ) -> float:
    """Return the signed change rotating a reference vector to its current pose."""

    if _length(reference) <= _GEOMETRIC_TOLERANCE or _length(current) <= _GEOMETRIC_TOLERANCE:
        raise KinematicClosureError("cannot determine rotation from a zero-length vector")
    return math.atan2(_cross(reference, current), _dot(reference, current))


def _rotate(point: PointXZ, angle_rad: float) -> PointXZ:
    """Rotate a planar linkage vector using its canonical XZ convention."""
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return PointXZ(
        cosine * point.x_mm - sine * point.z_mm,
        sine * point.x_mm + cosine * point.z_mm,
    )


def _add(first: PointXZ, second: PointXZ) -> PointXZ:
    return PointXZ(first.x_mm + second.x_mm, first.z_mm + second.z_mm)


def _subtract(first: PointXZ, second: PointXZ) -> PointXZ:
    return PointXZ(first.x_mm - second.x_mm, first.z_mm - second.z_mm)


def _length(point: PointXZ) -> float:
    return math.hypot(point.x_mm, point.z_mm)


def _dot(first: PointXZ, second: PointXZ) -> float:
    return first.x_mm * second.x_mm + first.z_mm * second.z_mm


def _cross(first: PointXZ, second: PointXZ) -> float:
    return first.x_mm * second.z_mm - first.z_mm * second.x_mm


def _wrap_angle(angle_rad: float) -> float:
    """Return the principal scalar-angle difference in [-pi, pi]."""

    return math.remainder(angle_rad, 2.0 * math.pi)


def _require_finite_angle(q1_rad: float) -> None:
    if (
        isinstance(q1_rad, bool)
        or not isinstance(q1_rad, Real)
        or not math.isfinite(q1_rad)
    ):
        raise ValueError(f"q1_rad must be a finite angle in radians; got {q1_rad!r}")


__all__ = ["KinematicClosureError", "LinkageKinematics", "LinkagePose"]
