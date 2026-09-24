"""Closed-form kinematics for the CLEAR finger linkage."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from numbers import Real

from .linkage import FingerLinkage
from .linkage_parameter import AnchorYZ


_GEOMETRIC_TOLERANCE = 1e-10


class KinematicClosureError(ValueError):
    """Raised when a requested linkage pose cannot satisfy a fixed coupler."""


@dataclass(frozen=True)
class LinkagePose:
    """One posed YZ configuration; q angles are changes from full extension."""

    q1_rad: float
    q2_rad: float
    q3_rad: float
    o: AnchorYZ
    a: AnchorYZ
    b: AnchorYZ
    c: AnchorYZ
    op: AnchorYZ
    ap: AnchorYZ
    bp: AnchorYZ
    cp: AnchorYZ


@dataclass(frozen=True)
class LinkageKinematics:
    """Solve the coupling loops from flexion measured at the extended pose.

    ``q1``, ``q2``, and ``q3`` are angular changes from the neutral extended
    configuration, not the absolute link orientations. The physical hinge at
    ``A`` remains in the model; distal motion comes from both loop closures.
    """

    linkage: FingerLinkage
    _loop_1_branch: int = field(init=False, repr=False)
    _loop_2_branch: int = field(init=False, repr=False)
    _reference_theta1_rad: float = field(init=False, repr=False)
    _reference_theta2_rad: float = field(init=False, repr=False)
    _reference_theta3_rad: float = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.linkage, FingerLinkage):
            raise TypeError("linkage must be a FingerLinkage instance")

        reference = self.linkage.reference_positions
        reference_theta1 = math.atan2(
            reference.a.z_mm - reference.o.z_mm,
            reference.a.y_mm - reference.o.y_mm,
        )
        reference_theta2 = math.atan2(
            reference.b.z_mm - reference.a.z_mm,
            reference.b.y_mm - reference.a.y_mm,
        )
        reference_theta3 = math.atan2(
            reference.c.z_mm - reference.b.z_mm,
            reference.c.y_mm - reference.b.y_mm,
        )
        loop_1_branch = _reference_branch_sign(
            reference.a,
            reference.op,
            reference.bp,
            "O'-B'",
        )
        loop_2_branch = _reference_branch_sign(
            reference.b,
            reference.ap,
            reference.cp,
            "A'-C'",
        )
        object.__setattr__(self, "_loop_1_branch", loop_1_branch)
        object.__setattr__(self, "_loop_2_branch", loop_2_branch)
        object.__setattr__(self, "_reference_theta1_rad", reference_theta1)
        object.__setattr__(self, "_reference_theta2_rad", reference_theta2)
        object.__setattr__(self, "_reference_theta3_rad", reference_theta3)

    def solve(self, q1_rad: float) -> LinkagePose:
        """Return the deterministic closed-chain pose for ``q1_rad``.

        The extended pose defines the zero of all three returned flexion
        angles. ``q1_rad`` rotates the proximal body from its OA' reference;
        ``q2_rad`` and ``q3_rad`` are principal relative joint-angle changes
        from neutral, wrapped to ``[-pi, pi]``.
        """

        _require_finite_angle(q1_rad)
        reference = self.linkage.reference_positions
        parameters = self.linkage.parameters
        main = parameters.main_chain
        coupling = parameters.coupling

        o = reference.o
        theta1 = self._reference_theta1_rad + q1_rad
        a = _add(o, _rotate(AnchorYZ(main.proximal_length_mm, 0.0), theta1))
        ap = _add(o, _rotate(coupling.proximal_anchor, theta1))

        middle_anchor_radius = reference.a.distance_to(reference.bp)
        bp_candidates = _circle_intersections(
            a,
            reference.op,
            middle_anchor_radius,
            self.linkage.coupler_1_length_mm,
            "O'-B'",
        )
        selected_bp = _select_branch(
            a,
            reference.op,
            bp_candidates,
            self._loop_1_branch,
            "O'-B'",
        )
        theta2 = _signed_rotation(coupling.middle_anchor, _subtract(selected_bp, a))
        q2_rad = _wrap_angle(
            theta2
            - theta1
            - (self._reference_theta2_rad - self._reference_theta1_rad)
        )
        bp = _add(a, _rotate(coupling.middle_anchor, theta2))
        b = _add(a, _rotate(AnchorYZ(main.middle_length_mm, 0.0), theta2))

        distal_anchor_radius = reference.b.distance_to(reference.cp)
        cp_candidates = _circle_intersections(
            b,
            ap,
            distal_anchor_radius,
            self.linkage.coupler_2_length_mm,
            "A'-C'",
        )
        selected_cp = _select_branch(
            b,
            ap,
            cp_candidates,
            self._loop_2_branch,
            "A'-C'",
        )
        theta3 = _signed_rotation(coupling.distal_anchor, _subtract(selected_cp, b))
        q3_rad = _wrap_angle(
            theta3
            - theta2
            - (self._reference_theta3_rad - self._reference_theta2_rad)
        )
        cp = _add(b, _rotate(coupling.distal_anchor, theta3))
        c = _add(b, _rotate(AnchorYZ(main.distal_length_mm, 0.0), theta3))

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


def _circle_intersections(
    center_0: AnchorYZ,
    center_1: AnchorYZ,
    radius_0: float,
    radius_1: float,
    loop_name: str,
) -> tuple[AnchorYZ, ...]:
    """Return zero, one, or two intersections of two planar circles."""

    center_delta = _subtract(center_1, center_0)
    center_distance = math.hypot(center_delta.y_mm, center_delta.z_mm)
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
        center_delta.y_mm / center_distance,
        center_delta.z_mm / center_distance,
    )
    midpoint = AnchorYZ(
        center_0.y_mm + along * unit[0],
        center_0.z_mm + along * unit[1],
    )
    if height <= math.sqrt(squared_tolerance):
        return (midpoint,)

    perpendicular = (-unit[1], unit[0])
    offset = AnchorYZ(height * perpendicular[0], height * perpendicular[1])
    return (_add(midpoint, offset), _subtract(midpoint, offset))


def _reference_branch_sign(
    center_0: AnchorYZ,
    center_1: AnchorYZ,
    reference_point: AnchorYZ,
    loop_name: str,
) -> int:
    """Return the reference cross-product sign used to select an assembly branch."""

    center_delta = _subtract(center_1, center_0)
    point_delta = _subtract(reference_point, center_0)
    cross = _cross(center_delta, point_delta)
    scale = max(
        1.0,
        math.hypot(center_delta.y_mm, center_delta.z_mm)
        * math.hypot(point_delta.y_mm, point_delta.z_mm),
    )
    if abs(cross) <= _GEOMETRIC_TOLERANCE * scale:
        raise KinematicClosureError(
            f"{loop_name} reference assembly branch is degenerate"
        )
    return 1 if cross > 0.0 else -1


def _select_branch(
    center_0: AnchorYZ,
    center_1: AnchorYZ,
    candidates: tuple[AnchorYZ, ...],
    reference_sign: int,
    loop_name: str,
) -> AnchorYZ:
    if len(candidates) == 1:
        return candidates[0]

    center_delta = _subtract(center_1, center_0)
    for candidate in candidates:
        if _cross(center_delta, _subtract(candidate, center_0)) * reference_sign > 0.0:
            return candidate
    raise KinematicClosureError(
        f"{loop_name} closure has no candidate on the reference assembly branch"
    )


def _signed_rotation(local: AnchorYZ, world: AnchorYZ) -> float:
    """Return the signed planar rotation from ``local`` to ``world``."""

    local_norm = math.hypot(local.y_mm, local.z_mm)
    world_norm = math.hypot(world.y_mm, world.z_mm)
    if local_norm <= _GEOMETRIC_TOLERANCE or world_norm <= _GEOMETRIC_TOLERANCE:
        raise KinematicClosureError("cannot recover link orientation from a zero anchor vector")
    return math.atan2(_cross(local, world), _dot(local, world))


def _rotate(point: AnchorYZ, angle_rad: float) -> AnchorYZ:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return AnchorYZ(
        cosine * point.y_mm - sine * point.z_mm,
        sine * point.y_mm + cosine * point.z_mm,
    )


def _add(first: AnchorYZ, second: AnchorYZ) -> AnchorYZ:
    return AnchorYZ(first.y_mm + second.y_mm, first.z_mm + second.z_mm)


def _subtract(first: AnchorYZ, second: AnchorYZ) -> AnchorYZ:
    return AnchorYZ(first.y_mm - second.y_mm, first.z_mm - second.z_mm)


def _dot(first: AnchorYZ, second: AnchorYZ) -> float:
    return first.y_mm * second.y_mm + first.z_mm * second.z_mm


def _cross(first: AnchorYZ, second: AnchorYZ) -> float:
    return first.y_mm * second.z_mm - first.z_mm * second.y_mm


def _wrap_angle(angle_rad: float) -> float:
    """Return the principal scalar-angle difference in [-pi, pi]."""

    return math.remainder(angle_rad, 2.0 * math.pi)


def _require_finite_angle(q1_rad: object) -> None:
    if (
        isinstance(q1_rad, bool)
        or not isinstance(q1_rad, Real)
        or not math.isfinite(q1_rad)
    ):
        raise ValueError(f"q1_rad must be a finite angle in radians; got {q1_rad!r}")


__all__ = ["KinematicClosureError", "LinkageKinematics", "LinkagePose"]
