"""Hard reference-assembly packaging checks for hand-mounted XM335 actuators.

The check uses oriented bounding boxes derived from the actuator's local
geometry bounds. This is conservative: it may report a D-profile pair as
overlapping when the actual curved solids do not, but it cannot miss overlap
between the bounding volumes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import combinations

from ..geometry import RigidTransform, Vector3
from ..hardware import Actuator, XM335Actuator
from ..morphology.hand_layout import HandLayout


_SEPARATION_TOLERANCE_MM = 1.0e-9


@dataclass(frozen=True)
class ActuatorPackagingResult:
    """Reference-pose feasibility; depths describe OBB overlap proxies."""

    feasible: bool
    violated_actuator_pairs: tuple[tuple[str, str], ...]
    obb_overlap_depths_mm: tuple[tuple[str, str, float], ...]


@dataclass(frozen=True)
class _OrientedBox:
    center_hand_mm: Vector3
    half_extents_mm: Vector3
    axes_hand: tuple[Vector3, Vector3, Vector3]


def _dot(first: Vector3, second: Vector3) -> float:
    return sum(first[index] * second[index] for index in range(3))


def _cross(first: Vector3, second: Vector3) -> Vector3:
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _oriented_box(actuator: Actuator, pose_hand_actuator: RigidTransform) -> _OrientedBox:
    bounds = actuator.geometry.bounds_xyz_mm
    center_local = tuple((minimum + maximum) / 2.0 for minimum, maximum in bounds)
    half_extents = tuple((maximum - minimum) / 2.0 for minimum, maximum in bounds)
    rotation = pose_hand_actuator.rotation_matrix
    axes_hand = tuple(
        tuple(rotation[row][column] for row in range(3))
        for column in range(3)
    )
    return _OrientedBox(
        center_hand_mm=pose_hand_actuator.transform_point(center_local),
        half_extents_mm=half_extents,
        axes_hand=axes_hand,  # type: ignore[arg-type]
    )


def _projection_radius(box: _OrientedBox, axis: Vector3) -> float:
    return sum(
        box.half_extents_mm[index] * abs(_dot(box.axes_hand[index], axis))
        for index in range(3)
    )


def _boxes_overlap_depth_mm(
    first: _OrientedBox,
    second: _OrientedBox,
) -> float | None:
    """Return the SAT overlap depth, or ``None`` when OBBs are separate.

    The minimum projection overlap is a bounding-box penetration proxy, not an
    exact penetration depth for the underlying D-shaped actuator solids.
    """

    candidate_axes = [*first.axes_hand, *second.axes_hand]
    candidate_axes.extend(
        _cross(first_axis, second_axis)
        for first_axis in first.axes_hand
        for second_axis in second.axes_hand
    )
    center_delta = tuple(
        second.center_hand_mm[index] - first.center_hand_mm[index]
        for index in range(3)
    )

    minimum_overlap_mm = math.inf
    for candidate in candidate_axes:
        norm = math.sqrt(_dot(candidate, candidate))
        if norm <= 1.0e-12:
            continue
        axis = tuple(component / norm for component in candidate)
        center_distance = abs(_dot(center_delta, axis))
        reach = _projection_radius(first, axis) + _projection_radius(second, axis)
        overlap_mm = reach - center_distance
        if overlap_mm <= _SEPARATION_TOLERANCE_MM:
            return None
        minimum_overlap_mm = min(minimum_overlap_mm, overlap_mm)
    return minimum_overlap_mm


def check_actuator_packaging(
    layout: HandLayout,
    actuator: Actuator | None = None,
) -> ActuatorPackagingResult:
    """Check transformed XM335 OBBs in the reference hand assembly.

    This does not check swept volume over runtime joint motion or collisions
    with finger links, fingertips, or external objects.
    """

    if not isinstance(layout, HandLayout):
        raise TypeError("layout must be a HandLayout instance")
    actuator = XM335Actuator() if actuator is None else actuator
    if not isinstance(actuator, Actuator):
        raise TypeError("actuator must be an Actuator instance")

    boxes = {
        name: _oriented_box(actuator, pose)
        for name, pose in layout.actuator_transforms.items()
    }
    violations: list[tuple[str, str]] = []
    overlap_depths: list[tuple[str, str, float]] = []
    for first_name, second_name in combinations(boxes, 2):
        overlap_depth_mm = _boxes_overlap_depth_mm(
            boxes[first_name],
            boxes[second_name],
        )
        if overlap_depth_mm is not None:
            violations.append((first_name, second_name))
            overlap_depths.append((first_name, second_name, overlap_depth_mm))

    return ActuatorPackagingResult(
        feasible=not overlap_depths,
        violated_actuator_pairs=tuple(violations),
        obb_overlap_depths_mm=tuple(overlap_depths),
    )


__all__ = ["ActuatorPackagingResult", "check_actuator_packaging"]
