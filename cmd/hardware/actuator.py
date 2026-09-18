"""Solver-neutral actuator hardware geometry primitives.

All dimensions are in millimetres.  These classes intentionally describe only
lightweight geometry and frame semantics needed by hand-layout optimization and
collision/clearance screening.  They do not create simulator-specific bodies.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real


Vector3 = tuple[float, float, float]


def _finite_real(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number; got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite real number; got {value!r}")
    return value


def _vector3(name: str, value: object) -> Vector3:
    if not isinstance(value, tuple) or len(value) != 3:
        raise TypeError(f"{name} must be a length-3 tuple")
    return tuple(_finite_real(f"{name}[{i}]", component) for i, component in enumerate(value))  # type: ignore[return-value]


@dataclass(frozen=True)
class BoxEnvelope:
    """One oriented-box keep-out primitive expressed in an actuator frame.

    ``center_mm`` and ``size_mm`` are local coordinates.  Orientation is supplied
    by the pose of the owning actuator, so this primitive is an OBB once placed
    in a hand model even though no rotation is stored here.
    """

    size_mm: Vector3
    center_mm: Vector3 = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        size = _vector3("size_mm", self.size_mm)
        center = _vector3("center_mm", self.center_mm)
        if any(length <= 0.0 for length in size):
            raise ValueError(f"size_mm components must be strictly positive; got {size!r}")
        object.__setattr__(self, "size_mm", size)
        object.__setattr__(self, "center_mm", center)

    @property
    def half_extents_mm(self) -> Vector3:
        return tuple(length / 2.0 for length in self.size_mm)  # type: ignore[return-value]

    @property
    def min_corner_mm(self) -> Vector3:
        return tuple(
            center - half
            for center, half in zip(self.center_mm, self.half_extents_mm)
        )  # type: ignore[return-value]

    @property
    def max_corner_mm(self) -> Vector3:
        return tuple(
            center + half
            for center, half in zip(self.center_mm, self.half_extents_mm)
        )  # type: ignore[return-value]

    def expanded(self, clearance_mm: float) -> "BoxEnvelope":
        """Return a box expanded uniformly by ``clearance_mm`` on every face."""

        clearance = _finite_real("clearance_mm", clearance_mm)
        if clearance < 0.0:
            raise ValueError("clearance_mm must be non-negative")
        return BoxEnvelope(
            size_mm=tuple(length + 2.0 * clearance for length in self.size_mm),  # type: ignore[arg-type]
            center_mm=self.center_mm,
        )


@dataclass(frozen=True)
class RevoluteActuatorGeometry:
    """Minimal solver-neutral geometry for a revolute actuator."""

    model_name: str
    joint_axis: Vector3
    keepout_boxes: tuple[BoxEnvelope, ...]
    mass_g: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.model_name, str) or not self.model_name.strip():
            raise ValueError("model_name must be a non-empty string")
        axis = _vector3("joint_axis", self.joint_axis)
        norm = math.sqrt(sum(component * component for component in axis))
        if norm <= 0.0:
            raise ValueError("joint_axis must be non-zero")
        boxes = self.keepout_boxes
        if not isinstance(boxes, tuple) or not boxes:
            raise ValueError("keepout_boxes must contain at least one BoxEnvelope")
        if not all(isinstance(box, BoxEnvelope) for box in boxes):
            raise TypeError("keepout_boxes must contain only BoxEnvelope instances")
        if self.mass_g is not None:
            mass = _finite_real("mass_g", self.mass_g)
            if mass <= 0.0:
                raise ValueError("mass_g must be strictly positive")
            object.__setattr__(self, "mass_g", mass)
        object.__setattr__(self, "joint_axis", axis)

    @property
    def unit_joint_axis(self) -> Vector3:
        norm = math.sqrt(sum(component * component for component in self.joint_axis))
        return tuple(component / norm for component in self.joint_axis)  # type: ignore[return-value]


__all__ = ["BoxEnvelope", "RevoluteActuatorGeometry", "Vector3"]
