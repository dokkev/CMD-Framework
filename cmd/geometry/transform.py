"""Small immutable SE(3) transforms shared across geometry consumers.

Transforms map coordinates from a child frame into a parent frame. Translation
components are millimetres; rotations are active right-handed 3x3 matrices.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Sequence


Vector3 = tuple[float, float, float]
Rotation3 = tuple[Vector3, Vector3, Vector3]
IDENTITY_ROTATION: Rotation3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


def _finite_float(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be finite; got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite; got {value!r}")
    return value


def _vector3(name: str, values: object) -> Vector3:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)) or len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    return tuple(_finite_float(f"{name}[{i}]", value) for i, value in enumerate(values))  # type: ignore[return-value]


def _rotation3(values: object) -> Rotation3:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)) or len(values) != 3:
        raise ValueError("rotation_matrix must be 3 by 3")
    rows = tuple(_vector3(f"rotation_matrix[{i}]", row) for i, row in enumerate(values))
    for first in range(3):
        for second in range(3):
            dot = sum(rows[row][first] * rows[row][second] for row in range(3))
            expected = 1.0 if first == second else 0.0
            if not math.isclose(dot, expected, rel_tol=0.0, abs_tol=1.0e-9):
                raise ValueError("rotation_matrix must be orthonormal")
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("rotation_matrix must have determinant +1")
    return rows  # type: ignore[return-value]


def _matvec(rotation: Rotation3, vector: Vector3) -> Vector3:
    return tuple(
        sum(rotation[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _matmul(first: Rotation3, second: Rotation3) -> Rotation3:
    return tuple(
        tuple(
            sum(first[row][k] * second[k][column] for k in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


@dataclass(frozen=True)
class RigidTransform:
    """Pose ``T_parent_child`` mapping child-frame points into the parent.

    Composition follows frame notation: ``T_A_B @ T_B_C == T_A_C``.
    """

    rotation_matrix: Rotation3 = IDENTITY_ROTATION
    translation_mm: Vector3 = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "rotation_matrix", _rotation3(self.rotation_matrix))
        object.__setattr__(self, "translation_mm", _vector3("translation_mm", self.translation_mm))

    @classmethod
    def identity(cls) -> "RigidTransform":
        return cls()

    @classmethod
    def from_translation(cls, translation_mm: Sequence[float]) -> "RigidTransform":
        return cls(translation_mm=_vector3("translation_mm", translation_mm))

    @classmethod
    def rotation_about_axis(
        cls,
        axis_xyz: Sequence[float],
        angle_rad: float,
    ) -> "RigidTransform":
        axis = _vector3("axis_xyz", axis_xyz)
        axis_norm = math.sqrt(sum(component * component for component in axis))
        if axis_norm <= 1.0e-12:
            raise ValueError("axis_xyz must have non-zero length")
        x, y, z = (component / axis_norm for component in axis)
        angle = _finite_float("angle_rad", angle_rad)
        cosine = math.cos(angle)
        sine = math.sin(angle)
        one_minus_cosine = 1.0 - cosine
        return cls(
            rotation_matrix=(
                (
                    cosine + x * x * one_minus_cosine,
                    x * y * one_minus_cosine - z * sine,
                    x * z * one_minus_cosine + y * sine,
                ),
                (
                    y * x * one_minus_cosine + z * sine,
                    cosine + y * y * one_minus_cosine,
                    y * z * one_minus_cosine - x * sine,
                ),
                (
                    z * x * one_minus_cosine - y * sine,
                    z * y * one_minus_cosine + x * sine,
                    cosine + z * z * one_minus_cosine,
                ),
            )
        )

    def transform_point(self, point_xyz_mm: Sequence[float]) -> Vector3:
        point = _vector3("point_xyz_mm", point_xyz_mm)
        rotated = _matvec(self.rotation_matrix, point)
        return tuple(
            rotated[index] + self.translation_mm[index] for index in range(3)
        )  # type: ignore[return-value]

    def transform_vector(self, vector_xyz: Sequence[float]) -> Vector3:
        return _matvec(self.rotation_matrix, _vector3("vector_xyz", vector_xyz))

    def inverse(self) -> "RigidTransform":
        inverse_rotation: Rotation3 = tuple(
            tuple(self.rotation_matrix[column][row] for column in range(3))
            for row in range(3)
        )  # type: ignore[assignment]
        inverse_translation = _matvec(
            inverse_rotation,
            tuple(-component for component in self.translation_mm),  # type: ignore[arg-type]
        )
        return RigidTransform(inverse_rotation, inverse_translation)

    def __matmul__(self, child: "RigidTransform") -> "RigidTransform":
        if not isinstance(child, RigidTransform):
            return NotImplemented
        return RigidTransform(
            rotation_matrix=_matmul(self.rotation_matrix, child.rotation_matrix),
            translation_mm=self.transform_point(child.translation_mm),
        )


__all__ = ["IDENTITY_ROTATION", "RigidTransform", "Rotation3", "Vector3"]
