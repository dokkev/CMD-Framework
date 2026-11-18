"""Immutable planar SE(2) rigid-body transforms in the finger (x, z) plane.

Coordinates are millimetres in the planar basis ``(+X distal, +Z dorsal)``;
thus ``-Z`` is the palmar direction. Standard rotations ``R(alpha)`` are
active counter-clockwise turns from ``+X`` toward ``+Z``. Joint flexions
``phi >= 0`` turn ``+X`` toward ``-Z`` (palmar), represented as
``Transform2D.flexion(phi) == Transform2D.rot(-phi)``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from core._validation import require_finite


def _wrap_angle(angle_rad: float) -> float:
    """Wrap an angle (rad) into ``(-pi, pi]``."""

    wrapped = math.remainder(angle_rad, 2.0 * math.pi)
    if wrapped <= -math.pi:
        return wrapped + 2.0 * math.pi
    return wrapped


@dataclass(frozen=True)
class Transform2D:
    """Planar rigid transform ``T_parent_child in SE(2)`` in ``(x, z)`` [mm, rad].

    Composition follows standard frame notation: ``T_A_B @ T_B_C == T_A_C``.
    """

    x_mm: float = 0.0
    z_mm: float = 0.0
    angle_rad: float = 0.0

    def __post_init__(self) -> None:
        x = require_finite("x_mm", self.x_mm)
        z = require_finite("z_mm", self.z_mm)
        a = _wrap_angle(require_finite("angle_rad", self.angle_rad))
        object.__setattr__(self, "x_mm", float(x))
        object.__setattr__(self, "z_mm", float(z))
        object.__setattr__(self, "angle_rad", float(a))

    @classmethod
    def identity(cls) -> Transform2D:
        return cls(0.0, 0.0, 0.0)

    @classmethod
    def trans(cls, x_mm: float, z_mm: float) -> Transform2D:
        return cls(x_mm, z_mm, 0.0)

    @classmethod
    def trans_x(cls, s_mm: float) -> Transform2D:
        return cls(s_mm, 0.0, 0.0)

    @classmethod
    def trans_z(cls, w_mm: float) -> Transform2D:
        return cls(0.0, w_mm, 0.0)

    @classmethod
    def rot(cls, angle_rad: float) -> Transform2D:
        """Counter-clockwise rotation in ``(x, z)`` (``+X`` toward ``+Z``)."""

        return cls(0.0, 0.0, angle_rad)

    @classmethod
    def flexion(cls, phi_rad: float) -> Transform2D:
        """Palmar flexion rotation ``R_flex(phi) = R(-phi)`` (``+X`` toward ``-Z``)."""

        return cls(0.0, 0.0, -require_finite("phi_rad", phi_rad))

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> Transform2D:
        mat = np.asarray(matrix, dtype=float)
        if mat.shape != (3, 3):
            raise ValueError("SE(2) homogeneous matrix must have shape (3, 3)")
        r = mat[:2, :2]
        if not np.allclose(r.T @ r, np.eye(2), atol=1e-9) or not math.isclose(
            float(np.linalg.det(r)), 1.0, abs_tol=1e-9
        ):
            raise ValueError("top-left 2x2 block must belong to SO(2)")
        if not np.allclose(mat[2], (0.0, 0.0, 1.0), atol=1e-9):
            raise ValueError("bottom row of SE(2) matrix must be [0, 0, 1]")
        angle = math.atan2(float(r[1, 0]), float(r[0, 0]))
        return cls(float(mat[0, 2]), float(mat[1, 2]), angle)

    @property
    def pos(self) -> tuple[float, float]:
        """Translational component ``pos(T) = (x_mm, z_mm)``."""

        return (self.x_mm, self.z_mm)

    @property
    def pos_array(self) -> np.ndarray:
        return np.array((self.x_mm, self.z_mm), dtype=float)

    @property
    def ori(self) -> float:
        """Rotational component ``ori(T) = alpha`` in ``(-pi, pi]`` [rad]."""

        return self.angle_rad

    def rotation_matrix(self) -> np.ndarray:
        c, s = math.cos(self.angle_rad), math.sin(self.angle_rad)
        return np.array(((c, -s), (s, c)), dtype=float)

    def matrix(self) -> np.ndarray:
        c, s = math.cos(self.angle_rad), math.sin(self.angle_rad)
        return np.array(((c, -s, self.x_mm), (s, c, self.z_mm), (0.0, 0.0, 1.0)), dtype=float)

    def apply_point(self, point_xz_mm: Sequence[float]) -> tuple[float, float]:
        if len(point_xz_mm) != 2:
            raise ValueError("point_xz_mm must have 2 coordinates")
        px = require_finite("point_xz_mm[0]", point_xz_mm[0])
        pz = require_finite("point_xz_mm[1]", point_xz_mm[1])
        c, s = math.cos(self.angle_rad), math.sin(self.angle_rad)
        return (c * px - s * pz + self.x_mm, s * px + c * pz + self.z_mm)

    def apply_points(self, points_xz_mm: np.ndarray) -> np.ndarray:
        pts = np.asarray(points_xz_mm, dtype=float)
        c, s = math.cos(self.angle_rad), math.sin(self.angle_rad)
        x, z = pts[..., 0], pts[..., 1]
        return np.stack((c * x - s * z + self.x_mm, s * x + c * z + self.z_mm), axis=-1)

    def apply_vector(self, vector_xz: Sequence[float]) -> tuple[float, float]:
        if len(vector_xz) != 2:
            raise ValueError("vector_xz must have 2 coordinates")
        vx = require_finite("vector_xz[0]", vector_xz[0])
        vz = require_finite("vector_xz[1]", vector_xz[1])
        c, s = math.cos(self.angle_rad), math.sin(self.angle_rad)
        return (c * vx - s * vz, s * vx + c * vz)

    def inverse(self) -> Transform2D:
        c, s = math.cos(self.angle_rad), math.sin(self.angle_rad)
        inv_x = -(c * self.x_mm + s * self.z_mm)
        inv_z = s * self.x_mm - c * self.z_mm
        return Transform2D(inv_x, inv_z, -self.angle_rad)

    def __matmul__(self, child: Transform2D) -> Transform2D:
        if not isinstance(child, Transform2D):
            return NotImplemented
        tx, tz = self.apply_point((child.x_mm, child.z_mm))
        return Transform2D(tx, tz, self.angle_rad + child.angle_rad)


__all__ = ["Transform2D"]
