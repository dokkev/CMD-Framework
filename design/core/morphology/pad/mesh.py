"""A pad given directly as a triangle mesh (for example from CAD)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from core.morphology.pad.base import Pad, PointXZ
from core._validation import require_finite, require_positive


@dataclass(frozen=True, eq=False)
class MeshPad(Pad):
    """A triangle mesh in the fingertip frame, in millimetres.

    ``vertices`` is ``(n, 3)`` and ``faces`` is ``(m, 3)`` vertex indices. The
    mesh must already sit in the fingertip frame (``+X`` distal, ``+Z`` dorsal,
    contact surface facing ``-Z``) and start at ``x = 0``.
    """

    vertices: np.ndarray = field(repr=False)
    faces: np.ndarray = field(repr=False)
    rotation_axis_z_mm: float = 0.0

    def __post_init__(self) -> None:
        vertices = np.asarray(self.vertices, dtype=float)
        faces = np.asarray(self.faces, dtype=int)
        if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3:
            raise ValueError("vertices must be an (n, 3) array with n >= 3")
        if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
            raise ValueError("faces must be a non-empty (m, 3) array")
        if faces.min() < 0 or faces.max() >= len(vertices):
            raise ValueError("faces index outside vertices")
        if not np.all(np.isfinite(vertices)):
            raise ValueError("vertices must be finite")
        if abs(vertices[:, 0].min()) > 1e-6:
            raise ValueError("the mesh must start at x = 0 in the fingertip frame")
        require_finite("rotation_axis_z_mm", self.rotation_axis_z_mm)
        vertices.setflags(write=False)
        faces.setflags(write=False)
        object.__setattr__(self, "vertices", vertices)
        object.__setattr__(self, "faces", faces)

    @classmethod
    def from_file(
        cls, path: str | Path, rotation_axis_z_mm: float = 0.0, scale: float = 1.0
    ) -> MeshPad:
        """Load a mesh file (STL, OBJ, PLY, ...) already in the fingertip frame.

        ``scale`` converts the file's units to millimetres (``1000`` for metres).
        """

        import trimesh

        require_positive("scale", scale)
        mesh = trimesh.load_mesh(Path(path), force="mesh")
        return cls(np.asarray(mesh.vertices) * scale, np.asarray(mesh.faces), rotation_axis_z_mm)

    @property
    def length_mm(self) -> float:
        return float(self.vertices[:, 0].max())

    def side_silhouette_mm(self, samples: int = 24) -> tuple[PointXZ, ...]:
        """Side envelope: the lowest and highest mesh points at ``samples + 1`` stations.

        Each station cuts every edge that crosses it; the ends use the vertices
        there. Concave notches between stations are not resolved.
        """

        edges = self.vertices[np.concatenate((self.faces[:, [0, 1]], self.faces[:, [1, 2]],
                                              self.faces[:, [2, 0]]))]
        x0, x1 = edges[:, 0, 0], edges[:, 1, 0]
        z0, z1 = edges[:, 0, 2], edges[:, 1, 2]
        length = self.length_mm
        lows, highs = [], []
        for x in np.linspace(0.0, length, samples + 1):
            at_vertex = self.vertices[np.abs(self.vertices[:, 0] - x) <= 1e-9, 2]
            crossing = (np.minimum(x0, x1) <= x) & (x <= np.maximum(x0, x1)) & (x0 != x1)
            t = (x - x0[crossing]) / (x1[crossing] - x0[crossing])
            z = np.concatenate((at_vertex, z0[crossing] + t * (z1[crossing] - z0[crossing])))
            lows.append((float(x), float(z.min())))
            highs.append((float(x), float(z.max())))
        return tuple(highs) + tuple(reversed(lows))


__all__ = ["MeshPad"]
