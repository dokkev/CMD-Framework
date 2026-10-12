"""3D visualization of the shaft-centred CLEAR XM335 actuator."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d.axes3d import Axes3D

from ..hardware import ACTUATOR_SHAFT_AXIS_XYZ, Actuator, XM335Actuator
from .._validation import require_positive


@dataclass(frozen=True)
class ActuatorVisualizer:
    """Render the D-shaped body and canonical actuator frame.

    The shaft center is the origin. Local +X is lateral body width, +Y points
    toward the rounded cap, and +Z is the shaft axis.
    """

    actuator: Actuator = field(default_factory=XM335Actuator)
    axis_marker_length_mm: float = 16.0

    def __post_init__(self) -> None:
        if not isinstance(self.actuator, Actuator):
            raise TypeError("actuator must be an Actuator instance")
        object.__setattr__(
            self,
            "axis_marker_length_mm",
            require_positive("axis_marker_length_mm", self.axis_marker_length_mm),
        )

    def create_figure(self) -> Figure:
        """Create and return a 3D actuator-only figure."""

        figure = plt.figure(figsize=(9.5, 8.0), constrained_layout=True)
        axis = figure.add_subplot(111, projection="3d")

        self._plot_d_extrusion(axis)
        self._plot_shaft_axis(axis)
        self._plot_actuator_frame(axis)
        self._configure_axis(axis)
        figure.suptitle("CLEAR Hand XM335 actuator", fontsize=14)
        return figure

    def save(self, output_path: str | Path, dpi: int = 160) -> Path:
        """Render the figure to ``output_path`` and return the saved path."""

        if not isinstance(dpi, int) or isinstance(dpi, bool) or dpi <= 0:
            raise ValueError("dpi must be a positive integer")

        output_path = Path(output_path)
        figure = self.create_figure()
        try:
            figure.savefig(output_path, dpi=dpi, bbox_inches="tight")
        finally:
            plt.close(figure)
        return output_path

    def _profile_xy(self) -> list[tuple[float, float]]:
        geometry = self.actuator.geometry
        radius = geometry.radius_mm
        arc = [
            (
                radius * math.cos(math.pi * index / 32.0),
                radius * math.sin(math.pi * index / 32.0),
            )
            for index in range(33)
        ]
        rear_y_mm = -geometry.rear_extent_mm
        return [
            (-radius, rear_y_mm),
            (radius, rear_y_mm),
            *arc,
            (-radius, rear_y_mm),
        ]

    def _plot_d_extrusion(self, axis: Axes3D) -> None:
        geometry = self.actuator.geometry
        xy_profile = self._profile_xy()
        z_min, z_max = -geometry.depth_mm / 2.0, geometry.depth_mm / 2.0

        faces = [
            [(x, y, z_min) for x, y in xy_profile],
            [(x, y, z_max) for x, y in xy_profile],
            [
                (-geometry.radius_mm, -geometry.rear_extent_mm, z_min),
                (geometry.radius_mm, -geometry.rear_extent_mm, z_min),
                (geometry.radius_mm, -geometry.rear_extent_mm, z_max),
                (-geometry.radius_mm, -geometry.rear_extent_mm, z_max),
            ],
            [
                (-geometry.radius_mm, -geometry.rear_extent_mm, z_min),
                (-geometry.radius_mm, 0.0, z_min),
                (-geometry.radius_mm, 0.0, z_max),
                (-geometry.radius_mm, -geometry.rear_extent_mm, z_max),
            ],
            [
                (geometry.radius_mm, -geometry.rear_extent_mm, z_min),
                (geometry.radius_mm, 0.0, z_min),
                (geometry.radius_mm, 0.0, z_max),
                (geometry.radius_mm, -geometry.rear_extent_mm, z_max),
            ],
        ]

        def cylinder_point(z_mm: float, angle_rad: float) -> tuple[float, float, float]:
            return (
                geometry.radius_mm * math.cos(angle_rad),
                geometry.radius_mm * math.sin(angle_rad),
                z_mm,
            )

        for index in range(32):
            angle_0 = math.pi * index / 32.0
            angle_1 = math.pi * (index + 1) / 32.0
            faces.append(
                [
                    cylinder_point(z_min, angle_0),
                    cylinder_point(z_max, angle_0),
                    cylinder_point(z_max, angle_1),
                    cylinder_point(z_min, angle_1),
                ]
            )

        axis.add_collection3d(
            Poly3DCollection(
                faces,
                facecolor="#f4a261",
                edgecolor="#b5651d",
                linewidth=0.65,
                alpha=0.72,
            )
        )

    def _plot_shaft_axis(self, axis: Axes3D) -> None:
        shaft_axis = ACTUATOR_SHAFT_AXIS_XYZ
        axis.quiver(
            0.0,
            0.0,
            0.0,
            *(component * self.axis_marker_length_mm for component in shaft_axis),
            color="#d62828",
            arrow_length_ratio=0.14,
            linewidth=2.8,
        )
        axis.scatter([0.0], [0.0], [0.0], color="#d62828", s=58, depthshade=False)
        axis.text(0.8, 0.8, self.axis_marker_length_mm, "shaft axis +Z")

    def _plot_actuator_frame(self, axis: Axes3D) -> None:
        axis_length = self.axis_marker_length_mm * 0.72
        for direction, color, label in (
            ((1.0, 0.0, 0.0), "#d97706", "+X lateral width"),
            ((0.0, 1.0, 0.0), "#2a9d8f", "+Y longitudinal"),
        ):
            axis.quiver(
                0.0,
                0.0,
                0.0,
                *(component * axis_length for component in direction),
                color=color,
                arrow_length_ratio=0.15,
                linewidth=1.8,
            )
            axis.text(
                direction[0] * axis_length + 0.8,
                direction[1] * axis_length + 0.8,
                direction[2] * axis_length + 0.8,
                label,
            )

    def _configure_axis(self, axis: Axes3D) -> None:
        geometry = self.actuator.geometry
        (x_min, x_max), (y_min, y_max), (z_min, z_max) = geometry.bounds_xyz_mm
        padding_mm = 2.0
        frame_marker_length = self.axis_marker_length_mm * 0.72

        axis.set_title("XM335 D-shaped extrusion · shaft-centred frame")
        axis.set_xlabel("X lateral width [mm]")
        axis.set_ylabel("Y longitudinal [mm]")
        axis.set_zlabel("Z shaft axis [mm]")
        axis_limits = (
            (
                min(x_min - padding_mm, -frame_marker_length - 3.0),
                max(x_max + padding_mm, frame_marker_length + 3.0),
            ),
            (
                y_min - padding_mm,
                max(y_max + padding_mm, frame_marker_length + 3.0),
            ),
            (
                z_min - padding_mm,
                max(z_max + padding_mm, self.axis_marker_length_mm + 4.0),
            ),
        )
        axis.set_xlim(*axis_limits[0])
        axis.set_ylim(*axis_limits[1])
        axis.set_zlim(*axis_limits[2])
        axis.set_box_aspect(
            (
                axis_limits[0][1] - axis_limits[0][0],
                axis_limits[1][1] - axis_limits[1][0],
                axis_limits[2][1] - axis_limits[2][0],
            )
        )
        axis.view_init(elev=22.0, azim=-62.0)
        axis.grid(True, alpha=0.25)
        axis.legend(
            handles=[
                Patch(
                    facecolor="#f4a261",
                    edgecolor="#b5651d",
                    alpha=0.72,
                    label=(
                        f"D body R{geometry.radius_mm:g}, "
                        f"rear {geometry.rear_extent_mm:g}, "
                        f"depth {geometry.depth_mm:g} mm"
                    ),
                ),
                Line2D(
                    [0],
                    [0],
                    color="#d62828",
                    linewidth=2.8,
                    label="shaft center / axis +Z",
                ),
            ],
            loc="upper left",
            fontsize=8,
        )


__all__ = ["ActuatorVisualizer"]
