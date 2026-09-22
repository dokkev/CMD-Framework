"""3D visualization of the CLEAR XM335 actuator only."""

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

from ..hardware import ACTUATOR_WIDTH_AXIS_XYZ, Actuator, XM335Actuator


def _positive_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a positive finite value in mm")
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite value in mm")
    return value


@dataclass(frozen=True)
class ActuatorVisualizer:
    """Render only the actuator box, shaft axis, and actuator-local frame.

    The actuator shaft center is the visualization origin ``(0, 0, 0)``.
    The actuator-local frame uses +X across the body width, +Y from bottom to
    top, and +Z along the XM335 shaft axis. The box is centered in X and Z;
    its center is at ``(0, -8, 0)`` for the project XM335 values.
    """

    actuator: Actuator = field(default_factory=XM335Actuator)
    axis_marker_length_mm: float = 12.0

    def __post_init__(self) -> None:
        if not isinstance(self.actuator, Actuator):
            raise TypeError("actuator must be an Actuator instance")
        object.__setattr__(
            self,
            "axis_marker_length_mm",
            _positive_finite("axis_marker_length_mm", self.axis_marker_length_mm),
        )

    def create_figure(self) -> Figure:
        """Create and return a 3D actuator-only figure."""

        figure = plt.figure(figsize=(9.5, 8.0), constrained_layout=True)
        axis = figure.add_subplot(111, projection="3d")

        self._plot_actuator_box(axis)
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

    def _plot_actuator_box(self, axis: Axes3D) -> None:
        box = self.actuator.box
        x_min = -box.width_mm / 2.0
        x_max = box.width_mm / 2.0
        y_min = -self.actuator.shaft.center_from_bottom_mm
        y_max = y_min + box.height_mm
        z_min = -box.depth_mm / 2.0
        z_max = box.depth_mm / 2.0

        vertices = (
            (x_min, y_min, z_min),
            (x_max, y_min, z_min),
            (x_max, y_max, z_min),
            (x_min, y_max, z_min),
            (x_min, y_min, z_max),
            (x_max, y_min, z_max),
            (x_max, y_max, z_max),
            (x_min, y_max, z_max),
        )
        faces = (
            (vertices[0], vertices[1], vertices[2], vertices[3]),
            (vertices[4], vertices[5], vertices[6], vertices[7]),
            (vertices[0], vertices[1], vertices[5], vertices[4]),
            (vertices[1], vertices[2], vertices[6], vertices[5]),
            (vertices[2], vertices[3], vertices[7], vertices[6]),
            (vertices[3], vertices[0], vertices[4], vertices[7]),
        )
        axis.add_collection3d(
            Poly3DCollection(
                faces,
                facecolor="#f4a261",
                edgecolor="#b5651d",
                linewidth=1.0,
                alpha=0.38,
            )
        )

    def _plot_shaft_axis(self, axis: Axes3D) -> None:
        shaft_axis = self.actuator.shaft.axis_xyz
        axis.quiver(
            0.0,
            0.0,
            0.0,
            shaft_axis[0] * self.axis_marker_length_mm,
            shaft_axis[1] * self.axis_marker_length_mm,
            shaft_axis[2] * self.axis_marker_length_mm,
            color="#d62828",
            arrow_length_ratio=0.14,
            linewidth=2.8,
        )
        axis.scatter([0.0], [0.0], [0.0], color="#d62828", s=58, depthshade=False)
        axis.text(
            shaft_axis[0] * self.axis_marker_length_mm + 0.8,
            shaft_axis[1] * self.axis_marker_length_mm,
            shaft_axis[2] * self.axis_marker_length_mm,
            "shaft axis +Z",
            color="#b21f2d",
        )

    def _plot_actuator_frame(self, axis: Axes3D) -> None:
        axis_length = self.axis_marker_length_mm * 0.72
        width_axis = ACTUATOR_WIDTH_AXIS_XYZ

        axis.quiver(
            0.0,
            0.0,
            0.0,
            width_axis[0] * axis_length,
            width_axis[1] * axis_length,
            width_axis[2] * axis_length,
            color="#d97706",
            arrow_length_ratio=0.15,
            linewidth=1.6,
        )
        axis.quiver(
            0.0,
            0.0,
            0.0,
            0.0,
            axis_length,
            0.0,
            color="#2a9d8f",
            arrow_length_ratio=0.15,
            linewidth=1.6,
        )
        axis.text(axis_length + 0.8, 0.0, 0.0, "+X width")
        axis.text(0.0, axis_length + 0.8, 0.0, "+Y height")

    def _configure_axis(self, axis: Axes3D) -> None:
        box = self.actuator.box
        y_min = -self.actuator.shaft.center_from_bottom_mm
        y_max = y_min + box.height_mm
        x_min = -box.width_mm / 2.0 - 2.0
        x_max = box.width_mm / 2.0 + 2.0
        z_extent = box.depth_mm / 2.0
        y_margin = max(box.height_mm * 0.08, 2.0)

        axis.set_title("3D XM335 actuator model")
        axis.set_xlabel("X actuator width [mm]")
        axis.set_ylabel("Y actuator height [mm]")
        axis.set_zlabel("Z shaft axis [mm]")
        axis.set_xlim(x_min, x_max)
        axis.set_ylim(y_min - y_margin, y_max + y_margin)
        axis.set_zlim(-z_extent - 2.0, z_extent + 2.0)
        axis.set_box_aspect(
            (
                x_max - x_min,
                y_max - y_min + 2.0 * y_margin,
                2.0 * z_extent + 4.0,
            )
        )
        axis.view_init(elev=22.0, azim=-62.0)
        axis.grid(True, alpha=0.25)
        axis.legend(
            handles=[
                Patch(
                    facecolor="#f4a261",
                    edgecolor="#b5651d",
                    alpha=0.38,
                    label=(
                        f"XM335 box W{box.width_mm:g} × H{box.height_mm:g} "
                        f"× D{box.depth_mm:g} mm (X=W, Y=H, Z=D)"
                    ),
                ),
                Line2D(
                    [0],
                    [0],
                    color="#d62828",
                    linewidth=2.8,
                    label=(
                        "shaft axis / +Z "
                        f"({self.actuator.shaft.center_from_bottom_mm:g} mm from bottom)"
                    ),
                ),
            ],
            loc="upper left",
            fontsize=8,
        )


__all__ = ["ActuatorVisualizer"]
