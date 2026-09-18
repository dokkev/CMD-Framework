"""3D visualization of a CLEAR actuator and reference linkage."""

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

from ..hardware import Actuator, XM335Actuator
from ..morphology import FingerLinkage, ReferencePositions


def _positive_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a positive finite value in mm")
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite value in mm")
    return value


@dataclass(frozen=True)
class ActuatorVisualizer:
    """Render an actuator box, shaft axis, and reference CLEAR linkage.

    The hardware model remains the source of truth for the actuator dimensions
    and shaft location. The only extra value is a visual marker length used to
    make the shaft axis visible in the Matplotlib figure.

    The actuator box is centered about the reference O pivot in X and Y. Its
    bottom is placed at ``O.z - shaft.center_from_bottom_mm`` so the displayed
    shaft axis passes through O at the specified height.
    """

    linkage: FingerLinkage
    actuator: Actuator = field(default_factory=XM335Actuator)
    axis_marker_length_mm: float = 12.0

    def __post_init__(self) -> None:
        if not isinstance(self.linkage, FingerLinkage):
            raise TypeError("linkage must be a FingerLinkage instance")
        if not isinstance(self.actuator, Actuator):
            raise TypeError("actuator must be an Actuator instance")
        object.__setattr__(
            self,
            "axis_marker_length_mm",
            _positive_finite("axis_marker_length_mm", self.axis_marker_length_mm),
        )

    def create_figure(self) -> Figure:
        """Create and return a 3D actuator/linkage figure."""

        figure = plt.figure(figsize=(10.5, 8.0), constrained_layout=True)
        axis = figure.add_subplot(111, projection="3d")
        reference = self.linkage.reference_positions

        self._plot_main_chain(axis, reference)
        self._plot_coupling_rods(axis, reference)
        self._plot_reference_points(axis, reference)
        self._plot_actuator_box(axis, reference)
        self._plot_shaft_axis(axis, reference)
        self._plot_canonical_axes(axis, reference)
        self._configure_axis(axis, reference)
        figure.suptitle("CLEAR Hand XM335 actuator and reference linkage", fontsize=14)
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

    @staticmethod
    def _plot_main_chain(axis: Axes3D, reference: ReferencePositions) -> None:
        segments = (
            (reference.o, reference.a),
            (reference.a, reference.b),
            (reference.b, reference.c),
        )
        for start, end in segments:
            axis.plot(
                [0.0, 0.0],
                [start.y_mm, end.y_mm],
                [start.z_mm, end.z_mm],
                color="#264653",
                linewidth=5.0,
                solid_capstyle="round",
            )

    @staticmethod
    def _plot_coupling_rods(axis: Axes3D, reference: ReferencePositions) -> None:
        rods = (
            (reference.op, reference.bp),
            (reference.ap, reference.cp),
        )
        for start, end in rods:
            axis.plot(
                [0.0, 0.0],
                [start.y_mm, end.y_mm],
                [start.z_mm, end.z_mm],
                color="#e76f51",
                linewidth=3.0,
                linestyle="--",
            )

    @staticmethod
    def _plot_reference_points(axis: Axes3D, reference: ReferencePositions) -> None:
        main_points = (
            ("O", reference.o),
            ("A", reference.a),
            ("B", reference.b),
            ("C", reference.c),
        )
        coupling_points = (
            ("O'", reference.op),
            ("A'", reference.ap),
            ("B'", reference.bp),
            ("C'", reference.cp),
        )

        for label, point in main_points:
            axis.scatter(
                [0.0],
                [point.y_mm],
                [point.z_mm],
                color="#264653",
                s=42,
                depthshade=False,
            )
            axis.text(0.8, point.y_mm, point.z_mm, label, color="#264653")

        for label, point in coupling_points:
            axis.scatter(
                [0.0],
                [point.y_mm],
                [point.z_mm],
                color="#e76f51",
                marker="s",
                s=34,
                depthshade=False,
            )
            axis.text(-2.0, point.y_mm, point.z_mm, label, color="#b23a2f")

    def _plot_actuator_box(self, axis: Axes3D, reference: ReferencePositions) -> None:
        box = self.actuator.box
        origin = reference.o
        x_min = -box.width_mm / 2.0
        x_max = box.width_mm / 2.0
        y_min = origin.y_mm - box.depth_mm / 2.0
        y_max = origin.y_mm + box.depth_mm / 2.0
        z_min = origin.z_mm - self.actuator.shaft.center_from_bottom_mm
        z_max = z_min + box.height_mm

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
                alpha=0.35,
            )
        )

    def _plot_shaft_axis(self, axis: Axes3D, reference: ReferencePositions) -> None:
        origin = reference.o
        shaft_axis = self.actuator.shaft.axis_xyz
        axis.quiver(
            0.0,
            origin.y_mm,
            origin.z_mm,
            shaft_axis[0] * self.axis_marker_length_mm,
            shaft_axis[1] * self.axis_marker_length_mm,
            shaft_axis[2] * self.axis_marker_length_mm,
            color="#d62828",
            arrow_length_ratio=0.14,
            linewidth=2.6,
        )
        axis.scatter(
            [0.0],
            [origin.y_mm],
            [origin.z_mm],
            color="#d62828",
            s=52,
            depthshade=False,
        )
        axis.text(
            shaft_axis[0] * self.axis_marker_length_mm + 0.8,
            origin.y_mm + shaft_axis[1] * self.axis_marker_length_mm,
            origin.z_mm + shaft_axis[2] * self.axis_marker_length_mm,
            "shaft axis +X",
            color="#b21f2d",
        )

    def _plot_canonical_axes(self, axis: Axes3D, reference: ReferencePositions) -> None:
        origin = reference.o
        axis_length = self.axis_marker_length_mm * 0.72
        axis.quiver(
            0.0,
            origin.y_mm,
            origin.z_mm,
            0.0,
            axis_length,
            0.0,
            color="#2a9d8f",
            arrow_length_ratio=0.15,
            linewidth=1.4,
        )
        axis.quiver(
            0.0,
            origin.y_mm,
            origin.z_mm,
            0.0,
            0.0,
            axis_length,
            color="#457b9d",
            arrow_length_ratio=0.15,
            linewidth=1.4,
        )
        axis.text(0.0, origin.y_mm + axis_length + 1.0, origin.z_mm, "+Y")
        axis.text(0.0, origin.y_mm, origin.z_mm + axis_length + 1.0, "+Z")

    def _configure_axis(self, axis: Axes3D, reference: ReferencePositions) -> None:
        box = self.actuator.box
        origin = reference.o
        all_points = (
            reference.o,
            reference.a,
            reference.b,
            reference.c,
            reference.op,
            reference.ap,
            reference.bp,
            reference.cp,
        )
        y_values = [point.y_mm for point in all_points]
        z_values = [point.z_mm for point in all_points]
        z_min = origin.z_mm - self.actuator.shaft.center_from_bottom_mm
        z_max = z_min + box.height_mm
        x_extent_mm = box.width_mm / 2.0 + self.axis_marker_length_mm
        y_margin_mm = max(box.depth_mm / 2.0, 2.0)
        z_margin_mm = max(box.height_mm * 0.08, 2.0)

        axis.set_title("3D actuator box and reference linkage")
        axis.set_xlabel("X lateral [mm]")
        axis.set_ylabel("Y longitudinal [mm]")
        axis.set_zlabel("Z dorsal [mm]")
        axis.set_xlim(-x_extent_mm, x_extent_mm)
        axis.set_ylim(min(y_values) - y_margin_mm, max(y_values) + y_margin_mm)
        axis.set_zlim(
            min(min(z_values), z_min) - z_margin_mm,
            max(max(z_values), z_max) + z_margin_mm,
        )
        axis.set_box_aspect(
            (
                2.0 * x_extent_mm,
                max(y_values) - min(y_values) + 2.0 * y_margin_mm,
                max(max(z_values), z_max) - min(min(z_values), z_min)
                + 2.0 * z_margin_mm,
            )
        )
        axis.view_init(elev=23.0, azim=-64.0)
        axis.grid(True, alpha=0.25)
        axis.legend(
            handles=[
                Line2D([0], [0], color="#264653", linewidth=4, label="main chain O-A-B-C"),
                Line2D([0], [0], color="#e76f51", linewidth=3, linestyle="--", label="coupling rods"),
                Patch(
                    facecolor="#f4a261",
                    edgecolor="#b5651d",
                    alpha=0.35,
                    label=(
                        f"XM335 box W{box.width_mm:g} × H{box.height_mm:g} "
                        f"× D{box.depth_mm:g} mm"
                    ),
                ),
                Line2D(
                    [0],
                    [0],
                    color="#d62828",
                    linewidth=2.5,
                    label=(
                        "shaft axis / +X "
                        f"({self.actuator.shaft.center_from_bottom_mm:g} mm from bottom)"
                    ),
                ),
            ],
            loc="upper left",
            fontsize=8,
        )


__all__ = ["ActuatorVisualizer"]
