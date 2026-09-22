"""Matplotlib 3D visualization for the analytic fingertip geometry."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d.axes3d import Axes3D

from ..morphology import Fingertip


@dataclass(frozen=True)
class FingertipVisualizer:
    """Render the fingertip contact envelope and carrier as a 3D model.

    The visualizer consumes the analytic ``Fingertip`` representation directly
    and does not create a mesh or introduce a second geometry definition. The
    lower and upper contact surfaces are sampled from the analytic model. The
    carrier cross-section is extruded along ``Y`` to show the rigid core and
    its separate distal dorsal plate extension.
    """

    fingertip: Fingertip = field(default_factory=Fingertip)
    samples: int = 200

    def __post_init__(self) -> None:
        if not isinstance(self.fingertip, Fingertip):
            raise TypeError("fingertip must be a Fingertip instance")
        if not isinstance(self.samples, int) or isinstance(self.samples, bool):
            raise TypeError("samples must be an integer")
        if self.samples < 2:
            raise ValueError("samples must be at least 2")

    def create_figure(self) -> Figure:
        """Create and return a new 3D fingertip model figure."""

        figure = plt.figure(figsize=(10.5, 8.0), constrained_layout=True)
        axis = figure.add_subplot(111, projection="3d")
        self._plot_outer_envelope(axis)
        self._plot_carrier(axis)
        self._configure_3d_axis(axis)
        figure.suptitle("Analytic fingertip morphology", fontsize=14)
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

    def _plot_outer_envelope(self, axis: Axes3D) -> None:
        pad_color = "#4ea8de"
        lower_color = "#0077b6"

        for surface in ("body", "nose"):
            x_grid, y_grid, lower_grid = self._surface_grid(surface, upper=False)
            axis.plot_surface(
                np.asarray(x_grid),
                np.asarray(y_grid),
                np.asarray(lower_grid),
                color=lower_color,
                alpha=0.82,
                linewidth=0,
                antialiased=True,
            )

            _, _, upper_grid = self._surface_grid(surface, upper=True)
            axis.plot_surface(
                np.asarray(x_grid),
                np.asarray(y_grid),
                np.asarray(upper_grid),
                color=pad_color,
                alpha=0.18,
                linewidth=0,
                antialiased=False,
            )

        self._plot_lateral_walls(axis)

    def _plot_lateral_walls(self, axis: Axes3D) -> None:
        body_y_values = self._linspace(
            0.0,
            self.fingertip.parameters.pad.body_length_mm,
            self.samples,
        )
        for x_mm in (-self.fingertip.half_width_mm, self.fingertip.half_width_mm):
            axis.plot_surface(
                np.asarray([[x_mm] * len(body_y_values)] * 2),
                np.asarray([body_y_values, body_y_values]),
                np.asarray(
                    [
                        [
                            self.fingertip.lower_surface_z_mm(x_mm, y_mm)
                            for y_mm in body_y_values
                        ],
                        [
                            self.fingertip.upper_surface_z_mm(x_mm, y_mm)
                            for y_mm in body_y_values
                        ],
                    ]
                ),
                color="#4ea8de",
                alpha=0.18,
                linewidth=0,
            )

        nose_y_values = self._linspace(
            self.fingertip.parameters.pad.body_length_mm,
            self.fingertip.total_length_mm,
            self.samples,
        )
        for x_mm in (-self.fingertip.half_width_mm, self.fingertip.half_width_mm):
            axis.plot_surface(
                np.asarray([[x_mm] * len(nose_y_values)] * 2),
                np.asarray([nose_y_values, nose_y_values]),
                np.asarray(
                    [
                        [
                            self.fingertip.lower_surface_z_mm(x_mm, y_mm)
                            for y_mm in nose_y_values
                        ],
                        [
                            self.fingertip.upper_surface_z_mm(x_mm, y_mm)
                            for y_mm in nose_y_values
                        ],
                    ]
                ),
                color="#4ea8de",
                alpha=0.18,
                linewidth=0,
            )

    def _plot_carrier(self, axis: Axes3D) -> None:
        carrier_color = "#f4a261"
        core_length_mm = self.fingertip.carrier_length_mm
        total_length_mm = self.fingertip.dorsal_plate_length_mm

        self._add_extruded_polygon(
            axis,
            self.fingertip.carrier_cross_section,
            0.0,
            core_length_mm,
            face_color=carrier_color,
            edge_color="#c45a00",
            alpha=0.78,
        )

        half_width_mm = self.fingertip.half_width_mm
        carrier_height_mm = self.fingertip.parameters.phalanx.carrier_height_mm
        dorsal_plate = (
            (-half_width_mm, 0.0),
            (half_width_mm, 0.0),
            (half_width_mm, carrier_height_mm),
            (-half_width_mm, carrier_height_mm),
        )
        self._add_extruded_polygon(
            axis,
            dorsal_plate,
            core_length_mm,
            total_length_mm,
            face_color="#e76f51",
            edge_color="#b23a2f",
            alpha=0.78,
        )

    def _configure_3d_axis(self, axis: Axes3D) -> None:
        pad = self.fingertip.parameters.pad
        phalanx = self.fingertip.parameters.phalanx
        lower_z_mm = -pad.pad_straight_height_mm - pad.ellipse_height_mm
        upper_z_mm = phalanx.carrier_height_mm

        axis.set_title("3D analytic model")
        axis.set_xlabel("X [mm]")
        axis.set_ylabel("Y [mm]")
        axis.set_zlabel("Z [mm]")
        axis.set_xlim(-self.fingertip.half_width_mm * 1.15, self.fingertip.half_width_mm * 1.15)
        axis.set_ylim(0.0, self.fingertip.total_length_mm)
        axis.set_zlim(lower_z_mm - 1.0, upper_z_mm + 1.0)
        axis.set_box_aspect(
            (
                self.fingertip.half_width_mm * 2.3,
                self.fingertip.total_length_mm,
                upper_z_mm - lower_z_mm + 2.0,
            )
        )
        axis.view_init(elev=23.0, azim=-62.0)
        axis.grid(True, alpha=0.25)
        axis.legend(
            handles=[
                Patch(facecolor="#4ea8de", edgecolor="#0077b6", alpha=0.55, label="compliant envelope"),
                Patch(facecolor="#f4a261", edgecolor="#c45a00", alpha=0.78, label="rigid carrier core"),
                Patch(facecolor="#e76f51", edgecolor="#b23a2f", alpha=0.78, label="distal dorsal plate"),
            ],
            loc="upper left",
            fontsize=8,
        )

    def _surface_grid(
        self,
        surface: str,
        upper: bool,
    ) -> tuple[list[list[float]], list[list[float]], list[list[float]]]:
        if surface == "body":
            y_values = self._linspace(
                0.0,
                self.fingertip.parameters.pad.body_length_mm,
                self.samples,
            )
        elif surface == "nose":
            y_values = self._linspace(
                self.fingertip.parameters.pad.body_length_mm,
                self.fingertip.total_length_mm,
                self.samples,
            )
        else:
            raise ValueError(f"unknown surface region: {surface!r}")

        normalized_x_values = self._linspace(-1.0, 1.0, self.samples)
        x_grid: list[list[float]] = []
        y_grid: list[list[float]] = []
        z_grid: list[list[float]] = []

        for y_mm in y_values:
            half_width_mm = self.fingertip.half_width_at_y_mm(y_mm)
            x_row = [half_width_mm * normalized_x for normalized_x in normalized_x_values]
            z_row = [
                (
                    self.fingertip.upper_surface_z_mm(x_mm, y_mm)
                    if upper
                    else self.fingertip.lower_surface_z_mm(x_mm, y_mm)
                )
                for x_mm in x_row
            ]
            x_grid.append(x_row)
            y_grid.append([y_mm] * len(normalized_x_values))
            z_grid.append(z_row)

        return x_grid, y_grid, z_grid

    @staticmethod
    def _add_extruded_polygon(
        axis: Axes3D,
        polygon: tuple[tuple[float, float], ...],
        start_y_mm: float,
        end_y_mm: float,
        face_color: str,
        edge_color: str,
        alpha: float,
    ) -> None:
        side_faces = []
        for start_point, end_point in zip(polygon, (*polygon[1:], polygon[0])):
            x_start, z_start = start_point
            x_end, z_end = end_point
            side_faces.append(
                [
                    (x_start, start_y_mm, z_start),
                    (x_end, start_y_mm, z_end),
                    (x_end, end_y_mm, z_end),
                    (x_start, end_y_mm, z_start),
                ]
            )

        cap_faces = [
            [(x_mm, start_y_mm, z_mm) for x_mm, z_mm in polygon],
            [(x_mm, end_y_mm, z_mm) for x_mm, z_mm in polygon],
        ]
        collection = Poly3DCollection(
            (*side_faces, *cap_faces),
            facecolors=face_color,
            edgecolors=edge_color,
            linewidths=0.7,
            alpha=alpha,
        )
        axis.add_collection3d(collection)

    @staticmethod
    def _linspace(start: float, stop: float, count: int) -> list[float]:
        if count < 2:
            return [start]
        step = (stop - start) / (count - 1)
        return [start + index * step for index in range(count)]


__all__ = ["FingertipVisualizer"]
