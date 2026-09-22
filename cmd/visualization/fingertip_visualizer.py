"""Orthographic 2D and interactive 3D views of the analytic fingertip."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.patches import Patch, Polygon
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d.axes3d import Axes3D

from ..morphology import Fingertip


CARRIER_COLOR = "#999c9f"
PAD_COLOR = "#b8d5a8"
OUTLINE_COLOR = "#50575b"


@dataclass(frozen=True)
class FingertipVisualizer:
    """Render three orthographic silhouettes and one 3D fingertip view.

    The visualizer consumes the analytic ``Fingertip`` representation directly
    and does not create a mesh or introduce a second geometry definition. The
    lower and upper contact surfaces are sampled from the analytic model. The
    stem is extruded along ``Y``; the dorsal nail follows its own rounded
    footprint and extends beyond the stem.
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
        """Create end, side, top, and rotatable 3D views in one figure."""

        figure = plt.figure(figsize=(12, 9), constrained_layout=True)
        for position, view in enumerate(("end", "side", "top"), start=1):
            self._plot_2d(figure.add_subplot(2, 2, position), view)
        axis = figure.add_subplot(2, 2, 4, projection="3d")
        self._plot_outer_envelope(axis)
        self._plot_carrier(axis)
        self._configure_3d_axis(axis)
        figure.suptitle("Fingertip morphology", fontsize=16)
        figure.legend(handles=[
            Patch(facecolor=CARRIER_COLOR, edgecolor=OUTLINE_COLOR, label="Carrier / nail"),
            Patch(facecolor=PAD_COLOR, edgecolor=OUTLINE_COLOR, label="Pad"),
        ], loc="upper center", bbox_to_anchor=(0.5, 0.965), ncol=2, frameon=False)
        return figure

    def _plot_2d(self, axis: Axes, view: str) -> None:
        """Project the model envelope; carrier hides pad where they overlap."""

        f = self.fingertip
        phalanx = f.parameters.phalanx
        x = np.linspace(-f.half_width_mm, f.half_width_mm, self.samples)
        y = np.unique(np.r_[np.linspace(0.0, f.total_length_mm, self.samples),
                             f.parameters.pad.body_length_mm, f.carrier_length_mm])
        if view == "end":
            lower = [f.lower_surface_z_mm(float(value), 0.0) for value in x]
            axis.fill_between(x, lower, 0.0, color=PAD_COLOR, ec=OUTLINE_COLOR, lw=1)
            carrier = f.carrier_cross_section
            title, xlabel, ylabel = "End view · XZ", "X [mm]", "Z [mm]"
        elif view == "side":
            # Include the centre explicitly, even for an even sample count.
            x_samples = np.unique(np.r_[x, 0.0])
            lower, upper = [], []
            for value in y:
                bounds = [f.outer_envelope_z_bounds_mm(float(xx), float(value))
                          for xx in x_samples]
                lower.append(min(pair[0] for pair in bounds))
                upper.append(max(pair[1] for pair in bounds))
            axis.fill_between(y, lower, upper, color=PAD_COLOR, ec=OUTLINE_COLOR, lw=1)
            carrier = [(0.0, -phalanx.stem_height_mm),
                       (f.carrier_length_mm, -phalanx.stem_height_mm),
                       (f.carrier_length_mm, 0.0), (f.dorsal_plate_length_mm, 0.0),
                       (f.dorsal_plate_length_mm, phalanx.carrier_height_mm),
                       (0.0, phalanx.carrier_height_mm)]
            title, xlabel, ylabel = "Side view · YZ", "Y [mm]", "Z [mm]"
        elif view == "top":
            pad_width = [f.half_width_at_y_mm(float(value)) for value in y]
            axis.fill_betweenx(y, -np.asarray(pad_width), pad_width,
                               color=PAD_COLOR, ec=OUTLINE_COLOR, lw=1)
            nail_width = [f.dorsal_plate_half_width_at_y_mm(float(value)) for value in y]
            carrier = [(-width, float(value)) for width, value in zip(nail_width, y)]
            carrier += [(width, float(value)) for width, value in zip(nail_width[::-1], y[::-1])]
            title, xlabel, ylabel = "Top view · XY", "X [mm]", "Y [mm]"
        else:
            raise ValueError(f"unknown view: {view!r}")
        axis.add_patch(Polygon(carrier, closed=True, facecolor=CARRIER_COLOR,
                               edgecolor=OUTLINE_COLOR, linewidth=1.1, zorder=3))
        axis.set(title=title, xlabel=xlabel, ylabel=ylabel)
        axis.set_aspect("equal", adjustable="box")
        axis.margins(0.12)
        axis.set_axisbelow(True)
        axis.grid(True, color="#dce1e4", linewidth=0.6)
        axis.spines[["top", "right"]].set_visible(False)

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
        pad_color = PAD_COLOR
        lower_color = PAD_COLOR

        for surface in ("body", "nose"):
            x_grid, y_grid, lower_grid = self._surface_grid(surface, upper=False)
            axis.plot_surface(
                np.asarray(x_grid),
                np.asarray(y_grid),
                np.asarray(lower_grid),
                color=lower_color,
                alpha=1.0,
                linewidth=0,
                antialiased=False,
                shade=False,
            )

            _, _, upper_grid = self._surface_grid(surface, upper=True)
            axis.plot_surface(
                np.asarray(x_grid),
                np.asarray(y_grid),
                np.asarray(upper_grid),
                color=pad_color,
                alpha=1.0,
                linewidth=0,
                antialiased=False,
                shade=False,
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
                color=PAD_COLOR,
                alpha=1.0,
                linewidth=0,
                antialiased=False,
                shade=False,
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
                color=PAD_COLOR,
                alpha=1.0,
                linewidth=0,
                antialiased=False,
                shade=False,
            )

    def _plot_carrier(self, axis: Axes3D) -> None:
        fingertip = self.fingertip
        pad = fingertip.parameters.pad
        phalanx = fingertip.parameters.phalanx
        stem_end = fingertip.carrier_length_mm
        stem_half_width = phalanx.stem_width_mm / 2.0
        stem_bottom = -phalanx.stem_height_mm
        plate_top = phalanx.carrier_height_mm

        # Split at the stem end so the nail underside starts there. Do not
        # extrude the whole T section to the fingertip or add an internal cap.
        y_values = sorted(set([
            0.0,
            stem_end,
            *self._linspace(pad.body_length_mm, fingertip.dorsal_plate_length_mm, self.samples),
        ]))
        faces = []
        for y0, y1 in zip(y_values, y_values[1:]):
            w0 = fingertip.dorsal_plate_half_width_at_y_mm(y0)
            w1 = fingertip.dorsal_plate_half_width_at_y_mm(y1)
            faces.append([(-w0, y0, plate_top), (w0, y0, plate_top),
                          (w1, y1, plate_top), (-w1, y1, plate_top)])
            for sign in (-1, 1):
                faces.append([(sign * w0, y0, 0.0), (sign * w1, y1, 0.0),
                              (sign * w1, y1, plate_top), (sign * w0, y0, plate_top)])
            if y1 <= stem_end:
                for sign in (-1, 1):
                    faces.append([(sign * stem_half_width, y0, 0.0),
                                  (sign * w0, y0, 0.0), (sign * w1, y1, 0.0),
                                  (sign * stem_half_width, y1, 0.0)])
            else:
                faces.append([(-w0, y0, 0.0), (-w1, y1, 0.0),
                              (w1, y1, 0.0), (w0, y0, 0.0)])

        a = fingertip.half_width_mm
        s = stem_half_width
        faces.append([(-a, 0.0, 0.0), (a, 0.0, 0.0),
                      (a, 0.0, plate_top), (-a, 0.0, plate_top)])
        faces.append([(-s, 0.0, stem_bottom), (s, 0.0, stem_bottom),
                      (s, stem_end, stem_bottom), (-s, stem_end, stem_bottom)])
        for x in (-s, s):
            faces.append([(x, 0.0, stem_bottom), (x, stem_end, stem_bottom),
                          (x, stem_end, 0.0), (x, 0.0, 0.0)])
        for y in (0.0, stem_end):
            faces.append([(-s, y, stem_bottom), (s, y, stem_bottom),
                          (s, y, 0.0), (-s, y, 0.0)])

        axis.add_collection3d(Poly3DCollection(
            faces, facecolors=CARRIER_COLOR, edgecolors="none", alpha=1.0,
        ))

    def _configure_3d_axis(self, axis: Axes3D) -> None:
        pad = self.fingertip.parameters.pad
        phalanx = self.fingertip.parameters.phalanx
        lower_z_mm = -pad.pad_straight_height_mm - pad.ellipse_height_mm
        upper_z_mm = phalanx.carrier_height_mm

        axis.set_title("3D view")
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
    def _linspace(start: float, stop: float, count: int) -> list[float]:
        if count < 2:
            return [start]
        step = (stop - start) / (count - 1)
        return [start + index * step for index in range(count)]


__all__ = ["FingertipVisualizer"]
