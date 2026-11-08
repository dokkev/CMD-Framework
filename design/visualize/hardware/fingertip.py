"""Orthographic 2D and interactive 3D views of the analytic fingertip.

    python -m visualize.hardware.fingertip [--output fingertip.png] [--show]
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.patches import Patch, Polygon
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d.axes3d import Axes3D

from core.morphology import Fingertip


# Keep the carrier and compliant pad visually distinct in every view.
CARRIER_COLOR = "#999c9f"
PAD_COLOR = "#b8d5a8"
OUTLINE_COLOR = "#50575b"

# The 3D view keeps a cool gray carrier and green pad, with darker lines for
# mesh and silhouette readability.
CARRIER_3D_COLOR = "#7d8992"
CARRIER_3D_EDGE_COLOR = "#263238"
PAD_3D_UPPER_COLOR = "#c6e5be"
PAD_3D_LOWER_COLOR = "#6fa878"
PAD_3D_WALL_COLOR = "#98c99c"
PAD_3D_MESH_COLOR = "#52755a"
PAD_3D_EDGE_COLOR = "#2f5638"


@dataclass(frozen=True)
class FingertipVisualizer:
    """Render three orthographic silhouettes and one 3D fingertip view.

    The visualizer consumes the analytic ``Fingertip`` representation directly
    and does not create a mesh or introduce a second geometry definition. The
    lower and upper contact surfaces are sampled from the analytic model. The
    stem is extruded along ``X``; the dorsal nail follows its own rounded
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
            Patch(facecolor=CARRIER_3D_COLOR, edgecolor=CARRIER_3D_EDGE_COLOR,
                  label="Carrier / nail"),
            Patch(facecolor=PAD_3D_UPPER_COLOR, edgecolor=PAD_3D_EDGE_COLOR,
                  label="Pad (mesh + boundary)"),
        ], loc="upper center", bbox_to_anchor=(0.5, 0.965), ncol=2, frameon=False)
        return figure

    def _plot_2d(self, axis: Axes, view: str) -> None:
        """Project the model envelope; carrier hides pad where they overlap."""

        f = self.fingertip
        phalanx = f.parameters.family.phalanx
        y = np.linspace(-f.half_width_mm, f.half_width_mm, self.samples)
        x = np.unique(np.r_[np.linspace(0.0, f.total_length_mm, self.samples),
                             f.straight_length_mm, f.carrier_length_mm])
        if view == "end":
            lower = [f.lower_surface_z_mm(0.0, float(value)) for value in y]
            axis.fill_between(y, lower, 0.0, color=PAD_COLOR, ec=OUTLINE_COLOR, lw=1)
            carrier = f.carrier_cross_section
            title, xlabel, ylabel = "End view · YZ", "Y [mm]", "Z [mm]"
        elif view == "side":
            # Include the centre explicitly, even for an even sample count.
            lower, upper = [], []
            for value in x:
                half_width = f.half_width_at_x_mm(float(value))
                y_samples = np.unique(
                    np.r_[np.linspace(-half_width, half_width, self.samples), 0.0]
                )
                bounds = [f.outer_envelope_z_bounds_mm(float(value), float(yy))
                          for yy in y_samples]
                lower.append(min(pair[0] for pair in bounds))
                upper.append(max(pair[1] for pair in bounds))
            axis.fill_between(x, lower, upper, color=PAD_COLOR, ec=OUTLINE_COLOR, lw=1)
            carrier = [(0.0, -phalanx.stem_height_mm),
                       (f.carrier_length_mm, -phalanx.stem_height_mm),
                       (f.carrier_length_mm, 0.0), (f.dorsal_plate_length_mm, 0.0),
                       (f.dorsal_plate_length_mm, phalanx.carrier_height_mm),
                       (0.0, phalanx.carrier_height_mm)]
            title, xlabel, ylabel = "Side view · XZ", "X [mm]", "Z [mm]"
        elif view == "top":
            pad_width = [f.half_width_at_x_mm(float(value)) for value in x]
            axis.fill_between(x, -np.asarray(pad_width), pad_width,
                              color=PAD_COLOR, ec=OUTLINE_COLOR, lw=1)
            nail_width = [f.dorsal_plate_half_width_at_x_mm(float(value)) for value in x]
            carrier = [(float(value), -width) for width, value in zip(nail_width, x)]
            carrier += [(float(value), width) for width, value in zip(nail_width[::-1], x[::-1])]
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
        mesh_count = max(2, min(self.samples, 36))

        for surface in ("body", "nose"):
            x_grid, y_grid, lower_grid = self._surface_grid(surface, upper=False)
            axis.plot_surface(
                np.asarray(x_grid),
                np.asarray(y_grid),
                np.asarray(lower_grid),
                color=PAD_3D_LOWER_COLOR,
                alpha=0.96,
                edgecolor=PAD_3D_MESH_COLOR,
                linewidth=0.35,
                antialiased=True,
                shade=True,
                rcount=mesh_count,
                ccount=mesh_count,
            )

            _, _, upper_grid = self._surface_grid(surface, upper=True)
            axis.plot_surface(
                np.asarray(x_grid),
                np.asarray(y_grid),
                np.asarray(upper_grid),
                color=PAD_3D_UPPER_COLOR,
                alpha=0.72,
                edgecolor=PAD_3D_MESH_COLOR,
                linewidth=0.35,
                antialiased=True,
                shade=True,
                rcount=mesh_count,
                ccount=mesh_count,
            )

        self._plot_lateral_walls(axis)
        self._plot_pad_boundaries(axis)

    def _plot_lateral_walls(self, axis: Axes3D) -> None:
        mesh_count = max(2, min(self.samples, 36))
        body_x_values = self._linspace(
            0.0,
            self.fingertip.straight_length_mm,
            self.samples,
        )
        for y_mm in (-self.fingertip.half_width_mm, self.fingertip.half_width_mm):
            axis.plot_surface(
                np.asarray([body_x_values, body_x_values]),
                np.asarray([[y_mm] * len(body_x_values)] * 2),
                np.asarray(
                    [
                        [
                            self.fingertip.lower_surface_z_mm(x_mm, y_mm)
                            for x_mm in body_x_values
                        ],
                        [
                            self.fingertip.upper_surface_z_mm(x_mm, y_mm)
                            for x_mm in body_x_values
                        ],
                    ]
                ),
                color=PAD_3D_WALL_COLOR,
                alpha=0.94,
                edgecolor=PAD_3D_MESH_COLOR,
                linewidth=0.35,
                antialiased=True,
                shade=True,
                rcount=2,
                ccount=mesh_count,
            )

        nose_x_values = self._linspace(
            self.fingertip.straight_length_mm,
            self.fingertip.total_length_mm,
            self.samples,
        )
        half_widths = [
            self.fingertip.half_width_at_x_mm(x_mm) for x_mm in nose_x_values
        ]
        axis.plot_surface(
            np.asarray([nose_x_values, nose_x_values]),
            np.asarray([-np.asarray(half_widths), np.asarray(half_widths)]),
            np.asarray(
                [
                    [
                        self.fingertip.lower_surface_z_mm(x_mm, width)
                        for x_mm, width in zip(nose_x_values, half_widths)
                    ],
                    [
                        self.fingertip.upper_surface_z_mm(x_mm, width)
                        for x_mm, width in zip(nose_x_values, half_widths)
                    ],
                ]
            ),
            color=PAD_3D_WALL_COLOR,
            alpha=0.94,
            edgecolor=PAD_3D_MESH_COLOR,
            linewidth=0.35,
            antialiased=True,
            shade=True,
            rcount=2,
            ccount=mesh_count,
        )

    def _plot_pad_boundaries(self, axis: Axes3D) -> None:
        """Overlay dark silhouette and region-seam lines on the pad."""

        fingertip = self.fingertip
        body_x_values = np.asarray(self._linspace(
            0.0,
            fingertip.straight_length_mm,
            self.samples,
        ))
        for y_mm in (-fingertip.half_width_mm, fingertip.half_width_mm):
            axis.plot(
                body_x_values,
                np.full_like(body_x_values, y_mm),
                [fingertip.lower_surface_z_mm(x_mm, y_mm) for x_mm in body_x_values],
                color=PAD_3D_EDGE_COLOR,
                linewidth=1.15,
                zorder=5,
            )
            axis.plot(
                body_x_values,
                np.full_like(body_x_values, y_mm),
                [fingertip.upper_surface_z_mm(x_mm, y_mm) for x_mm in body_x_values],
                color=PAD_3D_EDGE_COLOR,
                linewidth=1.15,
                zorder=5,
            )

        nose_x_values = np.asarray(self._linspace(
            fingertip.straight_length_mm,
            fingertip.total_length_mm,
            self.samples,
        ))
        nose_half_widths = np.asarray([
            fingertip.half_width_at_x_mm(x_mm) for x_mm in nose_x_values
        ])
        for sign in (-1.0, 1.0):
            nose_y_values = sign * nose_half_widths
            axis.plot(
                nose_x_values,
                nose_y_values,
                [
                    fingertip.lower_surface_z_mm(x_mm, abs(y_mm))
                    for x_mm, y_mm in zip(nose_x_values, nose_y_values)
                ],
                color=PAD_3D_EDGE_COLOR,
                linewidth=1.15,
                zorder=5,
            )
            axis.plot(
                nose_x_values,
                nose_y_values,
                [
                    fingertip.upper_surface_z_mm(x_mm, abs(y_mm))
                    for x_mm, y_mm in zip(nose_x_values, nose_y_values)
                ],
                color=PAD_3D_EDGE_COLOR,
                linewidth=1.15,
                zorder=5,
            )

        for x_mm in (0.0, fingertip.straight_length_mm, fingertip.total_length_mm):
            half_width_mm = fingertip.half_width_at_x_mm(x_mm)
            y_values = np.asarray(self._linspace(
                -half_width_mm,
                half_width_mm,
                self.samples,
            ))
            axis.plot(
                np.full_like(y_values, x_mm),
                y_values,
                [fingertip.lower_surface_z_mm(x_mm, y_mm) for y_mm in y_values],
                color=PAD_3D_EDGE_COLOR,
                linewidth=1.0,
                zorder=5,
            )
            axis.plot(
                np.full_like(y_values, x_mm),
                y_values,
                [fingertip.upper_surface_z_mm(x_mm, y_mm) for y_mm in y_values],
                color=PAD_3D_EDGE_COLOR,
                linewidth=1.0,
                zorder=5,
            )

    def _plot_carrier(self, axis: Axes3D) -> None:
        fingertip = self.fingertip
        phalanx = fingertip.parameters.family.phalanx
        stem_end = fingertip.carrier_length_mm
        stem_half_width = phalanx.stem_width_mm / 2.0
        stem_bottom = -phalanx.stem_height_mm
        plate_top = phalanx.carrier_height_mm

        # Split at the stem end so the nail underside starts there. Do not
        # extrude the whole T section to the fingertip or add an internal cap.
        x_values = sorted(set([
            0.0,
            stem_end,
            *self._linspace(fingertip.straight_length_mm,
                            fingertip.dorsal_plate_length_mm, self.samples),
        ]))
        faces = []
        for x0, x1 in zip(x_values, x_values[1:]):
            w0 = fingertip.dorsal_plate_half_width_at_x_mm(x0)
            w1 = fingertip.dorsal_plate_half_width_at_x_mm(x1)
            faces.append([(x0, -w0, plate_top), (x0, w0, plate_top),
                          (x1, w1, plate_top), (x1, -w1, plate_top)])
            for sign in (-1, 1):
                faces.append([(x0, sign * w0, 0.0), (x1, sign * w1, 0.0),
                              (x1, sign * w1, plate_top), (x0, sign * w0, plate_top)])
            if x1 <= stem_end:
                for sign in (-1, 1):
                    faces.append([(x0, sign * stem_half_width, 0.0),
                                  (x0, sign * w0, 0.0), (x1, sign * w1, 0.0),
                                  (x1, sign * stem_half_width, 0.0)])
            else:
                faces.append([(x0, -w0, 0.0), (x1, -w1, 0.0),
                              (x1, w1, 0.0), (x0, w0, 0.0)])

        a = fingertip.tip_half_width_mm
        s = stem_half_width
        faces.append([(0.0, -a, 0.0), (0.0, a, 0.0),
                      (0.0, a, plate_top), (0.0, -a, plate_top)])
        faces.append([(0.0, -s, stem_bottom), (0.0, s, stem_bottom),
                      (stem_end, s, stem_bottom), (stem_end, -s, stem_bottom)])
        for y in (-s, s):
            faces.append([(0.0, y, stem_bottom), (stem_end, y, stem_bottom),
                          (stem_end, y, 0.0), (0.0, y, 0.0)])
        for x in (0.0, stem_end):
            faces.append([(x, -s, stem_bottom), (x, s, stem_bottom),
                          (x, s, 0.0), (x, -s, 0.0)])

        axis.add_collection3d(Poly3DCollection(
            faces,
            facecolors=CARRIER_3D_COLOR,
            edgecolors=CARRIER_3D_EDGE_COLOR,
            linewidths=0.55,
            alpha=0.96,
        ))

    def _configure_3d_axis(self, axis: Axes3D) -> None:
        pad = self.fingertip.parameters.family.pad
        phalanx = self.fingertip.parameters.family.phalanx
        lower_z_mm = -pad.vertical_pad_height_mm - pad.elliptical_pad_height_mm
        upper_z_mm = phalanx.carrier_height_mm
        maximum_half_width_mm = max(
            self.fingertip.half_width_mm,
            self.fingertip.tip_half_width_mm,
        )

        axis.set_title("3D view")
        axis.set_xlabel("X [mm]")
        axis.set_ylabel("Y [mm]")
        axis.set_zlabel("Z [mm]")
        axis.set_xlim(0.0, self.fingertip.total_length_mm)
        axis.set_ylim(-maximum_half_width_mm * 1.15, maximum_half_width_mm * 1.15)
        axis.set_zlim(lower_z_mm - 1.0, upper_z_mm + 1.0)
        axis.set_box_aspect(
            (
                self.fingertip.total_length_mm,
                maximum_half_width_mm * 2.3,
                upper_z_mm - lower_z_mm + 2.0,
            )
        )
        axis.view_init(elev=23.0, azim=-152.0)
        axis.grid(True, alpha=0.25)

    def _surface_grid(
        self,
        surface: str,
        upper: bool,
    ) -> tuple[list[list[float]], list[list[float]], list[list[float]]]:
        if surface == "body":
            x_values = self._linspace(
                0.0,
                self.fingertip.straight_length_mm,
                self.samples,
            )
        elif surface == "nose":
            x_values = self._linspace(
                self.fingertip.straight_length_mm,
                self.fingertip.total_length_mm,
                self.samples,
            )
        else:
            raise ValueError(f"unknown surface region: {surface!r}")

        normalized_y_values = self._linspace(-1.0, 1.0, self.samples)
        x_grid: list[list[float]] = []
        y_grid: list[list[float]] = []
        z_grid: list[list[float]] = []

        for x_mm in x_values:
            half_width_mm = self.fingertip.half_width_at_x_mm(x_mm)
            y_row = [half_width_mm * normalized_y for normalized_y in normalized_y_values]
            z_row = [
                (
                    self.fingertip.upper_surface_z_mm(x_mm, y_mm)
                    if upper
                    else self.fingertip.lower_surface_z_mm(x_mm, y_mm)
                )
                for y_mm in y_row
            ]
            x_grid.append([x_mm] * len(normalized_y_values))
            y_grid.append(y_row)
            z_grid.append(z_row)

        return x_grid, y_grid, z_grid

    @staticmethod
    def _linspace(start: float, stop: float, count: int) -> list[float]:
        if count < 2:
            return [start]
        step = (stop - start) / (count - 1)
        return [start + index * step for index in range(count)]


__all__ = ["FingertipVisualizer"]


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Show end, side, top, and 3D views of the analytic fingertip."
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="save a PNG/PDF/SVG figure instead of only opening a window",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=160,
        help="output resolution when --output is used (default: 160)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=200,
        help="number of longitudinal samples (default: 200)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="open the interactive Matplotlib window after saving",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    visualizer = FingertipVisualizer(Fingertip(), samples=args.samples)
    figure = visualizer.create_figure()

    if args.output is not None:
        figure.savefig(args.output, dpi=args.dpi, bbox_inches="tight")

    if args.show or args.output is None:
        plt.show()
    else:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
