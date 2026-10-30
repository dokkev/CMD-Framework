"""The serial 2R (revolute) finger's solids and the gaps between them.

The MCP is at ``O`` and the distal joint at ``A = (|OA|, 0)``; the distal
segment ``A-C`` turns by ``phi`` about ``A`` (positive flexion turns ``+X``
toward ``-Z``), the same rotation of the distal segment relative to the
proximal one as the linkage's ``q_pip + q_dip``. The distal servo's shaft is
on ``A`` (:attr:`JointChecks.joint_servo`), its body fixed to the proximal
segment and laid back along ``-X`` tilted dorsally by ``servo_tilt_deg``;
without it (the idealized 2R) only ``A``'s pin housing stands at the joint.
Each pad sits on its own segment, a drop below its line; the distal pad on a
carrier plate (``packaging.distal_carrier_height_mm``). The distal phalanx is
a truss from ``A`` to the pad's two ends, in other layers than the servo.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

from core.morphology.packaging.faces import fold_free_lengths
from core.morphology.packaging.gaps import footprint_gaps, mcp_servo_gaps, polygon_gaps, proximal_pad_mcp_gap
from core.morphology.packaging.hardware import Packaging
from core.morphology.packaging.shapes import (
    PadSpan,
    ServoFootprint,
    carrier_outline,
    pad_outline,
    polygon_to_disk,
    segment_polygon_gaps,
)


VARIABLE_NAMES = ("a_x_mm", "ac_length_mm", "app_x_mm", "bpp_from_a_mm", "distal_drop_mm", "proximal_drop_mm",
                  "servo_tilt_deg")
"""``|OA|``, ``|AC|``, the proximal pad's end ``A''_x``, the distal pad's
joint end ``B''`` from ``A`` along ``A->C``, how far below ``A-C`` the distal
pad's top face lies and below ``O-A`` the proximal pad's, and the distal
servo's dorsal tilt from ``-X`` (degrees; the goal's PIP servo tilt, fixed or
searched). The pads are plain blocks, each on its own segment and off the
joint's housing (:data:`DISTAL_DROP_MIN_MM`'s radius): the proximal pad ends
at ``A_x`` less it, the distal pad starts at it past ``A``."""


def servo_rear(tilt_deg: float) -> tuple[float, float]:
    """The direction the distal servo's body lies back in from its shaft on
    ``A``: ``-X`` turned dorsally by ``tilt_deg``."""

    tilt = math.radians(tilt_deg)
    return (-math.cos(tilt), math.sin(tilt))


@dataclass(frozen=True)
class RevoluteGeometry:
    """A design's solids in the reference pose: the proximal pad and servos in
    the proximal frame, the distal pad in the distal frame (``u`` from ``A``
    along ``A->C``, ``v`` dorsal)."""

    a_x_mm: float
    ac_length_mm: float
    proximal_pad: np.ndarray
    distal_pad_local: np.ndarray
    distal_drop_mm: float
    proximal_drop_mm: float
    distal_servo: ServoFootprint
    proximal_length_mm: float
    distal_length_mm: float
    proximal_face_mm: float
    distal_face_mm: float
    proximal_room_mm: float
    distal_room_mm: float
    distal_carrier_local: np.ndarray | None = None
    """The distal pad's carrier plate in the distal frame (``None``: none)."""
    proximal_carrier: np.ndarray | None = None
    """The proximal support's plate in the proximal frame
    (``goal.collision.param.proximal_carrier_height_mm``; ``None``: none)."""
    distal_mount_local: np.ndarray | None = None
    """The distal mounting line, ``B''`` then the fingertip end, in the distal
    frame (``None``: the pad's top face): the support, there whatever module it carries."""

    @property
    def finger_length_mm(self) -> float:
        return self.a_x_mm + self.ac_length_mm

    def distal_pad(self, phi_rad: np.ndarray) -> np.ndarray:
        """The distal pad at distal flexions ``phi`` (``(n,)``), ``(n, m, 2)`` in the proximal frame."""

        return self.posed(self.distal_pad_local, phi_rad)

    def posed(self, local: np.ndarray, phi_rad: np.ndarray) -> np.ndarray:
        """Points ``(m, 2)`` of the distal frame at ``phi``, ``(n, m, 2)`` in the proximal frame."""

        phi = np.atleast_1d(np.asarray(phi_rad, dtype=float))
        e = np.stack((np.cos(phi), -np.sin(phi)), axis=1)
        n = np.stack((-e[:, 1], e[:, 0]), axis=1)
        a = np.array((self.a_x_mm, 0.0))
        local = np.asarray(local, dtype=float)
        return a + local[None, :, :1] * e[:, None] + local[None, :, 1:] * n[:, None]

    @property
    def distal_reach_mm(self) -> float:
        """The distal pad's farthest point from ``A``: how fast it moves per
        radian of ``phi``, at most."""

        solids = self.distal_pad_local if self.distal_carrier_local is None else np.vstack(
            (self.distal_pad_local, self.distal_carrier_local))
        return float(np.hypot(solids[:, 0], solids[:, 1]).max())

    @property
    def distal_mount(self) -> np.ndarray:
        """The distal pad's mounting line (its top face), ``B''`` then ``C``'s
        end, in the distal frame: the truss's two pad-side nodes."""

        if self.distal_mount_local is not None:
            return self.distal_mount_local
        top = self.distal_pad_local[:, 1].max()
        return np.array(((self.distal_pad_local[:, 0].min(), top), (self.distal_pad_local[:, 0].max(), top)))

    def tip(self, phi_rad: np.ndarray) -> np.ndarray:
        """``C`` at distal flexions ``phi``, ``(n, 2)``."""

        phi = np.atleast_1d(np.asarray(phi_rad, dtype=float))
        return np.stack((self.a_x_mm + self.ac_length_mm * np.cos(phi), -self.ac_length_mm * np.sin(phi)), axis=1)


def geometry(x: Sequence[float], packaging: Packaging,
             proximal_carrier_height_mm: float | None = None) -> RevoluteGeometry:
    """The solids of a design vector; ``proximal_carrier_height_mm``: the
    proximal support as a plate (:func:`proximal_support`), if any."""

    v = dict(zip(VARIABLE_NAMES, (float(value) for value in x)))
    proximal, distal = packaging.proximal_pad, packaging.distal_pad
    start = packaging.proximal_pad_start_mm
    proximal_room = v["app_x_mm"] - start
    distal_room = v["ac_length_mm"] - distal.offset_mm - v["bpp_from_a_mm"]
    # A module of a set length (a fixed finger's module sweep), else the room's, at most its max.
    proximal_length = proximal.length_mm or min(proximal_room, proximal.max_length_mm or math.inf)
    distal_length = distal.length_mm or min(distal_room, distal.max_length_mm or math.inf)
    chamfer = {side: pad.chamfer_mm for side, pad in (("proximal", proximal), ("distal", distal))}  # 0: blocks
    outer = v["ac_length_mm"] - distal.offset_mm
    proximal_outline = pad_outline(PadSpan(start, start + max(proximal_length, 1e-6), proximal_room),
                                   proximal, "end", chamfer["proximal"]) - (0.0, v["proximal_drop_mm"])
    distal_span = PadSpan(outer - max(distal_length, 1e-6), outer, distal_room)
    distal_outline = pad_outline(distal_span, distal, "start", chamfer["distal"]) - (0.0, v["distal_drop_mm"])
    servo = ServoFootprint((v["a_x_mm"], 0.0), servo_rear(v["servo_tilt_deg"]),
                           packaging.pip_servo.radius_mm, packaging.pip_servo.rear_extent_mm)
    face = lambda length, k: max(length - min(k, max(length, 0.0)), 0.0)  # noqa: E731
    # The support: the mounting line from B'' to the fingertip end, whatever the module.
    support = pad_outline(PadSpan(outer - max(distal_room, 1e-6), outer, distal_room), distal, "start", 0.0) \
        - (0.0, v["distal_drop_mm"])
    mount = np.array(((outer - distal_room, -v["distal_drop_mm"]), (outer, -v["distal_drop_mm"])))
    return RevoluteGeometry(v["a_x_mm"], v["ac_length_mm"], proximal_outline, distal_outline, v["distal_drop_mm"],
                            v["proximal_drop_mm"], servo,
                            proximal_length, distal_length, face(proximal_length, chamfer["proximal"]),
                            face(distal_length, chamfer["distal"]), proximal_room, distal_room,
                            carrier_local(support, v["distal_drop_mm"], packaging),
                            proximal_support(start, proximal_room, v["proximal_drop_mm"], proximal_carrier_height_mm),
                            mount)


def proximal_support(start_mm: float, room_mm: float, drop_mm: float, height: float | None) -> np.ndarray | None:
    """The proximal support's plate (``height`` mm dorsal of the mounting
    line, from the pad's outer end to ``A''``), as the linkage's
    (``packaging.proximal_carrier_polygon``); ``None`` without one."""

    if height is None:
        return None
    end = start_mm + max(room_mm, 1e-6)
    return np.array(((start_mm, -drop_mm), (end, -drop_mm), (end, height - drop_mm), (start_mm, height - drop_mm)))


def carrier_local(distal_pad_local: np.ndarray, drop_mm: float, packaging) -> np.ndarray | None:
    """The distal pad's carrier plate (``packaging.distal_carrier_height_mm``
    dorsal of the pad's top face, over its span) in the distal frame, as the
    linkage's (``packaging.distal_carrier_polygon``); ``None`` without one."""

    if packaging.distal_carrier_height_mm <= 0.0:
        return None
    return carrier_outline(distal_pad_local + (0.0, drop_mm), packaging.distal_carrier_height_mm) - (0.0, drop_mm)


def contact_faces(geo, phi_rad: np.ndarray):
    """The two contact faces as ``packaging.fold_free_lengths`` takes them:
    each pad's palmar face, outer end first, at distal flexions ``phi``
    (any geometry with ``proximal_pad``, ``distal_pad_local`` and ``posed``)."""

    proximal = geo.proximal_pad
    z = float(proximal[:, 1].min())
    proximal_face = np.array(((proximal[:, 0].min(), z), (proximal[:, 0].max(), z)))
    local = geo.distal_pad_local
    v = float(local[:, 1].min())
    distal_face = geo.posed(np.array(((local[:, 0].max(), v), (local[:, 0].min(), v))), phi_rad)
    origin, palmar = np.moveaxis(geo.posed(np.array(((0.0, 0.0), (0.0, -1.0))), phi_rad), 1, 0)
    return proximal_face, np.array((0.0, -1.0)), distal_face, palmar - origin


def fold_free(geo, phi_rad: np.ndarray) -> dict[str, np.ndarray]:
    """Each face's length the other leaves unhidden at ``phi`` (``(n,)`` each)."""

    return fold_free_lengths(*contact_faces(geo, np.atleast_1d(phi_rad)))


@dataclass(frozen=True)
class JointChecks:
    """What a 2R's motion is checked against."""

    packaging: Packaging
    mcp_samples_rad: Sequence[float]
    """MCP flexions at which the pads must clear the MCP servo."""
    joint_servo: bool = True
    """The servo on ``A`` (else only ``A``'s pin housing, :attr:`joint_radius_mm`)."""
    joint_radius_mm: float = 0.0
    """The joint's envelope at ``A`` without the servo."""
    link_radius_mm: float | None = None
    """The bars as capsules of this radius against the pads (``None``: not checked)."""


def margins(geo: RevoluteGeometry, phi: np.ndarray, checks: JointChecks,
            until_unsafe: bool = False) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin of every checked pair beyond the clearance.

    ``until_unsafe``: the distal pad is checked against the MCP servo (the
    costly pair: every MCP angle) only up to the first sample another pair
    fails at; past it the motion is unsafe anyway, and its margin is ``inf``.
    """

    packaging = checks.packaging
    clearance = packaging.body_clearance_mm
    count = len(phi)
    distal = geo.distal_pad(phi)
    proximal = np.broadcast_to(geo.proximal_pad, (count, *geo.proximal_pad.shape))
    moving = {"distal_pad": distal}
    margins = {}
    if checks.joint_servo and geo.distal_carrier_local is not None:
        margins[("distal_carrier", "pip_servo")] = (
            footprint_gaps(geo.posed(geo.distal_carrier_local, phi), geo.distal_servo, packaging) - clearance)
    for name, solid in moving.items():
        if checks.joint_servo:
            margins[(name, "pip_servo")] = footprint_gaps(solid, geo.distal_servo, packaging) - clearance
        else:
            a = np.broadcast_to((geo.a_x_mm, 0.0), (count, 2))
            margins[(name, "a_pin")] = polygon_to_disk(solid, a, checks.joint_radius_mm) - clearance
        if packaging.pad_pad_check:
            margins[(name, "proximal_pad")] = polygon_gaps(solid, proximal, packaging) - clearance
    if checks.link_radius_mm is not None:  # the comparison model: the bars as capsules, in the side view
        margins.update(bar_capsule_margins(geo, phi, checks.link_radius_mm, clearance))
    if geo.proximal_carrier is not None:  # the comparison model: both supports solid
        support = np.broadcast_to(geo.proximal_carrier, (count, 4, 2))
        margins[("distal_pad", "proximal_carrier")] = polygon_gaps(distal, support, packaging) - clearance
        if checks.joint_servo:  # not through the servo on its own segment (ending on the pin A is its mounting)
            margins[("proximal_carrier", "pip_servo")] = footprint_gaps(support, geo.distal_servo, packaging) - clearance
        if geo.distal_carrier_local is not None:
            carrier = geo.posed(geo.distal_carrier_local, phi)
            margins[("distal_carrier", "proximal_carrier")] = polygon_gaps(carrier, support, packaging) - clearance
            margins[("distal_carrier", "proximal_pad")] = polygon_gaps(carrier, proximal, packaging) - clearance
    if packaging.pad_checks_mcp("distal_pad"):
        end = count
        if until_unsafe:
            failing = np.flatnonzero(np.min(np.stack(list(margins.values())), axis=0) < -1e-9)
            end = int(failing[0]) + 1 if failing.size else count
        for name, solid in moving.items():
            margin = np.full(count, np.inf)
            margin[:end] = (mcp_servo_gaps(solid[:end], packaging, checks.mcp_samples_rad)
                            - packaging.mcp_pad_clearance_mm)
            margins[(name, "mcp_servo")] = margin
    if packaging.pad_checks_mcp("proximal_pad"):  # fixed in the proximal frame
        gap = proximal_pad_mcp_gap(geo.proximal_pad, packaging, checks.mcp_samples_rad)
        margins[("mcp_servo", "proximal_pad")] = np.full(count, gap - packaging.mcp_pad_clearance_mm)
    return {pair: np.where(np.abs(margin) < 1e-9, 0.0, margin) for pair, margin in margins.items()}


def bar_capsule_margins(geo, phi: np.ndarray, radius_mm: float, clearance: float) -> dict[tuple[str, str], np.ndarray]:
    """Per-sample margin (mm) of each pad against the bars it is not mounted
    on, capsules of ``radius_mm``, in the side view: the proximal pad against
    the distal truss (``A`` to the distal pad's mounting ends, and between
    them), the distal pad against the proximal bar ``O-A``."""

    count = len(phi)
    a = np.broadcast_to((geo.a_x_mm, 0.0), (count, 2))
    near, far = np.moveaxis(geo.posed(geo.distal_mount, phi), 1, 0)
    proximal = np.broadcast_to(geo.proximal_pad, (count, *geo.proximal_pad.shape))
    margins = {("proximal_pad", f"link_{name}"): segment_polygon_gaps(proximal, start, end) - radius_mm - clearance
               for name, (start, end) in (("a_near", (a, near)), ("a_far", (a, far)), ("near_far", (near, far)))}
    margins[("distal_pad", "link_o_a")] = (segment_polygon_gaps(geo.distal_pad(phi), np.zeros((count, 2)), a)
                                           - radius_mm - clearance)
    return margins


def between_samples(geo, phi: np.ndarray, margins: dict[tuple[str, str], np.ndarray], verify_step_rad: float,
                    margins_of: Callable[[object, np.ndarray], dict[tuple[str, str], np.ndarray]]
                    ) -> dict[tuple[str, str], np.ndarray]:
    """``margins`` with the motion between samples checked on the verification
    grid where a collision could hide there.

    The distal solids turn rigidly about ``A``, so no point of them moves more
    than ``distal_reach_mm`` per radian: a margin falls at most that fast, and
    two neighbouring clear samples whose margins sum to less than the reach
    times their step may pass through a collision between them. Those steps
    are resampled on ``verify_step_rad`` by ``margins_of(geo, phi)``, up to
    the first unsafe sample; the first that collides inside lowers the
    margins at its end to the least inside. (On the verification grid itself
    nothing is resampled.) Any ``geo`` whose ``distal_reach_mm`` bounds its
    speed serves (the belt 2R's, the RCJ's).
    """

    fine = verify_step_rad
    least = np.min(np.stack(list(margins.values())), axis=0)
    unsafe = np.flatnonzero(least < 0.0)
    end = int(unsafe[0]) if unsafe.size else len(phi) - 1
    step = np.diff(phi[:end + 1])
    clear = least[:end]
    suspect = np.flatnonzero((clear + least[1:end + 1] < geo.distal_reach_mm * step)
                             & (step > fine * (1.0 + 1e-9)))
    if not suspect.size:
        return margins
    counts = np.ceil(step[suspect] / fine - 1e-9).astype(int)
    inner = np.concatenate([np.linspace(phi[i], phi[i + 1], count + 1)[1:-1] for i, count in zip(suspect, counts)])
    owner = np.repeat(suspect, counts - 1)
    fine_margins = margins_of(geo, inner)
    fine_least = np.min(np.stack(list(fine_margins.values())), axis=0)
    hit = np.flatnonzero(fine_least < 0.0)
    if not hit.size:
        return margins
    interval = owner[hit[0]]
    inside = owner == interval
    result = {pair: margin.copy() for pair, margin in margins.items()}
    for pair, margin in fine_margins.items():
        result[pair][interval + 1] = min(result[pair][interval + 1], float(margin[inside].min()))
    return result


def hyperextension_margin(geo, hyperextension_rad: float, step_rad: float,
                          margins_of: Callable[[object, np.ndarray], dict[tuple[str, str], np.ndarray]]) -> float:
    """The least margin of any checked pair while the joint hyperextends to
    ``hyperextension_rad`` on a ``step_rad`` grid (``margins_of(geo, phi)``)."""

    back = -np.unique(np.append(np.arange(step_rad, hyperextension_rad, step_rad), hyperextension_rad))
    return min(float(margin.min()) for margin in margins_of(geo, back).values())


__all__ = ["VARIABLE_NAMES", "JointChecks", "RevoluteGeometry", "bar_capsule_margins", "between_samples",
           "carrier_local", "contact_faces", "fold_free", "geometry", "hyperextension_margin", "margins",
           "proximal_support", "servo_rear"]
