"""Each pad's mounting depth set by the study's policy, not searched
(``goal.pads.mount_policy: shallowest_supported``, 2026-11-04).

The depth ``d`` is measured palmar of the pad's reference line: CoDe's ``OA``
and ``BC``, the 2R's segments' lines. Each pad gets the least ``d`` on the
resolution grid that clears its own joint hardware, given the pad's length
and placement:

- **A, on the line**: ``d = 0`` when the pad does not reach that hardware.
- **B, under the hardware**: the least ``d`` that clears it. The hardware is
  CoDe's pin housings on the pad's own segment (``A``, ``A'`` proximal; ``B``,
  ``B'`` distal) and the PIP servo against the proximal pad; the 2R's servo
  at ``A`` for both pads; the RCJ's rollers, each its own segment's, and its
  servo on ``A`` against the proximal pad. The segment's body carries the mounting surface
  only as deep as the joint hardware the pad overlaps reaches below the
  line, plus the clearance (its *native support*): a pin housing as its
  disk; a roller (the RCJ's, one body with its pad: no clearance) by its
  lowest point over the pad's span; a servo housing fixed on the pad's own segment (CoDe's PIP servo
  and the 2R's servo, for the proximal pad) by its footprint's lowest point
  over the pad's span along the line, none where the two do not overlap
  (2026-11-05: a housing the pad only skirts carries nothing). The 2R's
  distal pad hangs from a truss off the output at ``A``: the servo sets
  only its clearance, unless the two depths are aligned
  (``goal.pads.align_mount_offsets``: one depth, the deeper least one, for
  both pads; the distal pad then supported to the servo's reach).
- **C, deeper than that support** (a proximal pad needing clearance from a
  servo it does not run under, say): no design. No spacer is assumed.

The checks are the evaluators' own: ``packaging.own_pin_gaps`` and
``pip_servo_pad_gap_mm`` for CoDe, the servo's footprint and joint disk for
the 2R. A pad's check depends on its own depth alone, so the two depths are
independent. A closed form (a block under a disk) gives the first depth tried;
the checks then decide, one grid step at a time. That the pad, once clear, stays clear deeper is
assumed (its hardware lies above its lowest face). The pads against each
other, and everything over the motion, stay the evaluators' to check: a
depth is never deepened to dodge them.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

TOLERANCE_MM = 1e-6
"""A gap this far below 0 still counts as touching."""
SIDES = ("proximal", "distal")


@dataclass(frozen=True)
class Mounting:
    """Each pad's least clearing depth (``None``: none within the allowed
    range) and its native support (mm), proximal then distal."""

    depths_mm: tuple[float | None, float | None]
    native_mm: tuple[float, float]

    @property
    def cases(self) -> tuple[str, str]:
        """``A`` on the line, ``B`` under its own hardware, ``C`` deeper than that hardware supports."""

        return tuple("C" if d is None or d > native + TOLERANCE_MM else "A" if d == 0.0 else "B"
                     for d, native in zip(self.depths_mm, self.native_mm))

    @property
    def supported(self) -> bool:
        return "C" not in self.cases

    def violation_mm(self, depths_mm: Sequence[float], cap_mm: float) -> float:
        """How far ``depths_mm`` are from this mounting (0: they are it, and supported)."""

        worst = 0.0
        for depth, required, native in zip(depths_mm, self.depths_mm, self.native_mm):
            if required is None:
                worst = max(worst, cap_mm)
                continue
            worst = max(worst, abs(depth - required), required - native)
        return worst


def clearing_depth_mm(span: tuple[float, float], center: tuple[float, float], radius: float) -> float:
    """The least depth below the line of a block over ``span`` (along the line)
    that clears a disk at ``center`` (along, dorsal) of ``radius``: 0 when the
    block does not reach it. The first depth tried, not a check."""

    low, high = span
    along, dorsal = center
    reach = 0.0 if low <= along <= high else min(abs(along - low), abs(along - high))
    if reach >= radius:
        return 0.0
    return max(0.0, math.sqrt(radius * radius - reach * reach) - dorsal)


def shallowest(clear: Callable[[Sequence[float]], Sequence[bool]], guesses: Sequence[float],
               caps: Sequence[float], resolution: float) -> list[float | None]:
    """Each pad's least depth on the grid within ``[0, cap]`` that ``clear``
    accepts (``clear(depths)``: per pad, each on its own depth), from
    ``guesses``: up a step while not clear, then down while still clear."""

    grid = lambda value: round(round(value / resolution) * resolution, 9)  # noqa: E731
    depths = [grid(math.ceil(min(max(g, 0.0), cap) / resolution - 1e-9) * resolution)
              for g, cap in zip(guesses, caps)]
    ok = list(clear(depths))
    found: list[float | None] = list(depths)
    rising = [i for i, fits in enumerate(ok) if not fits]
    while rising:
        for i in list(rising):
            depths[i] = grid(depths[i] + resolution)
            if depths[i] > caps[i] + 1e-9:
                found[i] = None
                rising.remove(i)
        if not rising:
            break
        ok = list(clear(depths))
        for i in list(rising):
            if ok[i]:
                found[i] = depths[i]
                rising.remove(i)
    falling = [i for i, depth in enumerate(found) if depth is not None and depth > 0.0]
    while falling:
        trial = [grid(found[i] - resolution) if i in falling else (found[i] if found[i] is not None else 0.0)
                 for i in range(len(found))]
        ok = list(clear(trial))
        for i in list(falling):
            if ok[i]:
                found[i] = trial[i]
                if trial[i] <= 0.0:
                    falling.remove(i)
            else:
                falling.remove(i)
    return found


def footprint_reach_mm(outline: np.ndarray, span: tuple[float, float], clearance: float) -> float:
    """How deep a housing carries a pad's mounting surface: its outline's
    (``(n, 2)``, ``(along, dorsal)`` in the pad's line frame) lowest point over
    the pad's ``span`` along the line, below the line, plus ``clearance``; 0
    where the two do not overlap along the line."""

    low, high = span
    closed = np.vstack((outline, outline[:1]))
    lowest = math.inf
    for (u0, v0), (u1, v1) in zip(closed[:-1], closed[1:]):
        first, last = max(min(u0, u1), low), min(max(u0, u1), high)
        if first > last:
            continue
        for u in (first, last):  # the edge's ends inside the span (it is straight)
            v = v0 if u1 == u0 else v0 + (v1 - v0) * (u - u0) / (u1 - u0)
            lowest = min(lowest, v)
    return 0.0 if math.isinf(lowest) else max(0.0, clearance - lowest)


def on_grid(native: Sequence[float], resolution: float) -> tuple[float, float]:
    """The native supports on the depths' grid, rounded up: a surface made to
    the grid sits at the housing's lowest point, not a step short of it."""

    return tuple(round(math.ceil(value / resolution - 1e-6) * resolution, 9) for value in native)


def _caps(goal) -> tuple[float, float]:
    return tuple(float(goal.bounds[f"{side}_mount_offset_mm"][1]) for side in SIDES)


# CoDe -------------------------------------------------------------------------

def _linkage_hardware(finger, packaging) -> dict[str, list[tuple[float, float, float]]]:
    """Each pad's own pin housings, ``(along, dorsal, radius)`` in its line's frame."""

    joint = finger.joint
    a = np.array((joint.a.x_mm, joint.a.z_mm))
    e = a / np.linalg.norm(a)
    n = np.array((-e[1], e[0]))
    b, c = np.array((joint.b.x_mm, joint.b.z_mm)), np.array((finger.tip.x_mm, finger.tip.z_mm))
    u = (c - b) / np.linalg.norm(c - b)
    v = np.array((-u[1], u[0]))
    clearance = packaging.body_clearance_mm
    frame = lambda point, origin, along, dorsal: (float((point - origin) @ along),  # noqa: E731
                                                  float((point - origin) @ dorsal))
    pins = {"proximal": (("a", a), ("ap", np.array((joint.ap.x_mm, joint.ap.z_mm)))),
            "distal": (("b", b), ("bp", np.array((joint.bp.x_mm, joint.bp.z_mm))))}
    origins = {"proximal": (np.zeros(2), e, n), "distal": (b, u, v)}
    return {side: [(*frame(point, *origins[side]), packaging.pivot_radius_mm(name) + clearance)
                   for name, point in pins[side]] for side in SIDES}


def linkage_mounting(finger, config) -> Mounting:
    """CoDe's mounting: each pad's least clearing depth and native support
    (whatever depths ``finger`` has)."""

    bare = dataclasses.replace(finger, pad_offsets_mm=(0.0, 0.0))
    return _cached(("linkage", bare), config, lambda: _linkage_mounting(bare, config))


def _linkage_mounting(finger, config) -> Mounting:

    from core.morphology.mechanism.linkage.packaging import (own_pin_gaps, pad_spans, pip_footprint,
                                                              pip_servo_pad_gap_mm)

    packaging = config.packaging
    spans = pad_spans(finger, packaging)
    spans = {"proximal": (spans["proximal_pad"].start_mm, spans["proximal_pad"].end_mm),
             "distal": (spans["distal_pad"].start_mm, spans["distal_pad"].end_mm)}
    hardware = _linkage_hardware(finger, packaging)
    guesses, native = [], []
    for side in SIDES:
        needs = [(clearing_depth_mm(spans[side], (along, dorsal), radius), radius - dorsal)
                 for along, dorsal, radius in hardware[side]]
        guesses.append(max(need for need, _ in needs))
        # The body reaches as deep as the housings the pad runs into.
        native.append(max((reach for need, reach in needs if need > 0.0), default=0.0))
    if packaging.pip_servo_pad_check:
        # The PIP servo is fixed on the proximal segment: where the pad runs
        # under its housing, the housing carries it (2026-11-05), as deep as
        # its footprint reaches over the pad. Never the distal pad's.
        bare = dataclasses.replace(finger, pad_offsets_mm=(0.0, 0.0))
        if pip_servo_pad_gap_mm(bare, packaging) < -TOLERANCE_MM:
            a = np.array((finger.joint.a.x_mm, finger.joint.a.z_mm))
            e = a / np.linalg.norm(a)
            outline = pip_footprint(packaging, finger).outline @ np.column_stack((e, (-e[1], e[0])))
            native[0] = max(native[0], footprint_reach_mm(outline, spans["proximal"], packaging.body_clearance_mm))

    def clear(depths: Sequence[float]) -> list[bool]:
        posed = dataclasses.replace(finger, pad_offsets_mm=(float(depths[0]), float(depths[1])))
        gaps = own_pin_gaps(posed, packaging)
        proximal = gaps[0] >= -TOLERANCE_MM
        if proximal and packaging.pip_servo_pad_check:
            proximal = pip_servo_pad_gap_mm(posed, packaging) >= -TOLERANCE_MM
        return [proximal, gaps[1] >= -TOLERANCE_MM]

    depths = shallowest(clear, guesses, _caps(config), config.resolution_mm)
    return Mounting(_aligned(depths) if config.align_mount_offsets else tuple(depths),
                    on_grid(native, config.resolution_mm))


# 2R ---------------------------------------------------------------------------

def revolute_mounting(x: Sequence[float], config) -> Mounting:
    """The 2R's mounting: each pad's least depth clearing the joint's servo
    (its footprint, fixed with the proximal pad; its disk about ``A``, which
    the distal pad turns about), the servo's own depth below the line its
    native support (whatever drops ``x`` has)."""

    from core.optimization.revolute.config import VARIABLE_NAMES

    bare = tuple(0.0 if name in ("proximal_drop_mm", "distal_drop_mm") else float(value)
                 for name, value in zip(VARIABLE_NAMES, x))
    return _cached(("revolute", bare), config, lambda: _revolute_mounting(bare, config))


def _revolute_mounting(x: Sequence[float], config) -> Mounting:

    from core.morphology.packaging.gaps import footprint_gaps
    from core.morphology.packaging.shapes import polygon_to_disk
    from core.optimization.revolute.config import VARIABLE_NAMES
    from core.optimization.revolute.evaluate import geometry

    goal = config.goal
    packaging, clearance = goal.packaging, goal.packaging.body_clearance_mm
    radius = config.joint_radius_mm
    slots = (VARIABLE_NAMES.index("proximal_drop_mm"), VARIABLE_NAMES.index("distal_drop_mm"))

    def at(depths: Sequence[float]):
        values = list(x)
        for slot, depth in zip(slots, depths):
            values[slot] = float(depth)
        return geometry(values, config)

    def gaps(depths: Sequence[float]) -> tuple[float, float]:
        geo = at(depths)
        a = np.array([[geo.a_x_mm, 0.0]])
        if not config.joint_servo:  # a pin at A (as the evaluator checks it)
            proximal = float(polygon_to_disk(geo.proximal_pad[None], a, radius)[0])
        elif packaging.pip_servo_pad_check:
            proximal = float(footprint_gaps(geo.proximal_pad[None], geo.distal_servo, packaging)[0])
        else:  # the servo and the proximal pad in separate layers, as the evaluator takes them
            proximal = math.inf
        distal = float(polygon_to_disk(geo.distal_pad_local[None], np.zeros((1, 2)), radius)[0])
        return proximal - clearance, distal - clearance

    def clear(depths: Sequence[float]) -> list[bool]:
        return [gap >= -TOLERANCE_MM for gap in gaps(depths)]

    geo = at((0.0, 0.0))
    spans = ((float(geo.proximal_pad[:, 0].min()), float(geo.proximal_pad[:, 0].max())),
             (float(geo.distal_pad_local[:, 0].min()), float(geo.distal_pad_local[:, 0].max())))
    centers = ((geo.a_x_mm, 0.0), (0.0, 0.0))
    guesses = [clearing_depth_mm(span, center, radius + clearance) for span, center in zip(spans, centers)]
    # The servo's housing (or A's pin) as deep as it reaches below the line: the
    # proximal pad's support where it runs into it, being on its body. The
    # distal pad hangs from a truss off the output at A, in other layers: the
    # housing only sets its clearance, so it has no support under it, unless
    # its depth is held to the proximal pad's (aligned, one mounting datum).
    reach = (-geo.distal_servo.lowest_z_mm if config.joint_servo else radius) + clearance
    # The servo's housing carries the proximal pad as deep as its footprint
    # reaches over the pad (2026-11-05, as CoDe's PIP servo); A's pin as its disk.
    under = (footprint_reach_mm(geo.distal_servo.outline, spans[0], clearance) if config.joint_servo
             else reach)
    native = (under if not clear((0.0, 0.0))[0] else 0.0, reach if goal.align_mount_offsets else 0.0)
    depths = shallowest(clear, guesses, _caps(goal), goal.resolution_mm)
    return Mounting(_aligned(depths) if goal.align_mount_offsets else tuple(depths),
                    on_grid(native, goal.resolution_mm))


# RCJ --------------------------------------------------------------------------

def disk_reach_mm(center: tuple[float, float], radius: float, span: tuple[float, float]) -> float:
    """How deep a disk (``(along, dorsal)`` centre, in the pad's line frame)
    carries a pad's mounting surface: its lowest point over the pad's
    ``span``, below the line; 0 where the two do not overlap along the line."""

    along, dorsal = center
    low, high = span
    reach = 0.0 if low <= along <= high else min(abs(along - low), abs(along - high))
    if reach >= radius:
        return 0.0
    return max(0.0, math.sqrt(radius * radius - reach * reach) - dorsal)


def rcj_mounting(x: Sequence[float], config) -> Mounting:
    """The RCJ's mounting: each pad's least depth clearing its own roller (and
    the servo, for the proximal pad), and what those carry (whatever drops
    ``x`` has)."""

    from core.optimization.rcj.config import VARIABLE_NAMES

    bare = tuple(0.0 if name in ("proximal_drop_mm", "distal_drop_mm") else float(value)
                 for name, value in zip(VARIABLE_NAMES, x))
    return _cached(("rcj", bare), config, lambda: _rcj_mounting(bare, config))


def _rcj_mounting(x: Sequence[float], config) -> Mounting:

    from core.morphology.packaging.gaps import footprint_gaps
    from core.morphology.packaging.shapes import polygon_to_disk
    from core.optimization.rcj.config import VARIABLE_NAMES
    from core.optimization.rcj.evaluate import geometry

    goal = config.goal
    packaging, clearance = goal.packaging, goal.packaging.body_clearance_mm
    slots = (VARIABLE_NAMES.index("proximal_drop_mm"), VARIABLE_NAMES.index("distal_drop_mm"))

    def at(depths: Sequence[float]):
        values = list(x)
        for slot, depth in zip(slots, depths):
            values[slot] = float(depth)
        return geometry(values, config)

    def gaps(depths: Sequence[float]) -> tuple[float, float]:
        # As the evaluator: each pad touching its own roller at most (no
        # clearance: one body), the proximal pad clear of the servo.
        geo = at(depths)
        r = geo.roller_radius_mm
        proximal = float(polygon_to_disk(geo.proximal_pad[None], geo.a[None], r)[0])
        if packaging.pip_servo_pad_check and geo.servo is not None:
            proximal = min(proximal, float(footprint_gaps(geo.proximal_pad[None], geo.servo, packaging)[0])
                           - clearance)
        distal = float(polygon_to_disk(geo.distal_pad_local[None], np.zeros((1, 2)), r)[0])
        return proximal, distal

    def clear(depths: Sequence[float]) -> list[bool]:
        return [gap >= -TOLERANCE_MM for gap in gaps(depths)]

    geo = at((0.0, 0.0))
    r = geo.roller_radius_mm
    spans = ((float(geo.proximal_pad[:, 0].min()), float(geo.proximal_pad[:, 0].max())),
             (float(geo.distal_pad_local[:, 0].min()), float(geo.distal_pad_local[:, 0].max())))
    centers = ((geo.a_x_mm, 0.0), (0.0, 0.0))
    guesses = [clearing_depth_mm(span, center, r) for span, center in zip(spans, centers)]
    # Each roller is its own segment's body, the servo the proximal one's: each
    # carries its pad as deep as it reaches over the pad.
    proximal = disk_reach_mm(centers[0], r, spans[0])
    if packaging.pip_servo_pad_check and geo.servo is not None:
        proximal = max(proximal, footprint_reach_mm(geo.servo.outline, spans[0], clearance))
    distal = disk_reach_mm(centers[1], r, spans[1])
    at_rest = clear((0.0, 0.0))
    native = (0.0 if at_rest[0] else proximal, 0.0 if at_rest[1] else distal)
    depths = shallowest(clear, guesses, _caps(goal), goal.resolution_mm)
    return Mounting(_aligned(depths) if goal.align_mount_offsets else tuple(depths),
                    on_grid(native, goal.resolution_mm))


def _aligned(depths: Sequence[float | None]) -> tuple[float | None, float | None]:
    """Both pads at the deeper of their least depths (each still clear there: its hardware is above it)."""

    if any(depth is None for depth in depths):
        return (None, None)
    deepest = max(depths)
    return (deepest, deepest)


_CACHE: dict[tuple, tuple[object, Mounting]] = {}
CACHE_SIZE = 4096
"""Mountings kept per process: the search sets a design's depths, then its
evaluation checks them, the same design (its depths aside) twice."""


def _cached(key: tuple, config, solve: Callable[[], Mounting]) -> Mounting:
    """``solve()`` once per design and config. The config is keyed by identity
    and held with the result, so no other config can take its place."""

    key = (*key, id(config))
    found = _CACHE.get(key)
    if found is None or found[0] is not config:
        if len(_CACHE) >= CACHE_SIZE:
            _CACHE.clear()
        found = _CACHE[key] = (config, solve())
    return found[1]


def mounting(mechanism: str, x: Sequence[float], config) -> Mounting:
    """The mounting a design's pads get under the policy, whatever its vector's depths say."""

    from core.morphology.mechanism import base_mechanism

    base = base_mechanism(mechanism)  # a variant mounts as its base, its config telling the parts apart
    if base == "linkage":
        from core.morphology.mechanism.linkage.design import decode

        return linkage_mounting(decode(x, config.link_length_resolution_mm), config)
    if base == "revolute":
        return revolute_mounting(x, config)
    if base == "rcj":
        return rcj_mounting(x, config)
    raise ValueError(f"no mounting policy for mechanism {mechanism!r}")


__all__ = ["Mounting", "clearing_depth_mm", "disk_reach_mm", "footprint_reach_mm", "linkage_mounting", "mounting", "rcj_mounting", "revolute_mounting", "shallowest"]
