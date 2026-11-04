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
  at ``A`` for both pads. The segment's body carries the mounting surface
  only as deep as the joint hardware the pad overlaps reaches below the
  line (its *native support*): CoDe's pin housings; the 2R's servo for its
  proximal pad only (the servo is on that body; the distal pad hangs from
  a truss off the output, the servo only its clearance), unless the two
  depths are aligned (``goal.pads.align_mount_offsets``: one depth, the
  deeper least one, for both pads; the 2R's distal pad then supported to
  the servo's reach).
- **C, deeper than that support** (CoDe's proximal pad pushed under its PIP
  servo, say): no design. No spacer is assumed.

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

    from core.morphology.mechanism.linkage.packaging import own_pin_gaps, pad_spans, pip_servo_pad_gap_mm

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

    def clear(depths: Sequence[float]) -> list[bool]:
        posed = dataclasses.replace(finger, pad_offsets_mm=(float(depths[0]), float(depths[1])))
        gaps = own_pin_gaps(posed, packaging)
        proximal = gaps[0] >= -TOLERANCE_MM
        if proximal and packaging.pip_servo_pad_check:
            proximal = pip_servo_pad_gap_mm(posed, packaging) >= -TOLERANCE_MM
        return [proximal, gaps[1] >= -TOLERANCE_MM]

    depths = shallowest(clear, guesses, _caps(config), config.resolution_mm)
    return Mounting(_aligned(depths) if config.align_mount_offsets else tuple(depths), tuple(native))


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
    native = (reach if not clear((0.0, 0.0))[0] else 0.0, reach if goal.align_mount_offsets else 0.0)
    depths = shallowest(clear, guesses, _caps(goal), goal.resolution_mm)
    return Mounting(_aligned(depths) if goal.align_mount_offsets else tuple(depths), native)


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

    if mechanism == "linkage":
        from core.morphology.mechanism.linkage.design import decode

        return linkage_mounting(decode(x, config.link_length_resolution_mm), config)
    if mechanism == "revolute":
        return revolute_mounting(x, config)
    raise ValueError(f"no mounting policy for mechanism {mechanism!r}")


__all__ = ["Mounting", "clearing_depth_mm", "linkage_mounting", "mounting", "revolute_mounting", "shallowest"]
