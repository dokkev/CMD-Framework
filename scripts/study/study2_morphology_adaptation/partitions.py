"""Each mechanism's design vector split into external morphology ``E``,
mechanism geometry ``Q`` and actuation ``A`` (``core.optimization.analysis.adaptation``).

``E`` is the common descriptor every evaluator already measures the same
way: the finger's length ``|OC|``, the contact-bearing segments ``|OA''|``
and ``|B''C|``, and the articulation station, the extended centre of
rotation (``core.morphology.icr``: CoDe's crossing of ``A-B'`` and ``A'-B``,
the 2R's ``A``, the RCJ's roller contact ``A + r``). The mapping is explicit
per mechanism, never read off variable names.

| mechanism | ``Q`` searched with ``E`` fixed | ``A`` held |
|---|---|---|
| CoDe (``linkage``) | ``A``, ``B`` (with ``BC``), ``B'`` along the line through ``A`` and the ICR | ``A'``, tilt |
| 2R-DD, idealized 2R | none (``E`` fixes the 2R) | tilt |
| belt 2R | none | the servo on ``A'`` (fixed in the proximal frame), tilt |
| RCJ, idealized RCJ | the rollers' radius (``A`` and ``BC`` follow) | tilt |

Each design's mounting depths are the goal's policy for its interface
(``core.optimization.mounting.mounting``), and every design variable stays
within its mechanism's bounds (``config.bounds``), or there is no design.
"""

from __future__ import annotations

import math
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from core.optimization.analysis.adaptation import Partition  # noqa: E402
from core.optimization.mechanism import mechanism as find_mechanism  # noqa: E402
from core.optimization.mounting import mounting  # noqa: E402

TOLERANCE_MM = 1e-6
DROPS = ("proximal_drop_mm", "distal_drop_mm")
MOUNTS = ("proximal_mount_offset_mm", "distal_mount_offset_mm")


class _Partition(Partition):
    """Shared: the design vector by name, the mounting policy, the bounds."""

    depths: tuple[str, str] = DROPS

    def __init__(self, name: str) -> None:
        if name not in self.mechanisms:
            raise ValueError(f"{type(self).__name__} maps {self.mechanisms}, not {name!r}")
        self.name = name
        self.names = tuple(find_mechanism(name).variable_names)

    def values(self, x: Sequence[float]) -> dict[str, float]:
        return dict(zip(self.names, (float(v) for v in x)))

    def finish(self, values: Mapping[str, float], config) -> tuple[float, ...] | None:
        """The design with the policy's depths, within the mechanism's bounds (``None``: not)."""

        x = [float(values[name]) for name in self.names]
        bounds = config.bounds
        for name, value in zip(self.names, x):
            if name in self.depths:
                continue
            low, high = bounds[name]
            if not (low - TOLERANCE_MM <= value <= high + TOLERANCE_MM) or not math.isfinite(value):
                return None
        try:
            found = mounting(self.name, x, config)
        except ValueError:  # no mechanism (CoDe's B behind A, its bars not closing)
            return None
        if not found.supported:
            return None
        for slot, depth in zip(self.depths, found.depths_mm):
            x[self.names.index(slot)] = float(depth)
        return tuple(x)

    def coordinate_bounds(self, config) -> dict[str, tuple[float, float]]:
        return {name: tuple(config.bounds[name]) for name in self.coordinates}

    def template(self, external: Mapping[str, float], source_mechanism: str, source_x: Sequence[float],
                 config) -> tuple[dict[str, float], tuple[float, ...]]:
        """The matched reference's mechanism coordinates and template (its
        actuation) for ``external``, given the source design it is matched to."""

        raise NotImplementedError


class LinkagePartition(_Partition):
    """CoDe. ``E``: ``|OC|`` (``B`` and ``BC``), ``|OA''|`` (``A'`` and its
    offset), ``|B''C|`` (``B``, ``B''``, ``BC``) and the ICR (all four
    pivots). ``Q``: the pivots ``A``, ``B`` (with ``BC``) and ``B'``; ``B'``
    is searched by its ``x`` on the line from ``A`` through the ICR, which
    holds the ICR exactly. ``A``: the servo's shaft ``A'`` and its tilt."""

    mechanisms = ("linkage",)
    coordinates = ("a_x_mm", "a_z_mm", "b_z_mm", "bc_length_mm", "bp_x_mm")
    actuation = ("ap_x_mm", "ap_z_mm", "servo_tilt_deg")
    depths = MOUNTS

    def external(self, x, config) -> dict[str, float]:
        from core.morphology.mechanism.linkage.measures import extended_icr_x

        v = self.values(x)
        icr = extended_icr_x((v["a_x_mm"], v["a_z_mm"]), (v["ap_x_mm"], v["ap_z_mm"]),
                             (v["b_x_mm"], v["b_z_mm"]), (v["bp_x_mm"], v["bp_z_mm"]))
        return {"finger_length_mm": math.hypot(v["b_x_mm"] + v["bc_length_mm"], v["b_z_mm"]),
                "proximal_span_mm": v["ap_x_mm"] + v["app_offset_mm"],
                "distal_span_mm": v["bc_length_mm"] - (v["bp_x_mm"] - v["bpp_offset_mm"] - v["b_x_mm"]),
                "articulation_mm": math.nan if icr is None else icr}

    def mechanism(self, x, config) -> dict[str, float]:
        v = self.values(x)
        return {name: v[name] for name in ("a_x_mm", "a_z_mm", "b_x_mm", "b_z_mm", "bp_x_mm", "bp_z_mm",
                                           "bc_length_mm")}

    def coordinate_values(self, x, config) -> dict[str, float]:
        v = self.values(x)
        return {name: v[name] for name in self.coordinates}

    def compose(self, external, coordinates, template, config):
        v = {**self.values(template), **coordinates}
        length, proximal, distal, icr = (external[name] for name in ("finger_length_mm", "proximal_span_mm",
                                                                      "distal_span_mm", "articulation_mm"))
        b_x = math.sqrt(max(length ** 2 - v["b_z_mm"] ** 2, 0.0)) - v["bc_length_mm"]
        if b_x <= 0.0 or abs(b_x - v["ap_x_mm"]) < TOLERANCE_MM or abs(icr - v["a_x_mm"]) < TOLERANCE_MM:
            return None
        # The ICR on A'-B at x = icr; B' on the line from A through it.
        share = (icr - v["ap_x_mm"]) / (b_x - v["ap_x_mm"])
        icr_z = v["ap_z_mm"] + share * (v["b_z_mm"] - v["ap_z_mm"])
        v["b_x_mm"] = b_x
        v["bp_z_mm"] = v["a_z_mm"] + (v["bp_x_mm"] - v["a_x_mm"]) * (icr_z - v["a_z_mm"]) / (icr - v["a_x_mm"])
        v["app_offset_mm"] = proximal - v["ap_x_mm"]
        v["bpp_offset_mm"] = v["bp_x_mm"] - b_x - v["bc_length_mm"] + distal
        return self.finish(v, config)

    def template(self, external, source_mechanism, source_x, config):
        if source_mechanism != "linkage":
            raise ValueError("CoDe's matched reference takes its mechanism and actuation from a CoDe design")
        return self.coordinate_values(source_x, config), tuple(source_x)


class RevolutePartition(_Partition):
    """The 2R-DD and the idealized 2R: ``E`` fixes every length (``A`` at the
    articulation station), no ``Q`` is left. ``A``: the servo's tilt."""

    mechanisms = ("revolute", "revolute_idealized")
    coordinates = ()
    actuation = ("servo_tilt_deg",)

    def external(self, x, config) -> dict[str, float]:
        v = self.values(x)
        return {"finger_length_mm": v["a_x_mm"] + v["ac_length_mm"], "proximal_span_mm": v["app_x_mm"],
                "distal_span_mm": v["ac_length_mm"] - v["bpp_from_a_mm"], "articulation_mm": v["a_x_mm"]}

    def mechanism(self, x, config) -> dict[str, float]:
        return {}

    def coordinate_values(self, x, config) -> dict[str, float]:
        return {}

    def joint(self, external) -> dict[str, float]:
        length, proximal, distal, station = (external[name] for name in ("finger_length_mm", "proximal_span_mm",
                                                                          "distal_span_mm", "articulation_mm"))
        return {"a_x_mm": station, "ac_length_mm": length - station, "app_x_mm": proximal,
                "bpp_from_a_mm": length - station - distal}

    def compose(self, external, coordinates, template, config):
        return self.finish({**self.values(template), **self.joint(external)}, config)

    def template(self, external, source_mechanism, source_x, config):
        values = {name: 0.0 for name in self.names}
        values["servo_tilt_deg"] = config.bounds["servo_tilt_deg"][0]
        return {}, tuple(values[name] for name in self.names)


class ParallelPartition(RevolutePartition):
    """The belt 2R: the 2R's, its servo on ``A'`` held where it is in the
    proximal frame (the actuation's mount), ``A' - A`` following ``A``."""

    mechanisms = ("revolute_parallel",)
    actuation = ("servo_tilt_deg", "ap_from_a_x_mm", "ap_from_a_z_mm")

    def compose(self, external, coordinates, template, config):
        v = self.values(template)
        servo_x = v["a_x_mm"] + v["ap_from_a_x_mm"]  # where the servo's shaft is held
        joint = self.joint(external)
        return self.finish({**v, **joint, "ap_from_a_x_mm": servo_x - joint["a_x_mm"]}, config)

    def template(self, external, source_mechanism, source_x, config):
        """The servo where the source's is: CoDe's ``A'`` (its PIP servo's shaft)."""

        if source_mechanism != "linkage":
            raise ValueError("the belt 2R's matched servo is placed at a CoDe design's A'")
        source = dict(zip(find_mechanism("linkage").variable_names, source_x))
        values = {name: 0.0 for name in self.names}
        values["servo_tilt_deg"] = config.bounds["servo_tilt_deg"][0]
        values["a_x_mm"] = external["articulation_mm"]
        values["ap_from_a_x_mm"] = source["ap_x_mm"] - external["articulation_mm"]
        values["ap_from_a_z_mm"] = source["ap_z_mm"]
        return {}, tuple(values[name] for name in self.names)


class RcjPartition(_Partition):
    """The RCJ and the idealized RCJ: ``E`` fixes the contact ``A + r``, so
    the rollers' radius is ``Q``, ``A`` and ``BC`` following it. ``A``: the
    servo's tilt."""

    mechanisms = ("rcj", "rcj_idealized")
    coordinates = ("roller_radius_mm",)
    actuation = ("servo_tilt_deg",)

    def external(self, x, config) -> dict[str, float]:
        v = self.values(x)
        r = v["roller_radius_mm"]
        return {"finger_length_mm": v["a_x_mm"] + 2.0 * r + v["bc_length_mm"], "proximal_span_mm": v["app_x_mm"],
                "distal_span_mm": v["bc_length_mm"] - v["bpp_from_b_mm"], "articulation_mm": v["a_x_mm"] + r}

    def mechanism(self, x, config) -> dict[str, float]:
        v = self.values(x)
        r = v["roller_radius_mm"]
        return {"roller_radius_mm": r, "a_x_mm": v["a_x_mm"], "b_x_mm": v["a_x_mm"] + 2.0 * r}

    def coordinate_values(self, x, config) -> dict[str, float]:
        return {"roller_radius_mm": self.values(x)["roller_radius_mm"]}

    def compose(self, external, coordinates, template, config):
        r = coordinates["roller_radius_mm"]
        length, proximal, distal, contact = (external[name] for name in ("finger_length_mm", "proximal_span_mm",
                                                                          "distal_span_mm", "articulation_mm"))
        bc = length - contact - r
        return self.finish({**self.values(template), "roller_radius_mm": r, "a_x_mm": contact - r,
                            "bc_length_mm": bc, "app_x_mm": proximal, "bpp_from_b_mm": bc - distal}, config)

    def template(self, external, source_mechanism, source_x, config):
        """The smallest rollers its hardware allows (where every RCJ front settles)."""

        values = {name: 0.0 for name in self.names}
        values["servo_tilt_deg"] = config.bounds["servo_tilt_deg"][0]
        return {"roller_radius_mm": config.bounds["roller_radius_mm"][0]}, tuple(values[name] for name in self.names)


PARTITIONS = {name: cls for cls in (LinkagePartition, RevolutePartition, ParallelPartition, RcjPartition)
              for name in cls.mechanisms}
"""Mechanism name -> its partition class."""


def partition(name: str) -> _Partition:
    """The partition of a registered mechanism."""

    if name not in PARTITIONS:
        raise ValueError(f"no partition for mechanism {name!r}; known: {sorted(PARTITIONS)}")
    return PARTITIONS[name](name)


__all__ = ["LinkagePartition", "PARTITIONS", "ParallelPartition", "RcjPartition", "RevolutePartition", "partition"]
