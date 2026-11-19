"""The CFL's motion and coverage (problem formulation A.4, A.5).

:func:`sweep_flexion` samples the PIP flexion ``q_2`` (the distal segment's
rotation relative to the proximal one) on a uniform grid from extension to
pi, as far as the four-bar closes. Per sample it reports whether the pose is
valid: no pad collides with another body's part, the distal pins clear the
PIP servo, the links (when they are capsules) clear the pads, and the
fingertip ``C`` stays ahead of the MCP (the palm). It also reports the
transmission angle at ``B'`` and the PIP servo's angle. ``core.evaluation``
turns these into ``F_motion``, ``q_max``, the transmission angle range and
``MA``.

:func:`coverage` evaluates the cylinder-based contact pad coverage at given
poses.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from core.evaluation import MotionSamples, PoseCoverage, common_length_mm, posed_finger_length_mm
from core.evaluation.motion import resample_between, uniform_samples_rad
from core.geometry.intervals import IntervalSet, face_occluded_set
from core.morphology.finger import Finger
from core.morphology.mechanism.linkage.model import four_bar
from core.morphology.mechanism.linkage.packaging import (
    FIXED_MARGIN_PAIRS,
    LinkagePackaging,
    assembly,
    contact_faces,
    distal_pad_polygon,
    layout,
    link_capsule_margins,
    pad_margins,
    poses,
)
from core.morphology.packaging.assembly import margins
from core.morphology.packaging.hardware import MARGIN_TOLERANCE_MM

TOUCH_MM = MARGIN_TOLERANCE_MM
"""Gaps above ``-TOUCH_MM`` count as touching, not overlapping (``code_formulation.md`` §2)."""


@dataclass(frozen=True)
class MotionSettings:
    """How the flexion is sampled and what is checked along it (angles in radians)."""

    step_rad: float
    """The uniform ``q_2`` grid step."""
    transmission_angle_range_rad: tuple[float, float]
    mcp_samples_rad: Sequence[float] = (0.0,)
    """MCP flexions at which the pads must clear the MCP servo (the formulation holds ``q_1 = 0``)."""
    link_radius_mm: float | None = None
    """The links as capsules of this radius against the pads they are not
    mounted on (``None``: not checked)."""
    verify_step_rad: float | None = None
    """The finer step the motion between samples is checked on where a
    collision could hide (``None``: not checked)."""


@dataclass(frozen=True)
class LinkageMotion:
    """One CFL's sampled flexion (see :func:`sweep_flexion`)."""

    samples: MotionSamples
    b: np.ndarray
    bp: np.ndarray
    tip: np.ndarray
    least_margin_mm: np.ndarray
    """Least gap of the checked pairs at each sample (mm)."""
    reported_margin_mm: np.ndarray
    """As :attr:`least_margin_mm`, less the pairs whose gap the design does not set."""
    first_collision: str
    """The first pair that overlaps along the motion, ``"first/second"`` (``""``: none)."""

    @property
    def stop_reason(self) -> str:
        """What ends ``q_max``: ``"collision"``, ``"tip"`` (``C`` behind the MCP),
        ``"transmission"``, ``"closure"`` or ``"curl"`` (pi reached)."""

        q, valid, gamma_ok = self.samples.q_rad, self.samples.valid, self.samples.gamma_ok
        bad = np.flatnonzero(~(valid & gamma_ok))
        if not bad.size:
            return "curl" if q[-1] >= math.pi - 1e-9 else "closure"
        first = int(bad[0])
        if self.least_margin_mm[first] < -TOUCH_MM:
            return "collision"
        if self.tip[first, 0] < -TOUCH_MM:
            return "tip"
        return "transmission"


def _gaps(finger: Finger, packaging: LinkagePackaging, settings: MotionSettings,
          b: np.ndarray, bp: np.ndarray, tip: np.ndarray) -> dict[tuple[str, str], np.ndarray]:
    """Every checked pair's gap at the posed samples: the pads against the other
    body's parts, the distal pins against the PIP servo, the links as capsules."""

    found = pad_margins(finger, packaging, b, tip, bp, settings.mcp_samples_rad)
    pins = [pair for pair in (layout(packaging).pair(pin, "pip_servo") for pin in ("bp_pin", "b_pin"))
            if pair is not None]
    if pins:
        found.update(margins(assembly(finger, packaging), poses(finger, b, tip, bp), packaging, pins))
    if settings.link_radius_mm is not None:
        found.update(link_capsule_margins(finger, packaging, b, tip, bp, settings.link_radius_mm))
    return found


def _moved_mm(finger: Finger, packaging: LinkagePackaging, b: np.ndarray, bp: np.ndarray,
              tip: np.ndarray) -> np.ndarray:
    """How far the distal body's farthest points (``B``, ``B'``, ``C`` and the
    distal pad's corners) move from each sample to the next (mm)."""

    corners = distal_pad_polygon(finger, packaging, b, tip)
    points = np.concatenate((b[:, None], bp[:, None], tip[:, None], corners), axis=1)
    return np.max(np.linalg.norm(np.diff(points, axis=0), axis=2), axis=1)


def sweep_flexion(finger: Finger, packaging: LinkagePackaging, settings: MotionSettings) -> LinkageMotion:
    """Sample the flexion from extension to pi (as far as the four-bar closes) and check each pose."""

    joint = four_bar(finger)
    sweep = finger.sweep_pip(uniform_samples_rad(math.pi, settings.step_rad))
    count = len(sweep.q_pip_rad)
    q = np.asarray(sweep.q_pip_rad, dtype=float)
    b, bp, tip = sweep.b, sweep.bp, sweep.tip
    found = _gaps(finger, packaging, settings, b, bp, tip)
    if settings.verify_step_rad is not None and count > 1 and found:

        def gaps_at(inner: np.ndarray) -> dict[tuple[str, str], np.ndarray]:
            fine = finger.sweep_pip(inner)
            gaps = _gaps(finger, packaging, settings, fine.b, fine.bp, fine.tip)
            if len(fine.q_pip_rad) < len(inner):  # the four-bar stops closing inside: a collision there
                gaps = {pair: np.concatenate((gap, np.full(len(inner) - len(gap), -math.inf)))
                        for pair, gap in gaps.items()}
            return gaps

        found = resample_between(q, found, _moved_mm(finger, packaging, b, bp, tip), settings.verify_step_rad,
                                 gaps_at)
    least = np.min(np.stack(list(found.values())), axis=0) if found else np.full(count, math.inf)
    reported = [gap for pair, gap in found.items() if pair not in FIXED_MARGIN_PAIRS]
    reported_least = np.min(np.stack(reported), axis=0) if reported else np.full(count, math.inf)
    overlapping = [(int(np.flatnonzero(gap < -TOUCH_MM)[0]), "/".join(pair))
                   for pair, gap in found.items() if np.any(gap < -TOUCH_MM)]
    first = min(overlapping)[1] if overlapping else ""
    valid = (least >= -TOUCH_MM) & (tip[:, 0] >= -TOUCH_MM)
    a = np.array((joint.a.x_mm, joint.a.z_mm))
    samples = MotionSamples(
        q_rad=q,
        valid=valid,
        theta_rad=np.asarray(sweep.servo_rad, dtype=float),
        gamma_rad=transmission_angle(a, bp, b),
        gamma_range_rad=settings.transmission_angle_range_rad,
    )
    return LinkageMotion(samples, b, bp, tip, least, reported_least, first)


def coverage(finger: Finger, packaging: LinkagePackaging, poses_rad: Sequence[float],
             include_linkage_occlusion: bool) -> tuple[PoseCoverage, ...]:
    """The cylinder-based coverage at each pose (0 where the four-bar does not close).

    With ``include_linkage_occlusion`` the crossing bars ``A-B'`` and ``A'-B``
    hide what they pass in front of on either face.
    """

    joint = four_bar(finger)
    sweep = finger.sweep_pip(np.asarray(poses_rad, dtype=float))
    a = np.array((joint.a.x_mm, joint.a.z_mm))
    ap = np.array((joint.ap.x_mm, joint.ap.z_mm))
    l1 = finger.proximal.end_mm
    l2 = finger.distal.end_mm - finger.distal.start_mm
    found = []
    for index, q in enumerate(poses_rad):
        if index >= len(sweep.q_pip_rad):
            found.append(PoseCoverage(float(q), 0.0, 0.0))
            continue
        b, bp, tip = sweep.b[index], sweep.bp[index], sweep.tip[index]
        proximal_face, proximal_normal, distal_faces, distal_normals = contact_faces(finger, packaging, b, tip)
        distal_face, distal_normal = distal_faces[0], distal_normals[0]
        hidden = (IntervalSet(), IntervalSet())
        if include_linkage_occlusion:
            bars = [(a, bp), (ap, b)]
            hidden = (face_occluded_set(proximal_face, proximal_normal, segments=bars),
                      face_occluded_set(distal_face, distal_normal, segments=bars))
        distal_start = b + finger.distal.start_mm * (tip - b) / finger.distal.end_mm
        found.append(PoseCoverage(
            float(q),
            common_length_mm(proximal_face, distal_face, hidden),
            posed_finger_length_mm(l1, l2, (l1, 0.0), distal_start),
        ))
    return tuple(found)


def servo_sense(finger: Finger, step_rad: float) -> float:
    """The sign of the PIP servo's rotation that flexes the PIP (``+1`` or ``-1``)."""

    return 1.0 if finger.sweep_servo([0.0, step_rad]).q_driver_rad[-1] >= 0.0 else -1.0


def transmission_angle(a: np.ndarray, bp: np.ndarray, b: np.ndarray) -> np.ndarray:
    """The transmission angle ``gamma`` at ``B'`` between ``B'->A`` and ``B'->B`` per sample, 0 to pi."""

    first, second = a - bp, b - bp
    cosine = np.sum(first * second, axis=1) / (np.linalg.norm(first, axis=1) * np.linalg.norm(second, axis=1))
    return np.arccos(np.clip(cosine, -1.0, 1.0))


__all__ = [
    "LinkageMotion",
    "MotionSettings",
    "TOUCH_MM",
    "coverage",
    "servo_sense",
    "sweep_flexion",
    "transmission_angle",
]
