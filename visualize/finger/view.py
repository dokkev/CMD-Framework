"""The finger's view: what every mechanism shares, its joint's view for the rest."""

from __future__ import annotations

import dataclasses
import math
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np

import core.morphology.packaging.gaps as packaging_module
from core.morphology.finger import Finger
from core.optimization.mechanism import mechanism as find_mechanism
from core.optimization.study import load_run
from visualize import sketch
from visualize.finger import solid
from visualize.style import LABELS

EXACT_BAND_MM = 1.0e6
"""The figures' gaps are exact however far (the checks bound the far ones from below only)."""


@contextmanager
def exact_gaps():
    """Every gap exact while it lasts (``core.morphology.packaging.gaps.EXACT_BAND_MM``)."""

    band = packaging_module.EXACT_BAND_MM
    packaging_module.EXACT_BAND_MM = EXACT_BAND_MM
    try:
        yield
    finally:
        packaging_module.EXACT_BAND_MM = band

DISTAL_PARTS = frozenset(("distal_pad", "distal_roller", "distal_carrier", "b_pin", "bp_pin"))
"""The parts riding on the distal segment: a pair with none of them keeps
its gap through the motion (the MCP held at 0), so it says nothing of the pose."""
REACH_STEP_DEG = 0.05
"""The flexion grid a joint's reach is found on."""
K_STEP_RAD = 1.0e-4
"""Half the flexion step ``k`` is differenced over."""


class JointView(Protocol):
    """A mechanism's joint as the view needs it: built from a study config and
    a design vector (``load``), it poses, checks and draws its own parts."""

    state: Any
    """The mechanism's own model: a ``core.morphology.finger.Finger`` or its solids."""
    packaging: Any

    @classmethod
    def load(cls, config: Any, x: tuple[float, ...]) -> "JointView": ...

    def reachable_deg(self) -> float:
        """The most flexion the joint closes to, at most 180."""

    def margins(self, phi_deg: np.ndarray) -> dict[tuple[str, str], np.ndarray]:
        """Each checked pair's gap at the flexions, ``(n,)`` per pair."""

    def icr(self, phi_deg: np.ndarray) -> np.ndarray:
        """The distal segment's centre of rotation relative to the proximal, ``(n, 2)``."""

    def k(self, phi_deg: np.ndarray) -> np.ndarray:
        """``d(phi)/d(theta)``, ``(n,)``."""

    def tip(self, phi_deg: float) -> np.ndarray: ...

    def distal_pad(self, phi_deg: float) -> np.ndarray: ...

    def proximal_pad(self) -> np.ndarray: ...

    def draw_solid(self, axis, phi_deg: float, hit: set[str]) -> None:
        """The joint's own parts as solids (``visualize.finger.solid``), not the palm or the pads."""

    def draw_sketch(self, axis, phi_deg: float, hit: set[str]) -> None:
        """The joint's own parts as the paper's schematic (``visualize.sketch``)."""

    def status(self, phi_deg: float) -> list[str]:
        """The joint's own readings at the flexion, one line each (none: ``[]``)."""


class KinematicJoint:
    """A joint view posed through its ``core.morphology.finger.Finger``
    (:meth:`Finger.sweep_pip`): its reach, centre of rotation, ``k`` and
    fingertip, whatever the joint is."""

    finger: Finger

    def __init__(self, finger: Finger) -> None:
        self.finger = finger

    def reachable_deg(self) -> float:
        if not hasattr(self, "_reach"):
            grid = np.radians(np.arange(0.0, 180.0 + 1e-9, REACH_STEP_DEG))
            solved = self.finger.sweep_pip(grid).q_pip_rad
            self._reach = float(np.degrees(solved[-1])) if len(solved) else 0.0
        return self._reach

    def sweep(self, phi_deg) -> Any:
        """The finger's sweep at the flexions (past its reach, at its reach), in their order."""

        phi = np.radians(np.minimum(np.atleast_1d(np.asarray(phi_deg, dtype=float)), self.reachable_deg()))
        return self._solve(phi)

    def _solve(self, phi_rad: np.ndarray) -> Any:
        order = np.argsort(phi_rad, kind="stable")  # ascending: the first that fails is past every other
        solved = self.finger.sweep_pip(phi_rad[order])
        n = len(phi_rad)
        if len(solved.q_pip_rad) < n:
            raise ValueError("the joint does not close at every flexion asked")
        inverse = np.argsort(order)
        return dataclasses.replace(solved, **{
            field.name: value[inverse] for field in dataclasses.fields(solved)
            if isinstance(value := getattr(solved, field.name), np.ndarray) and value.shape[:1] == (n,)})

    def icr(self, phi_deg) -> np.ndarray:
        return np.asarray(self.sweep(phi_deg).icr, dtype=float)

    def k(self, phi_deg) -> np.ndarray:
        """By central differences of the servo angle (one-sided at extension and the reach)."""

        phi = np.radians(np.minimum(np.atleast_1d(np.asarray(phi_deg, dtype=float)), self.reachable_deg()))
        low = np.maximum(phi - K_STEP_RAD, 0.0)
        high = np.minimum(phi + K_STEP_RAD, math.radians(self.reachable_deg()))
        servo = self._solve(np.concatenate((low, high))).servo_rad
        return (high - low) / np.abs(servo[len(phi):] - servo[:len(phi)])

    def tip(self, phi_deg: float) -> np.ndarray:
        return np.asarray(self.sweep([phi_deg]).tip[0], dtype=float)

    def distal_start(self, phi_deg: float) -> np.ndarray:
        """Where the distal segment's body starts (``B''``), on its posed line (the finger's own)."""

        phi = math.radians(min(phi_deg, self.reachable_deg()))
        return np.asarray(self.finger.kinematics([phi]).distal_start[0], dtype=float)

    def status(self, phi_deg: float) -> list[str]:
        return []


@dataclass
class FingerView:
    """One design of one mechanism: its joint's view, and the front's row it came from (if any)."""

    mechanism: str
    joint: JointView
    row: dict[str, str] | None = None

    @property
    def label(self) -> str:
        return LABELS[self.mechanism]

    @property
    def state(self) -> Any:
        """The mechanism's own model (a ``core.morphology.finger.Finger`` or its solids)."""

        return self.joint.state

    @property
    def packaging(self) -> Any:
        return self.joint.packaging

    @property
    def phi_max_deg(self) -> float:
        """The safe flexion: the row's (verified), else the joint's reach."""

        return float(self.row["phi_max_deg"]) if self.row else self.reachable_deg()

    @property
    def pads_mm(self) -> tuple[float, float]:
        """The pads' effective lengths (the row's; NaN without one)."""

        if not self.row:
            return math.nan, math.nan
        return float(self.row["proximal_effective_mm"]), float(self.row["distal_effective_mm"])

    def reachable_deg(self) -> float:
        return self.joint.reachable_deg()

    def margins(self, phi_deg) -> dict[tuple[str, str], np.ndarray]:
        """Each checked pair's gap at the flexions, exact however far."""

        with exact_gaps():
            return self.joint.margins(np.atleast_1d(np.asarray(phi_deg, dtype=float)))

    def gaps(self, phi_deg: float) -> dict[tuple[str, str], float]:
        """Each checked pair's gap at ``phi_deg`` (the finite ones)."""

        return {pair: float(margin[0]) for pair, margin in self.margins([phi_deg]).items() if np.isfinite(margin[0])}

    def collisions(self, phi_deg: float, gaps: dict[tuple[str, str], float] | None = None) -> set[str]:
        """The parts of every pair whose gap is negative at ``phi_deg``."""

        gaps = self.gaps(phi_deg) if gaps is None else gaps
        return {name for pair, gap in gaps.items() if gap < 0.0 for name in pair}

    def icr(self, phi_deg) -> np.ndarray:
        return self.joint.icr(np.atleast_1d(np.asarray(phi_deg, dtype=float)))

    def k(self, phi_deg) -> np.ndarray:
        return self.joint.k(np.atleast_1d(np.asarray(phi_deg, dtype=float)))

    def tip(self, phi_deg: float) -> np.ndarray:
        return self.joint.tip(phi_deg)

    def distal_pad(self, phi_deg: float) -> np.ndarray:
        return self.joint.distal_pad(phi_deg)

    def proximal_pad(self) -> np.ndarray:
        return self.joint.proximal_pad()

    def closest(self, phi_deg: float, gaps: dict[tuple[str, str], float] | None = None,
                ) -> tuple[tuple[str, str], float] | None:
        """The pair the motion brings closest at ``phi_deg`` and its gap: of
        the pairs with a distal part (:data:`DISTAL_PARTS`; the others keep
        theirs throughout), unless a pair collides, then the deepest
        collision of any; ``None``: no pair measured."""

        gaps = self.gaps(phi_deg) if gaps is None else gaps
        colliding = {pair: gap for pair, gap in gaps.items() if gap < 0.0}
        moving = colliding or {pair: gap for pair, gap in gaps.items() if DISTAL_PARTS & set(pair)}
        return min(moving.items(), key=lambda item: item[1]) if moving else None

    def draw(self, axis, phi_deg: float, hit: set[str] | None = None, style: str = "solid") -> None:
        """The finger at one flexion: the palm, the joint's parts and the pads,
        ``hit`` marked as colliding (``None``: the parts colliding there).

        ``style``: ``solid``, the parts in their collision groups' colours on
        a mm grid, the points named and the pose's readings in a corner
        (``visualize.finger.solid``); ``sketch``, the paper's schematic
        (``visualize.sketch``, framed by the caller)."""

        gaps = self.gaps(phi_deg) if hit is None or style == "solid" else {}
        hit = self.collisions(phi_deg, gaps) if hit is None else set(hit)
        if style == "sketch":
            sketch.palm(axis, self.packaging)
            self.joint.draw_sketch(axis, phi_deg, hit)
            sketch.pad(axis, self.proximal_pad(), "proximal_pad" in hit)
            sketch.pad(axis, self.distal_pad(phi_deg), "distal_pad" in hit)
            return
        if style != "solid":
            raise ValueError(f"style {style!r}: solid or sketch")
        solid.servo(axis, self.packaging.mcp_footprint().outline, "palm", "mcp_servo" in hit)
        self.joint.draw_solid(axis, phi_deg, hit)
        solid.pad(axis, self.proximal_pad(), "proximal", "proximal_pad" in hit)
        solid.pad(axis, self.distal_pad(phi_deg), "distal", "distal_pad" in hit)
        found = self.closest(phi_deg, gaps)
        if found is None:
            closest = "closest   -"
        else:
            (pair, gap) = found
            closest = f"{'COLLISION' if gap < 0.0 else 'closest'} {gap:5.1f} mm ({'/'.join(pair)})"
        solid.status(axis, [f"phi {phi_deg:5.1f}°  k {float(self.k([phi_deg])[0]):4.2f}",
                            *self.joint.status(phi_deg), closest])
        solid.legend(axis)
        solid.plane(axis)


def view_of(mechanism: str, config: Any, x, row: dict[str, str] | None = None) -> FingerView:
    """The view of a design vector ``x`` of ``mechanism`` under a study ``config``."""

    from visualize.finger.mechanism import JOINTS

    if mechanism not in JOINTS:
        raise ValueError(f"no view of {mechanism!r}; one of {sorted(JOINTS)}")
    return FingerView(mechanism, JOINTS[mechanism].load(config, tuple(float(v) for v in x)), row)


def view_from_row(row: dict[str, str], study: Path, mechanism: str | None = None) -> FingerView:
    """The design of a front's ``row``; ``study``: the directory of its runs
    (``mechanism``: the run's, unless given)."""

    run = load_run(Path(study) / row["run"].split("/")[-1])
    return view_of(mechanism or run.mechanism.name, run.config, run.mechanism.read_design(row), row)


def view_of_vector(mechanism: str, config_path: Path, x, overrides=()) -> FingerView:
    """The view of a design vector under a config file (``load_yaml``)."""

    from core.optimization.study import load_yaml

    found = find_mechanism(mechanism)
    return view_of(mechanism, load_yaml(found, config_path, overrides)[0], x)


__all__ = ["FingerView", "JointView", "KinematicJoint", "exact_gaps", "view_from_row", "view_of", "view_of_vector"]
