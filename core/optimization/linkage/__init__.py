"""The four-bar linkage finger study (``MECHANISM``, registered as ``linkage``):
its ``goal`` / ``solver`` config, search coordinates and evaluation against
the goal. The finger itself (design vector, kinematics, packaging, motion,
pad extension) is ``core.morphology.linkage``'s."""

from core.optimization.linkage.config import LinkageConfig, config_from_goal
from core.optimization.linkage.evaluate import CONSTRAINT_NAMES, LinkageEvaluation, evaluate
from core.optimization.linkage.mechanism import MECHANISM, LinkageMechanism

__all__ = [
    "CONSTRAINT_NAMES",
    "MECHANISM",
    "LinkageConfig",
    "LinkageEvaluation",
    "LinkageMechanism",
    "config_from_goal",
    "evaluate",
]
