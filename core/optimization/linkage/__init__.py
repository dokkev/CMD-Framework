"""The four-bar linkage finger study (``MECHANISM``, registered as ``linkage``):
its ``goal`` / ``solver`` config, design vector, evaluation and pad extension."""

from core.optimization.linkage.config import LinkageConfig, config_from_goal
from core.optimization.linkage.design import DEFAULT_VALUES, VARIABLE_NAMES, decode, encode, read_design
from core.optimization.linkage.evaluate import CONSTRAINT_NAMES, LinkageEvaluation, evaluate
from core.optimization.linkage.mechanism import MECHANISM, LinkageMechanism

__all__ = [
    "CONSTRAINT_NAMES",
    "DEFAULT_VALUES",
    "MECHANISM",
    "VARIABLE_NAMES",
    "LinkageConfig",
    "LinkageEvaluation",
    "LinkageMechanism",
    "config_from_goal",
    "decode",
    "encode",
    "evaluate",
    "read_design",
]
