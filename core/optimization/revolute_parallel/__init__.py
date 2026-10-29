"""The serial 2R driven through a belt from a servo on ``A'`` (the actuator
moved off the joint), on the linkage study's goal (``MECHANISM``, registered
as ``revolute_parallel``)."""

from core.optimization.revolute_parallel.config import VARIABLE_NAMES, ParallelConfig
from core.optimization.revolute_parallel.evaluate import (
    CONSTRAINT_NAMES,
    ParallelEvaluation,
    ParallelGeometry,
    evaluate,
    geometry,
)
from core.optimization.revolute_parallel.mechanism import MECHANISM, ParallelMechanism

__all__ = ["CONSTRAINT_NAMES", "MECHANISM", "VARIABLE_NAMES", "ParallelConfig", "ParallelEvaluation",
           "ParallelGeometry", "ParallelMechanism", "evaluate", "geometry"]
