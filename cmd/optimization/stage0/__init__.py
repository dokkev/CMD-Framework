"""Stage-0: NSGA-II search for initial ordinary-finger linkage parameters.

The pymoo runner lives in :mod:`cmd.optimization.stage0.run` and is not
imported here, so configuration and evaluation work without pymoo.
"""

from .config import Stage0Config
from .design import VARIABLE_NAMES, bounds_around, decode, encode
from .evaluate import Stage0Evaluation, Stage0Evaluator, input_samples

__all__ = [
    "Stage0Config",
    "Stage0Evaluation",
    "Stage0Evaluator",
    "VARIABLE_NAMES",
    "bounds_around",
    "decode",
    "encode",
    "input_samples",
]
