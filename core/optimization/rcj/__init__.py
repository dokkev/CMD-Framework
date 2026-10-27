"""The rolling contact joint (RCJ) finger on the linkage study's goal
(``MECHANISM``, registered as ``rcj``; the retention study's RCJ is
``core.optimization.rolling_contact_joint``)."""

from core.optimization.rcj.config import VARIABLE_NAMES, RcjConfig
from core.optimization.rcj.evaluate import CONSTRAINT_NAMES, RcjEvaluation, RcjGeometry, evaluate, geometry
from core.optimization.rcj.mechanism import MECHANISM, RcjMechanism

__all__ = ["CONSTRAINT_NAMES", "MECHANISM", "VARIABLE_NAMES", "RcjConfig", "RcjEvaluation", "RcjGeometry",
           "RcjMechanism", "evaluate", "geometry"]
