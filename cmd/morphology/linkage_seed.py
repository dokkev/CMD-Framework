"""Named initial designs for CLEAR linkage studies."""

from .linkage import FingerLinkage
from .linkage_parameter import FingerLinkageParameters, PointYZ


# These translations were derived from the pre-refactor BASELINE_LINKAGE_V0
# reference positions. Keep their full precision so the seed geometry and
# coupler lengths remain unchanged.
BASELINE_LINKAGE_V0 = FingerLinkage(
    FingerLinkageParameters(
        o_prime_from_o=PointYZ(10.0, 10.0),
        a_from_o=PointYZ(35.005085389255939, 4.997358991597482),
        a_prime_from_o=PointYZ(30.001126645511164, 0.0),
        b_from_a=PointYZ(25.005787138053947, -4.996059407806905),
        b_prime_from_a=PointYZ(4.998018646790356, -4.996059407806905),
        bc_length_mm=20.0,
        c_prime_from_b=PointYZ(-10.0, 5.0),
    )
)


__all__ = ["BASELINE_LINKAGE_V0"]
