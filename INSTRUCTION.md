CODEX TASK
==========

Task:
Refactor the CLEAR finger linkage parameterization so the fully-extended
reference geometry is specified directly by explicit 2D YZ translations from
named reference points, instead of proximal/middle/distal link lengths plus
local anchor coordinates and recovered reference rotations.


OBJECTIVE
=========

Replace the current linkage design representation:

    MainChainParameters
    CouplingAnchorParameters
    proximal / middle / distal body terminology
    implicit recovery of reference body orientations

with a direct reference-coordinate representation.

The user-facing linkage inputs must make the reference origin of every point
explicit.

Canonical reference-pose ownership is:

    O frame / O reference point:
        O'
        A
        A'

    A reference point:
        B
        B'

    B reference point:
        C
        C'

There are no user-specified reference rotation angles.

The fully-extended reference assembly is defined by:

    BC parallel to OC

with BC pointing in the same distal direction as OC.

Do NOT require O-A'-B'-B-C collinearity.
Do NOT require OA to be horizontal.
Do NOT require OA' to be horizontal.
Do NOT recover reference body angles by forcing coupling anchors onto a line.


ESTABLISHED CONTEXT
===================

Repository:
    https://github.com/dokkev/CMD-Framework

Relevant files currently include:

    cmd/morphology/linkage_parameter.py
    cmd/morphology/linkage.py
    cmd/morphology/linkage_kinematics.py
    cmd/morphology/linkage_seed.py
    cmd/morphology/__init__.py

    tests/test_linkage.py
    tests/test_linkage_kinematics.py

    scripts/animate_linkage.py

    docs/design/morphology_source_of_truth.md

Canonical linkage plane:

    X = lateral / revolute-axis direction
    Y = distal
    Z = dorsal

The mechanism is represented in the YZ plane.

O is the linkage reference origin:

    O = (0, 0)

All coordinates described below are reference-pose YZ translations.

There is no hidden frame rotation associated with an input coordinate.

For example:

    a_from_o = (y, z)

means exactly:

    A_reference = O_reference + a_from_o

and:

    b_from_a = (y, z)

means exactly:

    B_reference = A_reference + b_from_a

The translation is expressed directly in the canonical YZ coordinates of the
fully-extended reference assembly.

Runtime linkage motion may rotate these stored reference vectors. That does
not mean a reference rotation is part of the user input.


CANONICAL PARAMETERIZATION
==========================

Use one small planar coordinate type with neutral terminology.

Prefer:

    PointYZ

or an equivalently small neutral name.

Do not retain `AnchorYZ` if it is then used for main joints A, B, and C;
these are not all coupling anchors.

The user-facing linkage parameters should be conceptually equivalent to:

    @dataclass(frozen=True)
    class FingerLinkageParameters:
        o_prime_from_o: PointYZ

        a_from_o: PointYZ
        a_prime_from_o: PointYZ

        b_from_a: PointYZ
        b_prime_from_a: PointYZ

        bc_length_mm: float
        c_prime_from_b: PointYZ

The exact field spelling may be adjusted only if clarity improves, but every
field must make its reference point unambiguous.

Do not use generic fields such as:

    proximal_anchor
    middle_anchor
    distal_anchor

Do not introduce:

    proximal_reference_angle
    middle_reference_angle
    distal_reference_angle

No reference angle input is needed.


REFERENCE GEOMETRY
==================

Construct the reference geometry only by translation.

Canonical construction:

    O = (0, 0)

    O' = O + o_prime_from_o

    A  = O + a_from_o
    A' = O + a_prime_from_o

    B  = A + b_from_a
    B' = A + b_prime_from_a

    C  = B + (bc_length_mm, 0)
    C' = B + c_prime_from_b

Because the linkage plane uses +Y as distal, the fixed reference BC vector is:

    B->C = (bc_length_mm, 0)

in YZ coordinates.

Therefore C is not an arbitrary 2D design point.

Only its positive distal translation from B is independently parameterized.

Do not expose `c_from_b` as an unconstrained 2D parameter when its Z component
is architecturally fixed to zero.


FULLY-EXTENDED DEFINITION
=========================

The reference pose is "fully extended" when:

    BC || OC

and BC and OC point in the same direction.

Implement this geometrically, not through coupling-anchor collinearity.

Equivalent robust validation is:

    cross_2d(BC, OC) ~= 0
    dot(BC, OC) > 0

using an appropriate small numerical tolerance.

Because BC is constructed along canonical +Y, this condition implies that B
and C lie on the O-C distal line in the reference pose.

This is intentional.

The following are NOT fully-extended requirements:

    OA horizontal
    AB horizontal
    OA' horizontal
    A'B' horizontal
    O-A'-B'-B-C collinear
    A' or B' lying on the O-B-C line

Remove any existing validation or reference construction that assumes those
conditions.


POINT OWNERSHIP
===============

The physical interpretation must remain explicit.

`a_from_o`
    A relative to O.
    A belongs to the first moving rigid body about O.

`a_prime_from_o`
    A' relative to O.
    A' is a coupling point rigidly attached to the same O-based moving body.

`b_from_a`
    B relative to A.
    B belongs to the next rigid body about A.

`b_prime_from_a`
    B' relative to A.
    B' is rigidly attached to the same A-based moving body.

`bc_length_mm`
    Positive B->C distal length in the reference pose.

`c_prime_from_b`
    C' relative to B.
    C' is rigidly attached to the B-based distal body.

`o_prime_from_o`
    Fixed base coupling point O' relative to O.

The important distinction is:

    the reference coordinate is specified relative to a named point,
    not relative to an implicitly rotated "proximal/middle/distal frame".


DERIVED GEOMETRY
================

The following must be derived, not independently parameterized:

    |OA| = norm(a_from_o)

    |AB| = norm(b_from_a)

    |BC| = bc_length_mm

    |O'B'| = distance(O', B')

    |A'C'| = distance(A', C')

Reference world positions:

    O, O', A, A', B, B', C, C'

must be derived by the translation chain above.

If useful for current callers, `FingerLinkage` may expose derived properties
such as:

    oa_length_mm
    ab_length_mm
    bc_length_mm
    coupler_1_length_mm
    coupler_2_length_mm

but these must not become duplicate stored design inputs.


LINKAGE KINEMATICS
==================

Update `linkage_kinematics.py` to consume the new reference-vector
representation directly.

Do not reconstruct hidden reference angles.

The stored vectors already describe the zero/reference configuration.

For runtime motion, treat rotations as changes from that reference pose.

For an input proximal rotation change `q1_rad`:

    A(q1)  = O + R(q1) * a_from_o
    A'(q1) = O + R(q1) * a_prime_from_o

O' remains fixed.

Loop 1
------

B' belongs to the rigid body rotating about A.

Its fixed radius is:

    r_AB' = norm(b_prime_from_a)

Its fixed coupling distance to O' is:

    L_O'B'

Use the existing circle-intersection / assembly-branch approach to solve the
current B' position.

After selecting the correct B' solution:

    delta_middle =
        signed_rotation(
            b_prime_from_a,
            B'_current - A_current
        )

Then:

    B_current =
        A_current + R(delta_middle) * b_from_a

The returned relative joint change should be:

    q2_rad = wrap(delta_middle - q1_rad)

so q2 = 0 in the reference pose.


Loop 2
------

C' belongs to the rigid body rotating about B.

Its fixed radius is:

    r_BC' = norm(c_prime_from_b)

Its fixed coupling distance to current A' is:

    L_A'C'

Use circle intersection again and preserve the reference assembly branch.

After selecting C':

    delta_distal =
        signed_rotation(
            c_prime_from_b,
            C'_current - B_current
        )

Then:

    C_current =
        B_current
        + R(delta_distal) * (bc_length_mm, 0)

and:

    q3_rad = wrap(delta_distal - delta_middle)

so q3 = 0 in the reference pose.

This should eliminate the current need for stored/recovered:

    _reference_theta1_rad
    _reference_theta2_rad
    _reference_theta3_rad

Do not preserve those merely for compatibility if the new representation makes
them unnecessary.


ASSEMBLY BRANCH
===============

Preserve deterministic branch selection.

The reference branch for loop 1 is defined by:

    centers:
        A_reference
        O'_reference

    reference point:
        B'_reference

The reference branch for loop 2 is defined by:

    centers:
        B_reference
        A'_reference

    reference point:
        C'_reference

Reuse the current cross-product branch-sign concept where appropriate.

Do not replace the deterministic branch logic with "choose nearest" or another
configuration-history-dependent heuristic.


BASELINE_LINKAGE_V0
===================

Rewrite `BASELINE_LINKAGE_V0` using the new explicit reference-coordinate
parameterization.

The existing seed should be geometrically preserved unless a new baseline
geometry has been explicitly supplied.

Do not invent a new linkage shape during this refactor.

Before deleting the old parameterization, derive its current fully-extended
reference positions and use those to construct the equivalent new relative
coordinates:

    o_prime_from_o = O' - O

    a_from_o       = A  - O
    a_prime_from_o = A' - O

    b_from_a       = B  - A
    b_prime_from_a = B' - A

    bc_length_mm   = |BC|

    c_prime_from_b = C' - B

Add a regression test demonstrating that the rewritten baseline produces the
same reference positions and coupler lengths within an appropriate numerical
tolerance.

Do not round the baseline into a different mechanism merely to make the seed
values look cleaner.

The design-space constraint that A' and B' should lie dorsally above B may be
added later when the design-space bounds/constraints are implemented.

Do not silently change the current seed to satisfy that future constraint.


REQUIRED CODE CHANGES
=====================

1. `cmd/morphology/linkage_parameter.py`

   - Remove `MainChainParameters`.
   - Remove `CouplingAnchorParameters`.
   - Replace `AnchorYZ` with a neutral planar point/offset type such as
     `PointYZ`.
   - Define `FingerLinkageParameters` directly from explicit reference-relative
     coordinates.
   - Make every reference point clear from the field name.
   - Validate only local structural requirements needed to construct the model:
       - all coordinates finite
       - `bc_length_mm` finite and positive
   - Do not put optimizer bounds or unrelated design-space constraints here.


2. `cmd/morphology/linkage.py`

   - Replace the current recovered-angle reference construction with direct
     translation composition.
   - Remove the O-A'-B'-B-C collinearity requirement.
   - Implement the BC || OC fully-extended validation.
   - Keep coupler lengths derived.
   - Expose derived main-link lengths if they are useful to callers.
   - Keep `ReferencePositions` or an equally clear small immutable result type.


3. `cmd/morphology/linkage_kinematics.py`

   - Rewrite the solver to rotate reference vectors by motion deltas from the
     reference configuration.
   - Remove recovered reference-angle state.
   - Preserve deterministic circle-intersection branch selection.
   - Preserve the meaning that q1, q2, q3 are changes from the reference pose.
   - `solve(0)` must reproduce the stored reference geometry.


4. `cmd/morphology/linkage_seed.py`

   - Port `BASELINE_LINKAGE_V0` to the new parameter representation.
   - Preserve the current baseline geometry numerically.
   - Do not invent new design values.


5. `cmd/morphology/__init__.py`

   - Update exports.
   - Remove obsolete public exports for deleted parameter classes.
   - Export only the neutral new linkage parameter API required by callers.


6. Callers

   Update directly affected callers, including:

       scripts/animate_linkage.py

   and any visualization/test code using the old parameter groups.

   Do not add compatibility wrappers unless a real external API requirement is
   documented in the repository.


7. Source of Truth

   Update:

       docs/design/morphology_source_of_truth.md

   so the linkage section clearly states:

       O is reference origin.

       Relative reference inputs:
           O' from O
           A from O
           A' from O
           B from A
           B' from A
           C from B only through positive BC distal length
           C' from B

       Fully extended:
           BC parallel to OC, same direction.

       No coupling-anchor collinearity assumption.
       No user-provided reference rotation angles.


IMPLEMENTATION RULES
====================

- Keep the mechanism planar in canonical YZ.
- X remains zero for all linkage points.
- Use millimetres.
- Keep data immutable.
- Prefer direct geometry over inferred hidden conventions.
- User-facing fields must communicate their reference point in the name.
- Do not use proximal/middle/distal terminology for the parameterization.
- Do not recover a reference rotation solely to interpret user input.
- Runtime rotations are allowed and required in `linkage_kinematics.py`;
  "no rotation" applies to how the reference geometry is specified by the
  user.
- Derived values must not be duplicated as independent inputs.
- Keep the change local to linkage morphology/kinematics and directly affected
  callers.
- Do not redesign the whole-hand morphology in this task.


VALIDATION
==========

Run focused tests for:

    tests/test_linkage.py
    tests/test_linkage_kinematics.py

and other directly affected linkage tests only.

Required behavioral checks:

1. Reference construction

       O = (0, 0)

       O' = O + o_prime_from_o
       A  = O + a_from_o
       A' = O + a_prime_from_o

       B  = A + b_from_a
       B' = A + b_prime_from_a

       C  = B + (bc_length_mm, 0)
       C' = B + c_prime_from_b


2. Fully-extended condition

   Verify:

       BC x OC ~= 0
       BC dot OC > 0

   and verify that a non-parallel reference configuration is rejected.


3. No obsolete collinearity requirement

   Add a valid reference linkage where:

       OA is not horizontal
       A' is not on O-B-C
       B' is not on O-B-C

   while:

       BC || OC

   and confirm construction succeeds.


4. Derived lengths

   Verify:

       OA
       AB
       BC
       O'B'
       A'C'

   are correctly derived.


5. Zero pose

   Verify:

       LinkageKinematics(linkage).solve(0)

   reproduces the reference positions and:

       q1 = 0
       q2 = 0
       q3 = 0


6. Motion

   Verify nonzero q1 closes both coupling loops and preserves:

       |OA|
       |AB|
       |BC|
       |O'B'|
       |A'C'|

   within tolerance.


7. Baseline regression

   Verify the new `BASELINE_LINKAGE_V0` reproduces the pre-refactor baseline
   reference positions and coupler lengths within tolerance.


HARD STOPS
==========

Stop and report instead of guessing if:

- preserving the current `BASELINE_LINKAGE_V0` geometry cannot be determined
  from the existing implementation;
- the current kinematics depends on an undocumented physical constraint beyond
  the two coupling loops;
- a caller requires old `MainChainParameters` /
  `CouplingAnchorParameters` semantics for a reason not visible in the repo;
- implementing the refactor would require changing the hand-level morphology,
  actuator geometry, or unrelated fingertip model.

Do not invent new baseline coordinates to get past a failure.


OUT OF SCOPE
============

Do not:

- optimize the linkage;
- define design-space bounds;
- enforce future manufacturability constraints;
- redesign actuator packaging;
- change hand layout;
- change fingertip morphology;
- add 3D linkage motion;
- add a new general-purpose kinematics library;
- require A' or B' to be above B as part of this refactor unless that constraint
  already has an explicit canonical numeric definition elsewhere.

The A'/B' dorsal-placement rule can be implemented later as a design-space
constraint.


FINAL REPORT
============

Report only:

1. New linkage parameter API
2. How reference positions are constructed
3. How the kinematics solver changed
4. How `BASELINE_LINKAGE_V0` was preserved
5. Focused test results
6. Any unresolved issue

STOP after completing the task.