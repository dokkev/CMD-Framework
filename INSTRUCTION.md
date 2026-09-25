CODEX TASK
==========

Task:
Add a solver-agnostic modular optimization layer to CMD-Framework and define
the first reusable optimization problem for selecting a kinematically healthy
CLEAR ordinary-finger linkage baseline.

The optimization framework must support:

    box-bounded design variables
    equality constraints
    inequality constraints
    multiple objectives

as independently composable modules.

Do NOT add or choose an optimization algorithm in this task.
Do NOT add pymoo, CMA-ES, scipy.optimize, or another solver dependency yet.


OBJECTIVE
=========

Build a small generic optimization abstraction that allows problems to be
assembled like:

    variables
        +
    parameterization
        +
    evaluator
        +
    objectives
        +
    equality constraints
        +
    inequality constraints

without putting optimizer-specific concepts inside `cmd/morphology`.

Then implement a linkage-specific problem definition using the current
`FingerLinkageParameters`, `FingerLinkage`, and `LinkageKinematics`.

The first linkage problem should expose two objectives:

    1. maximize worst-case transmission quality
    2. minimize coupled-motion ratio variation

while supporting modular hard constraints for:

    full workspace kinematic validity
    A' / B' dorsal placement
    minimum transmission-angle margin
    monotonic q2/q3 flexion
    maximum q2/q3 motion ratio
    minimum q2/q3 endpoint flexion

The framework must remain usable later for fingertip morphology, hand layout,
contact mechanics, or other CMD optimization problems.


ESTABLISHED REPOSITORY STATE
============================

Work from the current repository state:

    repository:
        dokkev/CMD-Framework

    inspected commit:
        66a391f20bc514f39893fb3244fe943d05a8116a

Relevant existing structure:

    cmd/
    ├── design/
    │   ├── constraints.py
    │   └── urdf.py
    ├── morphology/
    │   ├── linkage.py
    │   ├── linkage_parameter.py
    │   ├── linkage_kinematics.py
    │   ├── linkage_seed.py
    │   ├── hand.py
    │   └── ...
    └── visualization/

There is currently no generic optimization package and no optimization-solver
dependency in the repository.

Do not move or redesign the existing `cmd/design/constraints.py` actuator
packaging code in this task.


CURRENT LINKAGE API
===================

`cmd/morphology/linkage_parameter.py` currently defines:

    PointYZ

    FingerLinkageParameters(
        o_prime_from_o,
        a_from_o,
        a_prime_from_o,
        b_from_a,
        b_prime_from_a,
        bc_length_mm,
        c_prime_from_b,
    )

All inputs are direct YZ reference-pose translations.

`cmd/morphology/linkage.py` derives:

    O
    O'
    A
    A'
    B
    B'
    C
    C'

and:

    |OA|
    |AB|
    |O'B'|
    |A'C'|

The fully extended reference requires:

    BC || OC

with the same distal direction.

`cmd/morphology/linkage_kinematics.py` already solves the two closed loops
using deterministic reference-branch selection.

Do not replace that solver.

Use it as the canonical source of linkage kinematics.


FLEXION SIGN CONVENTION
=======================

The current repository already establishes the runtime convention in
`scripts/animate_linkage.py`:

    q1 = 0
        fully extended reference

    q1 < 0
        flexion

The existing default animation uses:

    maximum flexion magnitude = 0.8 rad

Do not reverse this convention.

For optimization/evaluation, introduce a positive sweep coordinate:

    s >= 0

where:

    q1 = -s

and:

    s in [0, s_max]

This avoids sign confusion in optimization metrics.

Define positive joint-flexion magnitudes from unwrapped relative joint angles:

    f1(s) = s
    f2(s) = -unwrap(q2(s))
    f3(s) = -unwrap(q3(s))

Therefore a normally flexing mechanism should have:

    df2/ds >= 0
    df3/ds >= 0


ARCHITECTURE
============

Add:

    cmd/optimization/
    ├── __init__.py
    ├── variable.py
    ├── objective.py
    ├── constraint.py
    ├── problem.py
    └── linkage/
        ├── __init__.py
        ├── parameterization.py
        ├── evaluator.py
        ├── objectives.py
        ├── constraints.py
        └── baseline.py

Do not create more subpackages than this.

Keep the generic optimization package independent from morphology-specific
implementation details.

`cmd/optimization/linkage/` may import from `cmd.morphology`.

`cmd/morphology` must not import from `cmd.optimization`.


GENERIC DESIGN VARIABLE API
===========================

In:

    cmd/optimization/variable.py

add a small immutable box-bounded scalar variable type conceptually equivalent
to:

    @dataclass(frozen=True)
    class BoxVariable:
        name: str
        lower: float
        upper: float

Validation:

    name must be non-empty
    lower and upper must be finite
    lower < upper

Provide a deterministic method/property for evaluating box violation.

Box bounds are part of the variable definition.

Do not represent box bounds as ordinary domain-specific inequality objects.

The optimization problem should still include box feasibility in its final
`feasible` result.


GENERIC OBJECTIVE API
=====================

In:

    cmd/optimization/objective.py

define an objective abstraction with:

    name
    sense
    evaluate(context) -> float

where:

    sense = "min" or "max"

Use one canonical convention.

Raw objective values must preserve their physical meaning.

Also provide a way for future solver adapters to obtain a minimization-form
value:

    minimization_value =
        raw                         for "min"
        -raw                        for "max"

Do not force all domain objectives to manually negate themselves.

Objectives may be independently composed into an optimization problem.


GENERIC CONSTRAINT API
======================

In:

    cmd/optimization/constraint.py

support two explicit constraint categories.


Equality constraint
-------------------

Canonical mathematical convention:

    h(x) = 0

Interface conceptually:

    name
    tolerance
    residual(context) -> tuple[float, ...]

Feasible when every element satisfies:

    |h_i| <= tolerance


Inequality constraint
---------------------

Canonical mathematical convention:

    g(x) <= 0

Interface conceptually:

    name
    residual(context) -> tuple[float, ...]

Feasible when:

    g_i <= 0

Do not mix >= and <= conventions between constraint implementations.

Vector-valued constraints must be supported.

For example, one monotonicity constraint may return one residual per workspace
sample instead of reducing the entire trajectory into one scalar.

Provide structured diagnostic results containing at least:

    constraint name
    residual values
    maximum violation
    satisfied / unsatisfied


GENERIC OPTIMIZATION PROBLEM
============================

In:

    cmd/optimization/problem.py

implement a solver-independent `OptimizationProblem`.

It should own:

    variables
    parameterization
    evaluator
    objectives
    equalities
    inequalities

Conceptual evaluation pipeline:

    x
      ↓
    map positional vector to named variable values
      ↓
    check box bounds
      ↓
    parameterization.decode(...)
      ↓
    evaluator.evaluate(...)
      ↓
    objectives
      ↓
    equalities
      ↓
    inequalities
      ↓
    ProblemEvaluation

`ProblemEvaluation` must provide enough diagnostic information to inspect a
candidate without an optimizer.

At minimum expose:

    raw objective values by name
    minimization-form objective values
    box violations
    equality results
    inequality results
    feasible
    domain evaluation/context

Do not flatten everything into anonymous F/G/H arrays inside the core API.

Future solver adapters can do that later.

Evaluation order and output ordering must be deterministic.


PARAMETERIZATION PROTOCOL
=========================

The generic optimization layer may define a small protocol/interface such as:

    variable_names
    decode(named_values)

Do not require every morphology type to implement optimization methods.

The optimization parameterization is an adapter between a solver design vector
and canonical morphology parameters.


LINKAGE OPTIMIZATION PARAMETERIZATION
=====================================

In:

    cmd/optimization/linkage/parameterization.py

implement a `LinkageBaselineParameterization`.

Do NOT expose the fully-extended relation as an equality constraint.

Instead parameterize the optimization design space so it is satisfied
identically.

Use explicit fully-extended reference positions as optimization coordinates.

Recommended optimization vector:

    o_prime_from_o_y_mm
    o_prime_from_o_z_mm

    a_from_o_y_mm
    a_from_o_z_mm

    a_prime_from_o_y_mm
    a_prime_from_o_z_mm

    b_from_o_y_mm

    b_prime_from_o_y_mm
    b_prime_from_o_z_mm

    bc_length_mm

    c_prime_from_o_y_mm
    c_prime_from_o_z_mm

This is a 12-scalar latent optimization representation.

Its coordinate references are intentionally explicit in the names.


Decoder
-------

Construct:

    O = (0, 0)

    O' = (
        o_prime_from_o_y_mm,
        o_prime_from_o_z_mm
    )

    A = (
        a_from_o_y_mm,
        a_from_o_z_mm
    )

    A' = (
        a_prime_from_o_y_mm,
        a_prime_from_o_z_mm
    )

    B = (
        b_from_o_y_mm,
        0
    )

    B' = (
        b_prime_from_o_y_mm,
        b_prime_from_o_z_mm
    )

    C = (
        b_from_o_y_mm + bc_length_mm,
        0
    )

    C' = (
        c_prime_from_o_y_mm,
        c_prime_from_o_z_mm
    )

Then convert these reference positions into the canonical morphology API:

    o_prime_from_o = O' - O

    a_from_o = A - O

    a_prime_from_o = A' - O

    b_from_a = B - A

    b_prime_from_a = B' - A

    bc_length_mm = bc_length_mm

    c_prime_from_b = C' - B

and return:

    FingerLinkageParameters(...)

This guarantees exactly:

    B_z = 0
    C_z = 0
    BC along +Y
    BC || OC

without an equality constraint.


Important
---------

This 12-variable representation is an optimizer coordinate system only.

Do NOT replace the canonical `FingerLinkageParameters` API with it.

The morphology layer must remain optimizer-independent.


Encoding / seed support
-----------------------

Provide an `encode(...)` or equivalent helper that maps an existing
`FingerLinkage` reference geometry into the 12-scalar optimization vector.

It should be usable with:

    BASELINE_LINKAGE_V0

The current V0 reference has only a tiny numerical residual from exact B_z=0
because it preserves the pre-refactor geometry.

Do not modify `BASELINE_LINKAGE_V0` merely to make the optimizer seed exact.

Encoding may project the reference B position onto the exact fully-extended
manifold used by the optimizer.

Document and test that this projection is tiny rather than silently changing
the canonical seed.


LINKAGE SWEEP EVALUATOR
=======================

In:

    cmd/optimization/linkage/evaluator.py

add an immutable configuration such as:

    LinkageSweepConfig(
        max_flexion_rad,
        sample_count,
    )

Requirements:

    max_flexion_rad > 0
    sample_count >= 3

The current repository visualization uses:

    max_flexion_rad = 0.8

so this is a reasonable default sweep value for the evaluator.

It is not a permanent mechanical joint limit.


Sweep
-----

Generate:

    s_i in [0, max_flexion_rad]

and solve:

    q1_i = -s_i

using:

    LinkageKinematics(linkage).solve(q1_i)

Do not duplicate closed-loop kinematics in optimization code.


Evaluation result
-----------------

Create a structured `LinkageEvaluation` containing at least:

    parameters
    linkage

    input_flexion_rad
    q1_rad

    solved poses

    unwrapped q2_rad
    unwrapped q3_rad

    q2_flexion_rad
    q3_flexion_rad

    dq2_flexion_dq1_flexion
    dq3_flexion_dq1_flexion

    transmission_angle_loop_1_rad
    transmission_angle_loop_2_rad

    transmission_quality_loop_1
    transmission_quality_loop_2

    complete_workspace
    solved_sample_count
    failure_index
    failure_reason

Use tuples rather than mutable lists in the final immutable result if practical.


ANGLE UNWRAPPING
================

`LinkagePose.q2_rad` and `q3_rad` are principal relative-angle differences.

Unwrap them continuously from extension toward flexion before computing motion
ratios.

The existing `scripts/animate_linkage.py` already contains simple continuity
logic for q2/q3 plotting.

Do not copy that code blindly into multiple places.

Implement one clear local helper in the evaluator and test branch-cut
continuity.


KINEMATIC FAILURE
=================

Evolutionary optimizers will intentionally sample invalid geometry.

An invalid candidate must not crash an optimization batch.

The linkage evaluator must convert failures such as:

    invalid FingerLinkage reference geometry
    degenerate reference branch
    KinematicClosureError during workspace sweep

into a structured incomplete `LinkageEvaluation`.

At minimum record:

    complete_workspace = False
    number of successfully solved samples
    failure index
    human-readable failure reason

Do not swallow the reason.

Do not change `LinkageKinematics` to make invalid geometry artificially valid.


TRANSMISSION ANGLE
==================

Compute transmission conditioning directly from the posed geometry.


Loop 1
------

At B', define the angle between:

    B' -> A
    B' -> O'

Call this:

    mu_1


Loop 2
------

At C', define the angle between:

    C' -> B
    C' -> A'

Call this:

    mu_2


Compute the full angle in [0, pi], preferably using a numerically robust form:

    atan2(abs(cross(u, v)), dot(u, v))

Define the singularity/transmission margin:

    margin(mu) = min(mu, pi - mu)

and transmission quality:

    Q(mu) = sin(margin(mu))

equivalently the normalized absolute cross-product magnitude.

Properties:

    90 deg -> Q = 1
    0 / 180 deg -> Q = 0

Do not use an arithmetic mean as the primary transmission objective.


MOTION RATIO
============

Use the positive input flexion coordinate:

    s = -q1

and positive relative output flexion:

    f2 = -unwrap(q2)
    f3 = -unwrap(q3)

Compute:

    r2 = df2 / ds
    r3 = df3 / ds

Use a stable finite-difference scheme with one-sided endpoint differences and
centered interior differences.

No new numerical dependency is required for this.


OBJECTIVE 1 — WORST-CASE TRANSMISSION QUALITY
=============================================

In:

    cmd/optimization/linkage/objectives.py

implement:

    WorstCaseTransmissionQuality

Sense:

    maximize

Definition:

    Q_trans =
        min over all workspace samples and both loops
        of transmission_quality

That is:

    Q_trans =
        min(
            Q(mu_1(s)),
            Q(mu_2(s))
        )

over the complete sweep.

If the workspace evaluation is incomplete, return the defined worst raw
quality:

    0.0

The kinematic-validity constraint will mark the candidate infeasible.


OBJECTIVE 2 — COORDINATION RATIO VARIATION
==========================================

Implement:

    CoordinationRatioVariation

Sense:

    minimize

Use:

    r2 = df2/ds
    r3 = df3/ds

Define:

    CV(r) = std(r) / (abs(mean(r)) + epsilon)

and:

    J_coord = CV(r2) + CV(r3)

Use a small numerical epsilon only to prevent division by zero.

This objective prefers approximately constant coupled flexion ratios without
requiring a manually invented target q2/q3 trajectory.

Do not compare against an anatomical target trajectory in this task.

If the full workspace is incomplete, return:

    +inf

or another explicitly documented worst-value representation compatible with
the generic problem API.


LINKAGE INEQUALITY CONSTRAINTS
==============================

Implement these as independent composable classes in:

    cmd/optimization/linkage/constraints.py


1. FullWorkspaceKinematicValidity
---------------------------------

Feasible only if every requested sweep sample was successfully solved on the
deterministic reference assembly branch.

A useful residual is based on solved fraction:

    g =
        1 - solved_sample_count / requested_sample_count

Therefore:

    g = 0
        complete workspace

    g > 0
        incomplete workspace

Use the generic convention:

    g <= 0


2. MinimumAnchorDorsalOffset
----------------------------

The design requirement is that, in the fully extended reference pose:

    A' is dorsal of B
    B' is dorsal of B

For configurable required offset `d_min`:

    A'.z - B.z >= d_min
    B'.z - B.z >= d_min

Return inequality residuals:

    d_min - (A'.z - B.z)
    d_min - (B'.z - B.z)

Do not hard-code a permanent physical value for `d_min`.

The caller/problem configuration chooses it.


3. MinimumTransmissionMargin
----------------------------

For configurable:

    mu_min

require every sample in both loops to satisfy:

    min(mu, pi - mu) >= mu_min

Return one residual per loop per workspace sample:

    mu_min - margin(mu)

This is intentionally vector-valued.

A starting study may use 30 degrees, but keep the value configurable.


4. MonotonicFlexion
-------------------

Make this reusable for q2 and q3.

For each requested output:

    r >= r_min

where normally:

    r_min = 0

Return:

    r_min - r

for every workspace sample.

Do not assume q2 or q3 themselves must be positive; use positive flexion
magnitudes from the evaluator.


5. MaximumMotionRatio
---------------------

Reusable for q2 and q3.

Require:

    r <= r_max

Return:

    r - r_max

for every sample.

A starting study may use:

    r_max = 3

but keep it configurable.


6. MinimumEndpointFlexion
-------------------------

Reusable for q2 and q3.

Require at maximum commanded flexion:

    f_joint(s_max) >= f_min

Return:

    f_min - f_joint(s_max)

Keep `f_min` configurable.

For an initial 0.8-rad input sweep, values corresponding to the previously
discussed screening ratios would be:

    q2 minimum:
        0.5 * 0.8 = 0.4 rad

    q3 minimum:
        0.25 * 0.8 = 0.2 rad

These are provisional screening settings, not canonical morphology constants.


EQUALITY CONSTRAINTS
====================

The generic framework must support equality constraints.

However, the initial linkage baseline problem should have:

    no equality constraints

because the important fully-extended relation:

    BC || OC

is enforced exactly by the optimization parameterization.

Do not add a redundant equality constraint for B_z = 0 or BC || OC.


BOX CONSTRAINTS
===============

Support box bounds through `BoxVariable`.

Do NOT invent canonical numeric linkage design-space bounds in this task.

The physical search bounds for:

    O'
    A
    A'
    B
    B'
    BC
    C'

have not yet been finalized.

The linkage baseline problem builder must therefore accept its variable bounds
from the caller.

Tests may use explicit test-only bounds that contain the current V0 seed.

Do not commit arbitrary 20–40 mm search ranges as source-of-truth design bounds.


BASELINE PROBLEM COMPOSITION
============================

In:

    cmd/optimization/linkage/baseline.py

provide a small convenience builder or configuration that assembles the
linkage problem from the reusable modules.

The intended composition is conceptually:

    problem = OptimizationProblem(
        variables=... caller supplied box variables ...,

        parameterization=LinkageBaselineParameterization(),

        evaluator=LinkageSweepEvaluator(
            LinkageSweepConfig(
                max_flexion_rad=0.8,
                sample_count=101,
            )
        ),

        objectives=(
            WorstCaseTransmissionQuality(),
            CoordinationRatioVariation(),
        ),

        equalities=(),

        inequalities=(
            FullWorkspaceKinematicValidity(),

            MinimumAnchorDorsalOffset(...),

            MinimumTransmissionMargin(...),

            MonotonicFlexion("q2", ...),
            MonotonicFlexion("q3", ...),

            MaximumMotionRatio("q2", ...),
            MaximumMotionRatio("q3", ...),

            MinimumEndpointFlexion("q2", ...),
            MinimumEndpointFlexion("q3", ...),
        ),
    )

The exact numeric inequality thresholds other than the existing 0.8-rad sweep
must be passed through a configuration object or builder arguments.

Do not hide them as unexplained module constants.


PROBLEM ASSEMBLY MUST REMAIN MODULAR
====================================

It must be possible to construct a different study without editing core code.

For example, a caller should be able to omit smoothness or change constraints:

    problem = OptimizationProblem(
        ...,
        objectives=(
            WorstCaseTransmissionQuality(),
        ),
        inequalities=(
            FullWorkspaceKinematicValidity(),
            MinimumTransmissionMargin(...),
        ),
    )

Likewise adding a future objective must not require changing
`OptimizationProblem`.


NO SOLVER IMPLEMENTATION YET
============================

Do not install or implement:

    MO-CMA-ES
    CMA-ES
    NSGA-II
    NSGA-III
    pymoo
    scipy.optimize

in this task.

The repository currently has no established optimization dependency or
dependency manifest for these solvers.

This task ends at a clean solver-independent problem representation.

A future adapter should be able to transform:

    objective minimization vector
    inequality residual vector
    equality residual vector
    box bounds

into whatever API MO-CMA-ES / pymoo / another solver requires.


PUBLIC API
==========

Expose the useful generic components from:

    cmd/optimization/__init__.py

Keep domain-specific linkage exports primarily under:

    cmd.optimization.linkage

Do not re-export every implementation detail at the package root.

Do not modify `cmd/__init__.py`.


TESTS
=====

Add focused tests, preferably:

    tests/test_optimization_problem.py
    tests/test_linkage_optimization.py


Generic framework tests
-----------------------

Verify:

1. BoxVariable rejects invalid bounds.

2. A vector is mapped to named variables deterministically.

3. Box violations affect `ProblemEvaluation.feasible`.

4. Equality convention is exactly:

       h = 0

   with configured tolerance.

5. Inequality convention is exactly:

       g <= 0

6. Vector-valued constraints are preserved and evaluated correctly.

7. Objective sense conversion produces:

       min objective -> raw
       max objective -> -raw

   in the minimization-form vector.

8. Objectives/constraints can be added or omitted without modifying the
   problem class.


Linkage parameterization tests
------------------------------

Verify:

1. Decoding always produces:

       B.z = 0
       C.z = 0
       BC along +Y
       BC || OC

2. Decoded `FingerLinkageParameters` use the canonical relative-coordinate
   representation correctly.

3. Encoding `BASELINE_LINKAGE_V0` and decoding it produces a geometry extremely
   close to V0, with only the already-known tiny projection onto exact B_z=0.

4. `BASELINE_LINKAGE_V0` itself is not modified.


Linkage evaluator tests
-----------------------

Verify:

1. Sweep uses:

       q1 = -s

2. Zero flexion reproduces the reference pose.

3. Current baseline can be evaluated over the existing intended flexion sweep.

4. q2 and q3 are unwrapped continuously.

5. positive flexion magnitudes are used.

6. finite-difference motion ratios have the expected sample count.

7. transmission angles are finite and in [0, pi].

8. transmission qualities are in [0, 1].

9. a deliberately invalid/unreachable candidate becomes a structured
   incomplete evaluation rather than crashing the batch.


Objective tests
---------------

Verify:

1. WorstCaseTransmissionQuality equals the minimum transmission quality across
   both loops and all samples.

2. CoordinationRatioVariation is non-negative.

3. incomplete workspace receives the documented worst objective values.


Constraint tests
----------------

Verify independently:

    FullWorkspaceKinematicValidity
    MinimumAnchorDorsalOffset
    MinimumTransmissionMargin
    MonotonicFlexion
    MaximumMotionRatio
    MinimumEndpointFlexion

For each, include one passing and one violating synthetic or real evaluation
where practical.


REGRESSION
==========

Do not break existing linkage behavior.

Run at least:

    pytest -q \
        tests/test_optimization_problem.py \
        tests/test_linkage_optimization.py \
        tests/test_linkage.py \
        tests/test_linkage_kinematics.py

Then run the full suite if the focused tests pass:

    pytest -q


IMPLEMENTATION RULES
====================

- Use current morphology and kinematics as the source of truth.
- Do not duplicate linkage closure equations in optimization code.
- Do not alter `FingerLinkageParameters`.
- Do not alter the definition of the fully extended pose.
- Do not change `BASELINE_LINKAGE_V0`.
- Do not reverse the q1 flexion sign convention.
- Do not add arbitrary optimizer bounds.
- Do not add a solver dependency.
- Do not move existing hand actuator packaging constraints.
- Do not use objectives as substitutes for hard feasibility requirements.
- Prefer parameterization over equality constraints when an exact relationship
  can be guaranteed structurally.
- Keep all optimization thresholds explicit and configurable.
- Keep diagnostic names stable and human-readable.
- Keep the generic optimization core free of NumPy if practical; the current
  task does not require adding a numerical dependency to the core.
- Do not introduce subagents for this task.


HARD STOPS
==========

Stop and report instead of guessing if:

- current `LinkageKinematics` cannot evaluate the existing 0.8-rad flexion
  sweep as expected by `scripts/animate_linkage.py`;
- the current q1/q2/q3 sign convention differs from the convention documented
  above;
- preserving deterministic assembly-branch selection would require changing
  morphology kinematics;
- a solver package must be added to implement the generic problem abstraction;
- physical box bounds are needed but are not present in the repository.

Do not invent missing physical bounds.


OUT OF SCOPE
============

Do not:

- run a large linkage optimization;
- choose a final `BASELINE_LINKAGE_V1`;
- add MO-CMA-ES yet;
- optimize fingertip morphology;
- optimize hand layout;
- couple this problem to contact mechanics;
- add actuator torque models;
- add force transmission models beyond geometric transmission-angle quality;
- add manufacturing constraints whose numeric limits have not been defined;
- modify URDF or hand morphology;
- rewrite the current linkage solver.


FINAL REPORT
============

Report:

1. New optimization package structure
2. Generic variable/objective/constraint/problem APIs
3. Exact linkage latent design vector
4. Linkage sweep/evaluation metrics
5. Implemented objectives
6. Implemented inequality constraints
7. How invalid candidates are handled
8. Focused and full test results
9. Objective/constraint values obtained when evaluating the projected
   `BASELINE_LINKAGE_V0` seed
10. Any unresolved issue

STOP after completing the task.