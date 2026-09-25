CODEX TASK
==========

Task:
Refactor the current CLEAR Stage-0 linkage optimization so workspace is no
longer represented only by a fixed full-sweep hard constraint.

Introduce a distinction between:

    core workspace
        the minimum flexion interval over which linkage quality requirements
        must be satisfied

    search horizon
        the larger flexion interval over which the optimizer measures how far
        the candidate remains usable

and add usable workspace as a third optimization objective.

The resulting Stage-0 multi-objective problem must be:

    maximize usable workspace

    maximize worst-case transmission quality over the core workspace

    minimize coordination-ratio variation over the core workspace

while keeping kinematic / transmission / motion-ratio requirements over the
core workspace as hard constraints.

Do NOT add NSGA-II or another solver in this task.
This task only changes the solver-independent optimization problem definition.


CURRENT REPOSITORY STATE
========================

Repository:

    dokkev/CMD-Framework

Current inspected commit:

    7189d03b1debf1c4fa21b57419eefdbb0c63ad00

The current implementation already contains:

    cmd/optimization/
        variable.py
        objective.py
        constraint.py
        problem.py

    cmd/optimization/linkage/
        parameterization.py
        evaluator.py
        objectives.py
        constraints.py
        baseline.py

and:

    tests/test_optimization_problem.py
    tests/test_linkage_optimization.py

Do not redesign the generic optimization layer unnecessarily.

Preserve:

    BoxVariable
    Objective
    EqualityConstraint
    InequalityConstraint
    OptimizationProblem
    ProblemEvaluation

and the existing 12-dimensional linkage optimizer parameterization.


MOTIVATION
==========

The current evaluator treats:

    q1 = -0.8 rad

as the end of the entire required workspace.

This creates an undesirable artifact.

For example:

    candidate A:
        excellent behavior until 0.78 rad
        severe transmission degradation near 0.80 rad

    candidate B:
        moderate quality
        remains usable to 1.00 rad

A fixed 0.8-rad full-workspace formulation does not represent this tradeoff
well.

Stage 0 should instead expose:

    how good the linkage is over a required core workspace

and separately:

    how far that useful behavior extends.


STAGE-0 FORMULATION
===================

Use positive input-flexion coordinate:

    s = -q1

with:

    s >= 0

Define:

    W_core
        required/core workspace

    W_search
        search horizon

with:

    0 < W_core < W_search

Both must be configurable.

Do NOT make either value a canonical morphology constant.

Do NOT invent a permanent 0.8-rad or 1.2-rad physical joint limit.

The caller chooses these values for each optimization study.


OBJECTIVES
==========

The baseline linkage problem now has THREE objectives.


Objective 1
-----------

    maximize usable workspace

Define:

    W_usable

as the largest contiguous interval:

    [0, W_usable]

starting from the fully extended pose over which all pointwise workspace
health criteria remain satisfied.

The raw objective value must be expressed in radians.

Do not normalize it to [0, 1] inside the domain objective.


Objective 2
-----------

    maximize worst-case transmission quality

Evaluate this only over:

    [0, W_core]

not over the entire search horizon.


Objective 3
-----------

    minimize coordination-ratio variation

Evaluate this only over:

    [0, W_core]

not over the entire search horizon.


WHY CORE QUALITY AND WORKSPACE ARE SEPARATE
===========================================

Do not evaluate transmission quality over the entire usable workspace.

That would strongly couple:

    workspace extent

and:

    worst-case transmission quality

because the usable-workspace boundary may itself be defined by the
transmission threshold.

Instead:

    quality objectives
        evaluate a fixed comparable core interval

    workspace objective
        measures how much farther the candidate remains usable

This allows meaningful Pareto tradeoffs.


SEARCH HORIZON
==============

Refactor the current:

    LinkageSweepConfig.max_flexion_rad

concept into an explicitly named search-horizon concept.

Prefer:

    search_horizon_rad

The sweep still uses:

    q1 = -s

for:

    s in [0, search_horizon_rad]

The current historical 0.8-rad default may be retained only if needed for
backward compatibility, but it must not be documented as a physical limit.

The baseline optimization builder must explicitly distinguish:

    core_workspace_rad
    search_horizon_rad


CORE SAMPLE MUST BE EXACT
=========================

Do not silently evaluate the core workspace at the nearest sampled point.

The exact value:

    s = core_workspace_rad

must be evaluated.

If the uniform search grid does not contain the core endpoint exactly, insert
the core endpoint into the sampled flexion coordinates and keep the final
sample sequence sorted and unique.

Therefore:

    sample_count

may describe the nominal uniform grid size, while the actual requested sample
count may contain one additional core point.

Document this behavior clearly.


NEW LINKAGE WORKSPACE MODULE
============================

Add:

    cmd/optimization/linkage/workspace.py

This module owns the definition of "usable workspace".

Do not place these semantics inside the generic optimization package.


WORKSPACE CRITERIA
==================

Add an immutable configuration type conceptually equivalent to:

    LinkageWorkspaceCriteria

containing:

    minimum_transmission_margin_rad

    q2_minimum_motion_ratio
    q3_minimum_motion_ratio

    q2_maximum_motion_ratio
    q3_maximum_motion_ratio

These thresholds define whether one sampled pose is mechanically usable.

Validation must ensure all values are finite and physically meaningful.

The same criteria object must be reused by:

    usable-workspace evaluation

and:

    core-workspace hard constraints

Do not duplicate the same threshold values in two unrelated configuration
objects.


USABLE WORKSPACE DEFINITION
===========================

A sample is usable only if:

1. canonical linkage kinematics successfully solved that sample;

2. both transmission-angle margins satisfy:

       margin_1 >= minimum_transmission_margin_rad
       margin_2 >= minimum_transmission_margin_rad

3. q2 motion ratio satisfies:

       q2_minimum_motion_ratio
       <= dq2_flexion / ds
       <= q2_maximum_motion_ratio

4. q3 motion ratio satisfies:

       q3_minimum_motion_ratio
       <= dq3_flexion / ds
       <= q3_maximum_motion_ratio


The usable workspace is the longest CONTIGUOUS usable prefix beginning at:

    s = 0

Do not count disconnected usable regions after the first unusable point.


For sampled coordinates:

    s_0 = 0
    s_1
    ...
    s_k
    s_{k+1}

if samples through s_k are usable and s_{k+1} is not:

    W_usable = s_k

This is intentionally a sampled approximation.

Do not claim continuous exactness.


CLOSURE WORKSPACE
=================

Also compute a separate diagnostic:

    W_closure

defined only by successful deterministic linkage kinematics.

This allows results such as:

    closure workspace = 1.05 rad
    usable workspace  = 0.79 rad

    usable limit:
        transmission margin

to be distinguished.

Do not merge closure workspace and usable workspace.


WORKSPACE DIAGNOSTICS
=====================

Add an immutable structured result such as:

    LinkageWorkspaceEvaluation

containing at least:

    closure_workspace_rad
    usable_workspace_rad

    closure_sample_count
    usable_sample_count

    first_unusable_index

    limiting_reasons

`limiting_reasons` should identify why the first unusable sample failed.

Supported reasons should distinguish at least:

    kinematic_failure

    transmission_loop_1
    transmission_loop_2

    q2_minimum_motion_ratio
    q3_minimum_motion_ratio

    q2_maximum_motion_ratio
    q3_maximum_motion_ratio

    search_horizon

If several criteria fail at the same first unusable sample, preserve all
relevant reasons in deterministic order.

If the candidate remains usable through the entire search horizon:

    usable_workspace_rad = search_horizon_rad

and:

    limiting_reasons = ("search_horizon",)

or an equivalently explicit representation.


WORKSPACE OBJECTIVE
===================

In:

    cmd/optimization/linkage/objectives.py

add:

    UsableWorkspace

or an equivalently clear class name.

Sense:

    max

Its raw value is:

    LinkageWorkspaceEvaluation.usable_workspace_rad

The class may evaluate workspace through the reusable helper in
`workspace.py`.

Do not duplicate workspace logic inside the objective.


CORE-WORKSPACE SLICING
======================

Add one shared helper for selecting evaluation samples within:

    [0, core_workspace_rad]

Do not duplicate index-selection logic across every objective and constraint.

The helper must include the exact core endpoint.

Use a small numerical tolerance only for floating-point comparison.


TRANSMISSION OBJECTIVE UPDATE
=============================

Update:

    WorstCaseTransmissionQuality

so it accepts/configures:

    core_workspace_rad

and computes:

    min transmission quality

across both loops only for samples in:

    [0, core_workspace_rad]

Do not include samples beyond the core workspace.


COORDINATION OBJECTIVE UPDATE
=============================

Update:

    CoordinationRatioVariation

so it evaluates:

    dq2_flexion / ds
    dq3_flexion / ds

only over the core workspace.

Do not let poor behavior beyond the required core interval directly change the
coordination-quality objective.

Behavior beyond the core interval belongs to the usable-workspace objective.


CORE KINEMATIC CONSTRAINT
=========================

The current:

    FullWorkspaceKinematicValidity

must no longer mean:

    every sample through search_horizon must solve.

Replace or generalize it into something conceptually equivalent to:

    CoreWorkspaceKinematicValidity(core_workspace_rad)

It is feasible if deterministic linkage kinematics successfully solves every
required sample through:

    core_workspace_rad

A candidate may fail after the core workspace and still be feasible.

That failure should reduce:

    UsableWorkspace

instead of automatically making the candidate infeasible.


EXAMPLE
=======

Suppose:

    core workspace = 0.60 rad
    search horizon = 1.20 rad

Candidate A:

    usable until 0.78 rad

Candidate B:

    usable until 1.02 rad

If both satisfy every hard constraint through 0.60 rad:

    both are feasible

but:

    workspace objective A = 0.78
    workspace objective B = 1.02

Their core transmission and coordination scores may still trade off.

This behavior is REQUIRED.


CORE TRANSMISSION CONSTRAINT
============================

Update:

    MinimumTransmissionMargin

so it can be evaluated over a specified workspace endpoint.

For the baseline problem, apply it only over:

    [0, core_workspace_rad]

The same:

    minimum_transmission_margin_rad

used by `LinkageWorkspaceCriteria` must be passed here.

Do not enforce this hard constraint through the full search horizon.


CORE MOTION-RATIO CONSTRAINTS
=============================

Likewise update:

    MonotonicFlexion
    MaximumMotionRatio

so the baseline problem applies them only through:

    core_workspace_rad

The same q2/q3 minimum and maximum ratios used by
`LinkageWorkspaceCriteria` must be reused.

Do not maintain duplicated independent values.


ENDPOINT FLEXION
================

The existing:

    MinimumEndpointFlexion

must now refer to the CORE endpoint, not the search-horizon endpoint.

Prefer making the requested input-flexion location explicit in the API.

For example:

    MinimumFlexionAtInput(
        joint="q2",
        input_flexion_rad=core_workspace_rad,
        minimum_flexion_rad=...
    )

or generalize the existing class equivalently.

Evaluate exactly at:

    s = core_workspace_rad

Do not use the last search-horizon sample.


ANCHOR CONSTRAINT
=================

Keep:

    MinimumAnchorDorsalOffset

unchanged.

It is a reference-geometry constraint and is independent of workspace extent.


INCOMPLETE SWEEP RESIDUALS
==========================

Fix the current use of:

    math.inf

for missing samplewise constraint residuals.

This is important before connecting the problem to NSGA-II.

Currently an incomplete candidate may produce infinite residuals for:

    MinimumTransmissionMargin
    MonotonicFlexion
    MaximumMotionRatio

This destroys useful graded feasibility information.

For samplewise constraints:

    CoreWorkspaceKinematicValidity

must own the kinematic-workspace infeasibility.

Other samplewise constraints should evaluate all samples that actually exist
inside the core interval.

If a core sample is unavailable because kinematics already failed:

    do not add +inf

Pad missing fixed-size residual entries with a finite neutral residual:

    0.0

or use another clearly documented finite neutral representation.

The candidate remains infeasible because:

    CoreWorkspaceKinematicValidity > 0

This preserves graded information such as:

    candidate A solves 95% of the required core workspace
    candidate B solves 20%

rather than making both candidates effectively have infinite violation.

Do NOT use NaN or infinity in inequality residual vectors.


OBJECTIVE FAILURE VALUES
========================

Do not introduce NaN objective values.

For incomplete CORE workspace:

    WorstCaseTransmissionQuality
        return 0.0

For:

    CoordinationRatioVariation

retain the existing documented failure behavior only if the generic optimizer
stack can safely accept it.

Prefer a finite, explicitly documented worst-value representation before a
solver adapter is added.

Do not silently introduce arbitrary physical meaning to the penalty.

If changing this cleanly requires solver-adapter policy, leave the current
behavior and report it as an unresolved issue rather than inventing a hidden
penalty.


BASELINE CONFIGURATION
======================

Refactor:

    LinkageBaselineConstraintConfig

so the baseline builder has one source of truth for workspace thresholds.

A clean conceptual arrangement is:

    LinkageWorkspaceCriteria(
        minimum_transmission_margin_rad=...,
        q2_minimum_motion_ratio=...,
        q3_minimum_motion_ratio=...,
        q2_maximum_motion_ratio=...,
        q3_maximum_motion_ratio=...,
    )

and:

    LinkageBaselineConstraintConfig(
        core_workspace_rad=...,
        minimum_anchor_dorsal_offset_mm=...,
        q2_minimum_endpoint_flexion_rad=...,
        q3_minimum_endpoint_flexion_rad=...,
        workspace_criteria=...,
    )

The exact dataclass arrangement may differ slightly if a cleaner API results,
but:

    workspace health thresholds must have one canonical owner.


BASELINE PROBLEM
================

The composed problem should now be conceptually:

    OptimizationProblem(
        variables=...,

        parameterization=LinkageBaselineParameterization(),

        evaluator=LinkageSweepEvaluator(
            LinkageSweepConfig(
                search_horizon_rad=...,
                sample_count=...,
                core_workspace_rad=... or exact required sample equivalent,
            )
        ),

        objectives=(
            UsableWorkspace(...),
            WorstCaseTransmissionQuality(core_workspace_rad=...),
            CoordinationRatioVariation(core_workspace_rad=...),
        ),

        equalities=(),

        inequalities=(
            CoreWorkspaceKinematicValidity(core_workspace_rad=...),

            MinimumAnchorDorsalOffset(...),

            MinimumTransmissionMargin(
                ...,
                workspace_rad=core_workspace_rad,
            ),

            MonotonicFlexion(
                "q2",
                ...,
                workspace_rad=core_workspace_rad,
            ),

            MonotonicFlexion(
                "q3",
                ...,
                workspace_rad=core_workspace_rad,
            ),

            MaximumMotionRatio(
                "q2",
                ...,
                workspace_rad=core_workspace_rad,
            ),

            MaximumMotionRatio(
                "q3",
                ...,
                workspace_rad=core_workspace_rad,
            ),

            MinimumFlexionAtInput(
                "q2",
                input_flexion_rad=core_workspace_rad,
                ...
            ),

            MinimumFlexionAtInput(
                "q3",
                input_flexion_rad=core_workspace_rad,
                ...
            ),
        ),
    )


OBJECTIVE ORDER
===============

Use deterministic objective order:

    1. usable_workspace
    2. worst_case_transmission_quality
    3. coordination_ratio_variation

Raw senses:

    usable_workspace:
        max

    worst_case_transmission_quality:
        max

    coordination_ratio_variation:
        min

Therefore minimization form will conceptually be:

    [
        -W_usable,
        -Q_trans,
        J_coord,
    ]


NO FIXED PHYSICAL NUMBERS
=========================

Do not introduce canonical values for:

    core_workspace_rad
    search_horizon_rad

in morphology source-of-truth.

These are optimization-study settings.

Tests may use convenient values such as:

    core = 0.4
    horizon = 0.8

or similar test-only values.

Do not interpret those as design requirements.


TESTS
=====

Update:

    tests/test_linkage_optimization.py

and add focused tests for the new semantics.


Required tests
--------------

1. Search horizon vs core workspace

Verify:

    core_workspace_rad < search_horizon_rad

is accepted.

Reject:

    core <= 0
    search <= core
    non-finite values.


2. Exact core sample

Choose core and horizon values where the uniform grid would NOT naturally
contain the core endpoint.

Verify the evaluator includes exactly:

    core_workspace_rad

in the requested sweep.


3. Closure workspace

Construct or synthesize an evaluation where kinematics fails after a known
sample.

Verify:

    closure_workspace_rad

equals the final successfully solved sample.


4. Usable workspace limited by transmission

Create an evaluation where:

    closure continues farther

but transmission margin first fails at a known sample.

Verify:

    closure_workspace_rad > usable_workspace_rad

and:

    limiting_reasons

contains the correct transmission loop.


5. Usable workspace limited by motion ratio

Test minimum and maximum q2/q3 motion-ratio failures independently.


6. Search-horizon limited

For a candidate satisfying all criteria through the complete sweep:

    usable_workspace_rad == search_horizon_rad

and the limiting diagnostic identifies the horizon.


7. Core-feasible but post-core failure

This is critical.

Construct a candidate/evaluation such that:

    all requirements pass through W_core

but:

    transmission or kinematics fails after W_core

Verify:

    ProblemEvaluation.feasible is True

and:

    UsableWorkspace < search_horizon

This is the central behavior change.


8. Failure before core

Construct a candidate/evaluation that becomes unusable before:

    W_core

Verify:

    ProblemEvaluation.feasible is False.


9. Transmission objective slicing

Give an evaluation where transmission is excellent over the core workspace
but terrible after the core.

Verify:

    WorstCaseTransmissionQuality

uses only core samples.


10. Coordination objective slicing

Likewise verify post-core motion-ratio behavior does not change:

    CoordinationRatioVariation


11. Endpoint flexion location

Verify q2/q3 minimum-flexion requirements are checked at:

    core_workspace_rad

not:

    search_horizon_rad.


12. No infinite inequality residuals

For an incomplete kinematic sweep, verify every returned inequality residual is:

    finite

and:

    not NaN

The core kinematic-validity constraint must still report positive violation.


13. Three-objective composition

Verify the baseline problem objective names/order are exactly:

    usable_workspace
    worst_case_transmission_quality
    coordination_ratio_variation


14. Existing parameterization regression

Retain all current tests ensuring:

    12-dimensional latent linkage parameterization
    exact B.z = 0
    BC || OC
    BASELINE_LINKAGE_V0 is not mutated
    q1 = -s
    deterministic reference branch


REGRESSION
==========

Run focused tests:

    pytest -q \
        tests/test_optimization_problem.py \
        tests/test_linkage_optimization.py \
        tests/test_linkage.py \
        tests/test_linkage_kinematics.py

Then run:

    pytest -q

Do not claim test success unless actually executed.


IMPLEMENTATION RULES
====================

- Keep Stage 0 entirely analytic.
- Use the canonical `LinkageKinematics`.
- Do not duplicate closed-loop equations.
- Do not change `FingerLinkageParameters`.
- Do not change `BASELINE_LINKAGE_V0`.
- Keep q1 < 0 as physical flexion.
- Use positive s = -q1 for optimization metrics.
- Keep optimizer-specific logic out of morphology.
- Keep generic optimization code domain-neutral.
- Keep linkage workspace semantics inside `cmd/optimization/linkage`.
- Reuse one workspace-criteria configuration everywhere.
- Do not use disconnected valid regions after the first workspace failure.
- Do not add adaptive or binary-refined workspace boundaries yet.
- The workspace value is grid-based in this task.
- Dense/adaptive Pareto-candidate validation can be added later.
- Do not add NSGA-II yet.
- Do not add pymoo yet.
- Do not use subagents.


HARD STOPS
==========

Stop and report instead of guessing if:

- the existing linkage kinematics cannot distinguish post-core failure from
  core failure;

- changing the sweep semantics requires modifying canonical morphology;

- exact evaluation of the core endpoint cannot be added without duplicating
  linkage kinematics;

- a physical core workspace or search horizon must be chosen but is not
  explicitly present in the repository;

- a non-finite penalty is required by a future solver API that has not yet
  been implemented.

Do not invent missing physical requirements.


OUT OF SCOPE
============

Do not:

- add NSGA-II;
- add pymoo;
- run the final baseline optimization;
- choose `BASELINE_LINKAGE_V1`;
- optimize hand layout;
- optimize fingertip geometry;
- implement Stage 1;
- implement Stage 2;
- add adaptive workspace refinement;
- add surrogate optimization;
- change the morphology source-of-truth geometry.


FINAL REPORT
============

Report only:

1. Updated Stage-0 formulation
2. Core-workspace vs search-horizon API
3. Usable-workspace definition
4. Workspace diagnostics
5. Three objective definitions
6. Which hard constraints now apply only to the core workspace
7. How incomplete sweeps avoid infinite constraint residuals
8. Focused test results
9. Full test result
10. Any unresolved issue

STOP after completing the task.