CODEX TASK
==========

Task:
Finish the current CLEAR Stage-0 linkage optimization implementation by
connecting the existing solver-independent CMD optimization problem to pymoo,
adding a reusable NSGA-II execution layer, and providing an end-to-end Stage-0
runner/result-export path.

Do NOT redesign the linkage optimization formulation in this task.

Do NOT invent physical linkage design bounds.

Do NOT change the Stage-0 scientific objectives or workspace semantics unless
required to fix a concrete implementation bug found while integrating pymoo.

Do NOT use subagents.


CURRENT REPOSITORY STATE
========================

Repository:

    dokkev/CMD-Framework

Inspected HEAD:

    904f9af6da84434d2575baac51dd06c43946abae

Before editing, verify that the working HEAD is compatible with this inspected
state.

If HEAD has advanced, inspect the diff first.

If the optimization architecture or linkage formulation has materially changed,
stop and report the mismatch instead of blindly applying this instruction.


CURRENT CANONICAL LINKAGE CONVENTION
====================================

The XZ coordinate refactor is already complete.

Canonical hand/linkage coordinates are:

    +X = finger longitudinal / distal
    +Y = ordinary finger FE axis
    +Z = dorsal

The linkage operates in:

    XZ plane

with flexion axis:

    +Y

Current canonical planar point type:

    PointXZ(x_mm, z_mm)

Current ordinary hand/linkage reference alignment:

    ORDINARY_MCP_FE_TO_LINKAGE = identity

Do NOT reintroduce:

    PointYZ
    +Y distal
    +X flexion axis
    Rz(-pi/2) ordinary linkage mounting

The current optimization latent variables already use `_x_mm` names.


CURRENT STAGE-0 FORMULATION
===========================

The current implementation already has the desired scientific formulation.

Do not replace it.

The optimizer coordinates are the existing 12-dimensional
`LinkageBaselineParameterization`.

The positive optimization flexion coordinate is:

    s = -q1

so:

    q1 = 0
        fully extended

    q1 < 0
        flexion


Stage-0 uses:

    core_input_workspace_rad

for fixed, fair quality comparison between candidates, and:

    search_horizon_rad

for evaluating how far the mechanism remains usable.


Current objectives, in deterministic order:

    1. worst_case_transmission_quality
       sense = max

    2. coordination_ratio_variation
       sense = min

    3. distal_endpoint_excursion_mm
       sense = max

The distal endpoint is linkage point C.

It is a Stage-0 distal-link proxy.

Do NOT call it an actual fingertip trajectory.


Current usable input workspace is:

    a diagnostic
    +
    a minimum hard constraint

It is NOT a Pareto objective.


CURRENT OPTIMIZATION ARCHITECTURE
=================================

The generic solver-independent layer already exists:

    cmd/optimization/
        variable.py
        objective.py
        constraint.py
        problem.py

The linkage-specific problem already exists:

    cmd/optimization/linkage/
        parameterization.py
        evaluator.py
        workspace.py
        objectives.py
        constraints.py
        baseline.py

`OptimizationProblem.evaluate(x)` already produces:

    ProblemEvaluation

containing:

    design_vector
    variable_values
    decoded_parameters
    domain_evaluation
    raw_objective_values
    minimization_objective_values
    box_violations
    equality_results
    inequality_results
    feasible

Keep this solver-independent API canonical.

Do NOT add pymoo concepts to:

    cmd/morphology

or to the generic domain-facing objective/constraint definitions.


KNOWN INTEGRATION ISSUES
========================

Fix these as part of this task.


1. Non-finite coordination objective
------------------------------------

`CoordinationRatioVariation` currently returns:

    +inf

when the core workspace is incomplete.

This is intentional domain-level "worst" behavior but should not be passed
directly into pymoo.

Do NOT silently change its physical/domain semantics merely to satisfy pymoo.

The pymoo adapter must convert non-finite minimization-form objective values
to a configurable finite solver transport penalty.

Use an explicit adapter setting such as:

    nonfinite_objective_penalty

with a conservative finite default.

The penalty is NOT a physical metric.

Raw CMD evaluation must remain inspectable independently from the solver
transport value.


2. `bc_length_mm` decode safety
-------------------------------

`FingerLinkageParameters` requires:

    bc_length_mm > 0

but box variables are currently caller-provided.

If a production search bound allows:

    bc_length_mm <= 0

the parameterization can raise during `decode()` before the linkage evaluator
can convert the candidate into an incomplete structured evaluation.

Prevent this at problem/study construction time.

For a Stage-0 linkage problem, validate that the `bc_length_mm` BoxVariable
has:

    lower > 0

Do not invent a numeric lower bound.

Simply reject incompatible caller-supplied bounds with a clear error.


3. Constraint residual dimensions
---------------------------------

pymoo requires fixed objective/constraint dimensions.

The current linkage constraints are intentionally fixed-size for one configured
problem, but the generic CMD protocols do not expose static residual lengths.

The adapter must establish expected dimensions once from a caller-supplied
prototype design vector and assert that all subsequent evaluations preserve
those dimensions.

Do not infer dimensions independently for every candidate.


PYMOO DEPENDENCY
================

The user has installed pymoo in the development environment.

Before implementing against it:

    inspect the actually installed pymoo version

and verify the local API.

Do not assume an old pymoo version from memory.

Use the installed version's supported:

    Problem / ElementwiseProblem
    NSGA2
    minimize
    LHS or equivalent sampling APIs

as appropriate.

Do not add scipy.optimize or another optimization framework.

There is currently no established repository-wide dependency manifest.

Do NOT invent a new packaging/dependency-management system in this task.

If pymoo is missing from the active environment despite the user's installation,
stop and report the environment mismatch.


ADD SOLVER PACKAGE
==================

Add:

    cmd/optimization/solvers/
        __init__.py
        pymoo.py

This package is solver-specific.

The generic optimization core must remain solver-independent.


PYMOO PROBLEM ADAPTER
=====================

Implement a reusable adapter conceptually equivalent to:

    PymooProblemAdapter

The exact class/API may follow the installed pymoo version cleanly.

It must adapt:

    OptimizationProblem

to pymoo without changing the original problem.


Variable mapping
----------------

Map CMD variables in declared order:

    BoxVariable.lower -> xl
    BoxVariable.upper -> xu

Set:

    n_var = len(problem.variables)

Do NOT duplicate box bounds as G constraints.


Objective mapping
-----------------

Call:

    problem.evaluate(x)

exactly once per candidate.

Use:

    ProblemEvaluation.minimization_objective_values

in declared objective order for:

    out["F"]

For the current Stage-0 linkage problem this must produce conceptually:

    F = [
        -worst_case_transmission_quality,
         coordination_ratio_variation,
        -distal_endpoint_excursion_mm,
    ]

Do not manually reimplement objective signs in the linkage runner.


Non-finite objective transport
------------------------------

Before writing F to pymoo:

    every F value must be finite

If the CMD minimization-form value is non-finite:

    replace it with the configured finite
    nonfinite_objective_penalty

Do not modify the original `ProblemEvaluation`.

Do not replace ordinary large but finite values.


Inequality mapping
------------------

CMD inequality convention is already:

    g(x) <= 0

Flatten all inequality `ConstraintResult.residuals` in:

    declared constraint order
    then residual order

into pymoo G.

Preserve deterministic ordering.


Equality mapping
----------------

The current linkage baseline has no equalities, but the generic adapter should
correctly support the generic CMD equality API.

CMD equality semantics are:

    |h_i| <= tolerance

Convert each equality residual into a pymoo-compatible inequality residual:

    abs(h_i) - tolerance <= 0

and append it to G after ordinary inequalities.

This preserves CMD equality tolerance semantics without relying on pymoo's
separate equality tolerance behavior.

Do not expose the same equality a second time through H.


Constraint finiteness
---------------------

All values passed to pymoo G must be finite.

The current linkage implementation is designed to produce finite constraint
residuals even for incomplete core sweeps.

If a non-finite constraint residual nevertheless appears:

    treat it as an implementation error

and raise a clear exception including:

    constraint name
    residual index
    design vector

Do NOT silently hide an unexpected non-finite G behind a penalty in this task.


PROTOTYPE / SHAPE CONTRACT
==========================

The adapter constructor must accept a prototype design vector.

Evaluate it once and determine:

    n_obj
    number of flattened ordinary inequality residuals
    number of equality-derived G residuals
    total n_ieq_constr

Also record expected residual length per named constraint.

Every later candidate evaluation must produce the same residual shape.

If not:

    raise a clear deterministic error

identifying the offending constraint.

The prototype does NOT need to be feasible.

It only needs to be decodable/evaluable and dimensionally representative.


ADAPTER DIAGNOSTICS
===================

Provide a way to access the most recent or explicitly requested
`ProblemEvaluation` without reconstructing solver-specific physics.

Do not make the adapter cache unbounded evaluation history by default.

Avoid retaining every domain evaluation during a long NSGA-II run unless
explicitly requested.


NSGA-II RUN CONFIG
==================

Add a small immutable solver configuration, either in `pymoo.py` or a similarly
small solver-specific module.

Conceptually:

    NSGA2Config(
        population_size,
        generations,
        seed,
        eliminate_duplicates=True,
        nonfinite_objective_penalty=...
    )

Validate:

    population_size >= 2
    generations >= 1
    seed is an integer

Do not put linkage-specific parameters into this config.


NSGA-II RUNNER
==============

Provide a reusable function conceptually equivalent to:

    run_nsga2(
        problem,
        prototype_x,
        config,
        initial_designs=(),
    )

It should:

    construct PymooProblemAdapter
    construct NSGA2
    run pymoo.minimize
    return a structured CMD-friendly run result

Do not make linkage code import NSGA2 directly if the generic solver wrapper
can own it.


INITIAL POPULATION
==================

Support caller-provided initial designs.

For Stage 0 we want to be able to include:

    projected BASELINE_LINKAGE_V0

as one initial design.

Fill the remainder of the initial population using Latin Hypercube Sampling
over the full box bounds if supported cleanly by the installed pymoo API.

Preferred Stage-0 initialization:

    1 known baseline seed
    +
    globally distributed LHS samples

Do NOT generate the entire population as local perturbations around V0.

Do NOT clip an out-of-bounds supplied initial design.

If an initial design is outside the declared box bounds:

    raise a clear error

or require the caller to omit it.

Use the installed pymoo API's supported initialization mechanism rather than
depending on undocumented internals.


DUPLICATES
==========

Enable duplicate elimination by default if supported by the installed pymoo
version.

Do not write a custom duplicate-elimination algorithm unless pymoo requires it.


RUN RESULT
==========

Do not force downstream code to work directly with signed pymoo F values.

Create a small structured result representation that retains at least:

    pymoo result or minimal solver metadata
    Pareto design vectors X
    final-population design vectors
    solver config
    evaluation count if available

Then re-evaluate selected designs through the original:

    OptimizationProblem.evaluate(x)

when producing user-facing results.

The user-facing interpretation must use:

    raw_objective_values

not signed pymoo F.


NO-FEASIBLE-SOLUTION CASE
=========================

Handle the case where NSGA-II finds no feasible Pareto set.

Do NOT assume:

    result.X

always exists.

If no feasible nondominated set exists:

    return an empty Pareto candidate set
    preserve the final population
    preserve solver completion metadata

The Stage-0 export path must still produce useful diagnostics for the final
population.

Do not crash merely because all candidates are infeasible.


LINKAGE STAGE-0 STUDY BOUNDS
============================

There are currently NO production linkage design bounds in the repository.

The existing test code uses V0 +/- 1.0 only as a test fixture.

That is NOT an accepted physical design space.

Do NOT turn those test bounds into production defaults.

Do NOT invent ranges for:

    O'
    A
    A'
    B
    B'
    BC
    C'

Instead, make production Stage-0 execution require caller-supplied
`BoxVariable` bounds.

The software should be complete and runnable once those bounds are supplied.


LINKAGE STAGE-0 RUNNER
======================

Add a thin linkage-specific execution layer.

Prefer one of:

    cmd/optimization/linkage/stage0.py

or another comparably clear module.

It should compose EXISTING components rather than duplicate them:

    LinkageBaselineParameterization
    LinkageBaselineConstraintConfig
    LinkageSweepConfig
    build_linkage_baseline_problem
    BASELINE_LINKAGE_V0
    run_nsga2

Provide a function conceptually equivalent to:

    run_linkage_stage0(
        variables,
        constraint_config,
        sweep_config,
        solver_config,
        include_baseline_seed=True,
    )

Behavior:

    build the current Stage-0 problem
    encode/project BASELINE_LINKAGE_V0
    optionally include it in the initial population
    run NSGA-II
    return structured results

If the projected V0 is outside caller-supplied bounds and
`include_baseline_seed=True`:

    raise a clear error

Do not silently clip it.


RESULT RE-EVALUATION
====================

For every Pareto design vector returned by pymoo:

    call the original CMD problem.evaluate(x)

and retain the complete `ProblemEvaluation`.

This is the authoritative candidate interpretation.

Do not reconstruct linkage metrics manually from `res.F`.


RESULT EXPORT
=============

Provide a small standard-library export helper for Stage 0.

Do not introduce pandas merely for CSV export.

Given an output directory, write at least:

    run.json
    pareto.csv
    final_population.csv

If no feasible Pareto set exists:

    pareto.csv

should still be created with headers and zero candidate rows.


run.json
--------

Include at least:

    solver = "pymoo_nsga2"
    installed pymoo version
    population size
    generations
    seed

    search_horizon_rad
    sample_count
    core_input_workspace_rad
    minimum_required_input_workspace_rad

    variable names and bounds
    objective names and senses
    constraint names

    number of Pareto candidates
    number of final-population candidates
    number of feasible final-population candidates

Do not claim actual physical meaning for caller-selected bounds.


pareto.csv
----------

Each Pareto row should contain:

    candidate_id

    all 12 design-variable values by canonical name

    raw objective values:
        worst_case_transmission_quality
        coordination_ratio_variation
        distal_endpoint_excursion_mm

    workspace diagnostics:
        closure_input_workspace_rad
        usable_input_workspace_rad
        distal_path_length_mm
        distal_x_excursion_mm
        dorsal_z_excursion_mm
        first_unusable_index
        limiting_reasons

    feasibility diagnostics:
        feasible
        overall_max_constraint_violation
        overall_sum_positive_constraint_violation

    one maximum-violation column per named constraint

Use deterministic column ordering.

Serialize multiple limiting reasons with a simple stable delimiter such as:

    ;

Do not serialize Python repr objects into CSV cells unnecessarily.


final_population.csv
--------------------

Use the same useful columns where practical.

This file is important when no feasible Pareto set exists because it shows how
close the final population came to feasibility.


CONSTRAINT VIOLATION SUMMARY
============================

For a `ProblemEvaluation`, define user-facing diagnostics:

    overall_max_constraint_violation

as the maximum of:

    box violations
    equality maximum violations
    inequality maximum violations

and:

    overall_sum_positive_constraint_violation

as a deterministic sum of positive violation magnitudes.

For equality results, use their existing tolerance-aware violation amount.

This diagnostic is for reporting.

Do not replace pymoo's internal feasibility logic with a custom selection
algorithm.


OPTIONAL CLI
============

If a clean CLI can be added WITHOUT inventing physical bounds, add:

    scripts/optimize_linkage_stage0.py

The CLI must require the design bounds from the caller.

A standard-library JSON study configuration is acceptable.

Do not add YAML or another configuration dependency.

If implementing the CLI would substantially expand this task, prioritize the
reusable Python API and export helpers instead.

Do not embed arbitrary physical bounds inside the script.


TESTS: GENERIC PYMOO ADAPTER
============================

Add focused tests, e.g.:

    tests/test_pymoo_adapter.py

Use small synthetic deterministic problems where appropriate.

Required coverage:


1. Box bounds mapping

Verify:

    n_var
    xl
    xu

match declared CMD variables in order.


2. Objective sign mapping

Create one min and one max objective.

Verify pymoo F uses:

    raw for min
    -raw for max


3. Objective order

Verify declared CMD objective order is preserved exactly.


4. Inequality flattening

Use multiple vector-valued inequality constraints.

Verify G order is:

    constraint declaration order
    then residual order.


5. Equality tolerance conversion

For an equality:

    h = residual
    tolerance = tol

verify G contains:

    abs(h) - tol


6. Fixed residual shape

Infer shape from a prototype.

Then create a candidate producing a different residual length.

Verify the adapter rejects it clearly.


7. Non-finite objective transport

Return:

    +inf

from a synthetic objective.

Verify pymoo F receives the configured finite penalty while the original CMD
evaluation still contains +inf.


8. Non-finite constraint rejection

Return NaN or infinity from a synthetic constraint.

Verify the adapter raises a clear solver-integration error.


9. Initial design bounds

Verify an out-of-bounds supplied initial design is rejected rather than clipped.


10. Tiny NSGA-II smoke test

Run a cheap synthetic multi-objective problem for a few generations.

Verify:

    execution completes
    returned X dimensions are correct
    candidate CMD re-evaluation works

Do not test scientific convergence.


LINKAGE INTEGRATION TESTS
=========================

Extend linkage optimization tests or add a separate Stage-0 runner test.

Use TEST-ONLY bounds around the existing V0 projection.

It is acceptable for tests to use simple artificial bounds.

Mark clearly that these are:

    test fixtures
    not production physical bounds.


Required linkage coverage:


1. XZ regression

Verify the optimizer variable names remain:

    *_x_mm
    *_z_mm

and no Y-distal optimizer variable returns.


2. bc_length bound contract

A Stage-0 problem with:

    bc_length_mm.lower <= 0

must be rejected before optimization.


3. Baseline seed

Verify projected BASELINE_LINKAGE_V0 can be included when inside test bounds.


4. Finite pymoo transport

Evaluate a deliberately core-incomplete candidate.

Verify:

    CMD coordination raw/min objective may remain +inf
    pymoo F is finite
    pymoo G is finite


5. Stage-0 objective order

Verify exactly:

    worst_case_transmission_quality
    coordination_ratio_variation
    distal_endpoint_excursion_mm


6. Stage-0 NSGA-II smoke

Run a very small population/generation count under test-only V0-centered bounds.

The test only needs to verify:

    no integration crash
    result structure is valid
    final population is available
    Pareto handling works whether feasible candidates exist or not


7. Export smoke

Export a tiny run to pytest tmp_path.

Verify:

    run.json
    pareto.csv
    final_population.csv

are created and parseable.


DO NOT MAKE TESTS SLOW
======================

The full optimization is NOT a unit test.

Use small:

    sample_count
    population_size
    generations

for smoke coverage.

Keep runtime suitable for the normal pytest suite.


REGRESSION
==========

Run focused tests including at least:

    pytest -q \
        tests/test_optimization_problem.py \
        tests/test_linkage.py \
        tests/test_linkage_kinematics.py \
        tests/test_linkage_optimization.py \
        tests/test_pymoo_adapter.py

plus any new Stage-0 runner test file.

Then run:

    pytest -q

Do not claim success unless actually executed.


IMPLEMENTATION RULES
====================

- Keep `cmd/morphology` optimizer-independent.
- Keep `OptimizationProblem` solver-independent.
- Pymoo adaptation belongs only under `cmd/optimization/solvers`.
- Reuse `OptimizationProblem.evaluate`; do not duplicate objective or
  constraint equations.
- One domain evaluation per candidate inside the adapter.
- Preserve deterministic variable/objective/constraint order.
- Preserve raw domain objective values separately from solver F.
- Do not invent production design bounds.
- Do not change current XZ frame conventions.
- Do not change q1 flexion sign.
- Do not change Stage-0 objectives.
- Do not promote usable q1 range back into an objective.
- Do not call point C the fingertip.
- Do not implement Stage 1.
- Do not implement Stage 2.
- Do not add visualization in this task.
- Do not add parallel execution yet.
- Do not add checkpoint/restart yet.
- Do not add CMA-ES or another optimizer.
- Do not use subagents.


HARD STOPS
==========

Stop and report instead of guessing if:

- the installed pymoo API differs materially from the expected adapter model;

- pymoo is not available in the active Codex environment;

- actual production linkage box bounds are required to complete a code path;

- a current constraint has candidate-dependent residual dimension that cannot
  be made fixed without changing its scientific meaning;

- the XZ linkage convention is no longer the current repository source of
  truth;

- integrating pymoo would require changing canonical morphology;

- the current Stage-0 objective formulation has materially changed since
  inspected commit 904f9af6da84434d2575baac51dd06c43946abae.


OUT OF SCOPE
============

Do not:

- choose real linkage search bounds;
- choose the final Stage-0 constraint thresholds;
- run a large NSGA-II optimization;
- select a new baseline linkage;
- generate Pareto plots;
- optimize actual fingertip geometry;
- optimize hand layout;
- implement Stage 1 or Stage 2;
- implement surrogate optimization;
- implement MO-CMA-ES;
- modify BASELINE_LINKAGE_V0 merely to improve optimization.


FINAL REPORT
============

Report only:

1. Inspected HEAD
2. Installed pymoo version
3. Files added/modified
4. Pymoo adapter API
5. Exact F and G mapping
6. Non-finite objective handling
7. Fixed constraint-shape handling
8. NSGA-II runner API
9. Baseline-seed initialization behavior
10. Stage-0 result/export API
11. Focused test results
12. Full pytest result
13. Any unresolved issue
14. Explicit confirmation that no production physical bounds were invented

STOP after completing the task.