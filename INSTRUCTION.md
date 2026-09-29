CODEX TASK
==========

Task:
Finish the CLEAR Stage-0 linkage optimization workflow as a complete,
research-usable experiment pipeline.

The repository already contains:

    solver-independent CMD OptimizationProblem
    linkage Stage-0 formulation
    pymoo 0.6.2 adapter
    NSGA-II runner
    result export

This task must add the missing experiment layer so a Stage-0 study can be
defined, executed, diagnosed, reproduced, and iterated without editing source
code for every run.

The final workflow must be:

    Stage-0 study JSON
        ↓
    validated LinkageStage0StudyConfig
        ↓
    existing build_linkage_baseline_problem(...)
        ↓
    existing pymoo / NSGA-II implementation
        ↓
    existing CMD re-evaluation
        ↓
    run.json
    pareto.csv
    final_population.csv
    diagnostics.json
        ↓
    concise terminal summary

This task DOES include the executable optimizer workflow.

Do NOT implement a second NSGA-II algorithm.
Reuse and finish the existing pymoo implementation.

Do NOT invent physical linkage bounds.

Do NOT use subagents.


CURRENT REPOSITORY STATE
========================

Repository:

    dokkev/CMD-Framework

Inspected HEAD:

    54b0ef5246cfe2650fb23530a11242bcc36602ab

Installed pymoo:

    0.6.2

Before editing:

1. verify the current HEAD;
2. inspect changes if HEAD has advanced;
3. confirm the current Stage-0 formulation and pymoo adapter still match this
   instruction.

Stop and report if the architecture has materially changed.


CURRENT IMPLEMENTATION
======================

The current generic optimization layer is:

    cmd/optimization/
        variable.py
        objective.py
        constraint.py
        problem.py

The current pymoo solver layer is:

    cmd/optimization/solvers/
        __init__.py
        pymoo.py

It already contains:

    PymooProblemAdapter
    NSGA2Config
    NSGA2RunResult
    run_nsga2(...)

The current linkage optimization layer is:

    cmd/optimization/linkage/
        parameterization.py
        evaluator.py
        workspace.py
        objectives.py
        constraints.py
        baseline.py
        stage0.py

It already contains:

    LinkageBaselineParameterization
    LinkageBaselineConstraintConfig
    LinkageSweepConfig
    LinkageWorkspaceCriteria
    build_linkage_baseline_problem(...)
    run_linkage_stage0(...)
    export_linkage_stage0_results(...)


CURRENT STAGE-0 SCIENTIFIC FORMULATION
======================================

Do NOT change this formulation in this task.

Canonical linkage plane:

    XZ

Canonical directions:

    +X = distal
    +Y = flexion axis
    +Z = dorsal

Runtime input convention:

    s = -q1

so:

    q1 = 0
        fully extended

    q1 < 0
        flexion


The 12-dimensional design vector is the existing
`LinkageBaselineParameterization`.

Keep the current variable order exactly.


Current objectives, in order:

    1. worst_case_transmission_quality
       sense = max

    2. coordination_ratio_variation
       sense = min

    3. distal_endpoint_excursion_mm
       sense = max

Point C is only:

    distal-link proxy

Do NOT call it the actual fingertip.


Current workspace semantics:

    core_input_workspace_rad
        fixed quality-comparison interval

    search_horizon_rad
        larger evaluation horizon

    usable_input_workspace_rad
        diagnostic + minimum hard constraint

Do NOT make usable q1 range a Pareto objective.


PRIMARY GOAL
============

After this task, a user must be able to create a study configuration file and
run the study without command-line flags:

    python scripts/optimize/optimize_linkages.py

without editing Python source.

The script's default study path and output root are defined at its top. If the
study file is missing, the first invocation writes an incomplete template and
exits. Subsequent runs allocate a new numbered run directory.

The command must:

    load and validate the study
    execute NSGA-II
    export all results
    compute diagnostics
    print a concise summary


STUDY CONFIGURATION
===================

Add:

    cmd/optimization/linkage/study.py

Define an immutable dataclass conceptually equivalent to:

    LinkageStage0StudyConfig

containing at least:

    name
    variables
    constraint_config
    sweep_config
    solver_config
    include_baseline_seed

Recommended type structure:

    @dataclass(frozen=True)
    class LinkageStage0StudyConfig:
        name: str
        variables: tuple[BoxVariable, ...]
        constraint_config: LinkageBaselineConstraintConfig
        sweep_config: LinkageSweepConfig
        solver_config: NSGA2Config
        include_baseline_seed: bool = True


Validation
----------

Validate:

    name is non-empty

    variables exactly match
        LinkageBaselineParameterization.variable_names

    variable order is exact

    all BoxVariable validation remains active

    bc_length_mm lower > 0

    core workspace < search horizon

    minimum required input workspace <= search horizon

    existing constraint/sweep/solver dataclasses remain authoritative

Do not duplicate their numeric validation unnecessarily.


JSON STUDY FORMAT
=================

Provide:

    load_linkage_stage0_study(path)
    save_linkage_stage0_study(...)
    study_from_dict(...)
    study_to_dict(...)

Use only the Python standard library.

Do not add:

    pydantic
    yaml
    pandas

The JSON representation must be explicit and human-editable.


Use an ordered variable list, NOT an unordered mapping.

Conceptual JSON:

    {
      "name": "linkage_stage0_example",

      "variables": [
        {
          "name": "o_prime_from_o_x_mm",
          "lower": ...,
          "upper": ...
        },
        ...
      ],

      "workspace": {
        "core_input_workspace_rad": ...,
        "minimum_required_input_workspace_rad": ...,
        "search_horizon_rad": ...,
        "sample_count": ...
      },

      "criteria": {
        "minimum_transmission_margin_rad": ...,
        "q2_minimum_motion_ratio": ...,
        "q3_minimum_motion_ratio": ...,
        "q2_maximum_motion_ratio": ...,
        "q3_maximum_motion_ratio": ...
      },

      "constraints": {
        "minimum_anchor_dorsal_offset_mm": ...,
        "q2_minimum_endpoint_flexion_rad": ...,
        "q3_minimum_endpoint_flexion_rad": ...
      },

      "solver": {
        "population_size": ...,
        "generations": ...,
        "seed": ...,
        "eliminate_duplicates": true,
        "nonfinite_objective_penalty": 1e12
      },

      "include_baseline_seed": true
    }


Do not add values that do not already exist in the current formulation.


NO INVENTED PHYSICAL BOUNDS
===========================

This is a HARD requirement.

There are currently no accepted production physical bounds for the 12 design
variables.

Do NOT invent them.

Do NOT copy:

    BASELINE_LINKAGE_V0 +/- 1

from tests into a production study.

Do NOT add a default production Stage-0 study with arbitrary numeric bounds.


STUDY TEMPLATE
==============

Provide a way to generate a study template containing the canonical variable
names without inventing bounds.

The executable script should create the incomplete template at its configured
study path when that file is missing, then exit with instructions to fill it.
Keep the create-template helper available for direct Python use as well.

The generated template should contain:

    every canonical variable name

with:

    lower: null
    upper: null

or another clearly incomplete representation.

Likewise settings that require researcher choice may be null if no established
value exists.

The loader must reject incomplete/null values when an optimization run is
requested.

The template is a schema aid, not an executable study.


OPTIMIZER EXECUTION
===================

The actual optimizer execution must use the EXISTING:

    run_linkage_stage0(...)

which itself uses:

    run_nsga2(...)

Do NOT duplicate:

    PymooProblemAdapter
    NSGA2
    LHS initialization
    objective sign conversion
    constraint flattening

inside the run script or study layer.


The execution path must remain:

    LinkageStage0StudyConfig
        ↓
    run_linkage_stage0(...)
        ↓
    run_nsga2(...)
        ↓
    PymooProblemAdapter
        ↓
    pymoo 0.6.2 NSGA2


RUN SCRIPT
==========

Add:

    scripts/optimize/optimize_linkages.py

Do not use argparse or command-line options. Running the script with no
arguments loads the study at its top-level ``STUDY_PATH`` constant and writes
results under ``OUTPUT_ROOT``. If the study file is missing, create its
incomplete template and exit. Otherwise run the study and print a concise
summary.

Expose a small ``run_study(study_path, output_root, solver_overrides=None)``
function for direct Python use. Solver overrides, when provided, may change only
``NSGA2Config`` and must not mutate the source JSON. The script-level
``SOLVER_OVERRIDES`` mapping defaults to empty.

Allocate a new numbered run directory for each invocation so completed runs
are not overwritten. Keep the study path and output root configurable at the
top of the script.


RUN RESULT METADATA
===================

Extend the Stage-0 result/export path so run metadata records:

    study name
    include_baseline_seed

in addition to existing metadata.

Also record initialization semantics:

    initialization:
        baseline_seed_included
        remaining_population_method = "lhs"

Do not infer that the baseline seed was included merely from solver results.


SOURCE REVISION
===============

The run script should attempt to record the git commit SHA of the source used
for the run.

Use a small standard-library subprocess call equivalent to:

    git rev-parse HEAD

from the repository when available.

Store in run.json:

    source_git_commit

If the repository SHA cannot be determined:

    source_git_commit = null

Do NOT fail the optimization solely because git metadata is unavailable.

Do not put git subprocess logic inside the generic optimization core.


PYMOO VERSION
=============

Continue recording:

    pymoo_version

from the existing NSGA2RunResult.

Do not hard-code "0.6.2" into result files.

The installed runtime version is authoritative.


OPTIMIZER IMPLEMENTATION REVIEW
===============================

As part of this task, review the existing:

    cmd/optimization/solvers/pymoo.py

and make only concrete fixes needed for production Stage-0 execution.

Preserve the following behavior:

    CMD objectives converted through minimization_objective_values

    Stage-0 F:
        [
            -worst_case_transmission_quality,
             coordination_ratio_variation,
            -distal_endpoint_excursion_mm
        ]

    ordinary inequality residuals:
        passed to G in declaration/residual order

    CMD equality:
        abs(h) - tolerance
        appended to G

    box bounds:
        xl/xu only

    non-finite objective:
        finite solver transport penalty

    unexpected non-finite constraint:
        hard integration error

    fixed constraint dimensions:
        established from prototype

    baseline + LHS initialization

    no-feasible-Pareto case:
        empty Pareto candidate set
        final population retained


Do NOT replace these with a new architecture.


OBJECTIVE NaN POLICY
====================

Document the current policy clearly.

Current generic objective code treats:

    NaN

as an evaluation error.

The pymoo adapter handles non-finite objective values that successfully reach
the adapter, especially:

    +inf / -inf

Do NOT weaken NaN into an arbitrary penalty in this task.

NaN should remain fail-fast because it usually indicates a numerical or
implementation bug.


CONSTRAINT SCALING
==================

Do NOT implement solver-side constraint scaling yet.

Current residuals have different units:

    mm
    rad
    dimensionless

and samplewise constraints have different vector lengths.

This is an important research/solver issue, but normalization scales have not
yet been established.

The first real smoke optimization should diagnose whether scaling is actually
required.

Keep raw G behavior unchanged in this task.

Add a short TODO/doc comment where appropriate noting that future solver-only
constraint scaling may be added without changing CMD domain residuals.


DIAGNOSTICS
===========

Add:

    cmd/optimization/linkage/diagnostics.py

or an equivalently small module.

Implement a function conceptually equivalent to:

    summarize_linkage_stage0(result)

Return an immutable structure or plain serializable mapping containing at
least:

    final_population_count
    feasible_count
    feasible_fraction
    pareto_candidate_count


For every named constraint:

    violated_candidate_count
    violation_fraction
    mean_positive_max_violation
    maximum_violation


For final-population workspace diagnostics:

    closure_input_workspace_rad:
        finite min
        median
        max

    usable_input_workspace_rad:
        finite min
        median
        max


For each raw objective:

    finite_count
    nonfinite_count
    finite min
    median
    max


Do not assign "best" candidate rankings.

These diagnostics are descriptive only.


CONSTRAINT FAILURE FREQUENCY
============================

A constraint counts as violated for one candidate when:

    ConstraintResult.maximum_violation > 0

or a box variable has positive violation.

Stage-0 optimizer-generated candidates should normally satisfy box bounds, but
include box statistics if straightforward.

Preserve one diagnostic entry per NAMED domain constraint.

Do not count every sample residual as an independent candidate failure.

For example:

    minimum_transmission_margin

is violated by a candidate if ANY of its sampled residuals violate it.


POSITIVE VIOLATION STATISTICS
=============================

For one named constraint and one violating candidate, use:

    ConstraintResult.maximum_violation

for the summary statistics.

Do not sum hundreds of samplewise residuals for the per-constraint diagnostic.

The existing CSV:

    overall_sum_positive_constraint_violation

may remain as-is for detailed reporting.


WORKSPACE FAILURE REASONS
=========================

Also summarize:

    limiting_reasons

over the final population.

Count how many candidates terminate usable workspace because of:

    kinematic_failure
    transmission_loop_1
    transmission_loop_2
    q2_minimum_motion_ratio
    q3_minimum_motion_ratio
    q2_maximum_motion_ratio
    q3_maximum_motion_ratio
    search_horizon

and any existing valid reason.

If multiple reasons occur at the same limiting sample:

    increment each reason count

Do not force a single winner/reason.


DIAGNOSTICS EXPORT
==================

Extend:

    export_linkage_stage0_results(...)

or the run-script workflow so the output directory contains:

    run.json
    pareto.csv
    final_population.csv
    diagnostics.json

The diagnostics file must be valid JSON with:

    allow_nan=False

Therefore convert non-finite objective summary cases to explicit nulls or
separate counts rather than NaN/Infinity JSON literals.


TERMINAL SUMMARY
================

After a script-driven optimization completes, print a concise human-readable
summary.

Example shape:

    Stage-0 linkage optimization complete
    Study: ...
    Evaluations: ...
    Final population: ...
    Feasible: 17 / 100 (17.0%)
    Pareto candidates: ...

    Most frequently violated constraints:
      minimum_anchor_dorsal_offset: 61%
      minimum_transmission_margin: 43%
      monotonic_flexion_q3: 18%

    Output:
      ...

Do not print hundreds of sample residuals.

Do not declare a "best linkage".


KINEMATIC FAILURE DIAGNOSTICS
=============================

Add the current:

    LinkageEvaluation.failure_reason

to candidate CSV export.

Use a deterministic column such as:

    kinematic_failure_reason

For candidates with no failure:

    ""

This will help diagnose whether failed candidates are caused by:

    separated circles
    contained circle
    branch issues
    reference geometry failure

Do not parse these strings into new scientific constraint categories yet.


STUDY SNAPSHOT
==============

For reproducibility, write the fully resolved study used for the run into the
output directory:

    study.resolved.json

This file must reflect:

    original study values
    PLUS any solver overrides

and include:

    study name
    variables
    workspace
    criteria
    constraints
    solver
    include_baseline_seed

Do not modify the source input JSON.


OUTPUT DIRECTORY
================

A successful run directory should therefore contain:

    run.json
    study.resolved.json
    diagnostics.json
    pareto.csv
    final_population.csv


OVERWRITE POLICY
================

Do not silently overwrite an existing non-empty optimization output directory.

Preferred behavior:

    if output directory exists and contains files:
        fail clearly

Optionally support:

    --overwrite

if implemented cleanly.

If `--overwrite` is provided:

    overwrite only files owned by this Stage-0 run workflow

Do not recursively delete arbitrary directories.


SMOKE STUDY SUPPORT
===================

The optional ``run_study`` solver overrides are the intended smoke-test
mechanism. The no-argument script can also load small solver settings directly
from its JSON study.

For example, a researcher can define real study bounds once and run:

    solver.population_size = 50
    solver.generations = 10

or set those two values in ``SOLVER_OVERRIDES`` for one script invocation.

Do not add separate "smoke physical bounds."


NO PRODUCTION STUDY YET
=======================

Do not commit a production Stage-0 study with guessed design-space values.

Tests may construct artificial V0-centered bounds.

Those must remain clearly labeled:

    TEST FIXTURE ONLY


TESTS: STUDY CONFIG
===================

Add:

    tests/test_linkage_stage0_study.py

or equivalent.

Required coverage:


1. Round-trip

Construct a complete study config.

Verify:

    config -> dict -> JSON -> config

preserves all values and variable order.


2. Canonical variable order

Reject study JSON whose variables are:

    missing
    duplicated
    unexpected
    reordered


3. Null template

Verify generated template contains all 12 canonical variable names.

Verify a template with null bounds cannot be executed as a study.


4. bc_length domain

Reject:

    bc_length_mm.lower <= 0


5. workspace validation

Reject inconsistent:

    core >= search horizon

    minimum required workspace > search horizon


6. solver config

Verify JSON maps correctly to existing:

    NSGA2Config


7. include_baseline_seed

Verify round-trip and execution behavior.


TESTS: DIAGNOSTICS
==================

Add focused tests for:

    feasible fraction

    per-constraint violation count

    mean/max violation

    objective finite/non-finite counts

    workspace min/median/max

    limiting-reason frequencies


Use synthetic or small deterministic results.

Do not run a large optimizer.


TESTS: RUN SCRIPT
=================

Add a lightweight script smoke test.

Test template creation by invoking ``main()`` with a missing temporary study
path, then execute a tiny study through ``run_study`` or ``main()`` with:

    test-only V0-centered bounds
    tiny sample_count
    tiny population
    1-2 generations


Verify the output contains:

    run.json
    study.resolved.json
    diagnostics.json
    pareto.csv
    final_population.csv


Do not test convergence.


OPTIMIZER REGRESSION
====================

Retain all existing pymoo tests.

Explicitly verify current behavior remains:

    F sign mapping

    G flattening order

    equality tolerance conversion

    fixed residual shapes

    finite objective transport penalty

    non-finite constraint rejection

    out-of-bounds initial-design rejection

    no-feasible-Pareto support

    baseline + LHS initialization


REPRODUCIBILITY
===============

For the same:

    study
    pymoo version
    source commit
    seed

the run should be configured deterministically as far as supported by pymoo.

Do not claim bitwise reproducibility across:

    different pymoo versions
    different NumPy versions
    different platforms

Record enough metadata to identify the run.


REGRESSION TESTS
================

Run focused tests including at least:

    pytest -q \
        tests/test_optimization_problem.py \
        tests/test_pymoo_adapter.py \
        tests/test_linkage.py \
        tests/test_linkage_kinematics.py \
        tests/test_linkage_optimization.py \
        tests/test_linkage_stage0.py \
        tests/test_linkage_stage0_study.py

plus any new diagnostics or run-script test file.


Then run:

    pytest -q


The current environment previously required:

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

because ROS `launch_testing` plugin auto-loading failed on a missing `yaml`
dependency.

If that environment issue remains, it is acceptable to run:

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest ...

Document that fact accurately.

Do not claim ordinary pytest passed if plugin auto-loading failed before tests
started.


IMPLEMENTATION RULES
====================

- Keep morphology solver-independent.
- Keep generic OptimizationProblem solver-independent.
- Keep pymoo-specific code under cmd/optimization/solvers.
- Reuse existing run_nsga2().
- Reuse existing run_linkage_stage0().
- Do not duplicate objective or constraint math.
- Do not invent physical design bounds.
- Do not invent solver constraint scales.
- Keep XZ convention.
- Keep q1 < 0 flexion convention.
- Keep point C described as distal-link proxy.
- Preserve objective ordering.
- Preserve deterministic CSV/JSON field ordering where practical.
- Use standard library for config, JSON, CSV, run script, statistics, and git metadata.
- Do not add pandas.
- Do not add YAML.
- Do not implement visualization yet.
- Do not implement parallel evaluation yet.
- Do not implement checkpoint/resume yet.
- Do not implement CMA-ES.
- Do not implement Stage 1.
- Do not implement Stage 2.
- Do not use subagents.


HARD STOPS
==========

Stop and report instead of guessing if:

- production physical design bounds are required to complete the implementation;

- current HEAD has materially changed the Stage-0 scientific formulation;

- pymoo 0.6.2 APIs used by the existing solver no longer match the environment;

- the study layer would require moving optimizer concepts into morphology;

- a new numeric constraint normalization scale would need to be invented;

- a test reveals candidate-dependent G dimensions that cannot be fixed without
  changing scientific semantics.


OUT OF SCOPE
============

Do not:

- choose the 12 real design bounds;
- choose final scientific thresholds;
- run the final long optimization;
- select a new baseline linkage;
- rank Pareto candidates;
- generate Pareto figures;
- optimize hand layout;
- optimize fingertip outer geometry;
- optimize fingertip internal geometry;
- implement contact mechanics;
- implement Stage 1;
- implement Stage 2.


FINAL REPORT
============

Report only:

1. Inspected starting HEAD
2. Files added/modified
3. Study-config API
4. JSON study schema
5. Template behavior
6. Run-script invocation and Python API
7. Exact optimizer execution path
8. NSGA-II/pymoo changes, if any
9. Diagnostics implemented
10. Reproducibility metadata
11. Output-directory contents
12. Focused test results
13. Full test result
14. Any unresolved issue
15. Explicit confirmation that no production physical bounds or constraint
    normalization scales were invented

STOP after completing the task.
Stage 1/2까지 생각하면 지금 한 번 정리하고 가는 게 좋겠어.
가장 먼저 보이는 문제는 이거야:
cmd/optimization/linkage/
    evaluator.py
    workspace.py
    objectives.py
    constraints.py
    baseline.py
    stage0.py
    study.py
    diagnostics.py
여기에 study.py만 392줄, stage0.py도 300줄 가까이 되고, diagnostics + runner까지 합치면 optimization experiment plumbing이 실제 linkage logic보다 더 커지기 시작했어.
더 중요한 건 파일 개수보다 dependency 방향이야.
지금 가장 큰 architectural smell
현재 cmd/optimization/linkage/__init__.py가 거의 모든 걸 import하고 있어.
그중:
stage0.py
    -> pymoo

study.py
    -> NSGA2Config -> pymoo
를 끌고 들어오니까,
from cmd.optimization.linkage import LinkageSweepEvaluator
같이 그냥 linkage evaluator 하나 쓰려고 해도 package import 단계에서 pymoo까지 따라 들어와.
Stage 1/2에서 linkage kinematics만 쓰고 싶은데:
Stage 1 geometry/contact evaluator
        ↓
cmd.optimization.linkage
        ↓
pymoo
가 되는 건 dependency가 거꾸로야.
이건 지금 고치는 게 맞아.
내가 원하는 boundary
Stage 0/1/2 모두 공통으로 필요한 건 사실 optimizer가 아니라:
\[
\boxed{
\text{linkage parameters}
\rightarrow
\text{sampled linkage motion}
}
\]이거야.
즉 가장 중요한 reusable primitive는 지금의:
LinkageEvaluation
LinkageSweepEvaluator
이 둘이야.
얘들은 optimization 밑에 있으면 안 된다고 봐.
예를 들어:
cmd/morphology/
    linkage_parameter.py
    linkage.py
    linkage_kinematics.py
    linkage_motion.py
정도로 만들고:
linkage_kinematics.py
    q1 하나 solve

linkage_motion.py
    q1 sample sequence 전체 evaluate
    unwrap q2/q3
    motion ratio
    transmission angle/quality
로 나누는 게 자연스러워.
그러면 앞으로:
Stage 0
    LinkageMotionEvaluation
      ↓
    kinematic objectives

Stage 1
    LinkageMotionEvaluation
      +
    hand geometry
      +
    collision/contact opportunity

Stage 2
    LinkageMotionEvaluation
      +
    geometry
      +
    contact mechanics
모두 같은 lower-level result를 재사용할 수 있어.
그리고 지금 evaluator에 Stage-0 개념이 섞여 있음
현재:
LinkageSweepEvaluator(
    config,
    core_input_workspace_rad=core,
)
잖아.
근데 core_input_workspace_rad는 Stage 0 optimization formulation의 개념이지 linkage kinematics의 개념이 아니야.
현재 evaluator가 exact core point를 grid에 삽입하기 때문에 재사용성이 떨어져.
이걸:
LinkageSweepEvaluator(
    input_flexion_rad=(...)
)
정도로 바꾸는 걸 추천해.
또는:
evaluate_linkage_motion(
    parameters,
    input_flexion_rad,
)
이어도 되고.
Stage 0에서만:
samples = make_uniform_samples(
    search_horizon,
    sample_count,
    include=(core_workspace,),
)
해서 넘기면 됨.
이게 훨씬 깔끔해.
즉 reusable layer는 core가 뭔지도 몰라야 해.
workspace.py도 두 종류가 섞여 있어
현재 여기에는:
evaluate_linkage_workspace()
LinkageWorkspaceCriteria
라는 꽤 reusable한 부분과,
core_sample_indices()
core_workspace_is_complete()
core_motion_ratios()
라는 Stage-0-specific 부분이 같이 들어 있음.
이것도 나누는 게 좋아.
예를 들면:
cmd/morphology/linkage_motion.py
    LinkageMotionEvaluation
    evaluate linkage motion

    LinkageWorkspaceCriteria
    LinkageWorkspaceEvaluation
    evaluate_linkage_workspace
그리고:
cmd/optimization/linkage/stage0.py
    core_sample_indices
    core_motion_ratios
    core completeness
정도.
core workspace라는 단어 자체가 optimization study에만 남게 되는 거지.
Stage 0에는 얇은 wrapper 하나만 두면 됨
지금 OptimizationProblem의 context가 바로 LinkageEvaluation인데, 사실 Stage 0에서는 workspace를 여러 번 다시 계산하고 있어.
예를 들어:
DistalEndpointExcursion
    -> evaluate_linkage_workspace()

MinimumUsableInputWorkspace
    -> evaluate_linkage_workspace()

export
    -> evaluate_linkage_workspace()

diagnostics
    -> evaluate_linkage_workspace()
싸긴 하지만 구조적으로 약간 지저분해.
나는 작은 composite context 하나를 두는 게 제일 예쁠 것 같아.
@dataclass(frozen=True)
class LinkageStage0Evaluation:
    motion: LinkageMotionEvaluation
    workspace: LinkageWorkspaceEvaluation
그리고:
FingerLinkageParameters
        ↓
Stage0Evaluator
        ├─ evaluate linkage motion
        └─ evaluate usable workspace
        ↓
LinkageStage0Evaluation
이렇게 딱 한 번 계산.
그러면 objective/constraint는:
context.motion
context.workspace
만 보면 돼.
그리고 이 구조가 Stage 1/2랑도 예쁘게 연결돼.
@dataclass
class Stage1Evaluation:
    linkage_motion: LinkageMotionEvaluation
    ...
같이.
baseline.py와 stage0.py도 사실 겹침
지금:
baseline.py
    problem composition

stage0.py
    run + result + export
인데 baseline이라는 이름도 이제 좀 애매해.
V0 seed가 baseline이지, Stage-0 formulation 전체가 baseline은 아니잖아.
나는 둘을 정리해서:
cmd/optimization/linkage/stage0.py
안에:
LinkageStage0Config
LinkageStage0Evaluation
Stage0Evaluator
build_linkage_stage0_problem()
run_linkage_stage0()
정도만 두고 싶어.
그러면 baseline.py 자체는 없어져도 됨.
BASELINE_LINKAGE_V0는 morphology seed로만 남고.
그리고 study.py는 확실히 과함
이 부분은 네 감각이 맞아 ㅋㅋ
현재 study.py 392줄 중 대부분이:
JSON key validation
null validation
number validation
save/load
template
strict schema checking
이야.
연구 코드에서 이 정도 schema machinery가 지금 필요한지는 의문이야.
특히 이미:
BoxVariable
LinkageBaselineConstraintConfig
LinkageSweepConfig
LinkageWorkspaceCriteria
NSGA2Config
가 각각 validation을 하고 있는데 JSON layer에서 상당 부분을 다시 검증함.
나는 study concept 자체는 유지해도 된다고 봐. 실험 재현성에는 유용함.
다만 훨씬 얇게:
@dataclass(frozen=True)
class LinkageStage0Study:
    name: str
    variables: tuple[BoxVariable, ...]
    stage0: LinkageStage0Config
    solver: NSGA2Config
정도만 library에 두고,
JSON parsing/template은:
scripts/optimize/optimize_linkages.py
쪽으로 밀어도 됨.
혹은 study.py를 유지하더라도:
study_from_dict
study_to_dict
load
save
정도만 남기고 template/schema ceremony는 줄일 수 있어.
392줄까지 갈 필요는 없어 보여.
diagnostics.py도 core API는 아님
153줄짜리 diagnostics는 유용한데, Stage 1/2 reusable library abstraction은 아니야.
현재:
from .stage0 import LinkageStage0Result
까지 물고 있으니까 완전히 Stage-0 reporting code임.
그래서 이걸 package root에서:
from cmd.optimization.linkage import summarize_linkage_stage0
로 public export할 이유는 별로 없어.
나는:
scripts/optimize/
    optimize_linkages.py
    linkage_report.py
정도로 experiment tooling에 두거나,
그냥 script 내부 helper로 둬도 괜찮다고 봐.
특히 __init__.py는 확 줄이자
현재 30개 가까운 symbol을 re-export하는데, 이게 package coupling을 크게 만들어.
나는 cmd/optimization/linkage/__init__.py에서 많아야:
LinkageBaselineParameterization
build_linkage_stage0_problem
LinkageStage0Config
정도만 export하거나, 심지어 거의 비워도 괜찮다고 봐.
Reusable linkage motion은 애초에 optimization package 밖으로 옮기고.
모듈러하게 만드는 가장 쉬운 방법은 public API를 많이 만드는 게 아니라 dependency surface를 줄이는 거야.
내가 추천하는 최종 구조
과하게 새 abstraction을 만들지 않고 이 정도:
cmd/
├── morphology/
│   ├── linkage_parameter.py
│   ├── linkage.py
│   ├── linkage_kinematics.py
│   └── linkage_motion.py          # reusable Stage 0/1/2
│
├── optimization/
│   ├── variable.py
│   ├── objective.py
│   ├── constraint.py
│   ├── problem.py
│   │
│   ├── solvers/
│   │   └── pymoo.py
│   │
│   └── linkage/
│       ├── parameterization.py
│       ├── objectives.py
│       ├── constraints.py
│       └── stage0.py              # Stage-0 composition only
│
scripts/
└── optimize/
    └── optimize_linkages.py       # JSON / study / report / export
study.py, diagnostics.py, baseline.py는 없어져도 된다고 생각해.
물론 JSON parser를 script 밖에 두고 싶다면 study.py 하나 정도는 남겨도 됨. 중요한 건 domain/reusable path에 넣지 않는 것.
제일 중요한 dependency는 이렇게
morphology
    ↓
linkage motion / kinematics
    ↓
Stage-0 / Stage-1 / Stage-2 evaluation
    ↓
OptimizationProblem
    ↓
solver adapter
    ↓
pymoo
절대로:
Stage 1
    ↓
optimization.linkage
    ↓
pymoo
가 되면 안 돼.
그래서 지금 refactor priority
1. LinkageSweepEvaluator/LinkageEvaluation을 optimization 밖으로 이동
2. evaluator에서 core_input_workspace_rad 제거하고 explicit sample input으로 변경
3. reusable workspace 계산과 Stage-0 core_* 계산 분리
4. LinkageStage0Evaluation(motion, workspace) 도입해서 workspace 한 번만 계산
5. baseline.py + stage0.py 정리
6. study.py 대폭 축소 또는 script layer로 이동
7. diagnostics/export를 core API에서 제거
8. cmd.optimization.linkage.__init__ 거의 비우기
나는 특히 1~4는 지금 하는 게 맞고, 5~8은 code cleanup 성격이야.
지금 구현 자체가 잘못된 건 아니야. 오히려 기능 요구를 하나씩 잘 구현하다 보니 experiment infrastructure가 domain layer 안으로 조금씩 침투한 상태야. Stage 1 들어가기 전에 지금 한 번 dependency를 곧게 펴면 이후가 훨씬 편해질 거야.