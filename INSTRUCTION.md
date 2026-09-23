CODEX TASK
==========

Task:
Implement deterministic closed-form kinematics for the planar CLEAR finger linkage in `cmd/morphology`, using the existing reference geometry and coupler lengths.


OBJECTIVE
=========
Implement:

1. Solve the planar closed-chain linkage from a commanded proximal flexion angle `q1`.
2. Derive `q2` and `q3` geometrically from the two fixed-length coupler loops.
3. Return the posed main-chain and coupling-anchor positions for downstream Stage 1 evaluation.

Success means:

`FingerLinkage` continues to define only the mechanism/reference geometry, while `kinematics.py` can deterministically compute a valid posed linkage configuration without Newton, Pinocchio, SymPy, or iterative numerical solving.


ESTABLISHED CONTEXT
===================
Treat these as established inputs, not questions to reopen:

- The linkage is planar in the YZ plane.
- Flexion axes are along +X.
- `O` is the local origin `(0, 0)`.
- The main chain is `O-A-B-C`.
- `q1`, `q2`, and `q3` are relative revolute joint angles:
  - proximal absolute angle: `theta1 = q1`
  - middle absolute angle: `theta2 = q1 + q2`
  - distal absolute angle: `theta3 = q1 + q2 + q3`
- The fully extended reference configuration is `q1 = q2 = q3 = 0`.
- Existing `FingerLinkageParameters` contains only independent design variables.
- Existing `FingerLinkage` derives the fully extended reference positions and the fixed coupler lengths.
- `O'` is fixed to the base.
- `A'` is fixed to the proximal body.
- `B'` is fixed to the middle body.
- `C'` is fixed to the distal body.
- The two rigid couplers are:
  - `O' - B'`
  - `A' - C'`
- `|O'B'|` and `|A'C'|` are derived deterministically from the fully extended reference configuration and are not independent design variables.
- The current anchor-frame definitions in `linkage_parameter.py` are established for this task; do not reparameterize them.
- Stage 1 is intended for very fast kinematic search and later massive parallel evaluation.
- Newton/VBD belongs to Stage 2 and must not be introduced into this Stage 1 kinematics implementation.
- Design-space constraints such as maximum `OC`, min/max `OA'`, `B'B`, `BC`, manufacturability bounds, and feasible-design sampling are intentionally deferred.


REQUIRED CHANGES
================

1. `cmd/morphology/kinematics.py`
   - Implement a small numeric closed-form linkage solver.
   - Do not use an iterative root solver.
   - Do not use SymPy, SciPy optimization, Pinocchio, Newton, or another physics engine.
   - Use only direct planar geometry and standard numeric operations.

   Define a result representation for one posed linkage configuration containing at minimum:
   - `q1_rad`
   - `q2_rad`
   - `q3_rad`
   - posed positions of:
     - `O`
     - `A`
     - `B`
     - `C`
     - `O'`
     - `A'`
     - `B'`
     - `C'`

   Prefer a compact immutable dataclass representation consistent with the existing morphology code.

2. First closure loop: `O' - B'`
   - Treat `q1` as the commanded/input angle.
   - Compute the posed location of `A`.
   - `B'` is fixed in the middle-body frame, so its distance from `A` is constant.
   - The fixed coupler imposes:

     `|O'B'| = linkage.coupler_1_length_mm`

   - Solve `B'` as the intersection of:
     - a circle centered at `A` with radius equal to the fixed local distance `|AB'|`
     - a circle centered at `O'` with radius `|O'B'|`

   - Use the selected `B'` position and the existing local `B'` anchor vector to recover the absolute middle-link orientation `theta2`.
   - Compute:

     `q2 = theta2 - q1`

3. Second closure loop: `A' - C'`
   - Once `q1` and `q2` are known, compute posed `A`, `B`, `A'`, and the middle-link orientation.
   - `C'` is fixed in the distal-body frame, so its distance from `B` is constant.
   - The fixed coupler imposes:

     `|A'C'| = linkage.coupler_2_length_mm`

   - Solve `C'` as the intersection of:
     - a circle centered at `B` with radius equal to the fixed local distance `|BC'|`
     - a circle centered at posed `A'` with radius `|A'C'|`

   - Use the selected `C'` position and the existing local `C'` anchor vector to recover the absolute distal-link orientation `theta3`.
   - Compute:

     `q3 = theta3 - theta2`

4. Assembly branch
   - Circle-circle intersection generally produces two solutions.
   - The physical assembly branch must be deterministic.
   - Infer the branch independently for each closure loop from the existing fully extended reference configuration.
   - Use signed planar orientation / cross-product sign of the reference intersection to select the corresponding intersection during motion.
   - Do not choose a solution based on arbitrary list order.
   - Do not choose the branch independently at every call based only on smallest angle.
   - If the reference configuration is itself degenerate such that the assembly branch cannot be defined, fail clearly rather than silently selecting a branch.

5. Geometric helper
   - Keep the circle-circle intersection implementation small and internal to the kinematics module.
   - Handle:
     - separated circles
     - one circle completely inside the other
     - coincident centers
     - tangent configurations
     - small floating-point roundoff near tangency
   - An unreachable closure must produce an explicit kinematic-closure failure rather than NaN propagation.
   - A tangent configuration may be geometrically valid; do not classify all tangencies as invalid merely because they are singular.
   - Do not add design-space singularity-margin constraints in this task.

6. Main-chain pose reconstruction
   - Reconstruct the main pivots from the solved absolute link orientations:

     `A = O + R(theta1) * OA`
     `B = A + R(theta2) * AB`
     `C = B + R(theta3) * BC`

   - Reconstruct all primed anchors from their existing owning-body local coordinates.
   - The returned posed positions must satisfy both coupler lengths within numerical tolerance.

7. Public API
   - Provide a simple API centered on something equivalent to:

     `LinkageKinematics(linkage).solve(q1_rad)`

   - Exact naming may follow the existing package style, but keep the API small.
   - `FingerLinkage` must remain the reference/design geometry object.
   - Configuration-dependent solving belongs in `kinematics.py`.
   - Export the new public kinematics types through `cmd/morphology/__init__.py` if that file currently serves as the package public API.


IMPLEMENTATION RULES
====================
- Make the smallest change that satisfies the objective.
- Change behavior at the layer that owns it.
- Reuse `FingerLinkage`, `ReferencePositions`, `AnchorYZ`, and the existing linkage parameters.
- Do not duplicate reference-geometry or coupler-length calculations already owned by `FingerLinkage`.
- Keep all calculations in the existing YZ planar convention.
- Use radians for joint angles.
- Keep the implementation numeric and lightweight.
- Prefer `math` and simple scalar arithmetic.
- Structure the math so it can later be ported cleanly to a batched Warp kernel.
- Do not introduce NumPy solely for this scalar reference implementation.
- Do not introduce a generic IK abstraction.
- Do not add backend-specific FCL, Warp, or Newton code.
- Do not modify the linkage design parameterization as part of this task.
- Avoid unrelated refactors.


SUBAGENTS
=========

ADVISOR
-------
Use only when a genuinely non-obvious design, architecture, or semantic
decision is required.

Advisor is read-only.

Evaluate:
- consistency with the objective
- consistency with established context
- architectural fit
- semantic correctness
- unnecessary complexity
- scope creep

Output:

DECISION:
    APPROVE / BLOCK / CLARIFY

REASON:
    <concise reasoning>

CONSTRAINTS:
    <only necessary constraints>

Do not:
- edit code
- reopen established decisions
- broaden the task
- propose unrelated work
- request extra validation merely for confidence


REVIEWER
--------
After implementation and focused checks, use one independent reviewer.

Reviewer is read-only and reviews the actual implementation.

Check:
- objective satisfied
- required changes satisfied
- `FingerLinkage` remains reference geometry only
- kinematics owns configuration-dependent solving
- reference configuration reproduces exactly the existing reference geometry
- both fixed coupler lengths remain satisfied after solving
- assembly branch selection is deterministic
- no iterative solver or simulation dependency was introduced
- no design-space constraints were added prematurely
- no scope creep

Output:

VERDICT:
    PASS / PASS_WITH_NOTES / BLOCK

BLOCKERS:
    <real blockers only>

NOTES:
    <non-blocking observations only>

If BLOCK:
- fix only the blocker
- rerun only directly affected checks
- optionally perform one reviewer re-check


VALIDATION
==========
Run only checks justified by the change.

Required:
- Existing linkage tests continue to pass.
- Add focused kinematics tests verifying `solve(0.0)` reproduces the existing fully extended reference positions and returns approximately `q2 = q3 = 0`.
- Test at least one nonzero reachable `q1` configuration and verify:
  - all main-link lengths are preserved
  - `|O'B'|` equals the reference coupler-1 length
  - `|A'C'|` equals the reference coupler-2 length
  - reconstructed primed anchors agree with the solved points
- Test deterministic assembly-branch selection.
- Test at least one unreachable closure and verify a clear kinematic failure is produced instead of NaN or arbitrary geometry.

Rules:
- Prefer focused tests in `tests/test_linkage.py` or a small dedicated kinematics test file.
- Do not add large sweeps or performance benchmarks yet.
- Do not repeat completed validation.
- Reuse existing test fixtures where appropriate.


HARD STOPS
==========
Stop and report if:

- the current owning-frame definitions for `A'`, `B'`, or `C'` are internally inconsistent with `FingerLinkage.reference_positions`
- the existing fully extended reference geometry cannot determine a unique physical assembly branch
- implementing the closure requires changing the established linkage topology
- satisfying the task requires introducing a generic nonlinear solver
- a required assumption about the two rigid coupler loops is false

Do not silently expand scope.


OUT OF SCOPE
============
Do not:

- implement linkage design-space bounds or feasible-design sampling
- implement `OC`, `OA'`, `B'B`, or `BC` min/max constraints
- implement optimization
- implement Warp batching or GPU kernels
- implement FCL collision checking
- implement thumb opposition search
- implement Newton rigid-body simulation
- implement VBD or soft fingertip mechanics
- implement symbolic derivation
- implement velocity, acceleration, Jacobian, torque, or dynamics analysis

Also avoid unrelated cleanup, speculative improvements, and unsolicited
follow-up work.


FINAL REPORT
============
Report only:

1. What changed
2. Relevant checks and results
3. Advisor decision, if used
4. Reviewer verdict
5. Any blocker or unresolved issue

STOP after completing the task.