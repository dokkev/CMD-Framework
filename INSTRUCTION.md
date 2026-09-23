CODEX TASK
==========

Task:
Replace the current box-based XM335 actuator model with the shaft-centered D-shaped extruded geometry and coordinate convention shown in the established design.


OBJECTIVE
=========
Implement:

1. Replace the rectangular-box XM335 body representation with the exact D-shaped extruded body profile.
2. Change the actuator-local frame so the output shaft center is the origin and the shaft axis is local +Z.
3. Update all directly affected visualization, URDF, tests, and frame assumptions to use the new actuator geometry and coordinates consistently.

Success means:

XM335 geometry, visualization, and URDF compilation all use one canonical shaft-centered actuator frame:

- origin = output shaft center
- +X = actuator lateral width direction
- +Y = actuator longitudinal direction toward the rounded cap
- +Z = output shaft / rotation axis

and the actuator body matches the defined 35 x 19 x 22 mm D-shaped extrusion rather than a rectangular bounding box.


ESTABLISHED CONTEXT
===================
Treat these as established inputs, not questions to reopen:

- Repository:
  https://github.com/dokkev/CMD-Framework

- Relevant existing files include:
  - `cmd/hardware/actuator.py`
  - `cmd/hardware/xm335.py`
  - `cmd/hardware/__init__.py`
  - `cmd/visualization/actuator_visualizer.py`
  - `cmd/design/urdf.py`
  - `cmd/morphology/rigid_transform.py`
  - `cmd/morphology/hand_morphology_parameter.py`
  - `cmd/morphology/hand_morphology.py`
  - `cmd/morphology/hand_constraints.py`
  - `tests/test_xm335.py`
  - `tests/test_hand_urdf.py`

- The actuator hardware API uses a shaft-centred `DShapedExtrusion` and
  `MechanicalParameters`.

- The canonical XM335 frame is fixed as:

      origin = output shaft center

      +X = lateral body-width direction
      +Y = longitudinal body direction toward the rounded cap
      +Z = shaft / revolute axis

- The XY cross-section is a D-shaped profile.

- Exact profile geometry in millimetres:

      rectangular portion:
          -9.5  <= x <= 9.5
          -25.5 <= y <= 0

      rounded portion:
          x^2 + y^2 <= 9.5^2
          y >= 0

      extrusion:
          -11 <= z <= 11

- Therefore:

      rounded radius = 9.5 mm
      body extent behind shaft = 25.5 mm
      total X width = 19.0 mm
      total Y extent = 35.0 mm
      Z depth = 22.0 mm

- The shaft center is not a secondary offset inside the actuator anymore.
  It is the actuator frame origin.

- The canonical shaft axis is therefore:

      (0, 0, 1)

- Do not retain the old `center_from_bottom_mm` transformation model merely for compatibility.

- The D-shaped solid can be represented exactly as the union of:

  1. a rectangular prism

         X size = 19.0 mm
         Y size = 25.5 mm
         Z size = 22.0 mm
         center = (0, -12.75, 0)

  2. a cylinder

         radius = 9.5 mm
         length = 22.0 mm
         center = (0, 0, 0)
         axis = local Z

  The negative-Y half of the cylinder lies completely inside the rectangular
  prism, so this union is exactly the desired D-shaped extrusion.

- The reference hand morphology uses this actuator model directly for actuator
  placement and reference-pose packaging checks.

- MCP-AA, MCP-FE, CMC-RO, CMC-RAA, and IP-FE all use this XM335 actuator.

- The actuator-only task originally excluded hand morphology and packaging.
  Their follow-up reference model is implemented in `cmd/morphology/` and is
  governed by the established geometry addendum supplied with that task.

The current reference model uses `+X_H` distal, `+Y_H` along ordinary MCP-FE
axes, and `+Z_H` dorsal. Index MCP-AA is the identity frame; ordinary MCP-AA
and MCP-FE frames remain parallel, separated by the fixed translation
`(23.5, 0, 16.75) mm`. The thumb morphology is composed from CMC-RO placement
and `theta`, RO-to-RAA distance, RAA-to-MCP-FE distance, and `phi`; derived
downstream poses are not independent parameters. XM335 mounts are fixed per
joint and map actuator-local `+Z_A` onto each physical axis. The FE-style mount
keeps `+X_A` aligned with `+X_J` as its explicit clocking convention; the
supplied inputs do not specify actuator roll about the joint axis. Reference
packaging is an OBB screen, and the URDF serializes these morphology transforms
rather than maintaining a second placement graph.

- Existing finger linkage morphology/kinematics are separate and should not be
  redesigned here.


REQUIRED CHANGES
================

1. Actuator geometry representation
   - Replace the current rectangular-box-only actuator body representation with
     a representation capable of describing the established XM335 D-shaped
     extrusion.
   - Use a small geometry abstraction appropriate to the actual shape; do not
     preserve `RectangularBox` as the canonical XM335 geometry.
   - Store only meaningful independent dimensions where practical.
   - Redundant dimensions such as total width or total longitudinal extent
     should be derived rather than duplicated when that keeps the model clear.
   - Keep mechanical actuator properties such as gear ratio, torque, and mass.
   - Keep the representation solver-independent.

2. Canonical actuator frame
   - Make the shaft center `(0, 0, 0)`.
   - Set the shaft / rotation axis to local `+Z`.
   - Define local axes consistently:

         +X = lateral width
         +Y = toward rounded cap
         +Z = shaft axis

   - Remove any remaining assumptions that +Y is the shaft axis or +X points
     toward the rounded cap.
   - Remove the old need to recover the shaft location from
     `center_from_bottom_mm`.

3. XM335 parameters
   - Encode the established geometry:

         radius_mm = 9.5
         rear_extent_mm = 25.5
         depth_mm = 22.0

   - Ensure the resulting occupied volume has bounds:

         X: [-9.5, 9.5]
         Y: [-25.5, 9.5]
         Z: [-11.0, 11.0]

   - Preserve the existing project mechanical parameters unless they are
     already defined elsewhere:

         gear_ratio = 323.04
         max_output_torque_nm = 1.0
         mass_g = 27.0

4. Hardware public API
   - Update `cmd/hardware/__init__.py` exports to match the new representation.
   - Remove obsolete exports whose semantics belong only to the old box/frame
     convention.
   - Do not keep dead compatibility aliases unless an actual current caller
     requires them.
   - Update all in-repository callers instead.

5. Actuator visualization
   - Update `cmd/visualization/actuator_visualizer.py`.
   - Render the actual D-shaped extrusion rather than a 19 x 35 x 22 box.
   - The visualization origin must be the shaft center.
   - Display the new local axes correctly:

         +X lateral width
         +Y longitudinal / rounded-cap direction
         +Z shaft axis

   - Update labels, comments, axis ranges, and legend text so no old coordinate
     convention remains.
   - Do not introduce an unrelated visualization framework.

6. URDF geometry
   - Update `cmd/design/urdf.py` to consume the new actuator representation.
   - Represent the XM335 D-shaped body using standard URDF primitives as the
     exact union of:
       - one rectangular prism
       - one cylinder
   - Use both components for `visual` and `collision`.
   - For the rectangular component, use the established shaft-centered offset:

         center = (0, -12.75, 0) mm
         size   = (19.0, 25.5, 22.0) mm
                  along actuator-local (X, Y, Z)

   - For the cylindrical component:

         center = (0, 0, 0)
         radius = 9.5 mm
         length = 22.0 mm
         axis   = actuator-local Z

   - URDF cylinders are axis-aligned with their local Z axis by default, which
     already matches the actuator shaft axis; do not rotate the cylinder.

7. Hand/URDF actuator-frame assumptions
   - Inspect the current MCP-AA URDF path carefully.
   - The existing implementation places an actuator housing frame and then
     adds an 8 mm shaft-center offset derived from the old box-centered frame.
   - That offset must disappear under the new shaft-centered actuator frame.
   - The actuator link frame and MCP-AA revolute joint origin should coincide at
     the shaft center.
   - The MCP-AA revolute axis in the actuator frame must be `(0, 0, 1)`.
   - Preserve the existing physical MCP-AA rotation axis in the hand frame.
   - Therefore derive and apply the fixed actuator-to-hand mounting rotation
     needed to map actuator-local +Z onto the intended hand MCP-AA axis.
   - Derive the actuator mount rotation from the corrected local axes while
     preserving the established hand-frame MCP-AA axis.

8. Hand placement semantics
   - The current independent parameters live in
     `HandMorphologyParameters`; index MCP-AA is the identity root, and each
     supplied MCP-AA point is the joint/shaft center in the hand frame.
   - Do not add a hidden actuator body-center offset or independently expose
     derived MCP-FE positions.

9. Linkage-frame boundary
   - Re-derive the fixed transform between the new MCP-AA output/actuator frame
     and the existing linkage-local frame.
   - Do not reuse `LINKAGE_FRAME_RPY_IN_HAND_RAD` blindly if its assumptions
     depended on the old actuator frame.
   - Preserve the existing linkage orientation in the hand frame:

         linkage +X -> hand -Y
         linkage +Y -> hand +X
         linkage +Z -> hand +Z

     This is serialized as a fixed `Rz(-pi/2)` from the aligned MCP-FE frame.

   - Preserve physical behavior; only the coordinate representation should
     change.

10. Tests
   - Update `tests/test_xm335.py` for the new geometry and frame.
   - Remove assertions that intentionally require:
       - `RectangularBox`
       - `center_from_bottom_mm`
       - the obsolete +Y shaft axis
       - old width-axis convention
   - Add focused assertions for:
       - radius 9.5 mm
       - rear extent 25.5 mm
       - depth 22 mm
       - bounds X [-9.5, 9.5]
       - bounds Y [-25.5, 9.5]
       - bounds Z [-11, 11]
       - shaft axis `(0, 0, 1)`
       - immutable actuator values
       - invalid geometry values rejected

   - Update `tests/test_hand_urdf.py` to verify:
       - MCP-AA placement is the shaft/joint-frame position
       - no old 8 mm shaft offset remains
       - revolute axis uses the new actuator-local +Z convention
       - actuator mounting orientation maps that axis to the same intended
         physical hand axis
       - both box and cylinder geometry are emitted for visual and collision
       - box/cylinder dimensions and origins match the established geometry
       - cylinder orientation is correct

   - Update any focused actuator-visualizer tests if such tests already exist.


IMPLEMENTATION RULES
====================
- Make the smallest change that satisfies the objective.
- Change behavior at the layer that owns it.
- Reuse existing APIs and abstractions where appropriate.
- Avoid duplicate logic.
- Avoid speculative abstractions.
- Avoid unrelated refactors or cleanup.
- Preserve behavior not explicitly changed by this task.
- Use millimetres inside the Python design/hardware model and convert to metres
  only at the URDF serialization boundary, consistent with the existing code.
- Keep actuator geometry independent of URDF, matplotlib, Newton, or any other
  consumer.
- Do not approximate the D-shaped actuator with its rectangular bounding box.
- Do not introduce a mesh asset when the exact geometry can be represented by
  the established box-plus-cylinder union.
- Do not model screws, horns, cables, mounting tabs, or detailed CAD features.
- Reference actuator packaging currently uses conservative transformed OBBs
  derived from the XM335 bounds. It does not test swept motion or collisions
  with finger links, fingertips, or external objects.
- The low-dimensional thumb reference chain uses the supplied CMC-RO, CMC-RAA,
  and MCP-FE transforms; it is not runtime joint kinematics.
- Treat the new actuator frame convention as canonical; update downstream code
  rather than maintaining two competing coordinate conventions.


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

For this task, Advisor is appropriate only if the actuator-to-hand or
actuator-to-linkage frame transform cannot be determined unambiguously from the
existing coordinate conventions.

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
- correct architectural placement
- new shaft-centered coordinate convention is consistent everywhere
- D-shaped geometry is represented exactly
- URDF physical MCP-AA axis is preserved despite the local-frame change
- no stale old-coordinate assumptions remain in directly affected code
- no unintended behavior changes
- no duplicate/unnecessary abstraction
- no scope creep
- checks correspond to the changed behavior

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
- Run the focused XM335 hardware tests.
- Run the focused hand URDF tests.
- Run any existing actuator visualizer tests directly affected by the change.
- Render or inspect one actuator visualization sufficiently to verify the
  D-shaped body and axis convention if the existing visualization path supports
  a lightweight artifact check.
- Search directly affected source files for stale assumptions such as
  `center_from_bottom_mm`, `+Y shaft axis`, or old `X=longitudinal, Y=shaft, Z=width`
  documentation and confirm they are removed where no longer valid.

Rules:
- Prefer focused tests.
- Do not repeat completed validation.
- Reuse existing results when they already answer the question.
- Do not perform expensive work merely for reassurance.


HARD STOPS
==========
Stop and report if:

- the established actuator dimensions cannot produce the specified D-shaped
  geometry
- the intended physical MCP-AA axis in the hand frame cannot be determined from
  the existing code/conventions
- preserving the existing linkage physical orientation requires an
  undocumented architectural assumption
- satisfying the task requires redesigning the hand morphology model
- satisfying the task requires adding a general-purpose geometry engine
- the implementation task turns into a new research question

Do not silently expand scope.


OUT OF SCOPE
============
Do not:

- implement hand morphology optimization
- implement runtime hand kinematics or closed-loop linkage solving
- replace the conservative OBB packaging check with exact D-solid collision
  geometry
- redesign the finger linkage or its kinematics
- add detailed XM335 CAD features
- add simulator-specific actuator bodies
- add mesh generation or mesh assets

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
