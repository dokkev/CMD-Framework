# CoDe Hand — North Star

The publication goal: what the paper claims, for whom, and what is still open. One page.

The SSoT has four files, each with one job. When they disagree, the higher one wins and the lower one is fixed.

| File | Job |
|---|---|
| `north-star.md` (this file) | Publication goal: problem, approach, contributions and their status |
| `research-objective.md` | Experiment design: principles, what each study must show, claim discipline |
| `problem_formulation.md` | Mathematics: definitions, variables and their names, each study's formulation |
| `code_formulation.md` | Implementation: how the formulation is written as code |

## Venue

RSS 2027 (Athens, July 6–11). Extended abstract due 2026-12-04, stage-1 decision 2027-02-05, final paper due 2027-04-16 (dates from the call for papers; re-check on roboticsconference.org before each deadline).

## Problem

Contact is central to dexterous manipulation, and the contact interface (its geometry, size, compliance and sensing) sets that contact. Research hands use servos for maintainability, and the servo bodies take up the space inside the finger.

- **P1 — Space.** Servos and joints compete with the contact interface for the same space, so a large or thick interface costs either range of motion or contact surface.
- **P2 — Redesign.** Redesigning the finger for a different interface changes how its contact faces move relative to each other. For a joint turning about a fixed axis this cannot be avoided when the interface is moved to its own best depth: a thickness change Δh moves the faces' relative path by at least 2 sin(q/2)·Δh at flexion q.
- **P3 — Measurement.** So when interfaces are compared in manipulation, the difference cannot be attributed to the interface alone: the hand's motion changed with it.

## Goal

A servo-driven finger that, as the interface's size changes, keeps range of motion and contact surface together (P1) and changes its contact faces' relative path little when re-optimized (P2), so that the contact interface can be used as an independent variable in manipulation research (P3).

## Approach (CoDe)

- **Remote actuation:** the servo leaves the joint and drives it through a linkage. → P1
- **Polycentric articulation (the CFL):** the point the distal phalanx turns about moves with flexion. → P1, P2
- **The contact faces' relative path as the redesign target**, measured by D_c (`problem_formulation.md` A.6.3). → P2, P3

## Contributions and status

A contribution is a claim to be tested, not a result to be secured (`research-objective.md`, Claim Discipline).

| | Claim | Evidence | Status (2026-11-21) |
|---|---|---|---|
| C1 | A fixed-axis joint cannot keep the contact faces' relative path when an interface is moved to a different depth: the lower bound 2 sin(q/2)·Δh | Study 0 | Derived; 2R-remote's redesigns sit on the bound at 90° and 120° |
| C2 | The CFL finger keeps more contact surface than a servo-at-joint finger (2R-DD) in all 12 conditions (4 thicknesses × 3 flexions, +2.4 to +14.6 points), and is the only physical architecture with a usable design in all 12 | Study 1 | Shown in the 2D side model. Against 2R-remote the ranking depends on flexion and thickness, and is reported as such |
| C3 | Redesigned for a thicker interface, the CFL changes its contact faces' relative path least among the four physical architectures in all 9 conditions (10–51% of the fixed-axis bound), at near-best coverage | Study 2 | Shown in the 2D side model |
| C4 | The change in the contact faces' relative path alters learning-based interface evaluations | Study 3C | **Open.** Not yet designed in detail |
| C5 | Interfaces are exchanged on one physical hand | Hardware | **Open.** Extent undecided |

Supporting, not contributions: Study 3A (one hand, one learning pipeline, interfaces compared; pilot done, main experiment running) and Study 3B (contact-model sensitivity; pre-registered).

## Not claimed

A hand that manipulates better than others; a best contact interface; that the CFL beats every remote-actuated design everywhere; that keeping the relative path preserves a policy's performance; simulated compliant or tactile interfaces; physical accuracy of a contact model without validation.

## Open decisions

1. **The spacer baseline.** A finger designed once for the thickest interface, with thinner ones raised on spacers, has D_c = 0 on any architecture. The honest comparison is then the trade-off between D_c and coverage, with the spacer design at one end. Whether C1–C3 are restated in that form is undecided.
2. **Term.** "Contact-centric kinematics" promises more than D_c measures (the relative path over flexion, not the servo-to-contact map). "Relative contact trajectory" is the candidate replacement.
3. **How much the transmission changes under redesign** (servo angle to flexion, mechanical advantage), and whether the redesign target should include it. Not yet measured.
4. **Hardware scope** for the final paper (C5).
5. **"Interfaces concentrate on distal phalanges"** is not established (counterexamples exist); until a survey of hands supports it, the motivation rests on P1.
