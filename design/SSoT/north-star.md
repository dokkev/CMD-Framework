# CoDe Hand — North Star

What problem exists, what CoDe solves, and what the work claims. Concepts only: how the studies test the claims, their status and the venue plan are in `research-objective.md`.

The SSoT has four files, each with one job. When they disagree, the higher one wins and the lower one is fixed.

| File | Job |
|---|---|
| `north-star.md` (this file) | The problem, what CoDe solves, the contributions claimed |
| `research-objective.md` | Experiment design: principles, what each study must show, claim discipline, status and paper plan |
| `problem_formulation.md` | Mathematics: definitions, variables and their names, each study's formulation |
| `code_formulation.md` | Implementation: how the formulation is written as code |

## In one sentence

CoDe is a servo-driven hand on which contact interfaces of different size and shape can be exchanged while the relative contact trajectory (how one contact face moves as seen from the other) is kept, so that contact morphology can be studied as an independent variable in manipulation.

## The problem

Contact is central to dexterous manipulation, and the contact interface (its geometry, size, compliance and sensing) sets that contact. Research hands are built around servos for maintainability, and servo bodies and joints occupy the space a contact interface would need.

- **P1 — Space.** Servos and joints compete with the contact interface for the same space, so a large or thick interface costs either range of motion or contact surface.
- **P2 — Redesign.** Redesigning a finger for a different interface changes its relative contact trajectory. For a joint turning about a fixed axis this is unavoidable when the interface sits at its own best depth.
- **P3 — Measurement.** So when interfaces are compared in manipulation, a difference cannot be attributed to the interface alone: the hand's motion changed with it.

## Why the relative contact trajectory

The question every result must answer: why should a redesign keep the relative contact trajectory, rather than the skeleton's motion or nothing at all?

1. **The object meets the contact faces, not the skeleton.** To measure an interface's effect, everything the object meets except the interface should stay the same; for contact on more than one phalanx that is the relative pose of the contact faces over flexion, the relative contact trajectory.
2. **A redesign does not keep it unless it is the target.** Redesigned for coverage alone, a finger's relative contact trajectory changes by more than a fixed axis would force, and by an amount that depends on the search.
3. **It can be kept, at little cost.** A polycentric joint keeps it far inside the fixed-axis bound at near-best contact surface; a fixed axis cannot (C1, C3).

Whether a change of the relative contact trajectory cannot be undone by control, and whether its size changes what an interface experiment measures, are open questions (C4), worked out in `docs/experimental/`.

## What CoDe solves

- **Remote actuation:** the servo leaves the joint and drives it through a linkage, freeing the space under the joints for the interface. → P1
- **Polycentric articulation (the CFL):** the point the distal phalanx turns about moves with flexion, so a thick interface can sit under the joint without giving up range of motion or contact surface. → P1, P2
- **The relative contact trajectory as the redesign target:** when a new interface asks for a redesign, the finger is re-optimized to keep how its contact faces move, not how its skeleton moves. → P2, P3

## Contributions

A contribution is a claim to be tested, not a result to be secured.

- **C1 — A bound.** A fixed-axis joint cannot keep the relative contact trajectory when an interface changes depth; the change has a closed-form lower bound.
- **C2 — A finger.** A remote-actuated polycentric finger (the CFL) that keeps more contact surface than a servo-at-joint finger as interfaces thicken and the required flexion deepens.
- **C3 — Redesign that keeps contact.** Re-optimized for a new interface, the CFL keeps its relative contact trajectory far closer than the fixed-axis bound allows, at near-best contact surface.
- **C4 — Why it matters (open).** Whether a change of the relative contact trajectory changes what an interface experiment measures.
- **C5 — A platform.** Interfaces are exchanged on one physical hand.

## Not claimed

Keeping contact trajectories at the MCP (a revolute joint: the proximal pad's path relative to the palm still changes with the pad's thickness); a hand that manipulates better than others; a best contact interface; that the CFL beats every remote-actuated design under every condition; that keeping the relative contact trajectory preserves a policy's performance; simulated compliant or tactile interfaces; physical accuracy of a contact model without validation.
