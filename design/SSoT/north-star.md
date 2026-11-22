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

CoDe is a servo-driven hand on which contact interfaces of different size and shape can be exchanged while its contact surfaces keep moving relative to each other as before (the finger's contact Jacobian, seen in the proximal-phalanx frame), so that contact morphology can be studied as an independent variable in manipulation.

## The problem

Contact is central to dexterous manipulation, and the contact interface (its geometry, size, compliance and sensing) sets that contact. Research hands are built around servos for maintainability, and servo bodies and joints occupy the space a contact interface would need.

- **P1 — Space.** Servos and joints compete with the contact interface for the same space, so a large or thick interface costs either range of motion or contact surface.
- **P2 — Redesign.** Redesigning a finger for a different interface changes how its contact surfaces move relative to each other. For a joint turning about a fixed axis this is unavoidable when the interface sits at its own best depth.
- **P3 — Measurement.** So when interfaces are compared in manipulation, a difference cannot be attributed to the interface alone: the hand's motion changed with it.

## What is kept, and in which frame

In the proximal-phalanx frame (it turns with the proximal phalanx at the MCP), take the point under the fingertip on the distal contact surface. Its path over flexion, and its derivative in flexion together with the distal surface's rotation, describe how the contact surfaces move relative to each other; the derivative is the PIP column of the finger's contact Jacobian. The rotation part is the same on every design (the two surfaces turn by the flexion), so a design differs only in where the distal surface turns about (`problem_formulation.md` A.6.3). The MCP column, whose lever includes the pad depth, is not kept. The transmission from servo angle to flexion is not part of it: it is compensated by control.

## Why keep how the contact surfaces move

The question every result must answer: why should a redesign keep how the contact surfaces move, rather than the skeleton's motion or nothing at all?

1. **The object meets the contact surfaces, not the skeleton.** To measure an interface's effect, everything the object meets except the interface should stay the same; for contact on more than one phalanx that is the relative pose of the contact surfaces over flexion.
2. **A redesign does not keep it unless it is the target.** Redesigned for coverage alone, a finger's contact-surface motion changes by more than a fixed axis would force, and by an amount that depends on the search.
3. **It can be kept, at little cost.** A polycentric joint keeps it far inside the fixed-axis bound at near-best contact surface; a fixed axis cannot (C1, C3).
4. **Its change is what corrupts an interface measurement, not the skeleton's change.** When the same joint-position commands are reused across interfaces, the error in a measured interface effect follows the change in contact-surface motion, across pad thicknesses, and is not predicted by how much the skeleton changed (C4, shown in simulation).

## What CoDe solves

- **Remote actuation:** the servo leaves the joint and drives it through a linkage, freeing the space under the joints for the interface. → P1
- **Polycentric articulation (the CFL):** the point the distal phalanx turns about moves with flexion, so a thick interface can sit under the joint without giving up range of motion or contact surface. → P1, P2
- **Contact-surface motion as the redesign target:** when a new interface asks for a redesign, the finger is re-optimized to keep how its contact surfaces move, not how its skeleton moves. → P2, P3

## Contributions

A contribution is a claim to be tested, not a result to be secured.

- **C1 — A bound.** A fixed-axis joint cannot keep its contact-surface motion when an interface changes depth; the change has a closed-form lower bound.
- **C2 — A finger.** A remote-actuated polycentric finger (the CFL) that keeps more contact surface than a servo-at-joint finger as interfaces thicken and the required flexion deepens.
- **C3 — Redesign that keeps contact.** Re-optimized for a new interface, the CFL keeps its contact-surface motion far closer than the fixed-axis bound allows, at near-best contact surface.
- **C4 — Why it matters.** With reused joint-position commands, the error in a measured interface effect follows the change in contact-surface motion and not the skeleton's change; the dependence weakens as the actuation stiffness falls and is small under contact-force feedback.
- **C5 — A platform.** Interfaces are exchanged on one physical hand.

## Not claimed

Keeping the MCP column of the contact Jacobian (a revolute joint: the proximal pad's path relative to the palm still changes with the pad's thickness); a hand that manipulates better than others; a best contact interface; that the CFL beats every remote-actuated design under every condition; that keeping the contact-surface motion preserves a policy's performance; that it matters equally under every control protocol; that more skeleton change is better (only that skeleton change does not predict the error); simulated compliant or tactile interfaces; physical accuracy of a contact model without validation.
