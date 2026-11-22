# CoDe Hand — North Star

This is the top document of the SSoT. It fixes the paper's story and its contributions: the problem, what CoDe solves, the contributions claimed and the order in which the paper tells them. Concepts only; how they are shown is decided below it. Every other SSoT file and the manuscript follow it; when they disagree, this file wins.

## In one sentence

On servo-driven research hands a contact interface beyond the fingertip either does not fit, or, once the finger is redesigned to fit it, brings the hand's change into every measurement; CoDe makes room for thick proximal pads at the required flexion (C1) and redesigns the finger for a new interface by keeping its contact Jacobian rather than its skeleton (C2), so that the contact interface can be studied independently of the finger that carries it.

## The problem

Contact is central to dexterous manipulation, and the contact interface sets that contact. Many research hands are chains of servos at the joints, because such hands are easy to build, model and simulate.

- **P1 — No room beyond the fingertip.** Servos and joints occupy the space a contact interface needs. A proximal pad must be thin or cost range of motion, and every new interface is a redesign. Interfaces, and the manipulation studied with them, gather at the fingertip; the lack of room and the fingertip focus reinforce each other (an observation that motivates the work, not a claim).
- **P2 — The redesign becomes a second variable.** A finger redesigned for an interface changes with it, so a measured difference mixes the interface with the finger's kinematics. Keeping the skeleton does not help: the object meets the contact surfaces, not the skeleton.

## What CoDe does

- **Remote actuation:** the servo leaves the joint and drives it through a linkage, freeing the space under the joint. → P1
- **Polycentric articulation (the CFL):** the point the distal phalanx turns about can sit near the contact face, so a thick pad fits under the joint. → P1
- **The contact Jacobian as the redesign target:** a redesign keeps how the contact surfaces move relative to each other (the PIP column of the finger's contact Jacobian, seen from the proximal phalanx), not how the skeleton moves. → P2

## Contributions

Two contributions, one per problem. Each is a claim to be tested, with a stated way to fail, and each is checked on hardware in the next stage.

- **C1 — A finger with room for thick pads.** A remote-actuated CFL finger meets its required flexion and keeps more contact surface than a servo-at-joint finger at every pad thickness and required flexion. *Fails if* the servo-at-joint finger matches it in any condition.
- **C2 — A redesign that keeps the contact Jacobian.**
  1. A fixed axis cannot keep it when an interface changes depth, and the change has a lower bound.
  2. The CFL can keep it, well inside that bound, at near-best contact surface.
  3. Keeping it lets the interface be studied independently of the finger: under the commands a servo hand runs (a reused impedance setpoint), the part of a measured interface effect that a redesign adds follows the change in the contact Jacobian, not the skeleton's; the rest of the kinematics may change.

  *Fails if* the CFL reaches the bound, or if, under those commands, another design quantity predicts the added part as consistently.

## How the paper tells it

1. **Introduction:** the context, P1, P2, C1 and C2, without mathematics.
2. **Design:** one quantity, the point the distal phalanx turns about, answers both problems: where it lies sets the room for the pads (C1), and whether it keeps its place relative to the contact face sets the contact Jacobian (C2).
3. **Evidence for C1**, then **evidence for C2**: what keeping the contact Jacobian costs, then what it buys (the interface can be studied independently).
4. **Conclusion:** the hardware stage, and big-picture limitations with future work.

## Not claimed

Keeping the MCP column of the contact Jacobian; that interfaces gather at the fingertip because of the lack of room; interfaces of every shape and kind; a hand that manipulates better than others; a best contact interface; that the CFL beats every remote-actuated design; that keeping the contact Jacobian preserves a policy's performance or is the only thing to keep under every control protocol; exact independence (it holds within a tolerance); that more skeleton change is better; physical accuracy of a contact model without validation.
