# CoDe Hand — North Star

This is the top document of the SSoT. It fixes the paper's story and its contributions: the problem, what CoDe solves, the contributions claimed and the order in which the paper tells them. Every other SSoT file and the manuscript follow it; when they disagree, this file wins.

## In one sentence

On servo-driven research hands a contact interface beyond the fingertip either does not fit, or, once the finger is redesigned to fit it, brings the hand's change into every measurement; CoDe makes room for thick proximal pads at the required flexion (C1) and redesigns the finger for a new interface by keeping its contact Jacobian rather than its skeleton (C2), so that the contact interface can be studied as an independent variable.

## The problem

Contact is central to dexterous manipulation, and the contact interface (its geometry, size, compliance and sensing) sets that contact. Research hands are mostly chains of servos at the joints: they are easy to build, maintain, model and simulate. Tendon-driven and linkage-driven hands free the joints, but pay in modelling and simulation effort.

- **P1 — No room beyond the fingertip.** Servo bodies and joints occupy the space a contact interface needs, above all around the joint. A proximal pad beside a servo must be very thin or cost range of motion, and every new interface is a redesign with its own integration cost. Interfaces, and the manipulation studied with them, therefore gather at the fingertip, and the lack of room and the fingertip focus reinforce each other. This loop is the motivation, stated as an observation, not a claim.
- **P2 — The redesign becomes a second variable.** A finger redesigned to carry an interface changes its morphology with it. Interface studies often reuse one protocol across hands (scripted grasps, joint-space policies, the same actuator targets), so a measured difference mixes the interface's effect with the hand's change. Keeping the skeleton does not solve this: the object meets the contact surfaces, not the skeleton. For a joint turning about a fixed axis, a thicker interface at its own depth changes how the contact surfaces move whatever the redesign.

## What CoDe does

- **Remote actuation:** the servo leaves the joint and drives it through a rigid linkage. The hand keeps the servo's position control and a rigid-body model, and frees the space under the joints. → P1
- **Polycentric articulation (the CFL):** the point the distal phalanx turns about moves with flexion and can lie palmar of the joint hardware, near the contact face, so a thick pad fits under the joint. → P1
- **The contact Jacobian as the redesign target:** when a new interface asks for a redesign, the finger is re-optimized to keep how its contact surfaces move relative to each other, not how its skeleton moves. → P2

## What is kept, and in which frame

In the proximal-phalanx frame (it turns with the proximal phalanx at the MCP), take the point under the fingertip on the distal contact surface. Its path over flexion, and its derivative in flexion together with the distal surface's rotation, describe how the contact surfaces move relative to each other; the derivative is the PIP column of the finger's contact Jacobian. The rotation part is the same on every design (the two surfaces turn by the flexion), so a design differs only in where the distal surface turns about, and the same holds at every point of the distal face where an object may touch. The MCP column, whose lever includes the pad depth, is not kept. The transmission from servo angle to flexion is not part of it: it is compensated by control.

## Contributions

Two contributions, one per problem. Each is a claim to be tested, not a result to be secured; each states how it fails and what the hardware stage checks.

- **C1 — A finger that carries thick proximal pads at the required flexion.** Remote actuation and a polycentric PIP joint (the CFL) let a finger meet its required flexion and keep more contact surface than a servo-at-joint finger, at every pad thickness and required flexion. Behind it are clearance and coverage laws for planar finger joints: the best joint turns about a point on the contact face, and such a point can only be a virtual pole. *Fails if* a servo-at-joint finger matches the CFL in any condition. A belt-driven fixed axis can match it at shallow flexion; it pays in C2. *Hardware:* interfaces are exchanged on one physical finger by swapping the pad modules and the CFL links, and the parts, time and recalibration are recorded.
- **C2 — A redesign that keeps the contact Jacobian.** When the finger is redesigned for a new interface, it keeps the PIP column of its contact Jacobian in the proximal-phalanx frame, not its skeleton. Three parts:
  1. *It cannot be kept by a fixed axis.* A fixed axis keeps it only if its contact faces keep their distance from the axis; otherwise the change has a closed-form lower bound, and the contact-surface motion and the skeleton motion cannot both be kept.
  2. *It can be kept.* Re-optimized for thicker pads, the CFL keeps it well inside that bound, at near-best contact surface.
  3. *It is what matters.* When one joint command or joint torque is reused across redesigns, the error in a measured interface effect follows its change, not the change in skeleton, coverage, thickness or transmission; a fixed-axis finger that keeps its skeleton exactly still distorts the effect (shown in simulation).

  *Fails if* the CFL reaches the bound, or if another design quantity predicts the error in every setting. *Hardware:* the contact-surface motion of the redesigned finger is tracked, and the redesign that keeps it is compared with one redesigned for coverage alone under the same command.

## How the paper tells it

1. **Introduction.** The context, P1, P2, then C1 and C2. No mathematics: the pole does not appear here.
2. **Design.** One quantity answers both problems: the pole about which the distal phalanx turns relative to the proximal one. First the space (clearance and coverage laws; the best joint turns about a point on the contact face, which must be a virtual pole, hence the CFL; C1). Then the motion (the contact Jacobian, the contact-motion law, the sum rule and the fixed-axis bound; C2.1).
3. **C1 evidence.** Study 1: best coverage per architecture, pad thickness and required flexion.
4. **C2 evidence.** Study 2: can the contact Jacobian be kept (C2.2)? Study 3: why keep it (C2.3)?
5. **Conclusion.** The hardware stage for both contributions, and big-picture limitations with future work.

## Not claimed

Keeping the MCP column of the contact Jacobian (a revolute joint: the proximal pad's path relative to the palm still changes with the pad's thickness); that interfaces gather at the fingertip because of the lack of room (motivation only); interfaces of every shape and kind (the evidence is pad thickness, with one pad shape in simulation; camera-based tactile modules are future work); a hand that manipulates better than others; a best contact interface; that the CFL beats every remote-actuated design under every condition; that keeping the contact Jacobian preserves a policy's performance; that it matters equally under every control protocol; that more skeleton change is better (only that skeleton change does not predict the error); simulated compliant or tactile interfaces; physical accuracy of a contact model without validation.
