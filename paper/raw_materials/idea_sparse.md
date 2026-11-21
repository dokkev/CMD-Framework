# Idea (sparse)

Source of truth: `design/SSoT/research-objective.md`, `design/SSoT/problem_formulation.md` (A.6),
`report/study0/derivation.md`. Scope of this paper: Studies 0–2 only (no learning results).

## Problem

Functional contact interfaces (thick compliant pads, vision-based tactile modules, sensorized skins) need
physical volume, especially on the proximal phalanx and around the joint. On a conventional finger that
volume competes with the joint actuator, joint housing and transmission. Changing the interface therefore
often forces a finger redesign (link lengths, joint location, actuator placement). In an experiment that
compares two interfaces, that redesign is a confound: the measured difference mixes the intended
interface effect (geometry, compliance, friction, sensing) with an unintended embodiment change.

## Claim

Contact-interface measurements are only as valid as the embodiment is held fixed. We treat the contact
interface as a first-class design variable and ask which joint architectures can (1) leave room for
thick interfaces at deep flexion and (2) be redesigned for a thicker interface while keeping the
*contact-centric kinematics*: the motion of one contact face seen from the other, which is what an
object and a policy actually meet.

## Approach

1. **Formulation.** 2D side model of a 2-DoF finger, M = (K, C, A, S): mechanism, contact interface,
   actuation, structure. Objective: cylinder-based pad coverage C̄_pad, the share of the posed finger
   contour on which both pads can jointly hold a cylinder, averaged over flexions {90°, mid, q_req}.
   Constraints: self-collision-free assembly and motion up to required flexion q_req, transmission angle
   (four-bar), finger length L_f = 90 mm.
2. **Study 0 (analysis).** Planar finite displacement pole P(q). Exact pad clearance for any pole
   (separating axes); closed-form symmetric clearance law G_sym = max_q 2[d+h−z(q)]₊ tan(q/2);
   closed-form coverage; contact-centric kinematics identity ‖Δc(q)‖ = 2 sin(q/2)‖Δp(q)‖, giving a
   lower bound 2 sin(q_req/2)|Δh−Δd| on how much any fixed-axis redesign must change the contact faces'
   motion. Architectures enter only through the pole depth z(q), pole x-position P_x(q), and mount depth d.
3. **Study 1 (optimization).** For each architecture (CFL = crossed four-bar linkage used by CoDe,
   2R-DD = servo at joint, 2R-remote = belt-driven revolute, RCJ = rolling contact joint; plus
   servo-free "ideal" upper bounds for CFL and RCJ), maximize C̄_pad per pad thickness
   h ∈ {3,5,7,9} mm × q_req ∈ {90,120,150}°. Each architecture at its own best design (no shared
   morphology). Checks Study 0's predictions.
4. **Study 2 (redesign).** From each architecture's 3 mm design, redesign for 5/7/9 mm with two
   objectives: minimize contact-path change D_c (max distance of distal-face path seen from proximal
   face, normalized by L_f) and maximize C̄_pad. Compare with the fixed-axis lower bound.

## Expected message

- Architecture bounds the usable contact surface, and the gap grows with thickness and flexion. A fixed
  axis loses pad length at 2 tan(q_req/2) per mm of thickness; a moving pole (four-bar) need not.
- A fixed axis cannot keep the contact-centric kinematics under a thicker interface (closed-form bound,
  met exactly by 2R-remote). The four-bar keeps it to 10–51 % of that bound while keeping coverage at the
  best found for that thickness. The skeleton's motion changes instead: the internal mechanism absorbs the
  interface.
- Honest limits: 2D model; "not found" is about the search; 2R-remote wins at 90° flexion; fixed
  finger + thicker pad helps no architecture; four-bar absorption is partial at 9 mm.

## Naming

- CoDe = the hand/platform; CFL = its crossed four-bar finger joint in the studies. 2R-DD / 2R-remote /
  RCJ as above. "proximal pad", "distal pad".
- Hardware demonstration of interface exchange (criterion 4) is NOT claimed in this paper.
