# idea.md — CoDe Hand: designing a finger around its contact interfaces

Sources (synced with the SSoT on 2026-11-21, commit 6be3d93): `design/SSoT/north-star.md` (problem,
contributions C1–C5), `design/SSoT/research-objective.md` (§6 claims, §8 Study 3, §9 status),
`design/SSoT/problem_formulation.md` A.3–A.6, `report/study0/derivation.md`.
Study 3 enters only as hypothesis H3, stated as an open measurement problem (research-objective §8.1). Its design,
the frozen-policy protocol and the regrasp-geometry numbers are in the playground (`docs/experimental/`) and must
NOT be used in the paper.

Why the relative contact trajectory (north-star): (1) the object meets the contact faces, not the skeleton;
(2) a redesign does not keep it unless it is the target (Study 1's two independent 7 mm CFL designs at 120°:
1.28 and 3.91 mm vs 0.86 mm for the Study 2 redesign); (3) it can be kept at little cost (C1, C3).
Transmission (decided): compensated by control and recorded — flexion commands through each hand's θ(q) table with
gains K_θ = K_q/(dθ/dq)² hold flexion within 0.07° in simulation (MuJoCo-Warp, torque limits off); on hardware,
whether torque limits allow it where MA_wc falls (up to −18 %) is open.

Contributions (north-star): C1 a bound (Study 0); C2 a finger (Study 1); C3 redesign that keeps contact
(Study 2); C4 why it matters (open; Study 3, H3); C5 a platform (hardware, H4).

## 1. North star and the message

CoDe Hand is a contact-centric robotic hand: it is designed around its contact interfaces instead of
treating them as secondary components fitted into an existing hand architecture. It is NOT an attempt to
build a hand with more DoF, more strength or universally better manipulation. **CoDe is not intended to
make every contact interface better; it is intended to make contact interfaces easier to study.**

The contribution is the *validity of contact-interface measurements*, not the discovery that contact
matters (that a grasp depends on its contact interface is expected). CoDe enables controlled measurements
of contact-interface effects while minimizing confounding changes in the underlying morphology and
articulation kinematics.

## 2. The problem

1. **Limited space for functional interfaces.** Useful interfaces are physically large: thick compliant
   structures, integrated tactile modules (camera-based sensors with optics), sensorized surfaces,
   specialized geometries. On a conventional finger they compete with servo bodies, joint housings and
   transmissions for the same volume, especially along the proximal phalanx and around the joint.
2. **High redesign cost.** A different interface often requires changing phalanx dimensions, joint
   locations, actuator placement, linkage geometry: a mechanical redesign, not a module swap.
3. **Confounding (the key research consequence).** In an experiment comparing two interfaces, changes are
   of two kinds: *intended variation* caused by the interface (contact stiffness, friction, contact
   geometry, tactile observations, interface-dependent force transmission) and *unintended embodiment
   variation* caused by redesigning the hand to carry the interface (finger length, joint location,
   linkage geometry, actuation). CoDe does not remove the first; it controls the second so that the first
   can be studied.

Central question: how can a hand be designed so that contact interfaces can be varied and evaluated
systematically while its underlying morphology changes as little as possible?
Supporting questions answered in this paper: (Q1) how does hand architecture constrain the space and
geometry available to interfaces? (Q2) how much must the morphology change to accommodate a different
interface, and how much does its relative contact trajectory change? (Q3) does keeping that trajectory
change what an interface experiment measures?

## 3. Key concept: the relative contact trajectory

Robot configuration R = (H, C): underlying hand H = (E, Q, A) (structure, articulation geometry,
actuation) fixed; contact interface C variable. The kinematic target of a contact-centric hand is the
**relative contact trajectory: the path of the distal contact face seen from the proximal contact face, as a
function of flexion q (not of time)**. It excludes the transmission from servo angle to flexion, which is
recorded separately. (This term replaced "contact-centric kinematics" on 2026-11-21; do not use the old
term.) What a policy and an object meet is the contact face, not the skeleton. When an interface is thicker
than the finger was designed for, the relative contact trajectory has priority over the skeleton's: the mechanism's
internal dimensions may change to keep the contact faces moving as before. With identical rigid-body
kinematics, a thicker interface moves the contact face away from the point the phalanx turns about and so
changes the contact-point Jacobian — an effect of the hand's geometry, not of the interface under study,
and exactly what a redesign should remove. Preserving the relative contact trajectory does NOT mean preserving
contact mechanics (compliance, friction, sensing change with the interface — that is what is studied).

## 4. The CoDe approach (engineering means, not ends)

Remote actuator placement; polycentric articulation via a **crossed four-bar linkage (CFL)** as the PIP
joint; mechanically independent, replaceable proximal and distal contact modules. No mechanism is assumed
superior without comparative evidence.

## 5. Formulation (2D side model of a 2-DoF finger)

- Morphology M = (K, C, A, S): mechanism, contact interface C_i = (ℓ_i, h_i, p_i) (length, thickness,
  mount), actuation (MCP and PIP actuators, housing shapes and poses, transmission Γ), structure (phalanx
  bodies supporting each pad over its full length). The interface dimensions are given first as requirements.
- Lengths at the extended pose q = 0, phalanx axes collinear; L_f = L_1 + L_vp + L_2. Posed contour length
  L_f(q) = L_1 + ‖r_2,start(q) − r_1,end‖ + L_2 (the inter-phalanx gap shrinks with flexion for moving-pole joints).
- Feasibility: static (assembles without overlap at q = 0) and motion (collision-free for q ∈ [0, q_req]);
  touching allowed; elements on other lateral layers (links, belts) not checked against pads. CFL also keeps
  transmission angle γ ∈ [30°, 150°].
- **Coverage (objective).** In pose q the two contact faces, extended, meet at X; a cylinder inscribed in the
  V touches both faces at the same distance t from X. T_i = distances t ≥ 0 at which pad i's (unoccluded)
  face lies; T = T_1 ∩ T_2. C_pad(q) = 2|T| / L_f(q). C̄_pad = mean over Q_eval = {90°, (90°+q_req)/2, q_req}.
  Pad portions with no partner, or on the closed side of X, do not count. Numerator and denominator in the
  same pose (credits a gap that shrinks; charges a contour that lengthens).
- **Architectures compared** (fair: each at its own best design under one common goal — L_f = 90 mm, same
  actuators, pad requirements, collision rules; never fitted to another architecture's morphology):
  2R-DD (revolute PIP joint, actuator at the joint), 2R-remote (same joint, belt-driven from an actuator on
  the proximal phalanx, fixed pose 10 mm proximal/15 mm dorsal, tilt 15°), RCJ (rolling contact joint, two
  equal rollers), CFL (CoDe's crossed four-bar, pivots A, A' proximal, B, B' distal, driven at A').
  Idealized CFL ideal / RCJ ideal (no joint-actuator body) are declared upper bounds, reported apart. In the
  studies MA is reported, not constrained (2R ≡ 1, RCJ ≡ 1/2); occlusion not counted.

## 6. Study 0 — geometry of the joint–pad conflict (analysis; the paper's method/insight)

Coordinates fixed to proximal phalanx; x along the link line (through joint axes at q = 0), z dorsal. Pad i
at mount depth d_i, face depth t_i = d_i + h_i. Joint gap G = x_2s − x_1e (finger length neither pad can
occupy): (ℓ_1 + ℓ_2)/L_f = 1 − (x_0 + G)/L_f.

- **Lemma (finite pole).** For q ∈ (0, 2π) the displacement y ↦ R_q y + b of the distal phalanx is a rotation
  by q about the unique point P(q) = (I − R_q)^{-1} b = ½(b − cot(q/2) J b). Computed from the fingertip path
  alone (b = r_C(q) − R_q r_C(0)), so it applies to every architecture. Finite pole ≠ instantaneous center
  (coincide only for a fixed axis). z(q): pole depth below the link line; τ_i = t_i − z(q).
- **Prop. 1 (exact clearance).** Two rectangular pads are disjoint in pose q iff their projections separate
  on one of four axes {e_x, e_z, R_q e_x, R_q e_z} (separating axis theorem). Design condition:
  min_{0<q≤q_req} m(q) ≥ 0. Used to judge designs.
- **Prop. 2 (symmetric clearance law).** If both faces are equally deep (S1) and the joint-side lower corners
  stay equidistant from the pole in x (S2), the pads stay disjoint over (0, q_req] iff the gap is at least
  G_sym = max_{0<q≤q_req} 2[d + h − z(q)]_+ tan(q/2). Proof: about the pole, the rotated distal pad is the
  mirror image of the proximal pad in the line with normal (cos q/2, −sin q/2); a convex set and its mirror
  image are disjoint iff it stays in one half-plane; the joint-side corner crosses first. Critical flexion q*
  = argmax. G_sym convex in h, slope 2 tan(q*/2). Fixed axis: z ≡ 0, q* = q_req, gap grows by 2.00 / 3.46 /
  7.46 mm per mm thickness at q_req 90/120/150°. RCJ: z = r tan(q/4). CFL: z(q), P_x(q) set by linkage.
- **Prop. 3 (coverage).** Under S1, S2, equal pads:
  C_pad(q) = (L_f − x_0 − G) / (L_f − G(1 − cos(q/2)) + 2 z(q) sin(q/2)). Large fixed-axis gap shortens when
  flexed; palmar pole lengthens the contour. Lowering a constant-depth pole raises C_pad until z = t (G = 0):
  the best joint turns about a point on the contact face.
- **Prop. 4 (trajectory law).** p(q) = pole relative to the proximal face; a distal-face point
  moves as c(q) = p + R_q(c_0 − p). Two fingers whose poles differ by Δp: ‖Δc(q)‖ = 2 sin(q/2) ‖Δp(q)‖.
  Contact faces move alike iff the pole keeps its place relative to the face; then the same pad fits in the
  same place → keeping the relative contact trajectory and keeping the pads are one condition.
- **Corollary (fixed-axis bound).** Redesigning a fixed-axis finger so the faces move Δh − Δd away from the
  axis: max_q ‖Δc‖ ≥ 2 sin(q_req/2)|Δh − Δd| wherever the new axis is (equality if it does not move along the
  face). Same for rolling joint with fixed roller radius.
- Predictions: (H1-empirical) at a coverage optimum pad–pad contact is often active; (P2) C̄_pad reproduced
  from pole path + pads; (P3) fixed axes lose pad length as tan(q_req/2), a descending pole need not; (P4)
  under redesign only a joint that moves its pole can beat the bound.

## 7. Studies 1–2 (computational validation)

- **Study 1** (claims 1–2 of research-objective §6): for each architecture, h ∈ {3,5,7,9} mm, q_req ∈
  {90,120,150}°, maximize C̄_pad (pad lengths and positions are design variables). GA + NSGA-II, 3 seeds,
  CFL re-searched from neighbouring thicknesses and Study 2 designs; best of runs.
- **No spacers (decided 2026-11-21, excluded by requirement).** Every interface sits at its own shallowest
  depth. Raising a thinner pad on a spacer to the thickest pad's depth keeps D_c = 0 on any architecture, but
  fills the space the interface needs with a passive block, leaves every thinner interface the coverage of
  the deep design (120°, 3 mm on the 9 mm design: 8.6 points lost for CFL, 18.2 for 2R-remote), and fixes the
  thickest interface in advance. It is not a compared alternative.
- **Study 2** (claim 3): reference M_0 = each architecture's 3 mm Study 1 design. Trajectory change
  D_c(M, M_0) = max_{q≤q_req} ‖c(M,q) − c(M_0,q)‖ / L_f (c = path of the distal-face point below the fingertip
  seen from the proximal face); skeleton change D_s = same for fingertip. For 5/7/9 mm minimize D_c and maximize
  C̄_pad (NSGA-II, 2 seeds, no coverage floor); compare with fixed-axis bound 2 sin(q_req/2)(h − h_0).

## 8. Hypotheses for the paper (RSS Stage 1: hypotheses + methods complete, experiments may be preliminary)

- H1: Architecture bounds the usable contact surface; the spread grows with thickness and required flexion.
  (Study 1.) If false: interface volume is limited by packaging, not the joint.
- H2: Only a joint that moves its pole can be redesigned for a thicker interface while keeping the
  relative contact trajectory and the coverage. (Study 2 vs. Corollary.) If false: every interface
  comparison must report D_c as a covariate.
- H3 (C4, open; research-objective §8.1): keeping the relative contact trajectory keeps an interface measurement
  from depending on how the hand was adapted. Interface variable c from c₀ to c₁ on hand H, task measure M,
  effect Δ(H) = M(H,c₁) − M(H,c₀); decoupled when E(H) = |Δ(H) − Δ(H₀)| is small against Δ(H₀). A test must:
  (i) remove by control what control can undo (transmission) and declare what is not kept (MCP level);
  (ii) same task and controller on every hand, commands and stiffness in flexion, state whether command or
  contact force is held fixed; (iii) no re-training/re-tuning per hand (a policy moved between hands measures
  transfer); (iv) report E(H) vs Δ(H₀) with the raw between-hand difference at one interface value as control;
  (v) an interface change with a clearly non-zero effect on H₀. Design not settled; no result claimed.
- H4 (C5, Stage 2, hardware, criterion 4 of research-objective §5.7): interfaces can be exchanged on one physical
  CoDe hand with the same actuation and procedure, keeping the predicted motion. Planned measures: what an
  exchange touches (parts, time, whether the articulation is disassembled); reached flexion vs. model;
  contact-face motion tracked on the pads vs. model D_c; a common grasp protocol run on every interface;
  2–3 representative physical interfaces spanning the thickness range. Interfaces thicker than the design
  depth use a redesigned linkage, reported separately from the interface. If false: integration cost and
  what had to change are the result (planar feasibility is not buildability). No hardware results exist yet;
  prototype specs are not verified — do not state any.

## 9. Claim discipline (must be respected in writing)

Report negative results (2R-remote leads at 90°; unchanged fingers accept no thicker pad; CFL preservation
partial at 9 mm; CFL search hard). Candidate designs are design alternatives, not replicates; "none found"
is about the search. Planar feasibility is not buildability. Matched morphology reduces confounding; it
does not establish perfect causal isolation. Never claim universal superiority of CFL. Results are within
the 2D model, equal-thickness rigid pads, fixed L_f and the stated coverage definition. Occlusion and an MA
floor were not imposed. No learning results exist; H3 is an open problem, not a result. Do not claim that keeping the trajectory preserves policy performance.
Limitation (north-star Not claimed): the MCP level is not kept — a revolute MCP is a fixed axis, so the proximal
face's path relative to the palm changes with pad thickness on every redesign (C1 applies there).
