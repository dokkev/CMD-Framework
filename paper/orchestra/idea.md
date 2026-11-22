# idea.md — CoDe Hand: designing a finger around its contact interfaces

Sources (synced 2026-11-22 with the SSoT after the Study 3 update): `design/SSoT/north-star.md`,
`design/SSoT/research-objective.md` (§6 claims, §8 Study 3, §9 status), `design/SSoT/problem_formulation.md`
A.3–A.6, `report/study0/derivation.md`, and for Study 3 `docs/experimental/separability.md` (experiments 3–8,
pre-registered gates and amendments) with the logs in `sim/isaaclab/logs/experimental/two_pad/`.
Study 3 is now a simulation result (C2.3), with its scope stated: reused joint-position (impedance) commands,
CFL, q_req 120°, 3→5/7/9 mm, a free cylinder held by two pads, rigid contact (MuJoCo-Warp).

Story and contributions (north-star, rewritten 2026-11-22; the north star fixes the paper's story). Naming: P
problems, C contributions, H a study's hypotheses, R its safe claims (research-objective §6).
- P1 No room beyond the fingertip: research hands are mostly chains of servos at the joints (easy to build,
  model, simulate; tendon/linkage hands free the joints but pay in modelling). A proximal pad beside a servo
  must be thin or cost range of motion; each new interface is a redesign; interfaces and manipulation gather
  at the fingertip, and the two reinforce each other (an observation, not a claim).
- P2 The redesign becomes a second variable: with one protocol reused across hands, a measured difference
  mixes the interface with the finger's kinematics; keeping the skeleton does not help (the object meets the
  contact surfaces).
- C1 A finger with room for thick pads (remote actuation + CFL): more contact surface than a servo-at-joint
  finger at every thickness and q_req (Study 0 clearance/coverage laws, Study 1). Fails if 2R-DD matches it
  in any condition; 2R-remote matches at shallow flexion but pays in C2.
- C2 A redesign that keeps the contact Jacobian, not the skeleton: (1) a fixed axis cannot (bound, sum rule;
  Study 0), (2) the CFL can, within 10–51 % of the bound, at almost no cost in coverage (Study 2: what keeping
  it costs), (3) keeping it lets the interface be studied independently of the finger (Study 3: what keeping
  it buys; simulation; under the reused impedance setpoint a servo hand runs, the part of a measured effect the
  redesign adds follows Δ_contact; the fixed-axis family that keeps the skeleton errs 95–228 %). Fails if the
  CFL reaches the bound or, under those commands, another quantity predicts the added part as consistently.
  Framing (2026-11-22): the paper's question is whether an interface can be studied independently of the
  finger that carries it; Study 3's table is grouped by what a protocol reuses (setpoint: Δ_contact alone;
  ideal torque: Δ_contact at fixed force and Δ_skel through the force; touch: Δ_skel).
- Hardware (Stage 2) checks both: exchange cost (C1); tracked Δ_contact and R vs U on the bench (C2).
- Paper order: Introduction (context, P1, P2, C1, C2; no pole) → Design (the pole answers both: room → C1,
  motion → C2) → C1 evidence (Study 1) → C2 evidence (Study 2, Study 3) → Conclusion (Stage 2, big-picture
  limitations).
Transmission (decided): compensated by control and recorded — flexion commands through each hand's θ(q) table with
gains K_θ = K_q/(dθ/dq)² hold flexion within 0.07° in simulation (MuJoCo-Warp, torque limits off); on hardware,
whether torque limits allow it where MA_wc falls (up to −18 %) is open.

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
interface, and how much does its contact-surface motion change? (Q3) does keeping that motion
change what an interface experiment measures?

## 3. Key concept: how the contact surfaces move (the finger's contact Jacobian)

Robot configuration R = (H, C): underlying hand H = (E, Q, A) (structure, articulation geometry, actuation)
fixed; contact interface C variable. The kinematic target of a contact-centric hand is **how the distal contact
surface moves relative to the proximal one as the finger flexes**. Write it with an explicit frame, not a coined
term: in the **proximal-phalanx frame** F_p (it turns with the proximal phalanx at the MCP; origin on the proximal
contact face), c(q) is the point under the fingertip on the distal contact face, and its derivative in flexion
J(q) = d/dq [c(q); φ(q)] = [c′(q); 1] is the PIP column of the finger's contact Jacobian (rotation part 1 on
every design, Proposition 4; c′ carries the pole of the relative motion). Cite the standard contact Jacobian
(Montana 1988; Murray, Li, Sastry 1994). The standard one is taken at the moving contact point, but since φ = q the
PIP column at any distal-face point x is [R_{π/2}(x − p̂(q)); 1] (p̂: instantaneous centre in F_p), so wherever
the object touches the distal face the pole path sets the column; c stands for any such point, and the choice of
point and origin enters only the start offset Δc(0). Say "one path sets it" (the pole path), not "one point". D_c is the position-level change of
c over [0, q_req] (≈ the integral of the Jacobian change, since the q = 0 pose barely differs). The MCP column
(lever includes the pad depth) is NOT kept — the stated limitation. The transmission (servo angle → flexion) is
excluded, recorded and compensated by control. Keeping the contact Jacobian does NOT keep the contact mechanics
(compliance, friction, sensing change with the interface — that is what is studied). Do not write "relative
contact trajectory" or "RCT" in the paper; "contact-surface motion" for the quantity, "skeleton motion" for the
fingertip path. Symbols (SSoT 2026-11-22): Δ_contact (was D_c) and Δ_skel (was D_s), both in mm.

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
  same place → keeping the contact-surface motion and keeping the pads are one condition.
- **Corollary (fixed-axis bound).** Redesigning a fixed-axis finger so the faces move Δh − Δd away from the
  axis: max_q ‖Δc‖ ≥ 2 sin(q_req/2)|Δh − Δd| wherever the new axis is (equality if it does not move along the
  face). Same for rolling joint with fixed roller radius.
- Predictions: (H0.1, empirical) at a coverage optimum pad–pad contact is often active; (H0.2) C̄_pad reproduced
  from pole path + pads; (H0.3) fixed axes lose pad length as tan(q_req/2), a descending pole need not; (H0.4)
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

Naming (SSoT 2026-11-22): P problems and C contributions (north star); per study, H hypotheses and R safe claims
(research-objective §6). Study 0: R0.1 room laws, R0.2 motion laws, H0.1–H0.4 predictions. H1/R1 (Study 1) → C1;
H2/R2 (Study 2) → C2.2; H3/R3 (Study 3) → C2.3. The v2-era H1–H4 framing is superseded.
- C2.3 (Study 3, simulation): with reused position commands, the error in a measured interface effect follows
  the change in contact-surface motion, not the skeleton's change. Effect Δ(H) = F(H, c₁) − F(H, c₀) of an
  interface change (pad friction μ 1.3 → 2.0) on the load F a free cylinder held by two pads takes before the
  grasp gives way (4 load directions); error E(H) = |Δ(H) − Δ(O)| / |Δ(O)| against the reference hand O (3 mm),
  median over placements. Conditions: same flexion command (A, primary), oracle force matching (B, ground-truth
  contact force, an upper bound on compensation), joint-torque reuse (C), flexion-space impedance sweep (D).
  The test meets (i)–(v): transmission compensated, MCP declared; one evaluator, no re-tuning; raw between-hand
  differences reported; the friction effect on O is 1.2–2.7 N. Numbers in experimental_log.md (Study 3).
- Stage 2 (hardware, for C1 and C2; criterion 4 of research-objective §5.7): interfaces can be exchanged on one physical
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
floor were not imposed. No learning results exist. H3 is a simulation result with a scope: reused joint-position (impedance) commands; with reused joint torque the error shrinks 4–5× (still ordered by D_c); with oracle force feedback it is 3–7 %. Do not claim that keeping the contact surfaces' motion preserves policy performance, that it matters under every control protocol, or that more skeleton change is better (only that skeleton change does not predict the error). Rigid contact; the plateau of the impedance sweep is set by contact stiffness. Exploratory vs confirmatory: report the post-hoc amendments (impratio 1000, hand reselection, two added hands, symmetric metric) as such.
Limitation (north-star Not claimed): the MCP level is not kept — a revolute MCP is a fixed axis, so the proximal
face's path relative to the palm changes with pad thickness on every redesign (the fixed-axis bound, C2.1, applies there).
