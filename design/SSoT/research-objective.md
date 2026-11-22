# CoDe Hand — Research Objective

Experiment design: the principles, what each study must show, the discipline claims are held to, and the paper plan with each contribution's status (Section 9). The problem, what CoDe solves and the contributions are in [north-star.md](north-star.md), which takes precedence; the mathematics is in [problem_formulation.md](problem_formulation.md).

## 1. Purpose

CoDe is meant to be a research platform on which contact-interface experiments are possible and interpretable: researchers can study diverse contact morphologies if they wish, and can change only the contact interface while how its contact surfaces move relative to each other is kept. The problem it addresses, its approach and the contributions claimed are in [north-star.md](north-star.md); this file states how the studies test them.

It is not an attempt to build a hand with more degrees of freedom, higher grasping strength, or universally better manipulation performance.

---

## 2. Intended and Unintended Variation

When a manipulation experiment compares two contact interfaces, the observed change has two kinds of source:

- **Intended variation** comes from the contact interface itself: what happens on the contact face. Its geometry and profile, the contact's location, distribution and normals on the face, compliance, friction, tactile observations and interface-dependent force transmission. The contact-point Jacobian changes that follow from these (a contact elsewhere on the face, a different normal) are part of the effect under study.
- **Unintended embodiment variation** comes from how the hand was adapted to the interface: changes in finger length, joint location, linkage geometry, actuation or transmission, and above all a displacement of the contact faces relative to the articulation. A thicker interface on an unchanged joint moves its contact face away from the point the phalanx turns about, which changes how the contact surfaces move relative to each other, that is the finger's contact Jacobian. That change comes from the hand's geometry, not from the interface, and is what a redesign should remove.

The dividing line: a change on the contact face is the interface's; a change of where the contact face is, and how it moves, relative to the other contact face is the embodiment's. CoDe does not aim to remove the first. It aims to keep the second small, and to record what does change (Section 5.2), so that the first can be studied. The objective is to reduce embodiment confounding, not to claim perfect causal isolation.

---

## 3. Problem and Approach

See [north-star.md](north-star.md). Mechanisms are engineering means toward the research objective, not ends, and no mechanism is assumed superior without comparative evidence.

---

## 4. Supporting Questions

1. How does hand architecture constrain the available space and geometry of contact interfaces?
2. How much must the underlying hand change to accommodate different contact interfaces, and how much does its contact-surface motion change?
3. How do changes in contact-interface design influence physical interaction, tactile observations, and ultimately manipulation behavior?

The third question is the long-term research objective. The first two establish the hardware and experimental foundation required to investigate it.

---

## 5. Design Principles

### 5.1 Contact Interface First

Contact-interface requirements should influence the hand architecture from the beginning.

Do not treat contact interfaces as an afterthought that must fit into whatever space remains after actuator and joint placement.

### 5.2 Preserve the Underlying Embodiment

When comparing contact interfaces, maintain a common underlying hand morphology, actuation architecture, and kinematic configuration whenever practical.

Changes required to accommodate an interface must be recorded separately from the interface itself.

The objective is to reduce embodiment confounding, not to claim perfect causal isolation.

### 5.3 Modular and Mechanically Independent Interfaces

Proximal and distal contact interfaces should be independently configurable and replaceable.

Changing an interface should not unnecessarily require redesigning or disassembling the underlying articulation mechanism.

### 5.4 Functional Contact Structures

A contact interface is not limited to a passive friction pad.

It may include:

- Compliant structures.
- Tactile sensing modules.
- Sensor-integrated surfaces.
- Specialized contact geometries.
- Structures that influence contact-force transmission and deformation.

The platform should accommodate the physical requirements of these functional interfaces.

### 5.5 Experimental Accessibility

The hand should make controlled comparisons practical.

Researchers should be able to change contact-interface properties while keeping the underlying mechanical embodiment sufficiently consistent to interpret the resulting differences.

### 5.6 Operational Definition of Embodiment Invariance

CoDe aims to enable contact-interface variation while preserving the underlying robotic hand embodiment. In the studies' notation, the underlying hand H = (E, Q, A) is fixed and the contact interface C is variable. E is the structural morphology, Q the articulation geometry and A the actuation configuration.

For controlled contact-interface experiments, the following should remain fixed whenever possible:

- How the contact surfaces move relative to each other: in the proximal-phalanx frame, the path of the point under the fingertip on the distal contact surface over flexion and its derivative, the PIP column of the finger's contact Jacobian (its change is Δ_contact in mm, `problem_formulation.md` A.6.3). It excludes the transmission from servo angle to flexion, which is recorded separately. This is the kinematic target of a contact-centric hand; what a policy and an object meet is the contact surface, not the skeleton.
- Structural finger morphology and link geometry.
- Articulation kinematics and joint configuration.
- Actuator placement and transmission architecture.
- Contact-module mounting references.

When these cannot all be kept (an interface thicker than the finger was designed for asks for a redesign), the contact-surface motion has priority over the skeleton's: the mechanism's internal dimensions may change to keep the contact faces moving as before (`problem_formulation.md` A.6.0, A.6.3).

The following are intentionally variable:

- Contact-interface geometry and dimensions.
- Compliance, material properties and friction.
- Tactile sensing configuration.
- Contact location, distribution and surface response.

Preserving the contact-surface motion does not imply preserving contact mechanics or actuator-side contact responses: compliance, friction, sensing and the contact's location on the face change with the interface, which is precisely what the platform is designed to investigate. What should not change with them is where the contact faces are and how they move relative to each other (Section 2). Changes that a redesign does make (transmission from servo angle to flexion, mechanical advantage, the skeleton's motion) are recorded with the result.

The objective is to isolate contact-interface variation from unnecessary changes to the underlying hand embodiment, not to make contact behavior invariant.

### 5.7 Criteria for a Contact-Centric Research Platform

A successful contact-centric research platform should:

1. **Accommodate diverse interfaces.** Support meaningful variation in contact geometry, thickness, compliance and sensing hardware.
2. **Preserve the underlying embodiment.** Allow interface replacement without unnecessary modifications to finger structure, articulation or actuation.
3. **Preserve functional articulation.** Maintain the required range of motion and accessible contact surfaces across supported interface configurations.
4. **Enable controlled and repeatable experiments.** Allow different interfaces to be evaluated on the same physical hand with consistent actuation, sensing and experimental procedures.

These criteria define the platform's research utility. The first three are assessed computationally (Section 6; `problem_formulation.md` A.6). The fourth must be demonstrated on the physical hardware.

Mechanism compactness, transmission characteristics and contact-interface capacity are important evaluation metrics, but they are means of assessing these criteria rather than independent project objectives.

---

## 6. Evaluation Strategy

The project's experiments should provide evidence for the central research objective. This section states what must be shown; how each experiment is defined, searched and measured is in `problem_formulation.md` (A.6) alone.

**What the computational studies must show** (supporting questions 1 and 2):

1. **Architecture limits the contact interface.** Under one set of requirements (the same finger size, actuators, required flexion and collision rules), each architecture's best design leaves a different share of the finger usable for contact. Which architecture keeps more depends on the required flexion and the interface's thickness; the study reports the ranking per condition rather than one trend.
2. **The contact surface an architecture keeps depends on the interface's thickness and the required flexion.** With the finger optimized for each pad thickness and each required flexion, the best pad coverage differs by architecture, and so does how fast it falls as the pads get thicker or the flexion deeper.
3. **A redesign can keep the contact-surface motion far better than a fixed axis allows, with the contact surface kept too.** The contact-surface motion is what an interface experiment should hold; the skeleton's motion is a means to it. When a finger is redesigned for a thicker interface placed at that interface's best depth, a joint turning about a fixed axis changes the contact-surface motion by at least a closed-form lower bound, whatever the redesign; the CFL moves the point it turns about and changes it by a fraction of that bound, at a pad coverage near the best for that interface, within a range the study measures. Every interface is mounted at its own shallowest depth, without spacers. Raising a thinner interface on a spacer to the depth of the thickest one would keep the contact-surface motion on any architecture, but it fills the space the interface needs (the problem CoDe addresses) with a passive block, costs every thinner interface the coverage of the deep design (at 120°: 8.6 points for the CFL, 18.2 for 2R-remote, 3 mm against a 9 mm design), and fixes the thickest interface in advance. It is therefore excluded by requirement, not compared.

**What the learning-based studies must test** (supporting question 3; Section 8):

4. **Study 3:** with reused joint-position commands, the error in a measured interface effect follows the change in contact-surface motion and not the skeleton's change; how strongly depends on the actuation protocol (Section 8).

A geometric account (Study 0) gives the relation behind claims 1 to 3 (where the phalanx turns about, and how deep the pads sit, set what the joint takes from the pads); the optimization studies check its predictions.

**Conditions that make the comparison fair:**

- Every architecture is compared at its own best design under the common goal, never built to another architecture's morphology.
- The compared architectures are the CFL (CoDe's mechanism), 2R-DD, 2R-remote and RCJ. Idealized variants without the joint actuator are declared upper bounds and reported apart.
- A result is a statement within the 2D side model and the stated coverage definition. "Not found" is a statement about the search, not about existence.

**What must be shown on hardware:** that interfaces are in fact exchanged on one physical hand, with the same actuation and procedure (criterion 4, Section 5.7). The computational studies cannot show this.

Interface-effect experiments (question 3) should use matched or explicitly controlled morphologies.

The interface-measurement experiment (Study 3, simulation) is set out in Section 8.

---

## 7. Claim Discipline

- Keep physical CoDe, the physical alternatives and declared idealized ablations separate. An idealization removes only its named constraint.
- Report failures and negative results. Never tune objectives, constraints or comparison assumptions to make CoDe win.
- Candidate designs are design alternatives, not statistical replicates. A search that finds nothing is not proof that no design exists.
- Matched morphology reduces confounding; it does not establish perfect causal isolation.
- Separate demonstrated observations, model-derived findings, assumptions and hypotheses.
- Planar feasibility is not buildability.
- Quantitative figures must read saved results, with source commit or patch, configs, input artifacts and sampling provenance. Missing models or artifacts block their branch of work; do not invent measurements.
- The next useful research decision is one that improves controlled interface variation or exposes its limits, whether or not it favors CoDe.
- Fix the primary metric, the test and the interpretation criteria before the data that decide them exist, and record them in the experiment's note in `docs/experimental/` before the run. Never change a criterion after seeing its outcome; a failed validity condition is reported as such. A measurement detail changed after a validation run (e.g. a gate on the reference hand) is recorded with its reason and the data it was based on, and only before the data that decide the comparison exist.
- Use enough independent replicates (policy seeds) for the planned test to be able to reject (an exact sign-flip test over 5 seeds cannot reach p < 0.05).
- Verify what a metric measures before interpreting it (e.g. that a goal count counts rotation and not overlapping success regions).

---

## 8. Study 3: Does the Change in Contact-Surface Motion Change What an Interface Experiment Measures?

### 8.1 The problem: decoupling the contact interface from the morphology in manipulation evaluation

A manipulation evaluation measures a hand and its interface together. To compare interfaces, the hand must be adapted to each one (P1, P2), so the measured difference mixes the interface's effect (intended variation, Section 2) with the adaptation's (unintended variation). The problem Study 3 addresses is to obtain a measurement of an interface's effect on manipulation that does not depend on how the hand was adapted to carry it.

Stated as a measurement: an interface variable c (friction, surface profile, compliance, ...) is changed between two values on a hand H, and a task measure M gives the effect Δ(H) = M(H, c₁) − M(H, c₀). The effect is decoupled from the morphology when Δ(H) is the same on the reference hand H₀ and on the hands adapted from it for a different interface, so that the embodiment-dependent error E(H) = |Δ(H) − Δ(H₀)| is small against Δ(H₀).

What Study 3 must resolve:

1. **Which parts of an adaptation remain.** An adaptation changes the transmission (servo angle to flexion), the skeleton's motion, the contact-surface motion, and the proximal contact surface's place at the MCP. The error that matters is what is left after control has undone what it can; the transmission is undone by control (Section 9, decided), the MCP level is a declared limitation (Section 8.3).
2. **A protocol in which the interface is the variable.** The same task and the same controller on every hand, with commands in flexion and stiffness matched in flexion. What is held fixed between hands (the command, the torque or the contact force) is stated, because an interface's effect can be measured differently under each.
3. **No adaptation in the evaluator.** A controller that is re-trained or re-tuned per hand absorbs the morphology into itself, and a policy trained on one hand and tested on another measures transfer, not the interface. The evaluator is fixed across hands.
4. **A measure and a control.** E(H) is reported against Δ(H₀), together with the raw difference between hands at one interface value (what an identical surface profile shows: the artefact the adaptation alone produces).
5. **A real effect to measure.** The interface change must have an effect on H₀ that is clearly non-zero in the task; a decoupling result on an effect near zero is not informative.

Study 3 answers one question, for contribution C4 (`north-star.md`):

> **When the same interface change is measured on hands adapted from H₀ for a thicker interface, does the error E(H) follow the change in contact-surface motion Δ_contact, or the skeleton's change Δ_skel?**

### 8.2 Design and result (simulation)

- **Task and measure.** A cylinder free in the sagittal plane, held by the proximal and distal pads of the CoDe hand model's fingers (MCP held), is loaded in four directions until the grasp gives way; the interface change is the pads' friction (μ 1.3 → 2.0), and Δ(H) is the change in that load. Placements, gates, thresholds and amendments are pre-registered in `docs/experimental/separability.md` (experiments 3–8).
- **Hands.** CFL, q_req 120°. H₀ is the 3 mm design; for 3 → 5, 7 and 9 mm, the Study 2 redesign, Study 1's independent design and redesigns chosen on design metrics alone so that Δ_contact, Δ_skel and coverage vary apart (18 hands). The unchanged finger with thicker pads (the naive swap) is mechanically infeasible at every thickness (its pads collide at 47° of flexion).
- **Protocols.** Reused flexion command at the same flexion stiffness (A); the same flexion torque (C); a flexion-space impedance between the two (D); the distal contact force matched to H₀'s with the simulator's contact force (B, an upper bound on compensation).
- **Why the two path changes differ.** Δ_contact and Δ_skel are the change of the same pole of the contact Jacobian, seen from the proximal contact face and from the link line (`problem_formulation.md` A.6.3); keeping the pole relative to the face (the Study 2 redesign moves it down with a deeper face) keeps the contact-surface motion and changes the skeleton's.
- **Result.** Under A, over the 15 hands that hold the cylinder, E follows Δ_contact (Spearman ρ = 0.83, one-sided p = 0.0001; positive at each thickness) and the skeleton's change does not predict it; the Study 2 redesign has the smallest error at each thickness (3, 21, 41% at 5, 7, 9 mm) and the coverage-only design 52–124%. Three redesigns whose distal face retreats 1.5–2.6 mm from the placed cylinder cannot hold it under the same command. The error enters through the contact force: under C it is 3–40% (still ordered by Δ_contact, ρ = 0.75), under B 3–7%; under D the redesign stays at 14–26% at every stiffness while the coverage-only design rises from 44% at 0.1 of the nominal stiffness to 119% at the nominal stiffness and levels off (125, 131%; 7 mm hands), where the contact's own compliance takes the interference. The conclusion (the Study 2 redesign errs less than the coverage-only design, pair by pair) holds when the contact stiffness is changed tenfold (ke 1e3 and 1e4 N/m), the friction impedance ratio of the solver threefold either way, and the substeps doubled (one-sided Wilcoxon p ≤ 3×10⁻⁴ in each); the size of the coverage-only design's error moves between 127 and 184%. For pad compliance (contact stiffness 2.5e3 → 1e3 N/m, μ 1.3), the error under A follows Δ_contact over 14 holding hands (ρ = 0.80, one-sided p = 0.0004; partial 0.91 given Δ_skel). When each finger instead closes 5° past its own touch (E; no shared command), all 18 hands hold, the error falls to 6–61% (median about 20%), and what remains follows Δ_skel (ρ = 0.88; partial 0.90 given Δ_contact): the geometry an experiment must keep depends on what it reuses. Under reused joint commands it is the contact-surface motion. With the cylinder placed on a grid chosen independently of any hand (4 places × 7 commands per radius), all 45 redesigns (the 18 above and 27 more at 7 mm) hold it and the error still follows Δ_contact (ρ = 0.60 [0.36, 0.76], one-sided p < 10⁻⁵; partial 0.58 given Δ_skel), so the ordering is not an artefact of placing the object where H₀ touches it. On this grid a large skeleton change also raises the error by another route: the fingertip moves off the grid's placements, which then lose a pad (ρ = 0.86 between Δ_skel and the share lost); over the 26 redesigns that keep at least 80% of the placements, Δ_contact predicts the error (ρ = 0.48 [0.10, 0.78]) and Δ_skel does not (−0.04). Among five candidate invariants a designer might hold (Δ_contact, Δ_skel, coverage, pad thickness, transmission change), only Δ_contact predicts the error under A for both friction and stiffness (hand-bootstrap 95% intervals of all others contain zero; `invariants.py`), and only Δ_contact's interval excludes zero in every setting measured. For a third interface property, pad shape (flat to 2 mm crowned, μ 1.3), the error follows Δ_contact in the same order over six redesigns (ρ = 0.94, exact one-sided p = 0.008); few placements hold under the crowned pad (1–14 per hand), and in that set Δ_skel moves opposite to Δ_contact, so it repeats the ordering but does not by itself separate the two.

### 8.3 Claim boundaries

- The result holds in simulation (Isaac Lab 3 with Newton and MuJoCo-Warp, rigid contact), for one mechanism (the CFL), one required flexion, one task (a two-pad grasp of a free cylinder) and three interface properties (friction; compliance, as contact stiffness; shape). Its size depends on the protocol (largest with reused position commands) and, above the nominal stiffness, on contact stiffness. Under touch-referenced closure (E) the residual error follows the skeleton's change instead, so the claim is limited to reused joint commands.
- Do not claim that keeping the contact-surface motion preserves a policy's performance in general, that CoDe is needed for every interface experiment, or that more skeleton change is better.
- **The MCP level is not kept (limitation).** What is kept is finger-internal: the PIP column of the contact Jacobian in the proximal-phalanx frame. The proximal contact surface is carried by the MCP, a revolute joint with its hardware on the axis; a thicker pad at the shallowest mount moves that surface away from the axis, so the MCP column changes on every redesign by the change in pad thickness. Keeping it would need either a spacer (excluded) or the axis inside the pad (not buildable): C1 applies at the MCP as at any fixed axis. A polycentric MCP would lift this and is left as future work.
- Report null results as such; the post-hoc amendments of the experiment (contact-solver setting, hand reselection, two added hands, a symmetric error metric) are reported as exploratory.

### 8.4 Stage 2 plan (hardware; planned 2026-11-22, not yet run)

Hardware is not available for the extended abstract. The physical stage tests C5 and checks C3 and C4 on one finger.

- **Hands.** One CoDe finger with fixed servos, palm and phalanx bodies. Three configurations of the CFL are built: H₀ (3 mm pads), H_R (Study 2 redesign for 7 mm) and H_U (coverage-only 7 mm). Changing configuration swaps the pad modules and the CFL links only.
- **C5, exchange.** Record the parts swapped, the time taken and whether any recalibration is needed beyond loading the hand's θ(q) table.
- **C3, contact-surface motion.** Track markers on both contact faces with a camera over a slow flexion sweep. Compute c(q) in the proximal-phalanx frame and Δ_contact of H_R and H_U against H₀. Compare it with the model's value and with the fixed-axis bound; the manufacturing tolerance is stated from repeat sweeps.
- **C4, measurement.** Physical version of the Study 3 task: a cylinder on a planar slide, the finger closed by the reused flexion command, a pull in the four directions with a force gauge until slip. The interface change is the pad friction: bare silicone against a textured cover. Use 5 placements × 4 directions × 5 repeats per configuration and friction level, and compute E for H_R and H_U.
- **Negative results.** These are planned outcomes, not failures to hide:
  - Tracked Δ_contact above the bound: manufacturing tolerance dominates, and the redesign target needs a tolerance model.
  - H_R errs no less than H_U on hardware: pad compliance or friction variation outweighs the geometry. The claim is then limited to simulation, and the size of the physical effects is reported.
  - The exchange needs disassembly of the articulation: the integration cost is the result.

---

## 9. Paper Plan and Status

**Venue.** RSS 2027 (Athens, July 6–11). Extended abstract due 2026-12-04, stage-1 decision 2027-02-05, final paper due 2027-04-16 (from the call for papers; re-check on roboticsconference.org before each deadline).

**Evidence for each contribution** (contributions as stated in `north-star.md`; status 2026-11-22):

| | Evidence | Status |
|---|---|---|
| C1 | Study 0 (P4) | Derived. Study 2's 2R-remote redesigns sit on the bound at 90° and 120° (above it at 150° only for degenerate designs that no longer make contact) |
| C2 | Study 1 | Shown in the 2D side model: against 2R-DD, the CFL is higher in all 10 conditions where both have a design (+2.4 to +14.6 points; 2R-DD has none at 150° for 7 and 9 mm). Against 2R-remote the ranking depends on flexion and thickness (2R-remote higher at 90°; crossover near 5–6 mm at 120°; CFL higher at 150°) and is reported as such |
| C3 | Study 2 | Shown in the 2D side model: at the most-coverage end of each front, the CFL's change in contact-surface motion is the smallest of the four physical architectures in all 9 conditions, 10–51% of the fixed-axis bound, with coverage within 0.2 points of Study 1's best |
| C4 | Study 3 (Section 8) | Shown in simulation (Section 8.2): with reused position commands E follows Δ_contact (ρ = 0.83, p = 0.0002, partial 0.88 given Δ_skel; 15 hands, 3 → 5/7/9 mm), not Δ_skel; smaller under torque reuse (ρ = 0.75) and force matching (3–7%). Scope in Section 8.3 |
| C5 | Hardware | Planned for Stage 2 (Section 8.4); hardware not available for the extended abstract |

The earlier Study 3 programme (3A pilot and main experiment, 3B) is archived in `docs/experimental/study3_interface_platform.md` and is not evidence for C4.

**Decided.**

- No spacers (2026-11-21): every interface sits at its own shallowest depth, and redesign minimizes the change in contact-surface motion under that requirement (Section 6, claim 3). Study 3 has no spacer path.
- The transmission is compensated by control and recorded, not added to the redesign target (2026-11-21). Measured on the Study 3 hands (CFL, 120°, 3 → 7 mm; `design/scripts/study/exploratory/e15_transmission_change.py`): dθ/dq differs from H₀'s by up to 20% (H_R) and 21% (H_U), MA_wc by −11% and −18% (Study 2 overall: −7% to +16%). Commands in flexion mapped through each hand's θ(q) table (matching the design kinematics to 0.0003°), with the servo gains scheduled as K_θ = K_q/(dθ/dq)², hold the commanded flexion within 0.07° in simulation (MuJoCo-Warp, servo torque limits off).

**Open decisions.**

1. **Transmission on hardware.** Whether the servos' torque limits allow the compensation where MA_wc falls (up to −18% on the Study 3 hands); simulation runs without torque limits.
2. **Hardware scope** for the final paper (C5).
3. **"Interfaces concentrate on distal phalanges."** Not established (counterexamples exist); until a survey of hands supports it, the motivation rests on P1.
4. **C2's wording** ("the only physical architecture with a usable design in every condition" needs a threshold for "usable"; on hold).
