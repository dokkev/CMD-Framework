# CoDe Hand — Research Objective

Experiment design: the principles, what each study must show, the discipline claims are held to, and the paper plan with each contribution's status (Section 9). The problem, what CoDe solves and the contributions are in [north-star.md](north-star.md), which takes precedence; the mathematics is in [problem_formulation.md](problem_formulation.md).

## 1. Purpose

CoDe is meant to be a research platform on which contact-interface experiments are possible and interpretable: researchers can study diverse contact morphologies if they wish, and can change only the contact interface while the underlying relative contact trajectory is kept. The problem it addresses, its approach and the contributions claimed are in [north-star.md](north-star.md); this file states how the studies test them.

It is not an attempt to build a hand with more degrees of freedom, higher grasping strength, or universally better manipulation performance.

---

## 2. Intended and Unintended Variation

When a manipulation experiment compares two contact interfaces, the observed change has two kinds of source:

- **Intended variation** comes from the contact interface itself: what happens on the contact face. Its geometry and profile, the contact's location, distribution and normals on the face, compliance, friction, tactile observations and interface-dependent force transmission. The contact-point Jacobian changes that follow from these (a contact elsewhere on the face, a different normal) are part of the effect under study.
- **Unintended embodiment variation** comes from how the hand was adapted to the interface: changes in finger length, joint location, linkage geometry, actuation or transmission, and above all a displacement of the contact faces relative to the articulation. A thicker interface on an unchanged joint moves its contact face away from the point the phalanx turns about, which changes the relative contact trajectory and with it the contact-point Jacobian. That change comes from the hand's geometry, not from the interface, and is what a redesign should remove.

The dividing line: a change on the contact face is the interface's; a change of where the contact face is, and how it moves, relative to the other contact face is the embodiment's. CoDe does not aim to remove the first. It aims to keep the second small, and to record what does change (Section 5.2), so that the first can be studied. The objective is to reduce embodiment confounding, not to claim perfect causal isolation.

---

## 3. Problem and Approach

See [north-star.md](north-star.md). Mechanisms are engineering means toward the research objective, not ends, and no mechanism is assumed superior without comparative evidence.

---

## 4. Supporting Questions

1. How does hand architecture constrain the available space and geometry of contact interfaces?
2. How much must the underlying hand change to accommodate different contact interfaces, and how much does its relative contact trajectory change?
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

- The relative contact trajectory: the path of the distal contact face seen from the proximal one, as a function of flexion (its change is D_c, `problem_formulation.md` A.6.3). It excludes the transmission from servo angle to flexion, which is recorded separately. This is the kinematic target of a contact-centric hand; what a policy and an object meet is the contact face, not the skeleton.
- Structural finger morphology and link geometry.
- Articulation kinematics and joint configuration.
- Actuator placement and transmission architecture.
- Contact-module mounting references.

When these cannot all be kept (an interface thicker than the finger was designed for asks for a redesign), the relative contact trajectory has priority over the skeleton's: the mechanism's internal dimensions may change to keep the contact faces moving as before (`problem_formulation.md` A.6.0, A.6.3).

The following are intentionally variable:

- Contact-interface geometry and dimensions.
- Compliance, material properties and friction.
- Tactile sensing configuration.
- Contact location, distribution and surface response.

Preserving the relative contact trajectory does not imply preserving contact mechanics or actuator-side contact responses: compliance, friction, sensing and the contact's location on the face change with the interface, which is precisely what the platform is designed to investigate. What should not change with them is where the contact faces are and how they move relative to each other (Section 2). Changes that a redesign does make (transmission from servo angle to flexion, mechanical advantage, the skeleton's motion) are recorded with the result.

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
3. **A redesign can keep the relative contact trajectory far better than a fixed axis allows, with the contact surface kept too.** The relative contact trajectory is what an interface experiment should hold; the skeleton's motion is a means to it. When a finger is redesigned for a thicker interface placed at that interface's best depth, a joint turning about a fixed axis changes the relative contact trajectory by at least a closed-form lower bound, whatever the redesign; the CFL moves the point it turns about and changes it by a fraction of that bound, at a pad coverage near the best for that interface, within a range the study measures. Every interface is mounted at its own shallowest depth, without spacers. Raising a thinner interface on a spacer to the depth of the thickest one would keep the trajectory on any architecture, but it fills the space the interface needs (the problem CoDe addresses) with a passive block, costs every thinner interface the coverage of the deep design (at 120°: 8.6 points for the CFL, 18.2 for 2R-remote, 3 mm against a 9 mm design), and fixes the thickest interface in advance. It is therefore excluded by requirement, not compared.

**What the learning-based studies must test** (supporting question 3; Section 8):

4. **Keeping the relative contact trajectory keeps an interface measurement interpretable** (hypothesis, Study 3C). If an interface experiment is run on a hand adapted to the interface in two ways, one that keeps the relative contact trajectory (claim 3) and one that does not, the measured effect of the interface differs between them. Support: the effect measured on the unadapted path departs from the redesigned path's, and the departure grows with the change in contact-face motion. Refutation: the two paths measure the same effect within the stated precision, in which case keeping the relative contact trajectory is not what an interface experiment needs, and that is reported.

A geometric account (Study 0) gives the relation behind claims 1 to 3 (where the phalanx turns about, and how deep the pads sit, set what the joint takes from the pads); the optimization studies check its predictions.

**Conditions that make the comparison fair:**

- Every architecture is compared at its own best design under the common goal, never built to another architecture's morphology.
- The compared architectures are the CFL (CoDe's mechanism), 2R-DD, 2R-remote and RCJ. Idealized variants without the joint actuator are declared upper bounds and reported apart.
- A result is a statement within the 2D side model and the stated coverage definition. "Not found" is a statement about the search, not about existence.

**What must be shown on hardware:** that interfaces are in fact exchanged on one physical hand, with the same actuation and procedure (criterion 4, Section 5.7). The computational studies cannot show this.

Interface-effect experiments (question 3) should use matched or explicitly controlled morphologies.

Learning-based interface experiments (Studies 3A–3D) are set out in Section 8.

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
- Fix the primary metric, the test and the interpretation criteria before the data that decide them exist, and record them (`docs/decisions.md`). Never change a criterion after seeing its outcome; a failed validity condition is reported as such.
- Use enough independent replicates (policy seeds) for the planned test to be able to reject (an exact sign-flip test over 5 seeds cannot reach p < 0.05).
- Verify what a metric measures before interpreting it (e.g. that a goal count counts rotation and not overlapping success regions).

---

## 8. Learning-Based Manipulation Research

This section sets out the learning-based work under `sim/`. It extends question 3 (Section 4) and criterion 4 (Section 5.7) to learning-based dexterous manipulation. It does not replace Studies 1–2, and Section 7 applies to it in full.

### 8.1 Objective

**Demonstrate that CoDe Hand enables scalable and controlled contact-interface research for learning-based dexterous manipulation, with minimal changes to the underlying robotic embodiment and learning infrastructure.**

The goal is not to demonstrate that CoDe achieves universally higher manipulation performance than conventional hands. It is to establish why a contact-centric hand architecture is valuable to AI manipulation researchers.

**Central question:**

> What contact-interface experiments become practical with CoDe Hand that would otherwise require substantial changes to the robot embodiment, compromise functional articulation, or introduce additional experimental confounding?

The learning framework should produce evidence addressing this question, rather than simply maximizing task success.

**Scope of the current paper (2026-11-21).** Design studies (Studies 0–2) and simulation (Studies 3A–3C) with rigid contact interfaces. Compliant and tactile interfaces, which motivate CoDe (north-star.md), are not simulated; the learning results are statements about rigid interfaces of different geometry and thickness. The extent of hardware validation (Section 8.4, Hardware) is an open decision.

### 8.2 Contact as an Independent Variable

A robot configuration is R = (H, C), with H = (E, Q, A) as defined in Section 5.6. The primary research configuration holds H fixed while varying C.

Contact-interface variation may intentionally change contact geometry, dimensions, friction, compliance, sensing, contact locations and the contact-point Jacobians that follow from them. The objective is not to make contact mechanics invariant. It is to distinguish these intended contact-interface effects from unintended changes to the underlying embodiment, including a changed relative contact trajectory (Section 2).

Where practical, preserve the observation space, action space, controller, task definition, reward and learning algorithm across interface configurations. Changes that cannot be preserved must be reported explicitly.

### 8.3 Research Questions

**RQ1 — Manipulation-usable contact design space.** Which contact interfaces can each hand architecture support while preserving the articulation, contact accessibility and physical capabilities required for manipulation? Compare CoDe with reasonable conventional baselines, including fixed and morphology-adapted 2R-DD configurations. A geometrically attachable interface is not necessarily usable for manipulation.

**RQ2 — Embodiment and learning-infrastructure reusability.** How much additional engineering effort is required to introduce and evaluate a new contact interface? Evaluate:

- Changes to structural morphology and articulation.
- Changes to actuator and transmission configuration.
- Changes to simulation assets and robot descriptions.
- Changes to control, observations, actions and learning infrastructure.
- Additional interface-specific integration requirements.

Separate the cost of supporting a new interface from the cost of swapping an already-supported interface. Measured in Study 3A (what had to change to add each interface) and Study 3C (what a thicker interface costs on each adaptation path).

**RQ3 — Contact morphology and policy behavior.** With the underlying hand fixed, how does changing the contact interface influence:

- Manipulation task performance.
- Policy robustness and generalization.
- Zero-shot policy transfer across interfaces.
- Learning dynamics and adaptation requirements.

These experiments demonstrate the research questions enabled by CoDe, rather than assuming that CoDe or a particular interface must outperform alternatives.

**RQ4 — Cross-simulator and physical relevance.** To what extent do interface-dependent manipulation observations remain consistent across simulation engines and representative physical experiments? Cross-simulator agreement provides evidence about simulator dependence, not proof of physical accuracy. Current state: an independent engine (Newton VBD) and a distributed contact model (Newton hydroelastic) both failed validation on this hand (2026-11-21), so the present evidence is limited to contact-model sensitivity within one engine (Study 3B); agreement across engines remains open.

### 8.4 Evaluation Strategy

The learning-based studies are numbered 3A–3D, the same names in this file, in `sim/` and in `report/study3/`. (Earlier drafts used letters A–D; they are retired. A study once called 3C, optimizing an interface for a frozen policy, was discarded on 2026-11-21; Section 8.6.)

Throughout, J(π, H, C, S) is the task's pre-registered performance metric (for the cube-rotation task, goals reached per episode on a fixed goal sequence) of policy π on hand H with interface C under contact model S; arguments that do not vary are dropped.

**Roles and order.** Study 3C is the learning-based evidence specific to CoDe: it links the design studies' relative contact trajectory (claims 3 and 4) to interface experiments. Study 3A changes only an interface that any hand could exchange (a profile inside the same pad envelope), so it demonstrates the experimental platform and pipeline, not the need for CoDe (Section 8.6). Study 3B checks that 3A's and 3C's interface effects are not artifacts of one contact model. Order: 3A, then 3C, then 3B on the policies of both, then 3D.

**Study 3A — Contact-interface A/B test and transfer matrix** (RQ3). Keep the CoDe embodiment fixed and evaluate multiple contact interfaces with a common manipulation protocol, in two complementary ways:

- **Independent training:** train a policy per interface with a common algorithm and comparable training budgets.
- **Frozen policy:** run each policy on the other interfaces without retraining. This gives a policy-by-contact matrix M_ij = J(π_i, H, C_j), the performance of the policy trained on interface i when run on interface j. High transfer performance is not assumed or required.

Measure performance differences, sensitivity, adaptation, generalization and contact behaviour (where on the interface contact occurs), and record what had to change to add each interface (RQ2).

*Target evidence:* that controlled contact-interface experiments run on one embodiment and one learning pipeline, and show contact-dependent policy behaviour. Not, by itself, evidence that CoDe is needed.

**Study 3B — Contact-model sensitivity** (RQ4). Evaluate the policies of 3A (and 3C) under alternative contact models. The quantity of interest is the interface effect, not the absolute performance under each model: for a frozen policy π and contact models S, Δ_C^S(π) = J(π, C₁, S) − J(π, C₀, S), and Δ_model(π) = Δ_C^S′(π) − Δ_C^S(π). A contact model inside the same dynamics engine measures contact-model sensitivity, not generalization to an independent engine. Each model passes its own validation first (collider representation, contact force direction and magnitude, object equilibrium, closed-loop constraints), and its parameters are fixed in common for every interface, never tuned to one. The models compared are MuJoCo-Warp point contact with the training settings (elliptic friction cone, impratio 10), with a pyramidal friction cone, and with impratio 1; hydroelastic contact and Newton VBD failed validation on this hand and are reported as such.

*Target evidence:* whether a measured interface effect survives a change of the simulated contact model.

**Study 3C — Embodiment-dependent confounding of an interface effect** (RQ3, Section 2). Make the same interface change C₀ → C₁ (a thin interface to a thick one) along two morphology-adaptation paths: the redesigned path R keeps the relative contact trajectory (CoDe, Study 2's redesign); the unadapted path U lets it change with the interface (the same finger with thicker interfaces, then a fixed-axis architecture at its own best redesign). With the same task and protocol, each path measures

ΔJ_R = J(H_R,1, C₁) − J(H_R,0, C₀),  ΔJ_U = J(H_U,1, C₁) − J(H_U,0, C₀),

and Δ_interaction = ΔJ_R − ΔJ_U is how much the measured interface effect depends on the adaptation path. It is a candidate measure of embodiment confounding, not a pure kinematic one: the two paths may also differ in contact-point Jacobians, transmission and actuator loading, contact accessibility and the strategies the policies learn. The claim is limited to: *morphology adaptation can alter the measured effect of a contact-interface change, introducing embodiment-dependent confounding.* The kinematic share is then analysed through D_c and the measured contact motion. No direction is assumed, and a null Δ_interaction is reported as such.

*Target evidence:* whether, and by how much, a measured interface effect depends on how the hand was adapted to the interface.

**Study 3D — Architecture-dependent manipulation accessibility** (RQ1, RQ2; planned). Compare which contact interfaces support a predefined manipulation task under:

1. Conventional 2R-DD with fixed morphology.
2. Conventional 2R-DD with reasonable morphology adaptation.
3. CoDe with fixed morphology.

Evaluate both mechanical feasibility and downstream task execution. Use matched interface requirements, comparable actuator capabilities, declared size constraints and equivalent task objectives. Distinguish mechanical infeasibility from policy training failure. Mechanical feasibility comes from Studies 1–2.

*Target evidence:* whether CoDe expands the manipulation-usable interface design space while reducing the need for morphology adaptation.

**Hardware.** Select about 2–3 representative physical contact interfaces for hardware validation. Physical experiments should demonstrate interface interchangeability, retained functional articulation, repeatable manipulation evaluation and the limits of simulation predictions.

### 8.5 Success Criteria

The learning-based effort succeeds when it establishes a chain of evidence:

**Contact accommodation → Embodiment consistency → Learning-infrastructure reusability → Controlled manipulation research**

Specifically:

1. Demonstrate the range of contact interfaces that can be used for manipulation under stated architectural constraints.
2. Quantify the embodiment modifications and integration effort required by different architectures.
3. Demonstrate that multiple contact interfaces can be evaluated under a common CoDe embodiment and learning pipeline.
4. Show meaningful contact-dependent manipulation or policy behavior without assuming a favorable performance direction.
5. Validate representative observations across simulators and on physical hardware, reporting discrepancies and limitations.

### 8.6 Claim Boundaries

In addition to Section 7:

- Do not optimize benchmarks merely to demonstrate CoDe superiority.
- Do not optimize the contact interface to raise a given policy's performance and present it as evidence for CoDe: the platform's purpose is controlled measurement, not a better interface.
- Do not treat a contact-parameter sweep alone as evidence that CoDe is necessary.
- Do not claim that policy performance differences between different hand architectures are caused solely by their contact interfaces.
- Do not assume interface replacement preserves policy performance or eliminates the need for retraining.
- Do not claim that simulated contact response accurately represents physical compliant contact without validation.
- Do not confuse mechanical accommodation, manipulation feasibility and successful policy learning.
- Do not treat design candidates or training environments as independent statistical replicates.
- Report negative results, failed integrations, simulator disagreement and architecture trade-offs.

### 8.7 Intended Research Message

**CoDe Hand is not intended to make every contact interface better. It is intended to make contact interfaces easier to study.**

The ultimate demonstration is that AI manipulation researchers can change contact morphology, retain a consistent underlying embodiment and learning infrastructure, and systematically investigate the resulting manipulation behavior without repeatedly redesigning the robot.

---

## 9. Paper Plan and Status

**Venue.** RSS 2027 (Athens, July 6–11). Extended abstract due 2026-12-04, stage-1 decision 2027-02-05, final paper due 2027-04-16 (from the call for papers; re-check on roboticsconference.org before each deadline).

**Evidence for each contribution** (contributions as stated in `north-star.md`; status 2026-11-21):

| | Evidence | Status |
|---|---|---|
| C1 | Study 0 (P4) | Derived. Study 2's 2R-remote redesigns sit on the bound at 90° and 120° (above it at 150° only for degenerate designs that no longer make contact) |
| C2 | Study 1 | Shown in the 2D side model: against 2R-DD, the CFL is higher in all 10 conditions where both have a design (+2.4 to +14.6 points; 2R-DD has none at 150° for 7 and 9 mm). Against 2R-remote the ranking depends on flexion and thickness (2R-remote higher at 90°; crossover near 5–6 mm at 120°; CFL higher at 150°) and is reported as such |
| C3 | Study 2 | Shown in the 2D side model: at the most-coverage end of each front, the CFL's trajectory change is the smallest of the four physical architectures in all 9 conditions, 10–51% of the fixed-axis bound, with coverage within 0.2 points of Study 1's best |
| C4 | Study 3C | Open: not yet designed in detail |
| C5 | Hardware | Open: extent undecided |

Studies 3A (pilot done; main experiment running) and 3B (pre-registered) support C4 and the platform claim but are not contributions on their own (Section 8.4).

**Decided.** No spacers (2026-11-21): every interface sits at its own shallowest depth, and redesign minimizes the relative contact trajectory's change under that requirement (Section 6, claim 3). Study 3C has no spacer path.

**Open decisions.**

1. **Transmission under redesign** (servo angle to flexion, mechanical advantage: MA_wc changed by −7% to +16% in Study 2): how much it changes, whether it is argued as compensable and recorded (Section 5.6), or added to the redesign target. Not yet measured per design.
2. **Hardware scope** for the final paper (C5).
3. **"Interfaces concentrate on distal phalanges."** Not established (counterexamples exist); until a survey of hands supports it, the motivation rests on P1.
4. **Study 3D**: kept, or dropped from the RSS scope.
5. **C2's wording** ("the only physical architecture with a usable design in every condition" needs a threshold for "usable"; on hold).
