# CoDe Hand — Research Objective

Experiment design: the principles, what each study must show, and the discipline claims are held to. The publication goal (problem, approach, contributions and their status) is in [north-star.md](north-star.md), which takes precedence; the mathematics is in [problem_formulation.md](problem_formulation.md).

## 1. Purpose

**Design a robotic hand around its contact interfaces, rather than treating contact interfaces as secondary components constrained by an existing hand architecture.**

CoDe is a hand architecture intended to make contact-interface design an independent and experimentally accessible research variable. It is not primarily an attempt to build a hand with more degrees of freedom, higher grasping strength, or universally better manipulation performance.

The contribution is the validity of contact-interface measurements, not the discovery that contact matters. That a policy or a grasp depends on its contact interface is expected. **CoDe enables controlled measurements of contact-interface effects while minimizing confounding changes in the underlying robot morphology and articulation kinematics.** A change of interface itself changes contact points, normals and contact-point Jacobians; that is the intended effect under study, not a confound. What is minimized is the articulation's change made only to accommodate the interface, and the studies should show how large that confound would otherwise be.

---

## 2. The Core Research Problem

Contact is fundamental to dexterous manipulation. However, the physical design of contact interfaces remains constrained by conventional robotic hand architectures.

There are three interconnected problems.

### 2.1 Limited Space for Functional Contact Interfaces

Contact interfaces are often treated as thin friction pads or passive compliant surfaces.

In practice, useful contact interfaces can be substantially more complex and physically larger:

- Thick compliant structures.
- Integrated tactile sensor modules.
- Sensorized contact surfaces with internal electronics and supporting structures.
- Specialized geometries and materials designed for particular contact interactions.

These interfaces require physical volume.

However, conventional robotic fingers must also accommodate servo bodies, joint housings, transmission mechanisms, and structural components. These elements compete for the same limited space.

Consequently, integrating functional contact interfaces, especially along proximal phalanges and around articulated joints, can be mechanically difficult.

The problem is not simply attaching a sensor or a pad. It is providing sufficient usable space for the contact interface without compromising the hand's articulation and mechanical architecture.

### 2.2 High Redesign Cost When Contact Interfaces Change

Accommodating a different contact interface often requires changes to the underlying finger design.

These changes may include:

- Finger and phalanx dimensions.
- Joint locations and spacing.
- Actuator placement.
- Linkage and transmission geometry.
- Overall finger morphology.

As a result, changing a contact interface can become a substantial mechanical redesign rather than a simple module replacement.

This makes systematic exploration of contact-interface design expensive and time-consuming.

### 2.3 Difficulty Isolating the Effects of Contact Morphology

The most important research consequence is experimental confounding.

Suppose a manipulation experiment compares two contact interfaces.

If changing the interface also requires changing finger length, joint placement, linkage geometry, or actuator configuration, the resulting performance difference cannot be attributed solely to the contact interface.

The observed change may originate from:

- Contact-interface geometry or material.
- Tactile sensing characteristics.
- Changed finger kinematics.
- Changed hand morphology.
- Changed actuation or transmission behavior.

This makes it difficult to systematically investigate how contact-interface design itself influences manipulation.

The changes observed in such an experiment are of two kinds:

- **Intended variation** is caused by the contact interface itself: contact stiffness, friction, contact geometry, tactile observations and interface-dependent force transmission.
- **Unintended embodiment variation** is caused by redesigning the hand to accommodate the interface: finger length, joint location, linkage geometry, and actuation or transmission configuration.

CoDe does not aim to remove the first. It aims to control the second, so that the first can be studied.

**We need a hand architecture that reduces the coupling between contact-interface design and the underlying hand morphology.**

---

## 3. The CoDe Approach

CoDe introduces a contact-centric hand architecture in which contact interfaces are treated as first-class design components.

The architecture should provide sufficient space and mechanical freedom to accommodate diverse contact interfaces without requiring substantial redesign of the underlying finger structure.

The design philosophy is:

**Keep the hand architecture consistent. Redesign the contact interface.**

This does not mean that every interface must fit without any mechanical changes. Rather, the architecture should minimize the changes required to accommodate different interfaces and make those changes explicit and measurable.

The current engineering direction explores:

- Remote actuator placement to reduce competition between actuator bodies and contact interfaces.
- Polycentric articulation to provide additional freedom in finger motion and mechanical packaging.
- Linkage architectures that preserve useful ROM while accommodating functional contact structures.
- Mechanically independent proximal and distal contact-interface modules.
- Replaceable contact interfaces without requiring unnecessary disassembly of the underlying articulation mechanism.

These are engineering means toward the research objective, not independent ends.

No particular mechanism should be assumed superior without comparative evidence.

---

## 4. The Central Research Question

**How can a robotic hand be designed to enable systematic variation and evaluation of contact interfaces while minimizing changes to its underlying morphology and reducing experimental confounding?**

This leads to three supporting questions:

1. How does hand architecture constrain the available space and geometry of functional contact interfaces?
2. How much must the underlying hand morphology change to accommodate different contact interfaces?
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

Preserving the relative contact trajectory does not imply preserving contact mechanics or actuator-side contact responses: compliance, friction, sensing and the contact's location on the face change with the interface, which is precisely what the platform is designed to investigate. What should not change with them is where the contact faces are and how they move. With identical rigid-body kinematics, a thicker interface moves the contact face away from the point the phalanx turns about and so changes the contact-point Jacobian; that change is an effect of the hand's geometry, not of the interface under study, and is what a redesign should remove.

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

1. **Architecture limits the contact interface.** Under one set of requirements (the same finger size, actuators, required flexion and collision rules), each architecture's best design leaves a different share of the finger usable for contact, and the difference grows with the required flexion.
2. **The contact surface an architecture keeps depends on the interface's thickness and the required flexion.** With the finger optimized for each pad thickness and each required flexion, the best pad coverage differs by architecture, and so does how fast it falls as the pads get thicker or the flexion deeper.
3. **The relative contact trajectory can be kept when the interface changes, with the contact surface kept too.** The relative contact trajectory is what an interface experiment should hold; the skeleton's motion is a means to it. When a finger is redesigned for a thicker interface with that motion as its target, a joint turning about a fixed axis cannot keep it, whatever the redesign (a closed-form lower bound); the CFL moves the point it turns about and keeps it, at a pad coverage near the best for that interface, within a range the study measures. This is the claim the project makes for CoDe.

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

This section is the north star for the learning-based work under `sim/`. It extends question 3 (Section 4) and criterion 4 (Section 5.7) to learning-based dexterous manipulation. It does not replace Studies 1–2, and Section 7 applies to it in full.

### 8.1 Objective

**Demonstrate that CoDe Hand enables scalable and controlled contact-interface research for learning-based dexterous manipulation, with minimal changes to the underlying robotic embodiment and learning infrastructure.**

The goal is not to demonstrate that CoDe achieves universally higher manipulation performance than conventional hands. It is to establish why a contact-centric hand architecture is valuable to AI manipulation researchers.

**Central question:**

> What contact-interface experiments become practical with CoDe Hand that would otherwise require substantial changes to the robot embodiment, compromise functional articulation, or introduce additional experimental confounding?

The learning framework should produce evidence addressing this question, rather than simply maximizing task success.

**Scope of the current paper (2026-11-21).** Design studies (Studies 0–2) and simulation (Studies 3A–3C) with rigid contact interfaces. Compliant and tactile interfaces, which motivate CoDe (Section 2.1), are not simulated; the learning results are statements about rigid interfaces of different geometry and thickness. The extent of hardware validation (Section 8.4, Hardware) is an open decision.

### 8.2 Contact as an Independent Variable

A robot configuration is R = (H, C), with H = (E, Q, A) as defined in Section 5.6. The primary research configuration holds H fixed while varying C.

Contact-interface variation may intentionally change contact geometry, dimensions, friction, compliance, sensing, contact locations and contact-point Jacobians. The objective is not to make contact mechanics invariant. It is to distinguish these intended contact-interface effects from unintended changes to the underlying embodiment (Section 2.3).

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

**Study 3C — Embodiment-dependent confounding of an interface effect** (RQ3, Section 2.3). Make the same interface change C₀ → C₁ (a thin interface to a thick one) along two morphology-adaptation paths: the redesigned path R keeps the relative contact trajectory (CoDe, Study 2's redesign); the unadapted path U lets it change with the interface (the same finger with thicker interfaces, then a fixed-axis architecture at its own best redesign). With the same task and protocol, each path measures

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
