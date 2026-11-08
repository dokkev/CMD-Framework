# CoDe Hand — Project Goal

## 1. North Star

**Design a robotic hand around its contact interfaces, rather than treating contact interfaces as secondary components constrained by an existing hand architecture.**

CoDe Hand introduces a contact-centric robotic hand design and research platform for systematically investigating how contact-interface design influences physical interaction, tactile observations, and dexterous manipulation.

The central objective is to make diverse functional contact interfaces easier to design, integrate, replace, and evaluate while keeping the underlying hand morphology and actuation architecture as consistent as practical.

CoDe is not primarily an attempt to build a hand with more degrees of freedom, higher grasping strength, or universally better manipulation performance.

It is a hand architecture intended to make contact-interface design an independent and experimentally accessible research variable.

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

- Structural finger morphology and link geometry.
- Articulation kinematics and joint configuration.
- Actuator placement and transmission architecture.
- Contact-module mounting references.

The following are intentionally variable:

- Contact-interface geometry and dimensions.
- Compliance, material properties and friction.
- Tactile sensing configuration.
- Contact location, distribution and surface response.

Preserving the underlying embodiment does not imply preserving contact mechanics or actuator-side contact responses. For example, a contact-point Jacobian changes when the contact location or the interface thickness changes, even with identical rigid-body kinematics. These responses may change as a direct consequence of the contact interface, which is precisely what the platform is designed to investigate.

The objective is to isolate contact-interface variation from unnecessary changes to the underlying hand embodiment, not to make contact behavior invariant.

### 5.7 Criteria for a Contact-Centric Research Platform

A successful contact-centric research platform should:

1. **Accommodate diverse interfaces.** Support meaningful variation in contact geometry, thickness, compliance and sensing hardware.
2. **Preserve the underlying embodiment.** Allow interface replacement without unnecessary modifications to finger structure, articulation or actuation.
3. **Preserve functional articulation.** Maintain the required range of motion and accessible contact surfaces across supported interface configurations.
4. **Enable controlled and repeatable experiments.** Allow different interfaces to be evaluated on the same physical hand with consistent actuation, sensing and experimental procedures.

These criteria define the platform's research utility. The first three are assessed computationally (Studies 1–2). The fourth must be demonstrated on the physical hardware.

Mechanism compactness, transmission characteristics and contact-interface capacity are important evaluation metrics, but they are means of assessing these criteria rather than independent project objectives.

---

## 6. Evaluation Strategy

The project's experiments should provide evidence for the central research objective. Each study answers part of a supporting question in Section 4; its formulation and configuration live in its own README, not here.

- **Study 1: Fixed-Morphology Contact Accommodation** (question 1). Which contact interfaces can a frozen reference hand carry, with its ROM and usable contact coverage?
- **Study 2: Morphology Adaptation** (question 2). How much must the mechanism and the external morphology change to carry an interface that a reference hand cannot? Each kind of change is recorded separately. A compact-accommodation extension asks instead for the smallest finger that carries each interface, and the spans and articulation shifts that this costs.
- **Study 3: Transmission-Reflected Contact Mechanics** (toward question 3). How does each transmission map identical local contact mechanics to the actuator? This identifies an embodiment dependence to control for when interface experiments are interpreted.
- **Study 4: Joint Contact Interface** (question 1, around the joint). Can a bridge across the proximal and distal regions make the joint itself a continuous functional contact surface? A polycentric advantage is tested, not presumed.

Interface-effect experiments (question 3) should use matched or explicitly controlled morphologies.

Learning-based interface experiments (Studies A–D) are set out in Section 8.

---

## 7. Claim Discipline

- Keep physical CoDe, the physical alternatives and declared idealized ablations separate. An idealization removes only its named constraint.
- Report failures and negative results. Never tune objectives, constraints or comparison assumptions to make CoDe win.
- Candidate designs are design alternatives, not statistical replicates. A search that finds nothing is not proof that no design exists.
- Matched morphology reduces confounding; it does not establish perfect causal isolation.
- Separate demonstrated observations, model-derived findings, assumptions and hypotheses.
- Planar feasibility is not buildability. Local reflected stiffness is not validated actuator response. Geometric bridge continuity is not skin durability or usable loaded contact.
- Quantitative figures must read saved results, with source commit or patch, configs, input artifacts and sampling provenance. Missing models or artifacts block their branch of work; do not invent measurements.
- The next useful research decision is one that improves controlled interface variation or exposes its limits, whether or not it favors CoDe.

---

## 8. Learning-Based Manipulation Research

This section is the north star for the learning-based work under `sim/`. It extends question 3 (Section 4) and criterion 4 (Section 5.7) to learning-based dexterous manipulation. It does not replace Studies 1–4, and Section 7 applies to it in full.

### 8.1 Objective

**Demonstrate that CoDe Hand enables scalable and controlled contact-interface research for learning-based dexterous manipulation, with minimal changes to the underlying robotic embodiment and learning infrastructure.**

The goal is not to demonstrate that CoDe achieves universally higher manipulation performance than conventional hands. It is to establish why a contact-centric hand architecture is valuable to AI manipulation researchers.

**Central question:**

> What contact-interface experiments become practical with CoDe Hand that would otherwise require substantial changes to the robot embodiment, compromise functional articulation, or introduce additional experimental confounding?

The learning framework should produce evidence addressing this question, rather than simply maximizing task success.

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

Separate the cost of supporting a new interface from the cost of swapping an already-supported interface.

**RQ3 — Contact morphology and policy behavior.** With the underlying hand fixed, how does changing the contact interface influence:

- Manipulation task performance.
- Policy robustness and generalization.
- Zero-shot policy transfer across interfaces.
- Learning dynamics and adaptation requirements.

These experiments demonstrate the research questions enabled by CoDe, rather than assuming that CoDe or a particular interface must outperform alternatives.

**RQ4 — Cross-simulator and physical relevance.** To what extent do interface-dependent manipulation observations remain consistent across simulation engines and representative physical experiments? Cross-simulator agreement provides evidence about simulator dependence, not proof of physical accuracy.

### 8.4 Evaluation Strategy

**Study A — Architecture-dependent manipulation accessibility** (RQ1, RQ2). Compare which contact interfaces support a predefined manipulation task under:

1. Conventional 2R-DD with fixed morphology.
2. Conventional 2R-DD with reasonable morphology adaptation.
3. CoDe with fixed morphology.

Evaluate both mechanical feasibility and downstream task execution. Use matched interface requirements, comparable actuator capabilities, declared size constraints and equivalent task objectives. Distinguish mechanical infeasibility from policy training failure. Mechanical feasibility comes from Studies 1–2.

*Target evidence:* whether CoDe expands the manipulation-usable interface design space while reducing the need for morphology adaptation.

**Study B — Contact-interface A/B testing** (RQ3). Keep the CoDe embodiment fixed and evaluate multiple contact interfaces with a common manipulation protocol. Use two complementary evaluations:

- **Frozen policy:** change the contact interface without retraining.
- **Independent training:** train a policy per interface with a common algorithm and comparable training budgets.

Measure performance differences, sensitivity, adaptation and generalization.

*Target evidence:* controlled contact-interface experimentation without redesigning the underlying hand.

**Study C — Cross-interface policy transfer** (RQ3). Train policies under selected contact interfaces and evaluate each across the other supported interfaces. This gives a policy-by-contact performance matrix M_ij = P(π_i, H, C_j), the performance of the policy trained on interface i when run on interface j. The matrix should reveal how contact morphology influences policy robustness and generalization. High transfer performance is not assumed or required.

*Target evidence:* CoDe provides a reusable experimental embodiment for studying contact-dependent policy behavior.

**Study D — Sim-to-sim and representative sim-to-real** (RQ4). Use Isaac Lab for scalable policy training and manipulation experiments. Evaluate selected policies and interfaces in an independent simulator or contact model to characterize contact-model sensitivity. Select about 2–3 representative physical contact interfaces for hardware validation. Physical experiments should demonstrate interface interchangeability, retained functional articulation, repeatable manipulation evaluation and the limits of simulation predictions.

*Target evidence:* extensive simulation findings connected to a functioning physical research platform, without exhaustive real-world interface testing.

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
