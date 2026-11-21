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

4. **Keeping the relative contact trajectory keeps an interface measurement interpretable** (hypothesis, Study 3; Section 8). The same interface change, measured on a hand redesigned to keep the relative contact trajectory and on one redesigned without that target, gives different measurements, and the first is the closer to the interface's true effect.

A geometric account (Study 0) gives the relation behind claims 1 to 3 (where the phalanx turns about, and how deep the pads sit, set what the joint takes from the pads); the optimization studies check its predictions.

**Conditions that make the comparison fair:**

- Every architecture is compared at its own best design under the common goal, never built to another architecture's morphology.
- The compared architectures are the CFL (CoDe's mechanism), 2R-DD, 2R-remote and RCJ. Idealized variants without the joint actuator are declared upper bounds and reported apart.
- A result is a statement within the 2D side model and the stated coverage definition. "Not found" is a statement about the search, not about existence.

**What must be shown on hardware:** that interfaces are in fact exchanged on one physical hand, with the same actuation and procedure (criterion 4, Section 5.7). The computational studies cannot show this.

Interface-effect experiments (question 3) should use matched or explicitly controlled morphologies.

The learning-based experiment (Study 3) is set out in Section 8.

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

## 8. Study 3: Does Keeping the Relative Contact Trajectory Change What an Interface Experiment Measures?

Study 3 answers one question, for contribution C4 (`north-star.md`):

> **When the same interface change is measured, does the measurement differ between a hand redesigned to keep the relative contact trajectory and one redesigned without that target?**

The earlier, broader Study 3 (interface A/B tests, contact-model sensitivity, accessibility) is archived in `docs/experimental-idea/study3_interface_platform.md`.

### 8.1 Design

- **Interface change.** A thin rigid flat pad (C₀) to a thicker rigid flat pad (C₁) of the same geometry and material. On the contact face nothing changes, so the interface's own effect on the task is zero by construction (Section 2); any measured change is embodiment.
- **Hands**, all of one architecture (the CFL), so that the only difference between the two redesigns is the redesign target:
  - **H₀:** the finger designed for C₀ (Study 1).
  - **H_R:** H₀ redesigned for C₁ with the relative contact trajectory as the target (Study 2, small D_c).
  - **H_U:** the finger designed for C₁ for coverage alone (Study 1, larger D_c), at comparable coverage.
- **Measurement.** Policies trained on (H₀, C₀) are run, without retraining, on (H_R, C₁) and (H_U, C₁). With J the task's pre-registered metric,
  ΔJ_R = J(π, H_R, C₁) − J(π, H₀, C₀),  ΔJ_U = J(π, H_U, C₁) − J(π, H₀, C₀),
  paired over policy seeds.
- **Prediction.** |ΔJ_U| > |ΔJ_R|. Neither is expected to be exactly zero: a thicker pad moves every contact face away from the MCP axis and the palm by the same amount on both hands, which the comparison cancels.
- **Refutation.** ΔJ_R and ΔJ_U are equal within the stated precision. Then keeping the relative contact trajectory is not what such an experiment needs, and that is reported.
- **Statistics** (fixed before running, Section 7): at least 10 policy seeds; the primary test, its α and the precision that counts as "equal" are recorded in `docs/decisions.md` before the hands are evaluated.

### 8.2 Preconditions (checked before any training)

1. **The task exercises the difference.** D_c is the largest change over [0, q_req]. The flexions the task's policies actually use must include those where H_R's and H_U's trajectories part (Study 2's per-flexion profile); otherwise an equal measurement says nothing about the trajectory.
2. **The comparison is clean.** H_R and H_U differ in D_c by a clear margin and in coverage by little. Other changes between them (transmission from servo angle to flexion, finger outline, pad mass) are recorded. Actions are defined in flexion so that a transmission change does not enter the comparison.
3. **The hands are valid in simulation** (closed loops, contact, object equilibrium), as in the archived Gate 0.

### 8.3 Choices (2026-11-21)

- **Condition:** CFL, q_req = 120°, 3 → 7 mm (changed from 90°, 3 → 5 mm on 2026-11-21). D_c is 0.86 mm on H_R and 3.91 mm on H_U at similar coverage (0.725, 0.711); at H0's two-pad cylinder grasps the distal face is off by at most 0.33 mm on H_R and 3.10 mm on H_U (medians 0.12 and 2.01 mm), over the whole flexion range rather than only in deep flexion; and the transmission changes alike on both (dθ/dq by about 20%, MA_wc 0.520 and 0.479 against H0's 0.583), so the comparison is cleaner than at 90° (there MA_wc moved −8% and +23%). 120° is also the simulated hand's PIP range.
- **H_U is one fixed design:** Study 1's first independent coverage-only design for 7 mm at 120° (its D_c depends on the search: the Study 1 design Study 2 compared against has 1.28 mm, this one 3.91 mm; the choice is recorded, and a less favourable U would narrow the contrast).
- **Actions and observations in flexion q**, mapped to each hand's servo angle by its θ(q) table. Sent as servo angles, the 3 mm finger's commands bend H_R and H_U off by about the same amount (21.0° and 22.0° at 120°, 7 mm), which would hide the trajectory difference (`docs/decisions.md`, transmission entry).
- **Stiffness matched in flexion:** per-flexion PD gains so that the effective joint stiffness is equal across hands; the transmission ratio dθ/dq differs from H0's by up to 20% (H_R) and 21% (H_U) and MA_wc by −11% and −18%. Residual torque-limit differences are recorded.
- **Interface: compliant pads**, normal contact stiffness k_pad = 2 N/mm as the reference (1 and 5 N/mm as a sensitivity check), the same on every hand and both thicknesses. With rigid pads an offset of a few tenths of a millimetre already loads the servo heavily (quasi-static estimate), so the comparison would be decided by the simulator's contact softness; with k_pad 2 N/mm the force error at H0's grasps stays at or below 0.6 N on H_R and reaches 5.7 N on H_U at 120°, 7 mm (`report/study3/regrasp_geometry.md`). The simulated stiffness is checked in Gate 0.
- **Task:** two-pad grasps. Precondition 1 checked geometrically (`report/study3/regrasp_geometry.md`): at 120°, 7 mm the difference holds over the whole flexion range of the grasps (H_R at most 0.33 mm, H_U 3.10 mm). At 90°, 5 mm it was confined to flexions of 70° and above, one reason for the change of condition.

### 8.4 Claim boundaries

- The claim is about frozen-policy measurements of an interface change. Whether independently trained policies show the same is not answered.
- Do not claim that keeping the relative contact trajectory preserves a policy's performance in general, or that CoDe is needed for every interface experiment.
- Rigid interfaces in simulation only; compliant and tactile interfaces are not simulated.
- Report a null result as such.

---

## 9. Paper Plan and Status

**Venue.** RSS 2027 (Athens, July 6–11). Extended abstract due 2026-12-04, stage-1 decision 2027-02-05, final paper due 2027-04-16 (from the call for papers; re-check on roboticsconference.org before each deadline).

**Evidence for each contribution** (contributions as stated in `north-star.md`; status 2026-11-21):

| | Evidence | Status |
|---|---|---|
| C1 | Study 0 (P4) | Derived. Study 2's 2R-remote redesigns sit on the bound at 90° and 120° (above it at 150° only for degenerate designs that no longer make contact) |
| C2 | Study 1 | Shown in the 2D side model: against 2R-DD, the CFL is higher in all 10 conditions where both have a design (+2.4 to +14.6 points; 2R-DD has none at 150° for 7 and 9 mm). Against 2R-remote the ranking depends on flexion and thickness (2R-remote higher at 90°; crossover near 5–6 mm at 120°; CFL higher at 150°) and is reported as such |
| C3 | Study 2 | Shown in the 2D side model: at the most-coverage end of each front, the CFL's trajectory change is the smallest of the four physical architectures in all 9 conditions, 10–51% of the fixed-axis bound, with coverage within 0.2 points of Study 1's best |
| C4 | Study 3 (Section 8) | Open: designed (Section 8); hands not yet built |
| C5 | Hardware | Open: extent undecided |

The earlier Study 3 programme (3A pilot and main experiment, 3B) is archived in `docs/experimental-idea/study3_interface_platform.md` and is not evidence for C4.

**Decided.** No spacers (2026-11-21): every interface sits at its own shallowest depth, and redesign minimizes the relative contact trajectory's change under that requirement (Section 6, claim 3). Study 3 has no spacer path.

**Open decisions.**

1. **Transmission under redesign** (servo angle to flexion, mechanical advantage: MA_wc changed by −7% to +16% in Study 2): how much it changes, whether it is argued as compensable and recorded (Section 5.6), or added to the redesign target. Not yet measured per design.
2. **Hardware scope** for the final paper (C5).
3. **"Interfaces concentrate on distal phalanges."** Not established (counterexamples exist); until a survey of hands supports it, the motivation rests on P1.
4. **C2's wording** ("the only physical architecture with a usable design in every condition" needs a threshold for "usable"; on hold).
