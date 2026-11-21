# Experimental idea: Study 3 as a contact-interface platform study

Archived 2026-11-21 from `design/SSoT/research-objective.md` Section 8, when Study 3 was narrowed to one question (the current Section 8 there). Not part of the SSoT and not a plan for the current paper; kept as a source of later experiments.

What it was: a programme of learning-based experiments on one CoDe hand — an interface A/B test with a policy-by-contact transfer matrix (3A), contact-model sensitivity (3B), embodiment-dependent confounding (3C, which became the current Study 3) and architecture-dependent accessibility (3D).

What was run under it, all kept unchanged:

- 3A pilot (flat vs crowned pads, prototype hand pivot8, action EMA 1.0): `report/study3/study3_pilot.md`; confirmatory analysis on 10 seeds in `report/study3/tables/confirmatory.json` (flat → crowned transfer gap +0.196, 10/10 seeds, sign-flip p = 0.002).
- 3B′ (VBD replay, EMA 0.1): judged not interpretable by its pre-registered validity condition.
- Goal-count audit, budget pilot, hydroelastic and friction-model Gate 0, and the 3A main experiment (`-Main` tasks, 3000 iterations, seeds 0–9): `docs/decisions.md` (2026-11-21 entries), `logs/study3_main/`.

Lessons that carry over to any learning-based study here: verify what a metric counts (overlapping goal regions); pre-register the budget from a pilot; use at least 10 seeds for a sign-flip test; Newton VBD and hydroelastic contact failed validation on this hand; MuJoCo-Warp friction-model variants passed.

---

##  Learning-Based Manipulation Research

This section sets out the learning-based work under `sim/`. It extends question 3 (Section 4) and criterion 4 (Section 5.7) to learning-based dexterous manipulation. It does not replace Studies 1–2, and Section 7 applies to it in full.

### 1 Objective

**Demonstrate that CoDe Hand enables scalable and controlled contact-interface research for learning-based dexterous manipulation, with minimal changes to the underlying robotic embodiment and learning infrastructure.**

The goal is not to demonstrate that CoDe achieves universally higher manipulation performance than conventional hands. It is to establish why a contact-centric hand architecture is valuable to AI manipulation researchers.

**Central question:**

> What contact-interface experiments become practical with CoDe Hand that would otherwise require substantial changes to the robot embodiment, compromise functional articulation, or introduce additional experimental confounding?

The learning framework should produce evidence addressing this question, rather than simply maximizing task success.

**Scope of the current paper (2026-11-21).** Design studies (Studies 0–2) and simulation (Studies 3A–3C) with rigid contact interfaces. Compliant and tactile interfaces, which motivate CoDe (north-star.md), are not simulated; the learning results are statements about rigid interfaces of different geometry and thickness. The extent of hardware validation (Section 8.4, Hardware) is an open decision.

### 2 Contact as an Independent Variable

A robot configuration is R = (H, C), with H = (E, Q, A) as defined in Section 5.6. The primary research configuration holds H fixed while varying C.

Contact-interface variation may intentionally change contact geometry, dimensions, friction, compliance, sensing, contact locations and the contact-point Jacobians that follow from them. The objective is not to make contact mechanics invariant. It is to distinguish these intended contact-interface effects from unintended changes to the underlying embodiment, including a changed relative contact trajectory (Section 2).

Where practical, preserve the observation space, action space, controller, task definition, reward and learning algorithm across interface configurations. Changes that cannot be preserved must be reported explicitly.

### 3 Research Questions

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

### 4 Evaluation Strategy

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

### 5 Success Criteria

The learning-based effort succeeds when it establishes a chain of evidence:

**Contact accommodation → Embodiment consistency → Learning-infrastructure reusability → Controlled manipulation research**

Specifically:

1. Demonstrate the range of contact interfaces that can be used for manipulation under stated architectural constraints.
2. Quantify the embodiment modifications and integration effort required by different architectures.
3. Demonstrate that multiple contact interfaces can be evaluated under a common CoDe embodiment and learning pipeline.
4. Show meaningful contact-dependent manipulation or policy behavior without assuming a favorable performance direction.
5. Validate representative observations across simulators and on physical hardware, reporting discrepancies and limitations.

### 6 Claim Boundaries

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

### 7 Intended Research Message

**CoDe Hand is not intended to make every contact interface better. It is intended to make contact interfaces easier to study.**

The ultimate demonstration is that AI manipulation researchers can change contact morphology, retain a consistent underlying embodiment and learning infrastructure, and systematically investigate the resulting manipulation behavior without repeatedly redesigning the robot.

---

