# GOAL: Autonomous Discovery of CoDe Hand's Architectural Advantages

## MISSION

Act as an autonomous robotics research investigator, not merely a software implementation agent.

Your mission is to **design, execute, interpret, critically review, and strengthen 10 new independent computational studies** investigating the CoDe Hand architecture.

The objective is to discover previously unrecognized architectural properties, trade-offs, or advantages of CoDe that support the project's established North Star.

**Do not design studies to make CoDe win. Design studies to discover what CoDe actually does differently, why those differences arise, and whether they matter for contact-centric robotic hand research.**

A successful outcome is not necessarily 10 positive findings.

A successful outcome is 10 scientifically meaningful investigations, followed by a critical synthesis identifying the strongest defensible research contributions.

---

## 1. ESTABLISHED RESEARCH NORTH STAR

Read the repository's root `GOAL.md` first.

Treat its central research objectives as established.

The fundamental problem is:

1. Functional contact interfaces require physical space, but servo bodies, articulation mechanisms, and transmission components compete for the same limited finger volume.

2. Changing a contact interface often requires substantial redesign of the underlying hand morphology, creating high integration and experimentation costs.

3. When contact interfaces and underlying hand morphology change together, manipulation-performance differences become confounded. It becomes difficult to distinguish interface effects from embodiment effects.

CoDe Hand introduces a contact-centric hand architecture and research platform intended to make contact-interface morphology a more accessible and experimentally controllable design variable.

Its purpose is to reduce the spatial and mechanical coupling between contact-interface design and underlying hand embodiment.

**North Star:**

> The hand should be designed to accommodate contact, not force contact to accommodate the hand.

Every proposed study must explicitly explain how its research question relates to this goal.

The following are potential mechanisms for achieving the goal, not contributions to assume in advance:

- Remote actuator placement.
- Polycentric articulation and moving instantaneous centers of rotation.
- Crossed four-bar linkage geometry.
- Independently replaceable proximal and distal contact interfaces.
- Contact-oriented mechanical packaging.

Do not redefine the project around any one of these implementation choices.

---

## 2. ESTABLISHED CONTEXT

Repository:

`https://github.com/dokkev/CMD-Framework`

Primary branch:

`feature/2dof-fe`

Before modifying anything, inspect the actual repository state, including:

- `GOAL.md`
- Existing Study 1–4 implementations and reports.
- Existing optimization formulations.
- Mechanism kinematics and transmission models.
- Available Pareto archives and selected designs.
- Current contact-interface catalogs and morphology configurations.
- Existing tests, figures, result manifests, and documentation.
- Recorded source snapshots and uncommitted working-tree changes.

Treat existing validated results as established inputs.

Do not rerun expensive optimization merely to reproduce information already available.

### Previous Findings

Study 1 established that, under the evaluated fixed-morphology conditions, CoDe can preserve usable contact coverage and required ROM for a larger set of interfaces than several physical alternatives, particularly under high-flexion requirements.

Study 2 investigated morphology modifications required to accommodate fixed contact interfaces.

Its compactness extension found that CoDe was the shortest best-found physical architecture for the bilateral long-and-thick interface, while Belt 2R was shorter for most other evaluated interfaces.

CoDe also exhibited limitations for certain proximal-heavy interfaces.

These findings are conditional on the current geometric models, optimization bounds, and mechanical assumptions.

Study 3 investigates transmission-reflected contact dynamics.

Study 4 investigates whether the articulated joint region can support a functional contact interface.

**The 10 new investigations must not simply rename or repeat Studies 1–4.**

Use identifiers `E01`–`E10` to distinguish these exploratory investigations from the established studies.

---

## 3. PRIMARY OBJECTIVE

Discover and evaluate previously unexplored relationships between:

- Hand architecture.
- Contact-interface design freedom.
- Contact surface accessibility.
- Actuator and mechanism packaging.
- Articulation kinematics.
- Morphology preservation.
- Transmission characteristics.
- Mechanical robustness.
- Experimental controllability.

Prefer investigations that explain a general architectural principle rather than merely demonstrating that one optimized CoDe design outperforms one poorly chosen baseline.

Look especially for cases where conventional architectures face structural trade-offs that CoDe can reduce, avoid, or relocate.

Also identify cases where CoDe introduces additional costs or constraints.

The most valuable discovery would explain:

**Why a particular architectural feature changes the feasible design space for functional contact interfaces.**

---

## 4. AUTONOMOUS RESEARCH PROCESS

Execute the following process without requesting approval for routine research decisions.

### Phase A — Repository and Evidence Audit

Inspect the existing implementation and research artifacts.

Identify:

1. Mechanisms and design variables currently supported.
2. Available kinematic and geometric evaluation capabilities.
3. Existing Pareto optimization outputs.
4. Reusable contact-interface models.
5. Important physical assumptions.
6. Known limitations of the comparison methodology.
7. Open questions that have not yet been investigated.

Create a short evidence baseline distinguishing:

- Previously demonstrated findings.
- Existing hypotheses.
- Model assumptions.
- Missing experimental evidence.
- Potential implementation issues.

Do not treat past optimization failures as proof of mechanical impossibility.

### Phase B — Generate Candidate Research Questions

Brainstorm at least 20 distinct, falsifiable research questions.

Use the North Star to evaluate each candidate.

Possible exploration directions include:

- Actuator-inclusive packaging efficiency.
- Contact surface accessibility across articulation.
- Effect of articulation geometry on useful contact-normal directions.
- Design-space sensitivity to contact-interface dimensions.
- Robustness of feasible interfaces to manufacturing or assembly perturbations.
- Sensitivity of kinematic behavior to morphology adaptation.
- Spatial separation between contact surfaces and transmission hardware.
- Mechanism-induced contact-surface occlusion.
- Contact accommodation under different operating-posture distributions.
- Architecture-dependent constraints on tactile module installation.
- Coupling between external finger dimensions and internal mechanism geometry.
- Feasible-design diversity and sensitivity to small design changes.
- Contact accessibility under different object approach directions.
- Trade-offs between articulation range, interface thickness, and packaging.
- Conditions under which polycentric motion becomes advantageous or disadvantageous.

These are starting points, not a mandatory list.

Actively seek less obvious hypotheses that emerge from inspecting the repository, existing results, and actual mechanism geometry.

Do not limit exploration to the examples above.

### Phase C — Select Exactly 10 Independent Studies

Select 10 investigations with genuinely distinct primary hypotheses.

Avoid selecting multiple minor variations of the same experiment.

Each investigation must have:

- A unique research question.
- A falsifiable hypothesis.
- A meaningful null or alternative explanation.
- A clear relationship to the North Star.
- A measurable primary outcome.
- A fair comparison protocol.
- A feasible computational implementation.
- An interpretation that remains useful even if CoDe loses.

Prefer diversity across packaging, geometry, kinematics, contact accessibility, adaptation, and robustness.

Do not select all 10 from one category.

Register the study plan before inspecting newly generated outcome data.

After registration, changes are allowed when technically justified, but preserve the original question and document the reason for revision.

---

## 5. STUDY EXECUTION PROTOCOL

For each of `E01`–`E10`, follow the same research lifecycle.

### Step 1 — Formulate

Write a concise study specification containing:

**Question:** What previously unexplored property are we investigating?

**Hypothesis:** What do we expect and why?

**Alternative:** What competing explanation could produce the same observation?

**Mechanisms:** Which architectures should be compared?

**Controlled variables:** What must remain identical?

**Design variables:** What is allowed to change?

**Metrics:** What measurable quantities answer the question?

**Expected failure modes:** What could invalidate the comparison?

**North Star relevance:** Why does the result matter for contact-interface research?

### Step 2 — Reuse Existing Models

Prefer existing repository functionality.

Reuse:

- Mechanism forward kinematics.
- Contact-interface geometry.
- Collision and accessibility evaluators.
- Actuator geometry.
- Morphology descriptors.
- Transmission models.
- Existing optimization adapters.
- Pareto archives and saved candidate designs.
- Study plotting and result infrastructure.

Do not create parallel implementations of the same mechanism mathematics.

When a new model is required, implement the smallest reusable abstraction that answers the research question.

### Step 3 — Pilot Experiment

Start with a small, representative computation.

Use a few meaningful configurations or Pareto candidates to determine:

- Whether the metric is well-defined.
- Whether the model behaves physically.
- Whether the comparison is fair.
- Whether the hypothesis is experimentally distinguishable.
- Whether unexpected numerical artifacts appear.

Do not begin an expensive optimization before completing the pilot.

### Step 4 — Main Experiment

Run the actual comparison using sufficient design-space coverage.

Reuse existing feasible Pareto candidates where applicable.

If new optimization is necessary, justify:

- Why existing candidates are insufficient.
- Which objectives are optimized.
- The search variables and bounds.
- Population and generation budgets.
- Independent seeds.
- Candidate-selection criteria.
- Feasibility constraints.

Maintain fair design opportunities across architectures.

Do not intentionally restrict competing architectures to make CoDe appear better.

Likewise, do not silently grant one architecture extra design freedom.

### Step 5 — Analyze

Report:

- Quantitative results.
- Mechanism-by-mechanism comparisons.
- Design-space distributions, not only single winners.
- Active constraints.
- Sensitivity to important assumptions.
- Negative and unexpected outcomes.
- Possible physical explanations.

Explicitly separate observations from explanations.

For example:

Observation:

"CoDe achieves a shorter best-found morphology for this interface."

Explanation to test:

"Moving ICR reduces the required clearance near the articulated contact region."

Do not present the explanation as established until supported by an appropriate ablation, geometric analysis, or counterfactual.

### Step 6 — Critical Review

Perform an adversarial scientific review.

Ask:

1. Could the result be explained by different design bounds?
2. Could it be caused by an unrealistic competitor configuration?
3. Is it sensitive to actuator orientation?
4. Is it an artifact of an idealized mechanism?
5. Could it disappear under more complete collision geometry?
6. Are apparently feasible designs physically interpretable?
7. Are we comparing equivalent contact interfaces and ROM requirements?
8. Is the effect large relative to optimization variability?
9. Does the result depend on cherry-picked candidates?
10. Does the conclusion actually support the North Star?

Distinguish a serious methodological flaw from an ordinary study limitation.

### Step 7 — Strengthen Where Justified

If the study reveals a promising, interpretable result, perform a focused follow-up.

Choose the smallest experiment that discriminates between competing explanations.

Examples:

- Remove only one mechanical constraint as an ablation.
- Compare matched kinematic trajectories.
- Sweep actuator orientation.
- Test neighboring interface dimensions.
- Evaluate a second operating configuration.
- Compare against a simplified analytical bound.
- Repeat optimization with a justified alternative seed or initial population.
- Inspect configurations near a feasibility boundary.

Do not add more experiments merely to increase apparent evidence.

If the result is negative, record it and move on.

Do not alter the hypothesis after observing the result and present the altered hypothesis as the original one.

### Step 8 — Finalize

Save a reproducible record.

Each exploratory study must finish with one of:

- `SUPPORTED`
- `PARTIALLY_SUPPORTED`
- `NOT_SUPPORTED`
- `INCONCLUSIVE`
- `MODEL_OR_IMPLEMENTATION_BLOCKER`

All five outcomes are scientifically acceptable.

An investigation counts toward the 10 only if its attempted methods, evidence, and interpretation are documented.

A concept proposal with no attempted analysis does not count as a completed investigation.

---

## 6. COMPARISON FAIRNESS

This requirement is critical.

### Common Baselines

Where relevant, compare:

- CoDe crossed four-bar.
- Direct-drive 2R.
- Belt-driven 2R.
- Rolling Contact Joint (RCJ).

Use the repository's existing physical models.

Keep idealized variants explicitly separate.

For example, an ideal RCJ without joint actuator packaging is a theoretical comparison, not an ordinary physical hardware competitor.

### Matched Conditions

As appropriate, control:

- Total finger morphology.
- Proximal and distal contact interface geometry.
- Interface material assumptions.
- Required flexion ROM.
- Actuator size and model.
- Contact mounting policy.
- Object or contact geometry.
- Evaluation poses.
- Collision-clearance rules.

Not every study must freeze every variable.

However, every difference must be intentionally selected and documented.

### Fair Optimization

When architectures have different parameterizations, compare their feasible design spaces, not merely their parameter counts.

Do not equate one architecture's best-found solution with another architecture's arbitrary reference configuration.

Do not report best-found optimization results as globally optimal.

### Physical Realism

Retain a consistent abstraction level.

Use geometry, articulation, contact surfaces, hardware occupancy, and transmission behavior relevant to the research question.

Do not turn the exploration into detailed hardware production engineering.

However, do not ignore a physical constraint that directly determines the claimed advantage.

Treat 2D geometry as a screening approximation.

Claims about actual 3D contact accessibility, hardware integration, or mechanical feasibility require appropriately stronger evidence.

---

## 7. RESEARCH INTEGRITY

Do not optimize for positive conclusions.

Maintain a complete study registry, including unfavorable and inconclusive results.

Report all 10 investigations in the final synthesis.

Avoid:

- Post-hoc metric selection designed to favor CoDe.
- Selective exclusion of difficult interface configurations.
- Treating multiple Pareto candidates as independent statistical trials.
- Claiming significance from optimization seeds alone.
- Interpreting search failure as structural impossibility.
- Confusing smallest morphology with smallest morphology change.
- Confusing contact-interface accommodation with better manipulation performance.
- Claiming improvement in learned manipulation without corresponding evidence.
- Claiming that a mechanism property is caused by moving ICR without isolating that effect.
- Claiming physical contact mechanics from purely geometric overlap tests.

If a finding depends on a narrow or unrealistic assumption, state that explicitly.

A negative result that exposes an architectural limitation is more valuable than a misleading positive result.

---

## 8. RESOURCE AND IMPLEMENTATION DISCIPLINE

Use an evidence-first workflow.

1. Reuse existing results before launching new computations.
2. Run small pilots before expensive searches.
3. Validate new evaluators with focused checks.
4. Reuse existing analyses whenever the requested quantity is already available.
5. Prefer analytical or geometric calculations when they answer the question.
6. Avoid unnecessary FEM, high-fidelity simulation, or large optimization campaigns.
7. Use computationally expensive methods only when simpler models cannot resolve the scientific question.

Keep changes localized.

Avoid:

- Unrelated refactoring.
- Duplicate mechanism implementations.
- Large speculative abstraction layers.
- New optimization frameworks when the existing one is sufficient.
- Repeated broad test runs.
- Unnecessary repository restructuring.

Use the simplest implementation that produces defensible evidence.

If an investigation becomes disproportionately expensive relative to its scientific value, document the reason and select a different independent hypothesis.

Never silently simplify a physical assumption to force a result.

---

## 9. AUTONOMOUS SCIENTIFIC JUDGMENT

You are authorized to make routine research decisions autonomously.

Do not repeatedly ask the user:

- Which hypotheses to test.
- Which plots to generate.
- Which minor implementation approach to use.
- Whether a negative result should be retained.
- Whether a focused follow-up is worthwhile.
- Which study should run next.

Make those decisions based on scientific value, available evidence, and the established North Star.

However, do not silently redefine the main project objective.

Maintain two distinct levels of reasoning:

**Research exploration:** What new property might be worth investigating?

**Research validation:** What evidence would establish or falsify that property?

Keep these separate.

When new evidence suggests a more valuable direction, adapt the remaining exploratory studies while preserving completed work and documenting the change.

Use your independence to discover better questions, not to broaden the project indefinitely.

---

## 10. DELIVERABLES

Create a dedicated exploratory research workspace under the repository's existing study conventions.

Use the repository's current structure rather than imposing a large new hierarchy.

### A. Research Registry

Document:

- At least 20 initial candidate hypotheses.
- The selected 10 independent investigations.
- Selection rationale.
- Primary metrics and comparison controls.
- Any later methodological changes.
- Final status of each study.

### B. Ten Study Reports

Each study report must include:

1. Research question.
2. Hypothesis and alternative.
3. Model and assumptions.
4. Comparison methodology.
5. Quantitative results.
6. Interpretation.
7. Critical review.
8. Focused follow-up, if justified.
9. Limitations.
10. Final scientific takeaway.

### C. Reproducible Artifacts

For each study, save as appropriate:

- Configuration files.
- Candidate design identifiers.
- Optimization inputs and selected vectors.
- Machine-readable result tables.
- Generated figures.
- Run manifests.
- Source commit and working-tree provenance.
- Scripts needed to reproduce the analysis.

Never fabricate missing optimization results or regenerate figures from manually invented data.

### D. Cross-Study Research Synthesis

Produce a single synthesis report comparing all 10 investigations.

Identify:

- Consistent architectural trends.
- Unexpected advantages.
- Unexpected disadvantages.
- Findings replicated across different metrics or conditions.
- Potentially misleading optimization artifacts.
- Mechanism-level causal explanations supported by evidence.
- Remaining model limitations.

Distinguish between repeated evidence from genuinely different methods and multiple metrics derived from the same simulation.

### E. Paper-Relevance Ranking

Rank findings by:

1. Relevance to the North Star.
2. Strength and reproducibility of evidence.
3. Novelty of the architectural insight.
4. Clarity of the physical explanation.
5. Fairness against established alternatives.
6. Practical importance for contact-interface research.
7. Complexity required to communicate or validate the claim.

Prioritize findings that are both scientifically meaningful and easy to communicate.

Do not recommend adding all 10 studies to a paper.

Select only the strongest findings that reinforce a coherent contribution.

---

## 11. INDEPENDENT REVIEW

After completing the investigations, use one independent read-only reviewer to examine the actual study specifications, implementations, results, and synthesis.

The reviewer must not modify code.

Review:

- Whether 10 genuinely distinct studies were attempted.
- Whether results are supported by saved evidence.
- Whether comparisons are fair.
- Whether key assumptions are explicit.
- Whether conclusions exceed the model's validity.
- Whether negative outcomes were retained.
- Whether any purported advantage is explained rather than merely observed.
- Whether suggested contributions genuinely serve the North Star.

Reviewer output:

**VERDICT:** PASS / PASS_WITH_NOTES / BLOCK

**BLOCKERS:** Only concrete validity or implementation issues.

**NOTES:** Non-blocking scientific limitations and recommendations.

Fix real blockers only.

Do not repeatedly review completed work merely to increase confidence.

---

## 12. FINAL RESEARCH REPORT

The final report must begin with a compact overview of all 10 studies.

Use this structure:

| ID | Research Question | Result | Evidence Strength | North Star Relevance | Recommendation |
|---|---|---|---|---|---|

Then provide:

### 1. Top Three Discoveries

For each:

- What was discovered?
- Why was it unexpected?
- Which architecture comparisons support it?
- What mechanical principle explains it?
- What evidence remains missing?

### 2. Strongest Negative Findings

Identify important limitations of CoDe and conditions where competing architectures are preferable.

### 3. Potential Paper Contributions

Suggest a maximum of three findings that could materially strengthen the CoDe paper.

For each finding, draft a one-sentence claim supported by the available evidence.

Do not promote unsupported hypotheses into contributions.

### 4. North Star Assessment

Explain whether the new findings strengthen, weaken, or refine the original motivation for CoDe Hand.

Do not change the North Star merely because a result is disappointing.

### 5. Recommended Next Experiments

Recommend only experiments required to resolve important remaining uncertainty.

Distinguish:

- Essential before publication.
- Valuable but optional.
- Separate future research.

### 6. Implementation and Verification Summary

Report:

- Implementations added or changed.
- Focused tests and results.
- Studies completed.
- Inconclusive or blocked investigations.
- Reviewer verdict.
- Reproducibility status.

---

## 13. HARD STOPS

Stop the affected line of investigation and report its blocker if:

- The required mechanism model does not exist.
- Existing kinematics cannot support the proposed metric without substantial redesign.
- A necessary physical assumption cannot be justified.
- A result would require fabricating unavailable measurements.
- A comparison cannot be made scientifically fair within the existing abstraction.
- A proposed investigation requires substantially more complex simulation than its likely value justifies.
- The research question shifts into a fundamentally different project.

If possible, replace the blocked investigation with another registered, independent and feasible question.

Do not silently expand scope.

Do not discard valid negative results.

Do not commit or push repository changes without explicit authorization.

---

## 14. DEFINITION OF SUCCESS

Success means:

1. Ten distinct, falsifiable research studies have been attempted and critically interpreted.
2. The studies provide reproducible quantitative or analytical evidence.
3. Positive, negative, and inconclusive outcomes are all documented.
4. Important claims survive fair mechanism comparisons and focused scientific review.
5. The strongest findings have clear mechanical explanations.
6. New discoveries are connected explicitly to CoDe Hand's North Star.
7. Existing research infrastructure has been reused without unnecessary complexity.
8. A final synthesis identifies which findings deserve inclusion in the paper.

**The goal is not to prove that CoDe is universally better.**

**The goal is to discover what CoDe uniquely enables, under what conditions, why those conditions matter, and whether those properties justify a contact-centric hand architecture.**

Begin by inspecting `GOAL.md`, the current repository state, and the existing Study 1–4 evidence. Then register candidate hypotheses, select 10 independent investigations, and execute the full research workflow autonomously.