# Paper SSoT — how the manuscript is written

This file fixes how the RSS 2027 manuscript is written: its format, its words, its figures and how it is
revised. It does not decide what the paper claims. The problem, the contributions, what is not claimed and
the evidence are in `design/SSoT/` and are not repeated here.

## 1. Precedence and sync

| Order | Source | Decides |
|---|---|---|
| 1 | `design/SSoT/north-star.md` | the story, the contributions, what is not claimed |
| 2 | `design/SSoT/research-objective.md`, `problem_formulation.md` | the evidence each claim needs, the mathematics, the results' status |
| 3 | this file | how the manuscript says it |
| 4 | the manuscript (`../ContactCentricDesign-RSS2027`) | — |
| 5 | `paper/orchestra/` inputs (`idea.md`, `experimental_log.md`) | — |

- When a higher source changes, the lower ones are fixed to match, in this order. The manuscript always
  follows `design/SSoT`.
- If `design/SSoT` contradicts itself, flag it; do not pick one reading silently.
- A rule in this file overrides any note, memory or session that says otherwise. Change this file first,
  then the manuscript.

## 2. Format and build

- RSS 2027 extended abstract, Stage 1: at most 5 pages of content and 1 page of references (6 pages in all).
  Nothing beyond the 6 pages is reviewed.
  Length is deferred (user, 2026-11-24): write for the storyline first and fit the five pages in a final pass;
  do not cut content during revisions to stay within the page limit. Due 2026-12-04; re-check the dates on roboticsconference.org.
- Template: RSS (IEEEtran, natbib `plainnat`). Anonymous review.
- Manuscript repo: private `dokkev/ContactCentricDesign-RSS2027`, synced with Overleaf, cloned at
  `../ContactCentricDesign-RSS2027`. It is the only LaTeX copy: pull before editing.
- Local build: `conda run -n tex tectonic -o <scratch dir> main.tex`. Tectonic falls back to Latin Modern;
  the submission PDF is the Overleaf (pdfLaTeX) build, which gives the template's Times.
- `references.bib` in the manuscript carries no URL or DOI fields (page limit); the verified entries with
  URLs are in `paper/lit_review/refs.bib`. Cite only entries in that file.

## 3. Where the story goes

The story is north-star's "How the paper tells it". The manuscript places it as follows.

| Manuscript | North-star |
|---|---|
| Abstract | In one sentence; C1; C2.1–C2.3; the reversed conclusion as the one number |
| Introduction ¶1 | context: contact uncertainty compensated with the rest of the hand, hence servo hands; the interface is fitted to the room left |
| Introduction ¶2 | the interface matters, its own effect is hard to know; first obstacle, room (P1) |
| Introduction ¶3 | second obstacle, the finger changes (P2); a fixed axis cannot keep the contact surfaces' motion |
| Introduction ¶4 | "In this work we propose CoDe"; the two contributions as a list |
| III Design: Model, Pole, Optimization | the pole answers both problems; the two optimization problems |
| III Design: Coverage across joints | evidence for C1 |
| III Design: Redesign for thicker pads | C2.1, C2.2 (what keeping the contact Jacobian costs) |
| IV Experiment (simulation) | C2.3 (what keeping it buys) |
| V Conclusion; Planned hardware test | the hardware stage; big-picture limitations and future work |

Section structure: I Introduction, II Related Work, III Design, IV Experiment, V Conclusion. Design holds the
model, the analysis and the design studies; Experiment holds simulation only.

## 4. Words

### 4.1 Never in the manuscript

| Not written | Why / written instead |
|---|---|
| Study 0, 1, 2, 3, 4 (also in titles and captions) | codebase names; name what the part shows, refer by Sec./Fig. |
| C1, C2, P1, H1–H3, R-claims, Q1–Q3 | SSoT labels; write the claim in words |
| a "Hypothesis" label | state the expectation in prose: "If …, it should …; … would refute this" |
| relative contact trajectory, RCT, contact-centric kinematics, D_c, D_s | replaced by the terms below |
| mount depth d | one depth symbol, the face depth t |
| A'', B'' | construction points of the code; the paper uses r_{1,e}, r_{2,s} |
| center of rotation (as a term), skeletal motion, skeletal kinematics, contact-surface kinematics | one name per quantity: pole, skeleton motion, contact-surface motion |
| a mechanism synthesis method | the optimization formulation (two objectives solved with GA/NSGA-II) |

### 4.2 Terms (define once, never vary)

| Term | Meaning |
|---|---|
| contact-surface motion, c(q) | path of the distal-face point below the fingertip in the proximal-phalanx frame F_p |
| contact Jacobian (PIP column) | J(q) = [c'(q); 1]; used where the claim is stated, not as a running synonym |
| skeleton motion, r_C(q) | fingertip path written from the proximal link line |
| Δ_contact, Δ_skel | their largest changes over [0, q_req], in mm |
| pole, P(q) | the point about which the distal phalanx rotates relative to the proximal one (finite pole). Introduced once with this gloss; "center of rotation" is not used as a second name |
| face depth t | depth of a pad's contact face below the link line, set by the mounting rule |
| the mounting rule | each pad at the shallowest depth clear of its joint hardware, no spacers |
| the goal | the common requirements of every design search |
| CoDe | the crossed four-bar PIP finger in the design studies and the hand; "CFL" is not written (user, 2026-11-25: readers who skim should not need the acronym). In Fig. 2 the rolling contact joint is written out; elsewhere RCJ may stay |
| O, R, U, M | reference hand; redesign that keeps Δ_contact small; coverage-only design; a middle redesign (codebase labels) |
| CoDe, Conventional, Coverage | the three compared hands (user, 2026-11-27): **CoDe** = R, the contact-centred redesign (least Δ_contact; R is a contact-centred design, so the paper does not wait for the pins-free reruns); **Conventional** = skeleton minimization, the same mechanism and search with the least Δ_skel at pad coverage no more than 0.05 below CoDe's (`design/scripts/study/exploratory/e25_skeleton_min.py`); **Coverage** = U, coverage only. Before 2026-11-27 the paper called U "Conventional" |
| proximal pad, distal pad | never "near/far pad" |

In the introduction, "the joints and links" stands for the kinematic layout; no new term is introduced for it.

## 5. Writing

- References (Wensing's register takes precedence when they pull apart):
  - **Register and structure (how it reads as engineering):** Bhatia, Johnson and Mason, "Direct Drive Hands:
    Force-Motion Transparency in Gripper Design" (RSS 2019, the DDHand paper), and Wensing et al.,
    "Proprioceptive Actuator Design in the MIT Cheetah: Impact Mitigation and High-Bandwidth Physical
    Interaction for Dynamic Legged Robots" (IEEE T-RO 2017). Both are mechanism-design papers: a physical
    requirement motivates a design principle; the principle is made measurable by a design metric defined
    from first principles; simple models explain the trade-off against conventional designs; the comparison
    and the validation follow from that metric. Plain, precise engineering prose, the design rationale stated
    explicitly, each quantity defined before it is used.
  - **Voice:** "we" is allowed but not overused. The introduction and the conclusion are written to draw the
    reader in, and may say "we introduce", "our intuition", "our main contributions". In the design and
    experiment sections the subject is the design, the mechanism or the analysis ("The CFL keeps …"), with the
    passive where the actor does not matter.
  - **Presenting design decisions:** Chi et al., "Universal Manipulation Interface: In-The-Wild Robot
    Teaching Without In-The-Wild Robots" (RSS 2024). Each design choice is introduced by the requirement it
    serves and the failure it prevents, so the reader sees why the system looks as it does before seeing how
    well it works. Use it for the CoDe mechanism and for the design studies: requirement, choice, consequence.
  - **Delivery (how fast the message lands):** Vaswani et al., "Attention Is All You Need" (NeurIPS 2017).
    One message per paragraph, stated in its first sentence; sentences joined by explicit logical connectives
    (thus, in contrast, as a result, to this end) as in Wensing, not chopped into fragments; few parentheses and
    no semicolon chains; one or two numbers per sentence, the rest in tables and
    figures; nothing the reader must hold in mind before it pays off.
- Abstract: in the shape of the Attention abstract: the dominant approach, its limitation, the proposal ("This paper proposes …, dispensing with …"), what the analysis and the experiments show, one or two numbers.
- Introduction: no equations and no experimental numbers. Say a result in words and point to its section.
- State results directly. No sentence that concedes or cuts down the work to forestall a reviewer
  ("That much is expected", "admittedly"). Answer an expected objection with the stronger result. Scope that
  is part of a result ("in simulation", "under a reused setpoint") stays.
- No detailed self-critique in the body. Limitations go in one big-picture paragraph of the conclusion,
  each with its next step. Never make a claim that is true only with a caveat; narrow the claim instead.
- Headings: `\subsection` and `\subsubsection`, never bold run-in paragraph heads. Short titles that say
  what the part shows.
- The paper is a design-principle paper. The principle (place the pole by the contact surfaces; redesign by
  preserving contact-surface motion, not skeleton motion) and what follows from it (the fixed-axis bound, the
  CFL finger, the simulated test) are the contributions. Pole kinematics and the clearance relations are
  classical: cite them and state the final result, never list them as a contribution or an "analysis" of ours.
- Respect north-star's Not claimed, for example: interfaces gather at the fingertip is an observation, not a
  consequence of the lack of room; interface design exists and is not denied.

## 6. Figures

- Draw at print size and include at scale 1: 3.45 in for a column, 7.16 in for the page; labels 6–7 pt.
  Prefer column floats; define a float in the section that discusses it; look at the page before committing.
- PDF for Overleaf, PNG for preview (user, 2026-11-26): every figure is saved by
  `paper/figures/pngout.save_figure` as `figures/<name>.pdf`, which the manuscript includes, and
  `figures/<name>.png`, the opaque preview. No RGBA image may enter a figure: flatten it onto its
  background first (an alpha channel becomes a soft mask, which submission checkers flag; `save_figure`
  refuses one). Fonts embed as TrueType (`register_fonts` converts the CFF fonts once into
  ~/.cache/paper_fonts, never committed). Keep plots with many points raster, so no point becomes a PDF
  object.
- Hand, finger and sensor images are CAD renders (Onshape shaded views, `paper/figures/fetch_cad.py`) or renders of the visual USD used in Isaac Lab (`sim/isaaclab/scripts/code_hand/render_visuals.py`), never the Drake finger renders (render_hands.py); the Drake task renders in report/study3/figures are approved (2026-11-24).
- Scripts live in `paper/figures/` and write into the manuscript's `figures/`. Long computations are cached
  under `design/output/paper/` (git-ignored); delete a cache to recompute.
- Before submission, on the Overleaf PDF: `pdfimages -list` shows no `smask`, `pdffonts` shows every font
  embedded and no Type 3.
- Style reference (user, 2026-11-26): follow figures4papers
  (https://github.com/ChenLiu-1996/figures4papers, `scientific-figure-making/SKILL.md` and
  `references/design-theory.md`) when making or restyling a figure. Take from it: top and right spines off,
  frameless legends (a separate legend strip when panels are crowded), no grid or a faint one, explicit
  y-limits around the data, 2–4 curves per axis with `fill_between` bands for spread, the same font sizes,
  line widths and colour meaning across every panel of a row, the method of interest in the strong blue and
  the comparator in a warm red family with neutral greys for references, hatching or markers where print in
  grey must still separate series. The rules above win where they differ: print size with 6–7 pt labels
  (not its 16–24 pt canvases), PNG only (not its PDF export), Helvetica Light.

### 6.1 Figure plan (user, 2026-11-24)

Font: Helvetica Light for figure text (bold and math italic in Nimbus Sans, which the Light cut lacks). Physical meaning first: no trend lines, Pareto fronts or statistics (ρ, p) in figures; statistics, the selection rule
and coverage retention go in the text. White background, thin technical linework, LaTeX-style math, restrained
colour: CFL / R blue, 2R-remote / U amber, 2R-DD / reference grey, RCJ teal, pole purple. Final plots come from the
evaluator and the raw CSV/JSON (`paper/figures/fig_paper.py`), never from hand-drawn values.

| Figure | Role | Content |
|---|---|---|
| Fig. 1 | overview | the hand manipulating (USD), the CFL finger (CAD) with room for interfaces, contact- vs skeleton-centred redesign, interfaces incl. NARI-Touch |
| Fig. 2 | kinematic principle (double column, 45–50 mm) | (a) clearance set by the pole, (b) achievable poles (CFL, RCJ, fixed axis), (c) motion seen from the skeleton frame and the contact frame; no mechanism detail |
| Fig. 3 | design validation (double column, 65–75 mm) | (a) optimized geometry at 120°, 7 mm (evaluator: pads light, cylinder-holding surface dark, mechanism grey, extended + flexed) and coverage dots; (b) reference–redesign paths at 120°, 3→7 mm, and Δ_contact dots with the fixed-axis value dashed |
| Fig. 4 | scientific validation (double column, 70–80 mm) | one column per task: the task, R's and U's error, every redesign's error against Δ_contact |
| Fig. 2(b) bounds | every joint's achievable-pole region uses the same size rule as the CFL's design space (pivots on the link line within the finger; RCJ 2r <= 90 mm); no extra cap on any joint |
| Table I | contact-model robustness | R and U errors per contact model (medians per design's valid conditions, not matched condition by condition) |

## 7. Numbers and their sources

To be filled before submission: every number in the abstract, the design studies, the experiment and the
conclusion, with the report or output file it comes from (`report/`, `design/output/`) and the commit.

| Number in the manuscript | Section | Source | Checked |
|---|---|---|---|

## 8. Revision and review

- Reviews follow the PaperOrchestra AgentReview step with Claude reviewers: three independent reviewers,
  `paper/orchestra/prompt_reviewer.txt` verbatim, each reading the built PDF (for the figures) and its text.
  Reviewers see only the paper, no earlier reviews or repository files.
- Store each round as `paper/orchestra/reviews/vN_paper.txt` and `vN_r{1,2,3}.md`, and log the scores and
  the common critiques in `paper/orchestra/refinement_log.md`.
- During a revision round, commit and push both repositories after every revision (manuscript `main`,
  `cmd_framework` `feature/2dof-fe`). Commit messages are plain, without attribution lines. Never commit
  `INSTRUCTION.md` or PDFs.
