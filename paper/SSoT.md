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
  Nothing beyond the 6 pages is reviewed. Due 2026-12-04; re-check the dates on roboticsconference.org.
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

### 4.2 Terms (define once, never vary)

| Term | Meaning |
|---|---|
| contact-surface motion, c(q) | path of the distal-face point below the fingertip in the proximal-phalanx frame F_p |
| contact Jacobian (PIP column) | J(q) = [c'(q); 1]; used where the claim is stated, not as a running synonym |
| skeleton motion, r_C(q) | fingertip path written from the proximal link line |
| Δ_contact, Δ_skel | their largest changes over [0, q_req], in mm |
| pole, P(q) | finite pole of the distal phalanx relative to the proximal one (Lemma 1) |
| face depth t | depth of a pad's contact face below the link line, set by the mounting rule |
| the mounting rule | each pad at the shallowest depth clear of its joint hardware, no spacers |
| the goal | the common requirements of every design search |
| CFL | the crossed four-bar of the study; CoDe is the hand |
| O, R, U, M | reference hand; redesign that keeps Δ_contact small; coverage-only design; a middle redesign |
| proximal pad, distal pad | never "near/far pad" |

In the introduction, "the joints and links" stands for the kinematic layout; no new term is introduced for it.

## 5. Writing

- Two references, for two different things:
  - **Register and structure (how it reads as engineering):** Bhatia, Johnson and Mason, "Direct Drive Hands:
    Force-Motion Transparency in Gripper Design" (RSS 2019, the DDHand paper), and Wensing et al.,
    "Proprioceptive Actuator Design in the MIT Cheetah: Impact Mitigation and High-Bandwidth Physical
    Interaction for Dynamic Legged Robots" (IEEE T-RO 2017). Both are mechanism-design papers: a physical
    requirement motivates a design principle; the principle is made measurable by a design metric defined
    from first principles; simple models explain the trade-off against conventional designs; the comparison
    and the validation follow from that metric. Plain, precise engineering prose, the design rationale stated
    explicitly, each quantity defined before it is used.
  - **Voice, after Wensing:** "we" is not needed. The subject of a sentence is the design, the mechanism or the
    analysis ("The CFL keeps …", "This section derives …", "Each pad is mounted …"); the passive is used where
    the actor does not matter.
  - **Delivery (how fast the message lands):** Vaswani et al., "Attention Is All You Need" (NeurIPS 2017).
    One message per paragraph, stated in its first sentence; short declarative sentences; few parentheses and
    no semicolon chains; one or two numbers per sentence, the rest in tables and
    figures; nothing the reader must hold in mind before it pays off.
- Introduction: no equations and no experimental numbers. Say a result in words and point to its section.
- State results directly. No sentence that concedes or cuts down the work to forestall a reviewer
  ("That much is expected", "admittedly"). Answer an expected objection with the stronger result. Scope that
  is part of a result ("in simulation", "under a reused setpoint") stays.
- No detailed self-critique in the body. Limitations go in one big-picture paragraph of the conclusion,
  each with its next step. Never make a claim that is true only with a caveat; narrow the claim instead.
- Headings: `\subsection` and `\subsubsection`, never bold run-in paragraph heads. Short titles that say
  what the part shows.
- Respect north-star's Not claimed, for example: interfaces gather at the fingertip is an observation, not a
  consequence of the lack of room; interface design exists and is not denied.

## 6. Figures

- Draw at print size and include at scale 1: 3.45 in for a column, 7.16 in for the page; labels 6–7 pt.
  Prefer column floats; define a float in the section that discusses it; look at the page before committing.
- PNG only, never PDF figures. Save opaque RGB (`paper/figures/pngout.save_rgb`): an alpha channel becomes a
  soft mask in the PDF, which submission checkers flag. Keep plots with many points raster, so no point
  becomes a PDF object.
- Scripts live in `paper/figures/` and write into the manuscript's `figures/`. Long computations are cached
  under `design/output/paper/` (git-ignored); delete a cache to recompute.
- Before submission, on the Overleaf PDF: `pdfimages -list` shows no `smask`, `pdffonts` shows every font
  embedded and no Type 3.

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
