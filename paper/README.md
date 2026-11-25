# Paper workspace (RSS 2027)

**How the manuscript is written (format, words, figures, review loop) is fixed in [SSoT.md](SSoT.md), which takes precedence over this README.** The notes below on stages and scope are historical.

Written by following the PaperOrchestra stages (google-research/paper-orchestra) with Claude
instead of its Gemini/OpenAI backend; no project material is sent to an external LLM API.

| Stage | Output | Status |
|---|---|---|
| 0. Raw materials | `raw_materials/idea_sparse.md`, `raw_materials/experimental_log.md` | done |
| 1. Outline | `outline.md` | done |
| 2. Literature review | `lit_review/refs.bib` (copied to the manuscript repo as `references.bib`), `lit_review/related_work_notes.md` (each entry web-verified) | done |
| 3. Section writing | manuscript repo `dokkev/ContactCentricDesign-RSS2027` (Overleaf GitHub sync) | first draft, all sections |
| 4. Content refinement | review pass against SSoT and reports, numbers re-checked | first pass |
| 5. Plotting | paper-style figures | TODO (figures are read from `report/*/figures`) |

Scope (RSS 2027 Stage 1): hypotheses H1–H4. Studies 0–2 are the computational validation of H1–H2; Study 3C (H3) and the prototype interface exchange (H4) are specified as Stage-2 experiments, without results.

## Manuscript

The LaTeX lives in the private repo `dokkev/ContactCentricDesign-RSS2027`, synced with Overleaf, cloned at
`../ContactCentricDesign-RSS2027` (sibling of this repo). It is the only copy of the manuscript: edit it
there or on Overleaf, and pull before editing. Figures are copied from `report/` by `paper/sync_figures.sh`
(PNG only; `crop_figures.py` cuts compact strips from the report figures until they are redrawn).
Target: RSS 2027 extended abstract, **6 pages = 5 pages of content + 1 page of references**. As of
2026-11-21 the content ends on page 5 (≈3,700 words) and the references on page 6, so any addition must
be paid for by a cut. The manuscript `references.bib` has no URL/DOI fields (page limit); the verified
URLs stay in `lit_review/refs.bib`.

Build locally (no sudo): `conda run -n tex tectonic -X compile main.tex --outdir <scratch dir>` in the
manuscript repo. Do not commit the PDF.

Section files: `03_design.tex` holds the formulation and Study 0; `04_experiments.tex` holds Studies 1–2
and the discussion.

## Before submission

- Re-check every number against `design/output/study/` (the log copies the reports).
- Numbers from the 2026-11-20 partial pole-law run (`report/study2/study2.md`): the fixed-finger
  first-contact angles (65° vs 92°) and the adjacent-thickness contact/skeleton changes (0.84 / 3.54 mm).
  Re-run `pole_law.py` on the final Study 1 designs and update.
- Redraw the four figures for the paper (they are cropped report figures now).
- Search `\todo{` in the manuscript repo.

## References for writing

- [docs/howto_research_paper_guidelines_garg.md](docs/howto_research_paper_guidelines_garg.md): structure checklist (problem → why/so-what → prior work → intuition → contributions → technical details → setup & metrics → experiments; Fig. 1 = "money figure").
