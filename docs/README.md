# Documentation Index

Start with [GOAL.md](../GOAL.md) for the research north star and claim discipline, and the root [README.md](../README.md) for the repository layout and the separation between **prototype** and **study** optimizations.

## Active Documentation

| Location | Scope |
|---|---|
| [GOAL.md](../GOAL.md) | Research objective, embodiment invariance $R = (H, C)$, Studies 1–4 & A–D, claim discipline |
| [README.md](../README.md) | Monorepo layout, environment overview, prototype vs. study rules, coordinate conventions |
| [report/PROGRESS.md](../report/PROGRESS.md) | Current defensible claims (C1–C7), evidence boundaries, limitations, and next validation steps |
| [report/README.md](../report/README.md) | Index of all current and snapshot study/prototype reports under `report/` |
| [design/SSoT/problem_formulation.md](../design/SSoT/problem_formulation.md) | **Source of truth** (paper Appendix A, Korean): morphology $M=(K,C,A,S)$, static/motion feasibility, cylinder-based pad coverage, prototype formulation, Study 1 |
| [docs/formulation.md](formulation.md) | Pointer to `design/SSoT/problem_formulation.md` |
| [docs/decisions.md](decisions.md) | Chronological engineering and study-formulation decision log (through 2026-11-18) |
| [design/scripts/study/study1_fixed_morphology/README.md](../design/scripts/study/study1_fixed_morphology/README.md) | Study 1 specification: fixed-morphology contact accommodation and active tilt sweep |
| [design/scripts/study/study2_morphology_adaptation/README.md](../design/scripts/study/study2_morphology_adaptation/README.md) | Study 2 specification: E/Q adaptation hierarchy and ROM-conditioned baselines |
| [design/scripts/study/study2_compact_morphology/README.md](../design/scripts/study/study2_compact_morphology/README.md) | Study 2 extension: compact finger morphology search |
| [design/scripts/study/study3_transmission/README.md](../design/scripts/study/study3_transmission/README.md) | Study 3 specification: local transmission-reflected contact mechanics |
| [design/scripts/study/study4_joint_interface/README.md](../design/scripts/study/study4_joint_interface/README.md) | Study 4 specification: geometric screening of continuous joint bridges |
| [sim/isaaclab/README.md](../sim/isaaclab/README.md) | Isaac Lab 3.0 / Newton setup, physics backend presets, and contact-model experiments |
| [sim/isaaclab/scripts/code_hand/README.md](../sim/isaaclab/scripts/code_hand/README.md) | Closed four-bar loop closure, XM335 actuator law, and pad collider checks (E1–E7) |

## Archived / Legacy Notes (`docs/legacy/`)

Documents under `docs/legacy/` record earlier formulations (`palmar-pad-v3`, `linkage-v2`, `contact-retention-v1`) and the deleted `scripts/optimize/comparison/` pipeline from October 2026. Kept for provenance only—do not use them as specifications for current code:

- [legacy/math_formulation_2026-11-17.md](legacy/math_formulation_2026-11-17.md) — 2026-11-17 2D SSoT aligned to the evaluators; superseded on 2026-11-18 by `design/SSoT/problem_formulation.md`, which is written to intended function rather than to the code
- [legacy/linkage_v2_sweeps.md](legacy/linkage_v2_sweeps.md) — Formulation v2 (`linkage-v2-rev2`) ROM and pad-length sweeps
- [legacy/optimization/linkage.md](legacy/optimization/linkage.md) — Formulation v1 (`palmar-pad-v3`) linkage seed-generator design doc
- [legacy/optimization/retention.md](legacy/optimization/retention.md) — Early `contact-retention-v1` comparison metric
- [legacy/optimization/decision-points-2026-10-22.md](legacy/optimization/decision-points-2026-10-22.md) — Pre-v2 formulation decision memo
- [legacy/optimization/comparison-bounds.md](legacy/optimization/comparison-bounds.md) — Bound classification for the deleted `scripts/optimize/comparison/` study
- [legacy/tasks/design-comparison-task.md](legacy/tasks/design-comparison-task.md) & [legacy/tasks/paper-figures-task.md](legacy/tasks/paper-figures-task.md) — Superseded October 2026 comparison task briefs
