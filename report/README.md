# Reports

Reports and paper figures from runs under the current SSoT
(`design/SSoT/problem_formulation.md`, `design/SSoT/code_formulation.md`).

Layout: each study's reports and figures in its folder (`study1/`, `study2/`, figures in
`<study>/figures/`); the prototype's report and the formulation figures at the top (`figures/`);
anything superseded in `legacy/`. Study figures are made here, not under `design/output/`.

| Report | What |
|---|---|
| [code_prototype_2026-11-19.md](code_prototype_2026-11-19.md) | CoDe prototype, paper preset: $\bar C_{\mathrm{pad}}=0.720$ |
| [study1/study1_fixed_morphology_2026-11-19.md](study1/study1_fixed_morphology_2026-11-19.md) | Study 1: 71 modules × 5 architectures × tilt × $q_{\mathrm{req}}$ |
| [study1/study1_fixed_morphology_native_2026-11-20.md](study1/study1_fixed_morphology_native_2026-11-20.md) | Study 1 on native references (18 = 6 architectures × $q_{\mathrm{req}}$), pad coverage figure |
| [study2/figures/study2_quick_overview_2026-11-20.png](study2/figures/study2_quick_overview_2026-11-20.png) | Study 2a/2b quick (q_req 120°), preliminary; the full run's report follows |

## Legacy

Everything written before the SSoT rules (single objective $\bar C_{\mathrm{pad}}$,
cylinder coverage, $MA_{\min}=0.2$, Study 2 at 90/120/150 deg) is in
[`legacy/`](legacy/README.md), as it was on 2026-11-19. Its numbers come from
the old formulations and are not comparable with new runs. The matching run
records are in `design/output/legacy/pre_ssot_2026-11-19/` (not tracked).

Study 2 as the least change of morphology (2026-11-19/20: the Level 0–2 adaptation, its 8-target
report and the landscape pilot) was replaced by the re-optimization of Study 2a/2b (SSoT A.6.3–A.6.4):
[`legacy/study2_morphology_adaptation_2026-11-19.md`](legacy/study2_morphology_adaptation_2026-11-19.md),
the figures in `legacy/figures/study2_*`, and the runs in `design/output/legacy/study2_min_change_2026-11-20/` (not tracked).
