# Study 2: morphology adaptation, rerun under the SSoT (2026-11-19)

**Track: study** (`design/scripts/study/`). Formulation: `design/SSoT/problem_formulation.md` A.6.3.

## Setup

| Item | Value |
|---|---|
| Config | `design/scripts/study/study2_morphology_adaptation/config.yaml` |
| References | `references_nominal15.yaml`, kind `matched`: one frozen morphology ($L_f(0)=90$ mm), each architecture built to it |
| Targets | 8 contact-module pairs (`targets.yaml`: baseline, long, thick, long_thick, two asymmetric-long, two asymmetric-heavy) |
| $q_{\mathrm{req}}$ | 90°, 120°, 150° |
| PIP servo tilt sweep | 0°–90° in 15° steps for CoDe, 2R-DD, 2R-remote and RCJ; the idealized RCJ at its nominal (29 references) |
| Search | Level 1 (mechanism only, $\min \varepsilon_\kappa$), then Level 2 (external too, $\min \varepsilon_d$, then $\min \varepsilon_\kappa$); NSGA-II, population 100, 100 generations, seeds 0, 1, 2 (best of three kept) |
| Changes | normalized by the reference's $L_f$; Level 2 moves each external descriptor at most 20 mm |
| Output | `design/output/study/study2_morphology_adaptation_tilt_sweep/` (`summary_best.csv`: 696 reference–target–$q_{\mathrm{req}}$ pairs) |
| Source | commit `941c092` (the run's start) |

## Outcome at each architecture's nominal tilt

Each cell counts the 24 pairs (8 targets × 3 $q_{\mathrm{req}}$) by how the target was met. $\varepsilon_d$ is the
external change of the adapted pairs (fraction of $L_f$).

| Architecture (tilt) | Frozen feasible | Level 1 | Level 2 | Not found | $\varepsilon_d$ median (mean) |
|---|---|---|---|---|---|
| CoDe (15°) | 6 | 0 | 18 | **0** | 0.089 (0.096) |
| 2R-DD (0°) | 3 | 0 | 17 | 4 | 0.109 (0.112) |
| 2R-remote (15°) | 5 | 0 | 15 | 4 | 0.069 (0.092) |
| RCJ (0°) | 5 | 0 | 17 | 2 | 0.113 (0.119) |
| RCJ, idealized (upper bound) | 6 | 0 | 18 | 0 | 0.109 (0.103) |

- **CoDe meets every target at every $q_{\mathrm{req}}$**, as does only the idealized RCJ.
- **No target is met by the mechanism alone (Level 1) at the nominal tilts.** Every adapted pair changes the
  external morphology, then keeps the mechanism's internal geometry ($\varepsilon_\kappa = 0$ after Level 2's second
  step).
- CoDe's median external change (0.089 $L_f$, about 8 mm) is below the 2R-DD's and the RCJ's; the 2R-remote's is
  lower but it misses 4 pairs.

## External change at $q_{\mathrm{req}} = 150°$, by target

$\varepsilon_d$ (fraction of $L_f$); 0: met frozen; ✗: not found.

| Target | CoDe | 2R-DD | 2R-remote | RCJ | RCJ, idealized |
|---|---|---|---|---|---|
| baseline | 0 | 0.086 | 0 | 0 | 0 |
| long | 0.053 | ✗ | 0.053 | 0.192 | 0.053 |
| thick | 0 | 0.167 | 0.064 | 0.021 | 0 |
| long_thick | 0.053 | ✗ | ✗ | ✗ | 0.160 |
| asym_long_proximal | 0.069 | 0.222 | 0.069 | 0.110 | 0.069 |
| asym_long_distal | 0.109 | ✗ | 0.109 | 0.167 | 0.109 |
| asym_proximal_heavy | 0.124 | 0.124 | ✗ | 0.142 | 0.124 |
| asym_distal_heavy | 0.164 | ✗ | 0.164 | ✗ | 0.164 |

At 150° CoDe matches the idealized RCJ on every target but one, where it needs less change (long_thick: 0.053
against 0.160). The 2R-DD misses half the targets.

## What changes

Median absolute change of each external descriptor over the adapted pairs (mm):

| Architecture | $L_f$ | Proximal span | Distal span | Articulation $x_{\mathrm{art}}$ |
|---|---|---|---|---|
| CoDe | 2.3 | 3.5 | 4.8 | 2.0 |
| 2R-DD | 8.2 | 6.5 | 6.8 | 7.3 |
| 2R-remote | 5.1 | 4.2 | 5.8 | 2.3 |
| RCJ | 9.6 | 4.1 | 7.6 | 6.6 |
| RCJ, idealized | 4.6 | 4.8 | 6.2 | 3.8 |

CoDe adapts mostly through the distal span, with the least change in finger length and articulation.

## Tilt

Frozen feasible / not found / median $\varepsilon_d$ of the Level 2 pairs, over all targets and $q_{\mathrm{req}}$:

| Architecture | 0° | 15° | 30° | 45° | 60° | 75° | 90° |
|---|---|---|---|---|---|---|---|
| CoDe | 0 / 0 / 0.089 | 6 / 0 / 0.089 | 6 / 0 / 0.089 | 6 / 0 / 0.089 | 6 / 0 / 0.089 | 6 / 0 / 0.089 | 6 / 0 / 0.089 |
| 2R-DD | 3 / 4 / 0.109 | 4 / 4 / 0.115 | 5 / 2 / 0.124 | 5 / 1 / 0.118 | 5 / 1 / 0.112 | 5 / 1 / 0.109 | 5 / 1 / 0.109 |
| 2R-remote | 5 / 4 / 0.069 | same at every tilt | | | | | |
| RCJ | 5 / 2 / 0.113 | 5 / 1 / 0.119 | 6 / 1 / 0.124 | 6 / 1 / 0.113 | 6 / 1 / 0.113 | 6 / 1 / 0.113 | 6 / 1 / 0.113 |

As in Study 1, CoDe's results do not depend on the tilt from 15° up (at 0° its frozen hand fails statically, but
every target is still met after adaptation); the 2R-DD and the RCJ need the servo laid back to miss fewer targets.

## Limits

- The search budget is small (100 × 100 per level and seed); "not found" means not found within it.
- Level 1 never succeeding suggests the mechanism-only search may be too narrow at this budget, not that no
  mechanism change could work.
- Linkage occlusion is off in the study profile (SSoT A.6.1).
