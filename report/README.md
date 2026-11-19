# Reports

Each report belongs to one of two tracks, which are never mixed (repository README, "Two optimizations: prototype and
study"). **Study** reports answer GOAL.md's questions under the study's fixed packaging (`design/scripts/study/`).
**Prototype** reports describe the finger that gets built (`design/scripts/optimize/linkage/`, the hardware
workstream). Numbers from the two tracks are not compared.

Start with [PROGRESS.md](PROGRESS.md): it lists the current defensible claims, their evidence boundaries, the
unresolved limitations and the next validation steps against GOAL.md.

## Study

| File | Status | Contents |
|---|---|---|
| [ssot_alignment_2026-11-17.md](ssot_alignment_2026-11-17.md) | superseded (SSoT is now `design/SSoT/high-level-formulation.md`) | Canonical 2D mathematical SSoT (`docs/legacy/math_formulation_2026-11-17.md`) & `design/core/` alignment: 5-layer $SE(2)$ formulation, analytical collision, exact `IntervalSet` coverage, `profile`-based `include_linkage_occlusion`, and empirical $C_{\mathrm{endpoint}}$ vs $C_{\mathrm{robust}}$ analysis |
| [recheck_fold_rounding_2026-11-17.md](recheck_fold_rounding_2026-11-17.md) | current | Every study run affected by the ±0.25 mm fold-rounding error, re-evaluated or rerun: what changed (mostly belt 2R counts) and confirmation that C3, C4 and C7 are unchanged |
| [study1_faceoff_2026-11-17.md](study1_faceoff_2026-11-17.md) | current, appendix draft | Link-in-front occlusion rule ON vs OFF: Study 1 at the active 15-degree conditions (CoDe +2/+8 module pairs at 120/150 degrees, every other architecture unchanged) and the Study 2 face-off control; the rounding fix that capped the rule |
| [interface_family_2026-11-09.md](interface_family_2026-11-09.md) | current | Independent fixed-H proximal/distal thickness-family experiment, dense-posture validation and fold-rule sensitivity across all comparison mechanisms |
| [study2_rom_conditioned_2026-11-09.md](study2_rom_conditioned_2026-11-09.md) | current (pilot, one seed) | Separate 90/120/150-degree baseline preparation and interface adaptation, with saved-design readback and per-interface ROM cost plots |
| [transmission_sensitivity_2026-11-09.md](transmission_sensitivity_2026-11-09.md) | current | CoDe at nominal 15 degrees: proximal-heavy accommodation with relaxed transmission-angle limits, selected k values, same-geometry crosschecks and a face-rule-off control |
| [kinematic_study_freeze.md](kinematic_study_freeze.md) | decision, 2026-11-07 | The kinematic mechanism study (Studies 1-4, E01-E10, packaging trade-off) closed; its abstraction level and what goes to the hardware workstream |
| [study3_study4_2026-11-07.md](study3_study4_2026-11-07.md) | snapshot `4e77658` | Local transmission-reflected contact mechanics and matched joint-bridge geometric screening; figures in `figures/study34/` |
| [study2_compact_2026-11-07.md](study2_compact_2026-11-07.md) | snapshot `4e77658` | Given-interface compact morphology extension: relaxed length/proximity domain, best-found size, support-span and articulation changes; figures in `figures/study2_compact/` |
| [exploratory/synthesis.md](exploratory/synthesis.md), [exploratory/registry.md](exploratory/registry.md) | snapshot `eb767bd` | Exploratory studies E01-E10: pre-registration, one report per study (`exploratory/E*.md`) and the synthesis |
| [study1_study2_2026-11-06.md](study1_study2_2026-11-06.md) | historical: 10-degree PIP servo tilt | Study 1 (modules a frozen hand carries) and Study 2 (least morphology change to carry a module), all architectures on matched references. Not rerun under the active 15-degree nominal and 0-90-degree tilt sweep |
| [mechanism_study_2026-11-05.md](mechanism_study_2026-11-05.md) | superseded | CoDe vs 2R-DD vs RCJ, from before the link rule; its CoDe designs fail that rule. Use the reports above |

## Prototype

| File | Status | Contents |
|---|---|---|
| [prototype.md](prototype.md) | current | The CoDe finger in CAD (source run, CAD variables, pads, performance, re-evaluation under later rules) and the 8 mm-pivot design variant (2026-11-12) |
| [prototype_hardware.md](prototype_hardware.md) | audit, 2026-11-07 | What the CAD and the model do and do not represent of the real hardware; open hardware decisions |
| [contact_module_interface.md](contact_module_interface.md) | draft spec, 2026-11-07 | The mounting interface between the fixed hand and an exchangeable contact module, from the prototype model |

The figures are in `figures/`. Run outputs (`design/output/`) are not tracked by git, so each report records the run
paths and code commit it was made from.
