# Proximal accommodation under relaxed transmission-angle limits

2026-11-09. Model-derived sensitivity study, current CoDe nominal servo tilt 15 degrees.

The question is whether sacrificing transmission quality can recover proximal-heavy interfaces while
retaining the same contact-face rule. No k_max cap is imposed in any condition. The changed variable
is the permitted transmission angle, not a direct optimization objective or limit on k.

## Controlled conditions

- Interfaces: proximal 40 x 3 / distal 20 x 8 mm; proximal 45 x 5 / distal 15 x 3 mm.
- Required distal flexion 120 degrees, hyperextension 10 degrees; closure, collision and face checks retained.
- Servo shaft and 15-degree tilt fixed; Study 2 E/Q hierarchy and external range +/-20 mm unchanged.
- Current actual-envelope nonpenetration model, with no universal body allowance; planar assumptions remain.
- Pilot: population 32, 20 generations, seeds 0-2. Main: population 100, 100 generations, seeds 0-2.
- Positive control: same main budget, 30-150 degrees, links_block_face=False. It does not belong to the primary comparison.

## Saved results

| Run | Target | Found seeds / run | Status | eps_E | eps_Q | Selected k_max |
|---|---|---:|---|---:|---:|---:|
| pilot_margin30 | asym_long_proximal | 0/3 | not_found | nan | nan | - |
| pilot_margin30 | asym_proximal_heavy | 0/3 | not_found | nan | nan | - |
| pilot_margin20 | asym_long_proximal | 0/3 | not_found | nan | nan | - |
| pilot_margin20 | asym_proximal_heavy | 0/3 | not_found | nan | nan | - |
| pilot_margin10 | asym_long_proximal | 0/3 | not_found | nan | nan | - |
| pilot_margin10 | asym_proximal_heavy | 0/3 | not_found | nan | nan | - |
| main_margin30 | asym_long_proximal | 0/3 | not_found | nan | nan | - |
| main_margin30 | asym_proximal_heavy | 0/3 | not_found | nan | nan | - |
| main_margin20 | asym_long_proximal | 0/3 | not_found | nan | nan | - |
| main_margin20 | asym_proximal_heavy | 0/3 | not_found | nan | nan | - |
| main_margin10 | asym_long_proximal | 1/3 | level2 | 0.202222 | 0.171111 | 5.0628 |
| main_margin10 | asym_proximal_heavy | 0/3 | not_found | nan | nan | - |
| control_faceoff_margin30 | asym_long_proximal | 3/3 | level2 | 0.068889 | 0.022222 | 3.6248 |
| control_faceoff_margin30 | asym_proximal_heavy | 3/3 | level2 | 0.124444 | 0.021111 | 3.5220 |

## Interpretation and limits

At least one relaxed-angle search finds a face-rule-ON solution: the earlier proximal failure is not
an established architectural impossibility. This does not demonstrate a global optimum or robustness
across optimizer seeds. k_max is read back from each selected design rather than inferred from its angle bound.

For the asym_long_proximal main-margin10 candidate, k_max is about 5.063, eps_E about 0.202 and
eps_Q about 0.171. Rechecking that exact geometry at a 20-degree minimum fails only the required
hyperextension (about 2 degrees missing); at a 30-degree minimum both ROM and hyperextension fail.
Thus the relaxed limit matters at the extension end as well as at the requested 120-degree posture.

That candidate changes finger length by +8.5 mm, proximal support span by +15.6 mm, distal span by
-7.0 mm and articulation station by +18.2 mm. Its largest independent Q-coordinate change is 15.4 mm.
It therefore demonstrates recoverable accommodation with changed embodiment, not a fixed-H interface swap.

The historical face-off candidates rechecked at 15 degrees already satisfy the 30-degree minimum.
Turning the face rule on causes pad_usable_mm violations of 4 and 6.5 mm respectively. Those specific
geometries are limited by the face rule, not k or transmission-angle constraints.

All not_found entries mean no feasible design was found under the stated search domain and budget.
No physical torque, strength, 3D accessibility or hardware behavior was validated.

## Artifacts

- Root: design/output/study/transmission_sensitivity_2026-11-09/
- summary_runs.csv: per-seed outcomes; selected.csv: best selected outcomes and re-evaluated k.
- crosscheck.csv: same selected geometries under 30/20/10-degree limits.
- historical_candidate_recheck.csv: separate counterfactual diagnostic at 15 degrees.
- Each run stores manifest, source snapshot, exact stage configs, optimizer records and candidate CSVs.
- analysis_manifest.json hashes input tables and the analysis script; analysis_source stores its source snapshot.
- asym_long_proximal_candidate.png: geometry at 0/120 degrees and transmission-angle-margin trajectory.

Reproduce searches with sensitivity_transmission_{30,20,10}_{pilot,main}.yaml and
sensitivity_transmission_faceoff_control.yaml through the existing Study 2 run.py.
Summarize using scripts/study/study2_morphology_adaptation/summarize_transmission_sensitivity.py.
