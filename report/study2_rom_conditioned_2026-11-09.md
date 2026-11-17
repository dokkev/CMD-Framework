# Study 2: ROM-conditioned baseline pilot

2026-11-09. Every 90/120/150-degree ROM has a separately selected baseline before target-interface adaptation.
This is a one-seed pilot (population 32, 20 generations per search), not a final architecture ranking.

## Protocol

- Start from the same matched nominal reference descriptors; CoDe/belt tilt fixed at 15 degrees.
- DD and physical RCJ at 0 and 90 degrees are separate pilot conditions; ideal RCJ is an upper bound.
- Prepare the baseline interface (20x3 mm on both segments) for each ROM with the existing E-then-Q hierarchy.
- Freeze the chosen baseline, then adapt independently to each of eight interfaces at the same ROM.
- Retain the 30-150-degree transmission-angle range and 10-degree hyperextension for all CoDe conditions.
- No k_max cap; face usability at required ROM and actual nonpenetration checks remain active.
- Baseline cost is normalized by nominal length; interface cost by selected ROM-baseline length. They are never added.
- Net external change from nominal is independently measured, rather than summing two max norms.
- If a ROM baseline is not found, its target conditions are blocked, not labelled interface-infeasible.
- ROM-conditioned baselines may differ in E/Q across architectures; only the initial nominal descriptors are matched.

## Baseline preparation

| Reference | ROM | Status | eps_E from nominal | eps_Q from nominal |
|---|---:|---|---:|---:|
| code_matched | 90 | frozen_feasible | 0.0 | 0.0 |
| 2r_dd_tilt0_matched__tilt0 | 90 | frozen_feasible | 0.0 | 0.0 |
| 2r_dd_tilt0_matched__tilt90 | 90 | frozen_feasible | 0.0 | 0.0 |
| belt_2r_matched | 90 | frozen_feasible | 0.0 | 0.0 |
| rcj_tilt0_matched__tilt0 | 90 | frozen_feasible | 0.0 | 0.0 |
| rcj_tilt0_matched__tilt90 | 90 | frozen_feasible | 0.0 | 0.0 |
| rcj_ideal_matched | 90 | frozen_feasible | 0.0 | 0.0 |
| code_matched | 120 | frozen_feasible | 0.0 | 0.0 |
| 2r_dd_tilt0_matched__tilt0 | 120 | frozen_feasible | 0.0 | 0.0 |
| 2r_dd_tilt0_matched__tilt90 | 120 | frozen_feasible | 0.0 | 0.0 |
| belt_2r_matched | 120 | frozen_feasible | 0.0 | 0.0 |
| rcj_tilt0_matched__tilt0 | 120 | frozen_feasible | 0.0 | 0.0 |
| rcj_tilt0_matched__tilt90 | 120 | frozen_feasible | 0.0 | 0.0 |
| rcj_ideal_matched | 120 | frozen_feasible | 0.0 | 0.0 |
| code_matched | 150 | frozen_feasible | 0.0 | 0.0 |
| 2r_dd_tilt0_matched__tilt0 | 150 | level2 | 0.103333 | 0.0 |
| 2r_dd_tilt0_matched__tilt90 | 150 | frozen_feasible | 0.0 | 0.0 |
| belt_2r_matched | 150 | frozen_feasible | 0.0 | 0.0 |
| rcj_tilt0_matched__tilt0 | 150 | level2 | 0.008889 | 0.0 |
| rcj_tilt0_matched__tilt90 | 150 | frozen_feasible | 0.0 | 0.0 |
| rcj_ideal_matched | 150 | frozen_feasible | 0.0 | 0.0 |

## Interface outcomes

| Reference | ROM | Frozen | Adapted | Not found |
|---|---:|---:|---:|---:|
| code_matched | 90 | 2 | 6 | 0 |
| 2r_dd_tilt0_matched__tilt0 | 90 | 2 | 6 | 0 |
| 2r_dd_tilt0_matched__tilt90 | 90 | 2 | 6 | 0 |
| belt_2r_matched | 90 | 2 | 5 | 1 |
| rcj_tilt0_matched__tilt0 | 90 | 2 | 5 | 1 |
| rcj_tilt0_matched__tilt90 | 90 | 2 | 5 | 1 |
| rcj_ideal_matched | 90 | 2 | 6 | 0 |
| code_matched | 120 | 2 | 3 | 3 |
| 2r_dd_tilt0_matched__tilt0 | 120 | 1 | 6 | 1 |
| 2r_dd_tilt0_matched__tilt90 | 120 | 2 | 6 | 0 |
| belt_2r_matched | 120 | 2 | 3 | 3 |
| rcj_tilt0_matched__tilt0 | 120 | 1 | 4 | 3 |
| rcj_tilt0_matched__tilt90 | 120 | 2 | 5 | 1 |
| rcj_ideal_matched | 120 | 2 | 6 | 0 |
| code_matched | 150 | 2 | 0 | 6 |
| 2r_dd_tilt0_matched__tilt0 | 150 | 1 | 1 | 6 |
| 2r_dd_tilt0_matched__tilt90 | 150 | 1 | 3 | 4 |
| belt_2r_matched | 150 | 1 | 1 | 6 |
| rcj_tilt0_matched__tilt0 | 150 | 1 | 3 | 4 |
| rcj_tilt0_matched__tilt90 | 150 | 1 | 3 | 4 |
| rcj_ideal_matched | 150 | 1 | 7 | 0 |

## Artifacts and scope

Outputs: design/output/study/study2_rom_conditioned_2026-11-09/pilot/.
baseline_best.csv records ROM preparation; interface_best.csv records additional interface adaptation.
Each baseline.json includes the exact selected geometry and config; searches store per-seed stage configs and Pareto rows.
Readback validated 145 feasible baseline/interface designs; 0 target conditions were blocked by baseline search failure.
interface_costs.png shows per-interface external cost versus required ROM. Missing solutions are not interpolated.
not_found is search-budget/domain dependent. Pilot costs need not be monotone. In addition, endpoint-only face usability
means feasible sets at different required ROMs are not guaranteed nested, even though collision-free reach requirements are nested.
No physical torque, buildability, 3D accessibility or learning performance was measured.
