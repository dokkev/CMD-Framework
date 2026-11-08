# Exploratory studies E01–E10

These are small, independent investigations of the CoDe architecture,
judged against `GOAL.md`. The registry, revisions, per-study reports and
synthesis are in `report/exploratory/`.

All the scripts reuse the Study 1/2 infrastructure (`common.py` →
`../shared.py`, Study 1's `assembly`, the existing scenes and evaluators,
and the Study 3 transmission adapter). Each writes
`output/study/exploratory/<name>/`, containing a manifest with its source
record, its CSVs and, when the tree is dirty, a source snapshot.

| Script | Study | Runtime |
|---|---|---|
| `e01_packaging.py` | Palmar hardware intrusion, thickness, area share (extended pose) | s |
| `e02_polycentric_ablation.py`, `e02b_palmar_pivot.py` | CoDe vs an ideal 2R pinned at its ICR; palmar-pivot counterfactual; ICR path | s |
| `e03_module_independence.py` | Proximal × distal module grid, interaction and coupling | s |
| `e04_actuator_orientation.py` | Servo tilt sweep, −30° to 120° | s |
| `e05_tolerance.py`, `e05b_active_contacts.py` | Monte Carlo geometric perturbation, σ = 0.1–0.4 mm; active static contacts | s |
| `e06_clearance.py` | Body clearance 0–1.5 mm, all checks or pins exempt | s |
| `e07_occlusion.py` | Normal-probe occlusion of the faces, 0–150° | s |
| `e08_enclosing.py` | Two-face tangent circles (inconclusive metric) | ~20 s |
| `e09_hidden_transmission.py` | Lever change of Study 2 adaptations | s |
| `e10_design_drivers.py`, `e10b_bootstrap.py` | 150-design archive cover; correlations with bootstrap | ~10 s |
| `figures.py` | All figures, from the saved CSVs | s |

Run from the repository root in the `cmd` environment, with
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. E05b and E10b write side tables into
the E05 and E10 folders. The main scripts start from a fresh folder, so rerun
the follow-up after its main script.
