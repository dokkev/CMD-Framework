# Study 2 extension: compact accommodation of given interfaces

Question: with a contact interface fixed, how short a finger can each architecture
find, and what happens to its proximal/distal support spans and articulation?
This extends the original bounded-adaptation Study 2; it does not overwrite it.

## Formulation

All interface dimensions are fixed from the shared catalog. Existing mechanism
FK, collision/ROM/contact-coverage evaluators and mounting policy decide
feasibility. Required distal flexion is 120°. Keep actuation architecture,
servo envelopes and orientation fixed. CoDe/belt remote actuator shafts remain
at the matched reference's proximal-frame location; joint-mounted actuators
follow their joint as that architecture requires. There is no independent
actuator relocation optimization.

1. Minimize **absolute extended finger length L**, allowing E and Q to move.
2. Within `L <= primary best-found L + 0.1 mm`, minimize `eps_shape = max(|ΔS_p|,|ΔS_d|,|Δarticulation|)/L0`.
   Total length is excluded here: it is already controlled by the primary cap
   and would otherwise mask support/articulation changes. Original four-descriptor
   eps_E remains reported, but is not the secondary objective.
3. Pool saved primary and secondary incumbents across seeds, use their shortest
   actual discovered length to impose a common +0.1 mm eligibility cap, and
   select smallest eps_shape, then sorted absolute shape changes, length and eps_Q
   for deterministic retained-candidate ties. A primary candidate
   remains eligible even if its seed's secondary winner is outside that global
   cap. Secondary discoveries can lower the best-found length.

The first objective is compactness, not minimum change from the reference. Thus
compact designs can be shorter than 90 mm even for baseline interfaces. Report
both changes from the shared matched reference and changes from each architecture's
own compact baseline. Do not attribute ordinary baseline compaction to the new
interface.

The objective length is the existing partition's design descriptor, on its
0.1 mm search grid. CoDe's link-length rounding/reassembly can move its evaluated
fingertip slightly relative to that nominal descriptor. Finalized results retain
`assembled_length_mm` and the nominal/readback difference. Do not interpret
sub-grid rankings as a certified physical minimum.

E is the existing common descriptor: extended finger length, proximal
contact-bearing span, distal contact-bearing span, and extended articulation
station (CoDe ICR, 2R pin, RCJ roller contact). These spans are comparable support
lengths, not identically defined physical phalanx links. Q costs are retained
separately. Avoid summing external and mechanism costs with arbitrary weights.

## Relaxed domain, preserved constraints

The former +/-20 mm reference-proximity box is removed. Absolute length domain
starts at [40,160] mm for every reference/target. Proximal/distal span lower
bounds follow the fixed module/support requirements; their upper bounds and
articulation upper bound follow the current total-length domain. Existing
`with_finger_lengths` also propagates length changes into dependent mechanism
bounds, relative pivot boxes and rebuilt 2R/RCJ pad ranges. For the belt, the relative
shaft-to-joint coordinate additionally follows `shaft_x - A_x` over the full
allowed joint domain: the legacy [-20,30] mm relation was a sampling box, not a
physical maximum belt length. Shaft position and all pulley/clearance conditions
remain fixed. Diagnostic belt trials before this correction are retired and
excluded from the final results.

If no feasible design is found, or the best found is within 2 mm of the upper
limit, expand to [40,240] mm. Record every attempt. These remain finite searches;
"not found" does not establish impossibility. Existing physical hardware limits,
remaining mechanism layout boxes, roller radius, mounting policy and projected face-blocking
rules remain in force and must not be read as unlimited mechanical freedom.
The 40 mm lower domain edge is also flagged if approached. Source formulation
and resolved model bounds accompany every search.

Physical references include both original servo orientations, without choosing
one post hoc. The ideal RCJ is the existing no-joint-servo upper bound, displayed
apart. No ideal CoDe or hidden removal of linkage hardware is introduced.

## Search, provenance and interpretation

Main configuration: NSGA-II, population100, generations100, seeds0/1/2. Pilot:
population32, generations20, one seed, two difficult targets. Warm starts reuse
old Study2 target/reference optimizer fronts across seeds, plus the reference.
Select bounded, deterministic rows ordered by length/support descriptors and
record exact vectors, original run IDs and metadata. The inherited optimizer
admits only the first `population//2` warm vectors; the secondary search prepends
its primary incumbent. Main population100 admits all up-to40 recorded warm
vectors; the pilot admits at most16 (15 old vectors after prepending a primary).
Pilot failures are not final evidence.

The single-objective non-dominated archive now uses the exact identity "all rows
at the scalar minimum" rather than a general quadratic sort. Tests compare its
indices with the original general algorithm. This changes computational cost,
not objectives, constraints, evaluation count, formulations or selected fronts.

Three seeds assess search robustness, not statistical replication of physical
hands. Results are best-found compact designs, **not certified global minima**.
A wide length domain does not guarantee exploration reaches every feasible basin.
The second stage is limited to each seed's primary cap; final pooling selects
among retained candidates under the common cap, not a proof of the global
lexicographic optimum. Show individual seed lengths and failures.

All geometry is the existing 2D model with its layer assumptions. No strength,
manufacturing, assembly or real interface swapping claim follows from feasibility.

## Run and files

Use the installed cmd environment. Local interpreter:
`/home/dk/miniconda3/envs/cmd/bin/python`.

```
python scripts/study/study2_compact_morphology/run.py scripts/study/study2_compact_morphology/pilot.yaml
python scripts/study/study2_compact_morphology/run.py
python scripts/study/study2_compact_morphology/finalize.py
python scripts/study/study2_compact_morphology/figures.py
python scripts/study/study2_compact_morphology/report.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests/test_compact_morphology.py tests/test_study2_morphology_adaptation.py -q
```

Use `--targets` / `--references` for focused runs and a copied config with a fresh
output directory. `--resume` skips completed seed rows only after checking config
and filters, retaining original source provenance and recording resumed source.

Outputs include original and resumed source snapshots, resolved configs and
optimizer settings per stage, `targets/.../warm_start.json` with exact starting
vectors, all `run.json`/`pareto.csv`/`history.npz`, `summary.csv` (per seed), and
`summary_best.csv` (postprocessed global-cap selection). `aggregation.json`
records the postprocessing source and feasibility readback. Figures use these
saved tables and model configs, with input hashes and source snapshots.

For exact warm-start replay without the old Study2 fronts, copy the config and
set `warm_start.replay` to the archived result's `targets/` directory, keeping
`max_designs` and optimizer settings unchanged, and select a fresh output.
Run `finalize.py` once the entire selected dataset is complete. Original bounded
Study2 outputs and Study3/4 outputs are preserved.

Primary searches are reused after exact saved-config and optimizer-setting checks
when a secondary metric is corrected; old secondary stages are retained in a
diagnostic directory. Reuse never assumes the saved selected design feasible:
its design is read back through the existing evaluator.
