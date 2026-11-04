# Contact accommodation study

Can CoDe (crossed four-bar, mechanism `linkage`) carry a broader range of
contact modules than the direct-drive serial 2R (2R-DD, mechanism
`revolute`) while keeping its finger morphology?

## Stages

- **A. Cores** (`cores.yaml`). A core is a finished design, frozen. It
  includes the skeleton, the joints and linkage, the actuator's placement
  and tilt, the module supports and both mounting depths. Pick a core in
  one of three ways:
  - a portfolio's `seeds.csv` row by its role (`balanced`, `most_pad`,
    `best_transmission`, `most_curl`);
  - a run's `pareto.csv` row by its candidate number;
  - a design vector under a config file.

  The cores come from the `scripts/study/*.yaml` runs, all under the one
  goal in `scripts/study/base.yaml`. That goal sets the finger length, the
  required flexion and the same mounting-depth range for both mechanisms,
  and drops the segment and four-bar span ratios. The production config is
  not used.
- **B. Modules** (`modules.yaml`). A module is the rectangular pad block:
  a length along its segment from its outer end, and a thickness below the
  core's mounting surface. The catalog includes:
  - named pairs (baseline, long, thick, long and thick);
  - grids varying the proximal side, the distal side, or both.

  Every pair goes on every core. Only the modules' lengths and thicknesses
  change (`tradeoff.with_modules`). The design vector, and with it the
  mounting depths, is never touched. Nothing is re-optimized.
- **C. Evaluation** (`run.py`). For each core, module pair and required
  flexion, the study records:
  - whether the modules mount on the core's supports;
  - whether the assembly stays clear from extension to the required flexion
    (`contact_envelope.evaluate_fit`), with the first failing pose and pair;
  - the maximum safe flexion (`contact_envelope.max_flexion_deg`);
  - when it fits, the pads' effective lengths and the coverage at that
    flexion, as the mechanism's own evaluation counts them (each face up to
    where the two faces cross).

  A failed assembly is reported as infeasible. It is not redesigned.

## Mounting depth

The reference lines are the link lines: OA and BC for CoDe, and each
segment's line for the 2R-DD.

- `d_proximal_mm` and `d_distal_mm` are measured palmar of those lines.
- The contact face lies at `d + thickness`.
- The depths belong to the core: they come from its design vector, are
  searched in stage A and are held fixed in stage B.

The segment's body carries the mounting surface down to its joint hardware:
the pin housings for CoDe (3 mm) and the servo for the 2R-DD. A depth set
by that clearance therefore needs no extra structure.

## Physical rules

All cores must be checked under the same packaging: the servos, the pins,
the clearances and which pairs are checked. If they are not, `run.py`
stops. The checks are the idealized 2.5D side view of `contact_envelope`
(`COLLISION_POLICY`, recorded in the manifest).

## Run

```
python scripts/optimize/run.py scripts/study/code_tilt10.yaml      # the cores' runs first
python scripts/optimize/run.py scripts/study/2r_dd_tilt90.yaml
python scripts/study/contact_accommodation/run.py                  # then the study
```

The study writes to `output/study/contact_accommodation/`:

| file | contents |
|---|---|
| `manifest.json` | the config, cores and modules as given, the physical rules, the collision policy and the source commit |
| `core_designs.csv` | each core: its source, formulation, design vector, mounting depths and layout |
| `module_catalog.csv` | each module pair's id, group and dimensions |
| `assembly_results.csv` | one row per core, module pair and required flexion (`<core>__<module>__phi<deg>`) |

Equal inputs give equal files (values rounded to 6 decimals). Only the
manifest's commit can differ.
