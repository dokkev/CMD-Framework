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

  The cores are four roles each of CoDe at a 10 deg servo tilt and of the
  2R-DD at 0 and 90 deg. The comparison is therefore between those servo
  layouts, not the mechanism families at large. A design two roles share is
  evaluated once. The cores come from the `scripts/study/*.yaml` runs, all
  under one goal (`scripts/study/base.yaml`) on the study's own baseline,
  `scripts/study/baseline.yaml`. That baseline is a frozen copy of the
  production config, so a production change does not move the study. The
  goal sets the finger length, the required flexion and the mounting
  policy, and drops the segment and four-bar span ratios.
  `code_tilt10_ta10.yaml` relaxes CoDe's transmission-angle limit
  (30-150 to 10-170 deg) for a sensitivity run.
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
  An assembly also fails where the fingertip passes behind the MCP
  (`C_x < 0`, `tip_behind_mcp`), as in the optimization's evaluators.

  What this shows is which module requirements a fixed core accommodates
  without changing its morphology. It does not show the cost of
  redesigning a core for a module that does not fit. The collision model is
  the idealized 2.5D side view, with Y layers assumed, not a 3D stack-up.

## Mounting depth

The reference lines are the link lines: OA and BC for CoDe, and each
segment's line for the 2R-DD.

- `d_proximal_mm` and `d_distal_mm` are measured palmar of those lines.
- The contact face lies at `d + thickness`.

The study sets each depth by a policy instead of searching it
(`goal.pads.mount_policy: shallowest_supported`, `core.optimization.mounting`).
The two pads are set independently.

- **Case A:** the pad does not reach its own joint hardware, so `d = 0`.
- **Case B:** the pad runs under that hardware, so `d` is the least depth on
  the resolution grid that clears it. The depth depends on the pad's length
  and placement, and the evaluators' own collision checks decide it. The
  hardware supports the pad only on its own body:
  - CoDe: the pin housings of the pad's segment;
  - 2R-DD proximal pad: the servo, which sits on the proximal body;
  - 2R-DD distal pad: none, since it hangs from a truss off the output at A
    and the servo only sets its clearance. With `align_mount_offsets`, both
    pads take one depth (the deeper least one), and the distal pad is then
    carried to the servo's reach.
- **Case C:** a depth deeper than the hardware reaches below the line (the
  segment's native support) would need a spacer. The design is rejected.

The depths are the core's: they are written into its design vector and held
for every module. A module goes on the core's surfaces when, for each pad,
`d_required <= d_core <= d_native`: the module needs no deeper surface than
the core's, and the core's surface (off the line) still sits on hardware
that carries it under that module. Otherwise the assembly is reported as
`mount_incompatible`, and the module is not remounted. Two examples:
- A distal module too short to reach under B leaves the core's 3 mm
  surface over nothing, so it is incompatible.
- A module needing 2.5 mm, on a 3 mm core surface under a 3 mm housing,
  fits. Its collisions are then checked at 3 mm.

The rows record, for each pad:
- the depth;
- the depth that module would need;
- its case (A, B or C);
- its native support.

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
