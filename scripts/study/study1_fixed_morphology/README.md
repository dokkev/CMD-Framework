# Study 1: contact accommodation at a fixed morphology

Study 1 measures contact interface capacity: which modules a frozen core carries. Study 2
(`scripts/study/study2_morphology_adaptation`) measures morphology preservation: how far a
core must change to carry a given module. This directory was `contact_accommodation` before
2026-11-06; only its paths changed.

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

  The cores are CoDe at a 10 deg servo tilt in four roles, and the 2R-DD at
  0 and 90 deg in three: its servo drives the joint directly, its torque
  ratio is 1 for every design, so `best_transmission` would pick an
  arbitrary one. The comparison is therefore between those servo layouts,
  not the mechanism families at large. A design two roles share (a seed's
  role `most_pad;balanced`) is evaluated once. The cores come from the `scripts/study/*.yaml` runs, all
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
  - the maximum safe flexion (`contact_envelope.max_flexion_deg`), the
    least of the core's own limit and the module's first failure;
  - when it fits, the pads' effective lengths and the coverage at that
    flexion, as the mechanism's own evaluation counts them (each face up to
    where the two faces cross).

  A failed assembly is reported as infeasible. It is not redesigned.
  An assembly also fails where the fingertip passes behind the MCP
  (`C_x < 0`, `tip_behind_mcp`), as in the optimization's evaluators.

  The core's own limit is what no module changes. For the 2R-DD it is the
  flexion asked for. CoDe's motion is the evaluator's safe motion less the
  pads:
  1. its rising branch only, to the four-bar's closure or its first most
     flexion (the poses after it turn back), `kinematic_unreachable` short
     of the flexion asked for;
  2. on it, the optimizer's mechanism limits: the transmission angle at B',
     the DIP and the PIP turning with the servo, the distal pins clear of
     the PIP servo (`mechanism_limit`, the limit in `collision_pair`).

  The pads' collisions depend on the module, so each module's are checked
  on that motion with its own dimensions. The servo is sampled every
  0.1 deg (the optimizer's evaluator every 1 deg, so its limit can fall up
  to one of its steps short of the study's).

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
  - CoDe: the pin housings of the pad's segment, and for the proximal pad
    the PIP servo, which is fixed on the proximal segment (never the distal
    pad's);
  - 2R-DD proximal pad: the servo, which sits on the proximal body;
  - 2R-DD distal pad: none, since it hangs from a truss off the output at A
    and the servo only sets its clearance. With `align_mount_offsets`, both
    pads take one depth (the deeper least one), and the distal pad is then
    carried to the servo's reach.
- **Case C:** a depth deeper than the hardware reaches below the line (the
  segment's native support) would need a spacer. The design is rejected.

A pin housing reaches as deep as its disk. A servo housing reaches as deep
as its footprint's lowest point over the pad's span along the line, not its
whole reach, and carries nothing where the two do not overlap. Both include
the clearance and are rounded up to the depths' grid.

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

All cores must be checked under the same packaging (the servos, the pins,
the clearances and which pairs are checked) and the same mounting and
collision settings (policy, alignment, depth range and grid, link radius,
MCP poses). If they are not, `run.py` stops. The manifest lists the link
checks only when a link radius is set. The checks are the idealized 2.5D side view of `contact_envelope`
(`COLLISION_POLICY`, recorded in the manifest).

## Run

```
python scripts/optimize/run.py scripts/study/code_tilt10.yaml      # the cores' runs first
python scripts/optimize/run.py scripts/study/2r_dd_tilt0.yaml
python scripts/optimize/run.py scripts/study/2r_dd_tilt90.yaml
python scripts/study/study1_fixed_morphology/run.py                  # then the study
```

The study writes to `output/study/study1_fixed_morphology/`:

| file | contents |
|---|---|
| `manifest.json` | the config, cores and modules as given, the physical rules, the collision policy and the source commit |
| `core_designs.csv` | each core: its source, formulation, design vector, mounting depths and layout |
| `module_catalog.csv` | each module pair's id, group and dimensions |
| `assembly_results.csv` | one row per core, module pair and required flexion (`<core>__<module>__phi<deg>`) |

Equal inputs give equal files (values rounded to 6 decimals). Only the
manifest's commit can differ.
