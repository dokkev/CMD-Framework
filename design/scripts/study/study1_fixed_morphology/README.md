# Study 1: contact accommodation at a fixed morphology

## Active nominal and tilt-sweep configuration (2026-11-09)

Production and the active nominal CoDe hand use a CAD-derived **15 degree** PIP servo rear-body tilt
from -X toward dorsal +Z. The hand parameter is `finger.pip_servo_tilt_deg`; its servo-frame rotation is
`Rz(tilt - 90 degrees)`. Studies 1 and 2 share `references_nominal15.yaml`; the historical 10-degree
`references.yaml` and saved reports remain frozen inputs for the earlier studies.

The primary config sweeps 0, 15, 30, 45, 60, 75 and 90 degrees for CoDe, 2R-DD, belt 2R and physical RCJ.
Each angle creates a separate reference (`<base_id>__tilt<angle>`). E/Q and the actuator shaft position
are unchanged when creating the tilt conditions; pad seats follow the same mounting policy. Within each
Study 2 adaptation search the chosen tilt is fixed. This is an actuation sensitivity experiment: a best
result selected across tilts does not hold A fixed. The ideal RCJ stays a separate upper bound and is not
swept. CSVs record `base_reference_id` and `servo_tilt_deg`; `summary_best.csv` selects optimizer seeds
within each tilt, not across tilts. Study 2 `--references` accepts a base id or a particular tilt id.

The universal body-clearance allowance has been removed. Solids may touch, but signed overlap fails.
A short pad stays on its link reference line; when it overlaps its own pivot housing or fixed servo,
the seat follows the shallowest nonpenetrating position toward -Z, within the existing native-support
rule. The housing dimensions are physical envelopes, not an extra uniform clearance. No spacer is assumed.

**CoDe and the tilt (2026-11-17).** CoDe's servo turns about its shaft A', so the tilt only orients its body:
the four-bar, the pads and their faces do not depend on it. In Study 1, CoDe is identical in every assembly
from 15° to 90°. At 0° every assembly is invalid. The reference keeps the prototype's A'_z = 12.4 mm, designed for
a tilted servo, so the level body's lower face (12.4 - 9.5 = 2.9 mm) overlaps the 6 mm A pin (top at 3.0 mm) by
0.1 mm. This is a static check (`pivot_pin_mm`), so the motion is not swept. It says this reference geometry does
not fit a level servo, not that CoDe cannot use one. The belt 2R is identical at every tilt, 0° included. The tilt
sweep is informative for the 2R-DD and the RCJ, whose servo sits at the joint.

New defaults write to `*_tilt_sweep` output directories so they do not overwrite the published historical
runs. The sections below describe the original fixed-tilt study; their numerical results have not been
recomputed under the new sweep.

Study 1 measures **contact interface capacity**: which contact modules a
frozen hand carries. Study 2 (`../study2_morphology_adaptation`) measures
**morphology preservation**: how far a hand must change to carry a module it
does not. The two share their references and their modules
(`../shared.py`), so a hand or a module means the same in both. At the
required flexion, Study 1's verdict for a module is Study 2's first step,
"the reference as it is" (a test holds them equal).

## References

The hands are Study 2's matched references
(`../study2_morphology_adaptation/references.yaml`). One external morphology,
the current prototype's (finger 90.0 mm, proximal span `|OA''|` 43.3 mm,
distal span `|B''C|` 30.2 mm, articulation 55.05 mm), is held as a tracked
design vector. Every architecture is built to it once, in its own way
(`../partitions.py`):

| Reference | Mechanism | Actuation |
|---|---|---|
| `code_matched` | CoDe crossed four-bar (`linkage`), the prototype itself | PIP servo on A', tilt 10° |
| `2r_dd_tilt0_matched`, `2r_dd_tilt90_matched` | 2R-DD, servo at the joint (`revolute`) | servo laid back (0°) or standing (90°) |
| `belt_2r_matched` | belt 2R (`revolute_parallel`) | servo where CoDe's A' is, tilt 10°, 6 mm pulleys |
| `rcj_tilt0_matched`, `rcj_tilt90_matched` | RCJ, 1:1 rollers (`rcj`), the smallest its servo allows | servo at the proximal roller, 0° or 90° |
| `rcj_ideal_matched` | idealized RCJ (`rcj_idealized`): no joint servo | none at the joint. **An upper bound, reported apart** |

Every reference must share the comparison rules (`shared.comparison_rules`),
or the study stops:
- the required flexion and hyperextension;
- the finger's length range;
- the mounting policy and depth range;
- the pins' diameters and spacing;
- the servo envelopes;
- the fold and link face rules;
- the resolution.

The servo's tilt is each reference's own actuation, not a shared rule.

## Modules

`modules.yaml` lists the contact modules. A module is the rectangular pad
block: a length along its segment from its outer end, and a thickness below
the mounting surface. The catalog has 71 pairs:
- the named pairs;
- grids over length 15–45 mm and thickness 3, 5 and 8 mm, with the
  proximal side, the distal side, or both varied.

## Evaluation (`run.py`)

For each reference, module pair and required flexion (90, 120 and 150°):

1. **The frozen design carries the module.** The design vector is held:
   skeleton, joints, actuator and supports. Each pad's mounting depth is the
   goal's policy for that module on the hand's own hardware
   (`core.optimization.mounting`, `shallowest_supported`). A module that
   would need a spacer does not mount.
2. **The mechanism's own `evaluate()` judges it.** It decides whether the
   module mounts, and then whether the assembly is feasible.
   - A module mounts when the policy gives a depth and the module fits its
     segment's room (`proximal/distal_support_mm`, `mount_depth_mm`). This
     definition is the same for every mechanism.
   - The assembly is feasible when it is free of collisions through the
     motion (pads, pins, links, rollers, servos), reaches the flexion asked
     for, and keeps each module's length usable there. The usable length is
     each face up to the fold, less any part a crossing link or belt passes
     in front of.
3. **It records** the failing constraints, the maximum safe flexion φ_max,
   where the motion stops, and the usable contact coverage
   (`(proximal + distal effective) / finger length`).

A failed assembly is reported, never redesigned.

The previous Study 1 used a separate collision engine (`contact_envelope`)
written only for CoDe and the 2R-DD. It never produced results. It was
replaced on 2026-11-06 by the evaluators every mechanism already has, so
that the belt 2R and RCJ are compared under the same physics.

## Scope

This is a 2D side-view geometric and kinematic model, with Y layers assumed.
It does not cover:
- manufacturing tolerances;
- bearings and fasteners;
- strength;
- cable routing;
- 3D assembly.

A feasible assembly here shows that the planar geometry has room. It does
not show that the module can be built or swapped in practice.

## Run

```
python scripts/study/study1_fixed_morphology/run.py
```

The run writes to `output/study/study1_fixed_morphology/`:

| File | Contents |
|---|---|
| `manifest.json` | the config, references (with the morphology's provenance), modules, comparison rules, formulations and source commit |
| `references.csv` | each reference's E descriptors, formulation and design vector |
| `module_catalog.csv` | each module pair's id, group and dimensions |
| `assembly_results.csv` | one row per reference, module pair and required flexion |

Equal inputs give equal files, with values rounded to 6 decimals.
