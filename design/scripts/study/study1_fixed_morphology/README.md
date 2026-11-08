# Study 1: contact accommodation at a fixed morphology

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
