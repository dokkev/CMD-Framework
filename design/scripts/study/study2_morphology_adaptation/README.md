# Study 2: morphology adaptation for contact accommodation

How far must a reference hand change to carry a given contact interface?
This is not a search for a new best finger. It measures how much the existing
hand architecture has to change, and in which group of parameters.

**What it measures.** This is 2D, in-plane, kinematic morphology adaptation
under a fixed mounting and actuator architecture. The external change is the
change in four longitudinal descriptors (below). It is not the full
sacrifice in the hand's 3D morphology: section depth, the dorsal and palmar
envelope, a mounting standoff or spacer, and the finger's width are not
searched. Each pad's mounting depth is the mounting policy's, so a module
that needs a spacer to clear its joint's hardware has no design here, where
a hand could take that spacer. Read the results within this scope.

- Study 1 (`../study1_fixed_morphology`) measures **contact interface
  capacity**: which modules a frozen core carries.
- Study 2 measures **morphology preservation**: for a module the core does
  not carry, how far its mechanism, and then its external morphology, must
  change.

## Parameters

A hand is H = (E, Q, A). The contact interface C (each module's length and
thickness) is fixed per target. Each mechanism has its own mapping of its
design vector onto these groups (`../partitions.py`); the mapping is never read
off variable names.

- **E, external morphology**, the common descriptor every evaluator already
  measures the same way:
  - finger length `|OC|`;
  - proximal contact-bearing span `|OA''|`;
  - distal contact-bearing span `|B''C|`;
  - articulation station: the extended centre of rotation. For CoDe this is
    where A-B' crosses A'-B, for the 2R it is A, and for the RCJ it is the
    roller contact A + r.
- **Q, mechanism geometry**:
  - CoDe: the pivots A, B (with BC) and B'. B' moves on the line from A
    through the ICR, so the ICR stays where it is.
  - RCJ: the rollers' radius.
  - 2R (2R-DD and belt 2R): none. Fixing E fixes the 2R, so Level 1 has
    nothing to move. This is the mechanism's structure, not a gap in the code.
  Q is measured on these coordinates only. A position that follows E (CoDe's
  B_x from the finger length, B'_z from the ICR, the RCJ's pivots from the
  articulation station) is not Q, so Level 2's external change is not counted
  again as mechanism change.

  Level 1's freedom differs by architecture: three coordinates for CoDe
  (`a_z` and `b_z` are held at 0 by the study's bounds), one for the RCJ,
  none for the 2R. With E fixed, the spans are fixed, so Level 1 can carry a
  thicker module but never a longer one: a module longer than its span needs
  Level 2 on every architecture.
- **A, actuation**, held fixed throughout: the servo model, its mount (CoDe's
  A', the belt 2R's A' held in place in the proximal frame) and its tilt.

Each pad's mounting depth is set by the goal's mounting policy for that
interface (`core.optimization.mounting`). It is never searched. Every design
variable stays within its mechanism's bounds.

## The hierarchy

The engine is `core.optimization.analysis.adaptation.adapt`. At each step the
existing optimizer searches (`core.optimization.study.run_study`) and the
mechanism's own `evaluate()` alone decides feasibility: mounting, collisions,
the required flexion of 120°, and the module usable at that flexion.

1. **The reference as it is.** If it carries the module, stop.
2. **Level 1: change the mechanism only.** Minimize `eps_Q` with E = E0 and
   A = A0.
3. **Level 2: change the external morphology.** This runs only if Level 1
   found nothing. It has two searches:
   - minimize `eps_E` over E and Q, with A = A0;
   - then minimize `eps_Q` with `eps_E <= eps_E* + delta`, where `delta` is
     one grid step relative to the finger length. This is lexicographic: the
     external change comes first, so a redesigned mechanism cannot hide
     external change.

   Each external descriptor moves at most `external_range_mm` from the
   reference. The finger's length bounds widen to match, and so does every
   design bound the finger's length set (`adaptation.with_finger_lengths`:
   `|OA|`, `B_x`, `BC` and the box `A'` and `B'` span from them; each
   mechanism's pad ranges, built again from its goal). No other bound and no
   physical requirement changes. The finger's least length also drops by
   `external_range_mm` (to 70 mm): a shorter finger is an external change,
   and it is counted in `eps_E`.

   Box bounds that a module's length sets (the 2R's and RCJ's pad ends,
   `partitions.MODULE_BOUNDED`) are left to the evaluator's graded support
   constraints, as CoDe's always were. Otherwise a design past them would be
   rejected before evaluation, and only some mechanisms' searches would lose
   their slope.

Every change is the largest change in its group, normalized by the reference
finger's length:
- `eps_E = max_i |e_i - e0_i| / L_f0`
- `eps_Q = max_j |q_j - q0_j| / L_f0`

The two are reported separately, with the changes in mm, and are **never**
combined into one score.

A module longer than the reference's span forces part of `eps_E` whatever
the mechanism: the proximal span must reach `start + l_p` and the distal
span `l_d`. `summary.csv` records this floor (`external_floor`) and the
excess above it (`external_excess`). The mechanisms differ only in the
excess, and in whether they find a design at all. "Not found" means the optimizer found no feasible
design in its range and budget. It is not a proof that none exists.

## References (`references.yaml`)

- **Matched (the primary result).** Every mechanism is built to one external
  morphology: the current prototype's (L_f 90.0, S_p 43.3, S_d 30.2,
  ICR 55.05 mm; `report/prototype.md`). Its design vector is held in
  `references.yaml` (tracked, with its source run, formulation and commit as
  a record), not read from an optimizer output, so a clean checkout runs the
  study and a rerun portfolio does not move the reference. Each mechanism
  keeps its own architecture:
  - CoDe is the prototype itself;
  - the belt 2R's servo sits where CoDe's A' is;
  - the RCJ takes the smallest rollers its hardware allows.

  The idealized RCJ is an upper bound and is reported apart: no roller wall,
  bearing or cable retention is modelled.
- **Native (secondary, reported apart).** Each architecture's own balanced
  design. These run once the study searches are rerun under the current rules
  (`config.yaml`, `kinds`).

## Targets (`targets.yaml`)

The targets are module pairs from Study 1's catalog, chosen by id. Study 2 has
no catalog of its own.

| Target | Proximal (length × thickness, mm) | Distal (length × thickness, mm) |
|---|---|---|
| baseline | 20 × 3 | 20 × 3 |
| long | 35 × 3 | 35 × 3 |
| thick | 20 × 8 | 20 × 8 |
| long_thick | 35 × 8 | 35 × 8 |
| asym_long_proximal | 40 × 3 | 20 × 8 |
| asym_long_distal | 20 × 8 | 40 × 3 |
| asym_proximal_heavy | 45 × 5 | 15 × 3 |
| asym_distal_heavy | 15 × 3 | 45 × 8 |

## Run

```
python scripts/study/study2_morphology_adaptation/run.py [config.yaml] [--targets a,b] [--references a,b]
```

| Config | Purpose |
|---|---|
| `config.yaml` | The study: NSGA-II, population 100, 100 generations, seeds 0, 1 and 2 (each search runs once per seed) |
| `quick.yaml` | A pilot while iterating, with population 32 and 20 generations. Its "not found" means less than the full run's |
| `sensitivity_links.yaml` | CoDe alone without the link face rule (`goal_overrides`), on the targets it does not carry as it is: how much of its cost the crossing links cause. Not a comparison between mechanisms |

The references and modules are the ones Study 1 uses (`../shared.py`). Every
reference must share the comparison rules (`shared.comparison_rules`), or the
run stops.

The run writes to `output/study/study2_morphology_adaptation/`:

| File | Contents |
|---|---|
| `manifest.json` | The config, references, targets, the partitions, the normalization, the optimizer and its seeds, the comparison rules, the formulations and the source commit |
| `references.csv` | Each reference: E0, Q0 and its design vector |
| `summary.csv` | One row per reference, target and seed (a reference that carries the module as it is runs no search, so it has one row). Columns: whether the reference carries the module as it is; each search's status and costs (Level 1; Level 2's external search and its mechanism refinement apart); the final `eps_Q` and `eps_E`; the change in finger length, spans and articulation (mm); the largest mechanism change (mm); the final φ_max and clearance; the final design |
| `summary_best.csv` | One row per reference and target, over its seeds. It holds the best outcome (frozen, then Level 1 by least `eps_Q`, then Level 2 by least `eps_E` and then `eps_Q`), how many seeds found one, and the spread of their costs |
| `targets/<target>/<reference>/seed_<s>/<level>/` | Each search's run (`run.json`, `pareto.csv`), saved as its base mechanism's so `core.optimization.study.load_run` reads it; `run.json`'s `adaptation` holds the level, status, reference design, partition, free group, objective, cap, range and scale |

## Scope

**Included:**
- the 2D finger morphology;
- the modules' lengths and thicknesses;
- the existing collision model, kinematics and mounting policy;
- mechanism and external adaptation.

**Excluded:**
- section depth, the dorsal and palmar envelope, mounting standoffs or
  spacers, and the finger's width;
- changing the actuator model or topology;
- changing the modules' size;
- policy learning and tactile sensing;
- 3D wiring and connector space;
- the physical path for removing and replacing a module.

The study shows that the geometry has room for a module. It does not show
that a module can be swapped in practice.

## Compact-accommodation extension

The separate `../study2_compact_morphology/` study asks how short a finger can
be while carrying each given interface, then selects a small-change morphology
within 0.1 mm of the best-found length. It relaxes the reference proximity box
and uses broad absolute lengths. It does not replace the formulation or results
of this original bounded-adaptation study.
