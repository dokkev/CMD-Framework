# Packaging vs. contact accommodation (preliminary paper figures)

These figures and tables describe a **geometric trade-off under the present
matched-reference model**: how many contact modules each finger architecture
accommodates, and how much side-view envelope its hardware occupies to do
so.
- **Not a ranking.** They do not rank mechanisms, and nothing was
  re-optimized.
- **The model.** Every result is a *nominal* planar model: zero body
  clearance, Y layers assumed, extended pose. The E06 clearance issues are
  left as they are; see Limitations.

## Contents

| File | Content |
|---|---|
| `packaging_comparison.{pdf,png}` | (a)–(f) Extended-pose side views of the Study 1 matched references, at one scale |
| `accommodation_vs_thickness.{pdf,png}` | (a) Feasible pairs at 120° against the full envelope H_bbox, over the E04 servo-tilt sweep; (b) H_bbox (solid) and H_finger (dashed) against tilt |
| `mechanism_comparison.tex` | Three tables: compact reference table; extended diagnostics (`table*`); thinnest evaluated tilt per architecture |
| `mechanism_comparison.csv` | Reference-configuration numbers, machine-readable |
| `tilt_sweep.csv`, `best_observed.csv` | Per-tilt measurements and counts; the thinnest evaluated tilt per architecture |
| `manifest.json` | Source commit and patch records, the SHA-256 of the input CSVs, and the commands that regenerate them |

## Metrics
All are measured in the extended pose, with the baseline 20 × 3 mm pads at
policy depths, in mm, relative to the proximal reference line (z = 0). The
palm-mounted MCP servo is excluded for every architecture.

| Quantity | Definition |
|---|---|
| `h_bbox_mm` | **Primary envelope metric.** Highest minus lowest point over all finger hardware (links, pins, servo, pulleys, rollers, carrier) and both pads |
| `h_finger_mm` | Dorsal hardware top to the pad (contact) surface. Equals H_bbox unless hardware hangs below the pads |
| `dorsal_top_mm`, `pad_bottom_mm`, `hardware_bottom_mm` | The extremes used above |

H_finger alone understates the envelope wherever hardware sits below the
contact surface:
- 2R-DD: by 6.5 mm (the joint servo);
- belt 2R: by 3.0 mm (the pulley).

H_finger is kept as a secondary quantity, because it measures how much
height lies behind the contact surface.

## Figures

**Packaging comparison.**
- Every outline is an existing scene solid (`core.morphology.contact.loss`),
  redrawn by category:
  - finger servo: grey;
  - links and belt: mechanism colour;
  - pins, pulleys and rollers: pale yellow;
  - pads: pink and blue;
  - MCP servo: dashed, excluded.
- The arrow in every panel is H_bbox. Where hardware hangs below the pads, a
  red line marks it and H_finger is given.

**Accommodation vs. envelope.**
- **The sweep.** Each architecture's matched reference is frozen and only
  the servo tilt varies (−30° to 120° in 10° steps), as in E04. Counts come
  from E04's saved `summary.csv` (71 pairs, 120°).
- **Markers.**
  - Filled points: the reference is feasible at that tilt.
  - Open points: the reference itself is infeasible there. These are not
    connected by lines.
  - Stars: the Study 1 reference configurations.
  - Large rings: the **thinnest evaluated tilt** (smallest H_bbox) that
    reaches the architecture's highest 120° count. This is an observation
    of the sweep, not an optimum.
- **Missing points.** RCJ tilts ≤ −10° are absent: the baseline pads do not
  mount there.
- **Pad depth follows the tilt.** Pads are re-seated at policy depth for
  each tilt, so the pad surface moves with tilt (for example, 2R-DD at
  +10°: −8.2 mm).

## Observations
These are observations under the model, with no ranking implied.

**Study 1 references:**

| Reference | H_bbox | H_finger | Pairs at 120° | Pairs at 150° |
|---|---|---|---|---|
| CoDe 10° | 30.4 | 30.4 | 35 | 28 |
| 2R-DD 0° | 19.0 | 12.5 | 17 | 0 |
| 2R-DD 90° | 35.0 | 28.5 | 31 | 13 |
| Belt 2R 10° | 32.2 | 29.2 | 33 | 17 |
| RCJ 0° | 22.0 | 22.0 | 20 | 5 |
| RCJ 90° | 36.3 | 36.3 | 32 | 22 |

**Thinnest evaluated tilt reaching each architecture's highest count:**

| Architecture | Tilt | Pairs at 120° | H_bbox |
|---|---|---|---|
| 2R-DD | **30°** | 31 | 30.5 mm |
| Belt 2R | **0°** | 33 | 27.9 mm |
| CoDe | 10° | 35 | 30.4 mm |
| RCJ | 90° | 32 | 36.3 mm |

For the 2R-DD, 30° reaches the same 31 pairs as 90° with a 4.5 mm smaller
envelope. Highlighting only 90° would hide this.

- **A comparable envelope.** Under these configurations, CoDe reaches its
  count at an envelope comparable to the 2R-DD's thinnest 31-pair tilt
  (30.4 vs 30.5 mm).
- **The belt is thinnest.** The belt 2R at 0° is the thinnest
  configuration reaching 33 pairs (27.9 mm).
- **Where CoDe differs.** CoDe has only pin housings below the reference
  line (3.1 mm, inside the 4.2 mm pad depth). Its height is dorsal.
- **Orientation.** CoDe's count does not change from 10° to 120°, while its
  envelope grows to 43.8 mm.

**Suggested reading:**
- **Joint-mounted actuators.** They can improve accommodation through servo
  reorientation, at a packaging cost.
- **CoDe.** It arranges its hardware differently. It keeps the contact-
  facing side clear, with an overall envelope comparable to the alternatives
  in these configurations.
- **No compactness claim.** CoDe is not more compact, and a thin finger has
  its own value.
- **Scope.** All of this applies to one 90 mm reference morphology.

## Validation (focused)
1. **Agreement with E01.** The metric agrees with E01's raster envelope
   (0.1 mm grid) to within 0.2 mm for every reference.
2. **The two measures stay distinct.** H_finger and H_bbox differ exactly
   where hardware lies below the pads.
3. **The sweep reproduces Study 1.** At the reference tilts, the counts (35,
   33, 17, 31, 20, 32) and the thicknesses match the reference
   measurements.
4. **The tables match Study 1.** The counts were recomputed from Study 1's
   `assembly_results.csv` and match.
5. **One independent review.** Its material issue (H_finger alone hid the
   2R-DD envelope in the sweep) is addressed by making H_bbox primary.

## Limitations
- **2D projection.** These are planar side-view heights, not 3D volumes.
  Finger width is not represented; a laid-back servo may widen the finger.
- **Matched references, not optima.** These are matched references to one
  90 mm morphology, not architecture-wide optima. The tilt sweep holds every
  other coordinate, so the "thinnest evaluated tilt" is only the thinnest
  among the evaluated tilts.
- **Nominal zero clearance (E06).**
  - The prototype has zero-clearance static contacts: the A pin against the
    PIP servo, and the PIP servo against the MCP servo.
  - The mounting policy does not apply `body_clearance_mm`.
  - Both are left as they are, so the counts are nominal-model counts.
- **No further claims.** Nothing here shows manufacturability, 3D assembly,
  or manipulation performance.

## Reproduce
The inputs are untracked run outputs. Regenerate them if they are absent,
and check them against the SHA-256 values in `manifest.json`:
```
python scripts/study/study1_fixed_morphology/run.py            # output/study/study1_fixed_morphology/assembly_results.csv
python scripts/study/exploratory/e04_actuator_orientation.py   # output/study/exploratory/e04_actuator_orientation/summary.csv
```
Then:
```
python scripts/study/exploratory/packaging_tradeoff.py   # measurements -> output/study/exploratory/packaging_tradeoff/
python scripts/study/exploratory/packaging_figures.py    # figures, tables -> report/figures/packaging_tradeoff/
```
Run everything in the `cmd` environment with
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

The references come from
`design/scripts/study/study2_morphology_adaptation/references.yaml`, and the
catalog from `design/scripts/study/study1_fixed_morphology/modules.yaml`.
