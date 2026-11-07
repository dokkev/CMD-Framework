# Packaging vs. contact accommodation (preliminary paper figures)

These figures and the table show a **trade-off under the present
matched-reference model**: how much contact-module accommodation each finger
architecture achieves, and how much side-view thickness its hardware takes
to achieve it. They do not rank mechanisms. Nothing was re-optimized.

## Contents

| File | Content |
|---|---|
| `packaging_comparison.{pdf,png}` | Extended-pose side views of the Study 1 matched references, at one scale |
| `accommodation_vs_thickness.{pdf,png}` | (a) Feasible pairs at 120° against H_finger, over the E04 servo-tilt sweep, with whiskers to H_bbox; (b) H_finger (solid) and H_bbox (dashed) against tilt |
| `mechanism_comparison.tex` | Compact table (single column) and extended table (`table*`) |
| `mechanism_comparison.csv` | The same numbers, machine-readable |
| `tilt_sweep.csv` | The per-tilt thickness and accommodation behind figure (a)/(b) |
| `manifest.json` | Source commit/patch records for the data and the figures |

### Packaging comparison panels
- **References.** CoDe (servo 10°), 2R-DD (servo 0° and 90°),
  belt 2R (servo 10°), RCJ (servo 0° and 90°).
- **Hardware.** Every outline is the existing scene solid
  (`core.morphology.contact.loss`), redrawn by category:
  - finger servo: grey;
  - links and belt strands: mechanism colour;
  - pins, pulleys and rollers: pale yellow;
  - proximal and distal pads: pink and blue.
- **Excluded from the measures.** The palm's MCP servo is dashed, because it
  is excluded.
- **Annotations.**
  - The dashed black line is the proximal reference line (z = 0).
  - Each panel marks the dorsal hardware top, the pad bottom and H_finger.
  - Where hardware extends below the pad bottom, a red line marks it and
    gives H_bbox.

### Accommodation vs. thickness
- Each architecture's matched reference is frozen; only the servo tilt
  varies (−30° to 120°), as in E04.
- Accommodation is read from E04's saved `summary.csv` (71 pairs, 120°).
- **Markers.**
  - Filled points: the reference is feasible at that tilt.
  - Open points: the reference itself is infeasible at that tilt, so it
    carries 0 for that reason.
  - Stars: the Study 1 reference configurations.
- **Missing points.** RCJ tilts ≤ −10° are absent, because the baseline
  pads do not mount there (no policy depth), so no thickness exists.
- **Read H_bbox alongside H_finger.** At most tilts the 2R-DD joint servo
  hangs below the pad surface, so H_finger alone understates its envelope.
  At −30°, H_finger is 12.5 mm but H_bbox is 30.5 mm. Whiskers in (a) and
  dashed lines in (b) show H_bbox. The dip of the 2R-DD H_finger curve
  around 0° is partly this metric effect.
- **Pad depth follows the tilt.** The pads are re-seated at policy depth
  for each tilt, so the pad bottom also moves with tilt (for example,
  2R-DD at +10°: −8.2 mm).
- **Not a Pareto front.** The lines connect tilts in order. They are fixed-
  reference sweeps, not optimized fronts.

## Metric definitions
All quantities are measured in the extended pose, with the baseline 20 × 3
mm pads at policy depths, in mm, relative to the proximal reference line.
The MCP servo is excluded.

| Quantity | Definition |
|---|---|
| `dorsal_top_mm` | Highest point of the finger hardware (links, pins, servo, pulleys, rollers, carrier) |
| `pad_bottom_mm` | Lowest point of the two pads, i.e. the palmar contact surface |
| `h_finger_mm` | `dorsal_top − pad_bottom` (primary) |
| `h_bbox_mm` | Highest minus lowest point over hardware and pads |

The two thickness measures differ when hardware lies below the pad surface:
- 2R-DD, both tilts: by 6.5 mm (the joint servo);
- belt 2R: by 3.0 mm (the pulley).

Bars count as lines, because the study configs set no link radius.

## Key observations
These are observations under the model, with no ranking implied.
- **Thinnest in H_finger: the 2R-DD at 0°** (12.5 mm). But its
  joint servo protrudes 6.5 mm beyond the pad surface (H_bbox 19.0), and it
  carries 17 pairs at 120°.
- **Standing the 2R servo up (90°)** raises accommodation to 31 pairs and
  H_finger to 28.5 mm. The joint servo still protrudes beyond the pads.
- **CoDe** carries 35 pairs at H_finger = H_bbox = 30.4 mm. Below the
  reference line it has only its pin housings (3.1 mm), within the 4.2 mm
  pad depth. Its cost is dorsal (26.2 mm top). Its accommodation does not
  change between 10° and 120° of tilt, while its thickness grows to 43.8 mm.
- **RCJ** at 0° is 22.0 mm thick and carries 20 pairs. At 90° it is
  36.3 mm and carries 32. Its pads sit deep (10.8–12.5 mm) below the
  rollers.
- **Belt 2R** carries 33 pairs at 29.2 mm (H_bbox 32.2).

## Validation (focused)
1. **Agreement with E01's raster envelope.** The metric agrees with E01 (0.1
   mm grid) to within 0.2 mm for every reference; the raster reads
   systematically lower by its grid.
2. **The two measures stay distinct.** H_finger and H_bbox differ exactly
   where hardware lies below the pads (2R-DD, belt 2R).
3. **The sweep reproduces Study 1.** At the reference tilts (CoDe 10°,
   belt 10°, 2R-DD 0° and 90°, RCJ 0° and 90°), the sweep gives
   the Study 1 counts (35, 33, 17, 31, 20, 32) and the same thickness as the
   reference measurement.
4. **The table matches Study 1.** Its counts were recomputed from
   `assembly_results.csv` and match.

## Assumptions and limitations
- **2D projection.** These are planar side-view heights with Y layers
  assumed. They are not 3D volumes, and they say nothing about finger
  width, which a 0° servo may increase.
- **Matched reference configurations.** These are references to the
  prototype's external morphology (90 mm), not architecture-wide optima.
  Other designs of each architecture could be thinner or carry more.
- **Tilt sweep holds everything else.** It does not re-optimize the other
  coordinates, and it applies no dorsal-envelope limit.
- **Clearance (E06).** The model allows touching at zero clearance. The
  prototype has zero-clearance contacts (A pin–PIP servo, PIP servo–MCP
  servo), and the mounting policy does not apply `body_clearance_mm`. Both
  are left as they are, not repaired here.
- **No further claims.** Nothing here shows manufacturability, 3D assembly,
  or manipulation performance.

## Reproduce
```
python scripts/study/exploratory/packaging_tradeoff.py   # measures -> output/study/exploratory/packaging_tradeoff/
python scripts/study/exploratory/packaging_figures.py    # figures, table -> report/figures/packaging_tradeoff/
```
Run these in the `cmd` environment with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

**Inputs:**
- Study 1 `output/study/study1_fixed_morphology/assembly_results.csv`;
- E04 `output/study/exploratory/e04_actuator_orientation/summary.csv`;
- references `scripts/study/study2_morphology_adaptation/references.yaml`;
- catalog `scripts/study/study1_fixed_morphology/modules.yaml`.
