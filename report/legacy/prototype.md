# Current prototype (CoDe)

**Track: prototype optimization** (`design/scripts/optimize/linkage/`). Nothing here is a study result, and its numbers
are not compared with the study's (repository README, "Two optimizations: prototype and study").

This is the CoDe finger design currently in CAD. It was logged on 2026-11-05. The design below was optimized with the
PIP servo tilt fixed at 10°. The CAD-derived servo tilt is 15° (`finger.pip_servo_tilt_deg` in
`design/core/morphology/code_hand/prototype.yaml`; commit `ef2a0f8`), so the performance numbers below use the
optimization's 10°.

![prototype](figures/prototype.png)

*From left to right: extended, at the required flexion of 120° (the pose where coverage is measured), and at φ_max = 176.1°. The middle panel shows the result of the 2026-11-05 link check (`links_block_face`): green points are kept. Both pads are kept in full.*

## Source

| Item | Value |
|---|---|
| Design | `design/output/servo_close/linkage/proto_tilt10_align`, the `balanced` seed (run_004 #2113) |
| Optimizer | smsemoa, seed 0, population 200, 1000 generations |
| Formulation | linkage-v12 |
| Code | `7334085` (2026-11-04, "Search each pad's mounting offset below its link line") |

### Goal it was optimized under

| Item | Value |
|---|---|
| Finger length \|OC\| | 80–90 mm |
| Required flexion φ_req | 120° |
| Segment ratios | \|OA″\| ≥ 0.35, \|B″C\| ≥ 0.25, \|AB\| ≤ 0.35 of \|OC\| |
| Transmission angle | 30–150° |
| PIP servo tilt | fixed at 10° |
| Pad mounting depth | searched, the two pads aligned to one depth (`align_mount_offsets: true`) |
| Pads | at least proximal 20 mm and distal 25 mm long, 5 mm thick |
| Servos (PIP, MCP) | radius 9.5 mm, rear extent 25.5 mm |
| Objectives | transmission, pad_coverage, curl |

## Geometry

All values are in the proximal frame with the finger extended: O is the MCP at the origin, +X points distally and +Z dorsally. They match the CAD variables.

| CAD variable | Value | Note |
|---|---|---|
| A_x, A_z | 35.7, 0 mm | PIP pivot |
| Ap_x, Ap_z (A′) | 37.5, 12.4 mm | PIP servo shaft |
| B (absolute) | 67.2, 0 mm | DIP pivot |
| Bp_x, Bp_z (B′, relative to B) | −6.7, 6.5 mm | absolute (60.5, 6.5) |
| C_x, C_z (C, relative to B) | 22.8, 0 mm | fingertip, BC = 22.8 mm |
| pip_servo_tilt | 10° | |
| A′B | 32.2 mm | coupler |
| AB′ | 25.6 mm | driver link |

Values derived from these:
- \|OC\| = 90.0 mm (the top of its range)
- \|OA\| = 35.7 mm
- \|AB\| = 31.5 mm
- The extended centre of rotation sits at 0.612 of \|OC\|.

### Pads

| | Proximal pad | Distal pad |
|---|---|---|
| Span (along its link line) | x = 9.5 → A″ = 43.3 mm (A″ is 5.8 mm past A′_x) | B″ = 7.4 mm behind B → C (B″ is 0.7 mm short of B′_x) |
| Mounted length | 33.8 mm | 30.2 mm |
| Mounting depth below the line | 3.0 mm | 3.0 mm |
| Contact face depth (mounting + thickness) | 8.0 mm | 8.0 mm |

## Performance

| Item | Value |
|---|---|
| k_max (PIP servo torque per distal moment, 0–120°) | 2.449 (k_min 2.319) |
| Coverage | 0.711 (effective 33.8 + 30.2 mm, both pads usable in full) |
| φ_max | 176.1° (limited by DIP back-off) |
| Servo travel to 120° | 50.4° |
| Least pad clearance | 0.25 mm |
| Least transmission margin | 50.6° (toggle margin 15.3°) |
| Driver link to servo clearance | 1.9 mm |
| Fingertip at 120° | C_x = 35.4 mm, far in front of the MCP |

## Under the current rules (2026-11-05)

The prototype was optimized under linkage-v12. We re-evaluated it with the rules added since then.

- **Link rule** (`goal.loss.links_block_face`): neither crossing link passes in front of either contact face at 120°.
  - The coupler A′B runs outside A″ and comes down the back of the distal pad to B.
  - The driver link A-B′ runs past the end of the proximal pad to B′.
  - So no length is lost: the usable lengths stay 33.8 + 30.2 mm and coverage stays 0.711.
- **`pad_waste_mm` constraint removed**: the prototype has no pad length beyond what it can use, so the change does not affect it.
- **Mounting depth policy**: the prototype searched its depth (range 0–10 mm) and aligned the two pads, so it does not follow the study's `shallowest_supported` policy. The distal pad runs under B, and the depth found, 3.0 mm, equals the radius of B's pin housing.

What these results show: B sits 22.8 mm back from C, and both links pass behind the pads. The link rule demands exactly this shape. The study's earlier CoDe designs with B on C (BC 0.1 mm) do not have it, and neither do the balanced designs of the 2026-11-04 production search, which used |OC| 90–100 mm and 150°.

## Design variant: 8 mm pivots (2026-11-12)

This variant is not in CAD yet. It answers a hardware question: what finger the production search finds once every
pivot housing is 8 mm. It is the hand that `sim/` currently builds with pads
(`design/core/morphology/code_hand/pivot8_balanced.yaml`; see `sim/isaaclab/scripts/code_hand/README.md`, E7).

### Source

| Item | Value |
|---|---|
| Config | `design/scripts/optimize/linkage/config_pivot8.yaml` (extends `config.yaml`) |
| Runs | `design/output/linkage/config_pivot8/search/run_001`–`run_009`, merged in `portfolio_001` |
| Optimizer | nsga2, smsemoa and agemoea × seeds 0–2; population 200, 1000 generations each |
| Formulation | linkage-v14 |
| Code | `1c09b26` |

### Goal: what differs from `config.yaml`

| Item | `config.yaml` | `config_pivot8.yaml` |
|---|---|---|
| Pivot housings A, A′, B, B′ | 6, 10, 6, 6 mm | 8, 8, 8, 8 mm |
| Finger length \|OC\| | 90–100 mm | 84–91 mm |
| Pad thickness | 5 mm | 3 mm |
| Pad mounting depths | searched separately | searched, the two pads aligned |

Shared with `config.yaml`: required flexion φ_req 150°, PIP servo tilt 15°, pads at least proximal 20 mm and
distal 25 mm, objectives transmission, pad_coverage and curl. This goal differs from the CAD prototype's (above:
120°, 80–90 mm, tilt 10°, 5 mm pads), so the two designs' numbers are not a like-for-like comparison.

### Representative seeds

k_max is taken over 0–150°. φ_safe is how far the finger flexes before a stop.

| Seed | Run #candidate | k_max | Proximal pad + distal pad | φ_safe | Pad clearance | Stop |
|---|---|---|---|---|---|---|
| best_transmission | run_002 #1 | **1.92** | 32.0 + 27.4 mm | 150.2° | 0.047 mm | DIP back-off |
| most_pad | run_002 #2475 | 3.23 | **40.2 + 37.9 mm** | 158.2° | 0.015 mm | DIP back-off |
| most_curl | run_005 #1713 | 2.24 | 31.6 + 28.9 mm | **180.0°** | 0.211 mm | curl |
| balanced | run_009 #2099 | 2.39 | 35.6 + 33.8 mm | 177.4° | 0.042 mm | DIP back-off |

In every seed, the two pads come within 0.25 mm of each other. The search pushes the pads to the closest clearance it
allows.

### The balanced seed (`pivot8_balanced.yaml`)

![pivot-8 balanced](figures/prototype_pivot8_balanced.png)

*The balanced seed from 0° to φ_safe = 177.4°. Source: `design/output/finger/pivot8_search_balanced_poses.png`.*

The proximal frame's values are rounded to 0.1 mm.

| CAD variable | Value |
|---|---|
| A_x, A_z | 37.2, 0 mm |
| Ap_x, Ap_z (A′) | 40.1, 13.2 mm |
| B (absolute) | 69.0, 0 mm |
| Bp_x, Bp_z (B′, relative to B) | −8.5, 6.6 mm (absolute 60.5, 6.6) |
| C_x (C, relative to B) | 21.9 mm, \|OC\| = 90.9 mm |
| A′B, AB′ | 31.8, 24.2 mm |
| Proximal pad | x = 9.5 → 45.1 mm (35.6 mm), A″ 5.0 mm past A′_x |
| Distal pad | B″ 11.9 mm behind B → C (33.8 mm), B″ 3.4 mm short of B′_x |
| Pad mounting depth, thickness | 4.0 mm (the radius of the 8 mm housings), 3 mm |

Other results:
- k_min is 2.20.
- The servo travels 65.5° to reach 150°.
- The least transmission margin is 44.7°, and the toggle margin is 7.2°.
- The driver link clears the servo by 2.3 mm.
- The pad width for simulation (19 mm, the XM335 case width) is a placeholder and was not searched.
