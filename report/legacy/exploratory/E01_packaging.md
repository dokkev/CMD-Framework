# E01 — Actuator-inclusive packaging share

**Status: SUPPORTED** for palmar free depth, **with a relocation cost**:
dorsal height, and no gain in the pads' share of the side-view area.

## 1. Question
At matched morphology and modules, how much of the palmar (contact) side
of the extended finger is left free of hardware?

## 2. Hypothesis and alternative
- **Hypothesis.** Remote actuation (CoDe, belt 2R) leaves more palmar depth
  free than joint-mounted servos (2R-DD, RCJ).
- **Alternative.** Pins and links, which every architecture has, dominate,
  so the differences are small. Or the hardware is merely moved somewhere
  else.

## 3. Model and assumptions
- The existing scene solids at the extended pose, rasterized on a 0.1 mm
  grid.
- Hardware = links, pins, servos, pulleys, rollers and carrier. The MCP
  servo (palm) is excluded.
- Bars are drawn as thin capsules (no link radius in the study configs).
- 2D side view; Y layers are not resolved.

## 4. Method
- References: the seven matched references.
- Modules: baseline (20×3) and thick (20×8).
- Metrics:
  - h(x), the deepest hardware below the link line along the finger;
  - the "module-free length", where h ≤ d;
  - the dorsal–palmar thickness;
  - the pads' share of the occupied area.

Script: `design/scripts/study/exploratory/e01_packaging.py`. Data:
`design/output/study/exploratory/e01_packaging/`. Figure:
`report/figures/exploratory/e01_palmar_intrusion.png`.

## 5. Results (baseline module)

| Reference | Deepest palmar hardware | Module-free length, d = 3 mm | Dorsal top | Thickness | Pad share of area |
|---|---|---|---|---|---|
| CoDe | 3.0 mm (pin housings) | 90.0 mm (100%) | 26.1 mm | 30.2 mm | 0.15 |
| Belt 2R | 5.9 mm (pulley) | 78.8 (88%) | 26.1 | 32.0 | 0.14 |
| 2R-DD 90° | 9.4 (joint servo) | 72.0 (80%) | 25.4 | 34.8 | 0.17 |
| 2R-DD 0° | 9.4 | 55.5 (62%) | 9.4 | 18.8 | 0.17 |
| RCJ 90° | 9.4 (servo, rollers) | 37.5 (42%) | 25.4 | 36.2 | 0.12 |
| RCJ 0° | 9.4 | 21.0 (23%) | 9.4 | 21.9 | 0.12 |
| *Ideal RCJ* | *2.9* | *90.0 (100%)* | *2.9* | *5.9* | *0.67* |

## 6. Interpretation
- **Palmar side.**
  - CoDe is the only physical design whose palmar intrusion is limited to
    its pin housings (3 mm) along the whole finger.
  - The belt's pulley on the joint axis costs 5.9 mm.
  - A servo on the joint axis costs its full radius (9.4 mm) over about
    20 mm of the joint region, whichever way it is tilted.
- **Dorsal side.** Remote actuation does not remove hardware; it moves it
  dorsally. CoDe's A′ servo raises the dorsal top to 26 mm, against 9.4 mm
  for the laid-back joint servos. So CoDe is thicker overall than the 2R-DD
  at 0° and the RCJ at 0°.
- **Area share.** The pads' share of the occupied side-view area is no
  better for CoDe (0.15) than for the 2R-DD (0.17), because the PIP servo
  is counted wherever it sits.

## 7. Critical review
- **Joint axis on the link line.** Every model puts the joint axis on the
  link line (a_z = 0). A joint servo raised dorsally off the axis line is
  not modelled. That arrangement could reduce the 2R-DD's palmar intrusion,
  at the cost of a transmission.
- **Extended pose only.** Intrusion during motion is E07's question.
- **No buildability claim.** "Free" means no modelled hardware in the side
  view, not that a 3D module can be built there.

## 8. Follow-up
None needed: the result is direct geometry.

## 9. Limitations
- 2D only.
- One morphology.
- Link bars have no thickness.
- MCP servo excluded.

## 10. Takeaway
Remote actuation is what frees the palmar side: CoDe, and to a lesser
degree the belt 2R, keep the contact side clear of everything except pin
housings. The price is dorsal height, not palmar space. This supports
"accommodate contact" as a **relocation** of hardware away from the
contact side, not a reduction of hardware.
