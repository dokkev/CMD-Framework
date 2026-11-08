# CoDe contact-module mounting interface (specification draft)

2026-11-07. The interface between the fixed hand H = (E, Q, A) and the
variable contact interface C (GOAL.md §5.6). Values come from the prototype
model (`pad_spans`, `pad_offsets`, `hardware_audit.py`). Anything the CAD
must still establish is marked UNVERIFIED. No sensor or carrier dimension is
invented.

## 1. Datums and frames
Both frames are right-handed, in mm. +X runs along the segment from its outer
end toward the joint for the proximal frame, and along BC toward the tip for
the distal frame. +Z is dorsal, so a module extends toward −Z. Y is across
the finger, with the frame origin at the finger's mid-plane.

| | Proximal frame P | Distal frame D |
|---|---|---|
| Body | Proximal body (O–A–A′) | Distal body (B′–B–C) |
| Reference line (datum) | Line OA (z = 0 in the proximal frame) | Line BC |
| Origin | x = 9.5 mm from O (pad start: MCP servo shaft disk plus offset), on the reference line | B″ = 7.4 mm proximal of B along BC, on the reference line |
| Usable mounting length | 33.8 mm (to A″ at x = 43.3) | 30.2 mm (B″ to C) |
| Prototype mounting surface | 3.0 mm palmar of OA | 3.0 mm palmar of BC |
| What fixes the length | MCP servo at the outer end (0.46 mm gap); A″ at the joint end | B″ at the joint end; the fingertip C |

**The datum is the reference line, which is part of H.** The mounting-surface
depth belongs to the module and its carrier (C). It may differ between
modules, but it must clear the real pin housings: with 8 mm housings, at
least 4.0 mm under the A, B and B′ housings that a module passes beneath
(E06-8).

## 2. Allowable envelope (planar model, prototype geometry)
These limits hold for the current linkage at a 3.0 mm mounting surface, with
both modules alike (`thickness_envelope.csv`):

| Module length (from the outer end) | Max thickness, feasible to 120° | Max thickness, feasible to 150° |
|---|---|---|
| Full span (33.8 / 30.2 mm) | 5.0 mm | 5.0 mm |
| 25 mm | 7.5 mm | 6.5 mm |
| 20 mm (proximal only; the distal requirement is ≥ 25 mm) | 10 mm | 8 mm |

- **What limits it.** The binding interaction is the two modules meeting at
  the fold. Thick modules must leave the joint end free: they must be
  shorter, or chamfered at the joint end. A chamfer is not modelled; check it
  with the evaluator before use.
- **Width.** Each module must fit inside the central layer, between the
  driver and coupler plates. The crossing links pass over the pads in side
  view (E06-6), so module width ≤ the inner plate spacing minus running
  clearance. **UNVERIFIED**: plate positions are not modelled. The servo band
  (22 mm, a project parameter) is the most the model implies.
- **Tolerance margin.** At the 5 mm full-span limit, the pad-to-pad margin is
  0.12 mm. Leave at least the fabrication tolerance; this reduces the table's
  limits.

## 3. Attachment principle (proposed; UNVERIFIED against CAD)
- **Carrier and contact body.** Each module is a rigid carrier with the
  contact body (rigid, compliant or sensorized) bonded or fastened to it. The
  carrier, not the contact body, carries the attachment load into the
  segment body. The module takes no part in the linkage's structural load
  path: the bodies' walls and shafts carry the joint loads.
- **Seat.** The carrier seats on a flat mounting surface of its segment
  body, parallel to the datum at the module's depth. It is located by two
  dowels or a slot-and-pin, which fixes the frame repeatably.
- **Fasteners.** Use screws into threaded inserts in the segment body,
  driven from the **palmar side** at the carrier's outer-end tab and at its
  inner end.
  - Palmar access needs no linkage, shaft or bearing removal.
  - For the distal module, flex the finger so that its inner end is
    expected to clear the proximal module (to be confirmed in CAD).
  - For sensorized or compliant modules, the screw heads sit in pockets of
    the carrier, outside the active contact area, or under a cover.
- **Independence.** The proximal and distal modules share no fastener or
  carrier, so each can be replaced alone (GOAL.md §5.3).
- **Rejected alternative.** Lateral (±Y) fasteners are not used: the link
  plates cover the pad sides in parts of the motion (E06-6), so lateral
  access depends on pose and on the unmodelled plate geometry.

## 4. Replacement procedure (to be confirmed on the CAD)
1. Bring the finger to a pose with the module's fasteners exposed: extended
   for the proximal module, 90–120° flexed for the distal module.
2. Remove the module's palmar screws (two per module) and lift the carrier
   off its dowels along −Z.
3. Seat the new carrier on the dowels and refit its screws.
4. Check the module's thickness and length against section 2, or run its
   dimensions through the evaluator.

No linkage, shaft, bearing or servo is touched. **UNVERIFIED**: the CAD must
confirm the screw access paths and that no link plate covers a screw head in
the chosen pose.

## 5. Module families: integration requirements

| Family | Attachment | Must be checked |
|---|---|---|
| **A. Rigid / thin** (reference) | Carrier is the contact body; two palmar screws on dowels | Thickness and length within section 2; flatness of the seat |
| **B. Thick compliant** | Compliant body on a rigid carrier; screws in pockets outside the contact area | Section 2 envelope at the *compressed and uncompressed* thickness; joint-end length and chamfer; carrier stiffness; that bulging under load does not reach the other module at the fold (the 0.12–0.25 mm margins are geometric only) |
| **C. Sensorized** | Sensor in or under the contact body, on a carrier with a connector | Envelope as for B, plus connector and cable volume behind the carrier inside the central layer; a cable path from the distal to the proximal body across the joint. The path length changes strongly with flexion (Study 4: −20 to −60% chord/arc change around the joint), so a service loop or a flexible strain-relieved routing is needed. **UNVERIFIED**: no cable path exists in the model |

## 6. Open items before freezing the interface
- **Real geometry.** The real pin-housing diameter at A, B and B′, which sets
  the minimum mounting depth, and the inner spacing of the link plates,
  which sets the module width (see `prototype_hardware.md`, E06-1 and
  E06-6).
- **Fasteners.** The insert and screw size, the dowel positions, and the
  palmar access paths in CAD.
- **Cable channel.** The cable channel location for family C.
