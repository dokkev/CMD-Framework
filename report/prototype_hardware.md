# CoDe prototype — hardware feasibility, stack-up and release checklist

2026-11-07 · `feature/2dof-fe` at `47cb724` (plus this audit). Design: the
prototype in `report/prototype.md` (run_004 #2113, the tracked vector in
`design/scripts/study/study2_morphology_adaptation/references.yaml`).

Evidence: `design/scripts/study/prototype/hardware_audit.py` writes
`design/output/study/prototype/hardware_audit/audit.json` and
`thickness_envelope.csv`. It uses only the existing FK
(`decode`, `sweep_pip`) and the existing assembly and gap model. No geometry,
kinematics, constraint or design value was changed.

## Scope of this document

**This is a hardware-workstream document. It is not a gate for the research.**
The kinematic study is closed (`kinematic_study_freeze.md`), and these
findings do not invalidate its comparison:
- every mechanism was compared at the same idealized planar abstraction;
- every mechanism used zero clearance;
- every mechanism sampled MCP at 0° only.

The audit's findings fall into three kinds:

| Kind | Findings | Handling |
|---|---|---|
| **A. Affects how the study is read** | E06-2: the MCP range checked (0° only); the nominal zero clearance (E06, E06-7, E06-8); the thickness–length–ROM envelope (§4) | Stated as study limitations in `kinematic_study_freeze.md` |
| **B. Limits of the idealized model** | Bearing and flange diameter against the 6 mm housing disk; Y-layer thickness; shaft retention; bracket walls; fastener access | No action now; the model is stated as an idealized planar packaging model |
| **C. To resolve before fabrication (prototype CAD)** | E06-1: the A bearing against the servo housing; the finger-width budget; shaft and bearing support; module fastening; tolerances | Hardware workstream; prerequisites for building, not for the research |

## Fabrication release status: **UNVERIFIED**

The repository holds **no CAD assembly**: no STEP/STL/native files, no
drawings, no links to a CAD source. All geometry is the planar model. The 3D
stack-up, shaft support, fastener access and module replacement therefore
cannot be verified here.

For fabrication, two issues become blockers if the CAD confirms their
premise:

| Issue | Finding | Why it blocks | Detail |
|---|---|---|---|
| **B1** | The A joint's specified flanged bearing does not fit beside the PIP servo in the servo's own plane | A blocker if A's bearing shares the servo's Y band | E06-1 |
| **B2** | The PIP and MCP servos collide when the MCP extends past about −9° | A blocker if MCP hyperextension is required, or if no MCP stop exists | E06-2 |

Two smaller findings:
- **Width.** The finger-width budget with the modelled servo depth exceeds
  the ~25 mm target (stack-up section).
- **Thickness margin.** The current 5 mm full-span modules sit exactly at
  the model's thickness limit for the prototype's 3 mm mounting depth
  (articulation section).

## 1. Assembly inventory

| Component | Status | Evidence |
|---|---|---|
| Linkage joint centres O, A, A′, B, B′, C | **Modelled** (planar). A = (35.7, 0), A′ = (37.5, 12.4), B = (67.2, 0), B′ = (60.5, 6.5), C = (90, 0) mm | `references.yaml` x; `decode` |
| Coupler A′–B (32.2 mm), driver A–B′ (25.6 mm) | **Symbolic**: centre lines only. Plate width and thickness are not set | `linkage/packaging.py` PARTS: "plates' width is not set yet" |
| PIP servo (XM335) | **Modelled** in the side view as a D shape (r = 9.5, rear extent 25.5 mm, tilt 10°). Y depth 22.0 mm is a project parameter, **not a vendor drawing** | `design/core/hardware/xm335.py`: "project parameters, not a collision or simulator geometry model" |
| MCP servo (XM335, shaft on O, rear body +Z) | **Modelled** (planar), same caveat | `packaging/hardware.py` |
| Servo output horn and its coupling to the coupler at A′ | **Assumed, not modelled.** Only a 10 mm keep-out disk at A′ | `ap_pivot_diameter_mm = 10` in the config |
| Pin housings A, B, B′ | **Symbolic.** Side-view disks of 6 mm (design config) | `a/b/bp_pivot_diameter_mm = 6` |
| Bearings | **Assumed** in a docstring: McMaster 57155K424 (3 mm shaft, 6 mm OD, 2 mm wide, 7.2 × 0.6 mm flange), one each side of a joint, pressed into 1 mm aluminium walls. Implied housing 8 mm | `PIVOT_CLEARANCE_DIAMETER_MM` docstring |
| Shafts, spacers, washers, retaining rings | **Not modelled** | — |
| A-pivot bracket ("A hangs off the servo body on a bracket, in the same plane") | **Assumed, not modelled** | `linkage/packaging.py` docstring |
| Proximal and distal body walls / support structures | **Not modelled.** Bodies are rigid frames only | — |
| Y layers | **Symbolic.** Names only: `central` (servos, pads, pin housings), `driver` and `coupler` plates outside it on opposite sides. No thickness | `LAYERS` |
| Contact pads (blocks) | **Modelled** (planar): proximal x 9.5–43.3 mm (33.8 mm), distal B″ to C (30.2 mm), mounting surface 3.0 mm palmar of OA/BC, 5 mm thick | `pad_spans`, `pad_offsets` |
| Contact-module mounting features (carriers, fasteners, inserts) | **Not modelled** | — |
| Distal pad carrier | Disabled (`distal_carrier_height_mm = 0`) | `Packaging` |

## 2. E06 issue classification
Raw side-view gaps, with no clearance subtracted, are from `audit.json`. The
prototype's own config and the study config give identical numbers. Both
use **body clearance 0**: the prototype was optimized with touching allowed.

| # | Pair | Raw gap | Model status | Classification | Governing dimensions | Smallest practical resolution |
|---|---|---|---|---|---|---|
| E06-1 | A pin housing ↔ PIP servo body | **+0.03 mm** with the modelled 6 mm housing. With the specified bearing: **−0.57 mm** (7.2 mm flange) and **−0.97 mm** (8 mm housing) | Kept apart by a keep-out at zero clearance | **Unresolved hardware design problem.** A 6 mm housing around a 6 mm-OD bearing has no wall, so the modelled size is not physical. In the servo's plane the real housing overlaps the servo | \|AA′\| = 12.53 mm, servo radius 9.5 mm: 3.03 mm of radius left at A | Keep A where it is. Either (a) put A's bearings in walls outside the servo's Y band (costs width; see stack-up), or (b) replace the flanged bearing at A with a plain bushing or pin whose boss is ≤ 5.4 mm diameter (≈0.3 mm gap). The servo case cannot be relieved. **Needs a CAD decision** |
| E06-2 | PIP servo ↔ MCP servo | +1.24 mm at MCP 0°; +0.55 at −5°; **−0.22 at −10°**; −4.5 at −20°. Grows to +3.6 mm with flexion | Checked, but **only at MCP 0°** (`mcp_servo_angles_rad = (0,)`) | **Genuine collision in MCP hyperextension.** Both servos are in the central layer | PIP servo footprint against the MCP servo's dorsal rear body; zero crossing near −9° (between samples) | Specify the MCP ROM. If it includes extension, add a hard MCP stop at about −5° (0.5 mm left) or less. No kinematic change |
| E06-3 | B′ pin housing ↔ PIP servo body (motion −10° to 176°) | **+10.8 mm** | Exempt by a switch (`allow_collision_pip_servo_to_joint_bp`) | **Not a collision.** The exemption is not needed for this design | — | None |
| E06-4 | Driver link A–B′ ↔ PIP servo (motion −10° to 176°, side view) | +1.88 mm to the centre line, at the 10° hyperextension; +2.05 mm over 0–176° | Exempt: different Y layers | **Y separation assumed, unverified.** A plate wider than about 3.8 mm overlaps the servo in side view, so the plate must lie outside the servo's Y band | Link plate width, servo depth | Confirm in CAD that the driver plate lies outside the servo's Y extent |
| E06-5 | Coupler A′–B ↔ PIP servo | 0.0 (attached at A′) | Exempt: different Y layers | **Intended contact.** The servo drives the coupler at A′ through its horn | Horn thickness (not modelled) | Confirm the horn-to-coupler joint and the coupler plate's clearance to the servo face in CAD |
| E06-6 | Pads ↔ crossing links (motion, centre lines) | 0.0 in side view | Exempt: different Y layers | **Y separation assumed.** The crossing links pass over the pads in side view, so the pads must be narrower than the gap between the two link plates | Pad width, plate positions | Fix a module width envelope inside the plates (mounting spec) |
| E06-7 | Proximal ↔ distal pad (motion) | **+0.118 mm** minimum (at 126°); +0.249 at 120° | Checked | **Satisfied in the model, with a margin below realistic tolerances** | Pad thickness 5 mm at mounting depth 3 mm; full spans | Use the thickness and length envelope in the mounting spec |
| E06-8 | Pads ↔ own pin housings | 0.0 | Checked (touching allowed) | **An intended supported contact** in the model (each pad seats against its pins), **but at zero clearance** | Mounting depth 3.0 mm = pin housing radius | With a real 8 mm housing the 3.0 mm seat is too shallow by 1 mm; re-derive the seat depth from the real housing (it changes C, not E/Q/A) |
| E06-9 | Mounting-depth policy vs. evaluator under clearance | — | Model inconsistency (E06) | **A model limitation; it does not affect this prototype.** The prototype uses a searched depth of 3.0 mm, not the policy | — | Leave as recorded. Repair it in `design/core/optimization/mounting.py` before any study that uses non-zero clearance |

## 3. Mechanical stack-up (Y direction)
No CAD values are available, so this is a **specification of what the CAD
must contain**. Values without a source are marked "unknown"; none are
invented.

| Layer (−Y → +Y) | Component | Nominal Y thickness | Source | Adjacent | Running clearance | Retention |
|---|---|---|---|---|---|---|
| 1 | Driver link plate (A–B′) | unknown | — | Body wall / bearing flange at A and B′ | unknown | unknown |
| 2 | Bearing flanges at A (proximal) and B′ (distal) | 0.6 mm each | McMaster 57155K424 (docstring) | Plate / wall | — | Flange against the wall |
| 3 | Body walls carrying A, B′ bearings | 1.0 mm | Docstring | Bearing | — | Press fit |
| 4 | Central layer: PIP servo body, pads, pin housings | 22.0 mm (servo depth) | `xm335.py` (project parameter) | Walls 3 and 5 | — | Servo screws (not modelled) |
| 5 | Body walls carrying A′, B bearings | 1.0 mm | Docstring | Bearing | — | Press fit |
| 6 | Servo horn at A′ | unknown | — | Coupler plate | unknown | Horn screw |
| 7 | Coupler plate (A′–B) | unknown | — | Bearing flange at B | unknown | unknown |
| Shafts | 3 mm pins through walls and plates | 3 mm diameter | Docstring | — | — | unknown (e-clip, shoulder screw?) |

**Width budget.** The `driver` and `coupler` plates must lie outside the
central (servo) band, because each overlaps the servo in side view (E06-4,
E06-5). The finger width is therefore at least:
W ≥ 22.0 (servo) + 2 × 1.0 (walls) + 2 × 0.6 (flanges) + t_driver + t_coupler + t_horn + running clearances + retention.

Even with t_driver = t_coupler = 1.0 mm, t_horn = 0 and no clearances, that
is W ≥ 27.2 mm. The ~25 mm reference is therefore **not reachable with the modelled 22 mm
servo depth** and outer link plates. That reference comes from the task
brief; no finger-width value exists in the repository. Two
explanations are possible:
- the CAD uses a different layer arrangement (for example, walls inside the
  servo band only where the servo is absent in side view);
- the servo's real Y depth differs from 22 mm.

This is a discrepancy to settle in CAD, not a confirmed blocker.

**Load path.** Each joint's shaft carries a link plate that sits outside the
body walls: the driver at A and B′ on one side, the coupler at A′ and B on
the other. If the shaft is supported only by the two body walls and the
plate is outboard, the shaft is overhung by the flange, the plate and the
retention, and bends under contact load. The CAD must show either:
- a two-sided support for each plate (plate between two walls); or
- an overhang short enough for a 3 mm shaft at the expected joint moment.

The joint moments come from Study 3's lever arms (about 30–60 mm of distal
lever) and are not computed here. This is **UNVERIFIED**.

## 4. Articulation (planar model only, preliminary)
The prototype, under its own config:
- **Required flexion.** Feasible at 120°, with φ_max = 176.2° (limited by
  DIP back-off).
- **Hyperextension.** 10° is met.
- **Servo-linkage clearances, at zero clearance.**
  - Driver link to servo: 1.88 mm (centre line, at hyperextension).
  - Pads to each other: 0.118 mm minimum over −10° to 176°, and 0.249 mm
    within 0–120°. The evaluator's `pad_clearance_mm` (0.205 mm) is its own
    least margin over all pad pairs in its window; `report/prototype.md`
    rounds the 120° value to 0.25 mm.
  - Pads to the MCP servo: 0.46 mm.
- **Kinematics unchanged.** Joint centres equal the tracked vector, and φ_max
  and every constraint value equal the Study 1/2 evaluation of the same
  vector.
- **Missing 3D checks.**
  - Link plate to link plate at the crossing.
  - Plates to body walls.
  - Horn to coupler.
  - Shaft heads and retention to the pads and servos.
  - Cable paths.
  - Any mechanical stop.

**Thickness envelope** (prototype geometry, mounting surface 3.0 mm palmar of
OA/BC, both modules alike; `thickness_envelope.csv`):

| Module length | Feasible at 120° up to | Feasible at 150° up to |
|---|---|---|
| Full span (33.8 / 30.2 mm) | **5.0 mm** (5.5 mm fails on `rom_deg`, φ_max 109°; pad–pad contact at the fold is the likely cause, inferred from E06-7) | 5.0 mm |
| 25 mm | 7.5 mm | 6.5 mm |
| 20 mm | 10 mm | 8 mm |

The 20 mm row relaxes the prototype's distal minimum support (25 mm, a
requirement in `report/prototype.md`). It therefore applies to the proximal
module, or to a relaxed distal requirement. The 25 mm and full-span rows
relax nothing.

The current 5 mm full-span modules sit exactly at the limit. Thick modules
have to be shorter at the joint end, or mounted shallower; mounting
shallower is limited by the real pin housings (E06-8). This is the
prototype's own geometry and differs from Study 1, which used policy depths
of 0 and 1.1 mm.

## 5. Prototype release checklist

| Check | Status | Evidence / what is missing |
|---|---|---|
| Physical geometry and assembly complete (CAD with all parts) | **UNVERIFIED** | No CAD in the repository |
| Joint centres equal the tracked design | READY (model) | `references.yaml` vector; audit reproduces the Study 1/2 evaluation |
| Distal ROM 120° (and 10° hyperextension) | READY (planar) / UNVERIFIED (3D) | φ_max 176.2°; 3D interference unchecked |
| A pivot bearing beside the PIP servo | **BLOCKED if same Y band** | E06-1: −0.57 to −0.97 mm in plane; choose layer or bushing |
| PIP servo vs MCP servo over the MCP ROM | **BLOCKED if MCP extends < about −9°** | E06-2; needs an MCP ROM spec or stop |
| Non-zero clearance at every hardware pair | **UNVERIFIED** | The design was optimized at 0 clearance; the pad–pad margin is 0.12 mm |
| Shaft and bearing support and retention, load path | **UNVERIFIED** | Shaft, retention and wall positions not modelled; overhang unknown |
| Finger width budget | **UNVERIFIED** (discrepancy) | Modelled parts give ≥ 27.2 mm against a ~25 mm reference (task brief; not in the repository) |
| Servo mounting (screws, bracket) | **UNVERIFIED** | Not modelled |
| Fastener access | **UNVERIFIED** | Not modelled |
| Contact-module replacement without linkage disassembly | **UNVERIFIED** (principle specified) | `contact_module_interface.md` |
| Contact-interface support and load transfer | **UNVERIFIED** | Carrier and inserts not modelled |
| Manufacturing dimensions and tolerances | **UNVERIFIED** | No drawings |
| Remaining assumptions documented | READY | This report, sections 1–3 |

## 6. Focused validation performed
1. **Kinematics unchanged.** The audit evaluates the tracked vector with the
   existing evaluator and FK. φ_max (176.2°) and every constraint value
   match the Study 1/2 references and `report/prototype.md`. No file under
   `design/core/` was changed.
2. **E06 cases re-evaluated.** All nine are listed above, with raw gaps,
   under both configs.
3. **120° ROM preserved.** Feasible (planar).
4. **Stack-up against CAD.** Not possible (no CAD): **UNVERIFIED**.
5. **Mounting datums.** Read from `pad_spans` and `pad_offsets` (see the
   mounting spec).
6. **Module replacement.** **UNVERIFIED** (no assembly geometry).
7. **Exports and drawings.** None exist, so none were modified.
8. **Tests.** No shared code changed, so none were run beyond executing the
   audit script.

## Remaining assumptions
- The XM335 dimensions are project parameters.
- The bearing choice comes from a docstring, not a bill of materials.
- The pads are rectangular blocks.
- The links are centre lines.
- The model is planar with symbolic Y layers.
- The body clearance is 0.
- The MCP is sampled at 0° only in the design configs.
