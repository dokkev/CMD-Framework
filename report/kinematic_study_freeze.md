# Kinematic study freeze

2026-11-07 · `feature/2dof-fe`. Decision: the kinematic mechanism study
(Studies 1–4, E01–E10, the packaging trade-off) is closed. It is not a
fabrication-readiness review.
- **What the study is for.** It shows *why* the architecture suits contact-
  interface research (GOAL.md §4, questions 1–2). It is not proof of a
  finished finger.
- **Next research phase.** Contact Morphology Experiment Design.
- **Hardware.** Physical realization runs as a parallel engineering
  workstream (`prototype_hardware.md`).

## Kinematic study vs. hardware design

| Item | Kinematic study (closed) | Hardware design (parallel workstream) |
|---|---|---|
| Linkage closure / singularity | Required; done (existing FK) | — |
| 120° ROM / hyperextension | Required; done | Physical stops |
| Transmission ratio and angle | Required; done (k, Study 3 lever arms) | Measured torque and stiffness |
| Pad–pad interference | Required; done (planar) | 3D geometry |
| Servo–servo packaging | 2D approximation; **MCP 0° only** | Real assembly over the MCP ROM |
| Pivot–servo clearance | Idealized envelope (6 mm housing disks) | Bearing and bracket design |
| Shafts, bearings, spacers | Assumption stated | Detailed design |
| Y-direction stack-up | Layer assignment (names only) | Real dimensions |
| Contact-module mounting | Independent replaceability in principle | Fasteners and access |

**The same abstraction level applies to every mechanism.** For all seven
references, the comparison uses:
- an idealized planar packaging model;
- a zero body clearance;
- MCP sampled at 0°;
- symbolic Y layers.

CoDe is not held to a manufacturing standard that the 2R and RCJ references
are not held to.

## Completion criteria

| Criterion | Status | Evidence |
|---|---|---|
| Formulation and reference configurations fixed | Met | Matched references tracked in `design/scripts/study/study2_morphology_adaptation/references.yaml`; study configs `design/scripts/study/*.yaml`; formulations recorded in each manifest |
| Main results reproducible | Met | Every run saves its config and source commit/patch. Study 1 equal-inputs test. Study 2: seeds 0–2. Figures are generated from saved CSVs only (inputs hashed for the packaging figures) |
| Same comparison conditions across mechanisms | Met | `shared.check_comparable` stops a run on any differing goal field except actuation. The limitations below are shared by all references |
| Final figures and tables | Candidates ready; selection below | `report/figures/study/`, `report/figures/packaging_tradeoff/`, `report/figures/exploratory/` |
| Assumptions and model limits documented | Met | This file; per-study reports |
| No new objective, mechanism family or optimizer | Held | — |

## Results selected for the paper
Results are selected for a direct contribution to GOAL.md questions 1–2.
The rest stays in the repository as supporting evidence.

1. **Accommodation against ROM (Study 1).** Pairs carried at 90/120/150° at
   one matched morphology, and the accommodation maps.
   - State the length–thickness–ROM trade-off explicitly.
   - CoDe does not take every thick interface. On the prototype geometry
     (mounting depth 3.0 mm), the thickest feasible module at 120° is:

     | Module length | Thickest at 120° |
     |---|---|
     | Full span | 5.0 mm |
     | 25 mm | 7.5 mm |
     | 20 mm (proximal) | 10 mm |

     (`prototype_hardware.md` §4.) These are limits of this prototype and
     model, not of the architecture.
2. **Servo tilt against envelope (E04 and the packaging figures).**
   - Joint-servo designs gain accommodation by reorienting the servo, at an
     envelope cost.
   - The thinnest evaluated tilts are 2R-DD 30° and belt 0°.
   - CoDe keeps the contact side clear, at a comparable envelope.
3. **Design rationale from the virtual articulation (E02, E10).** The
   virtual pivot sinks 4–6 mm toward the contact faces at 90–120°, without
   hardware there. This is an ablation plus a sufficiency counterfactual and
   a correlational replication, in 2D, at one morphology.
4. **E06 findings that change how results are read.** These go in as study
   limitations (below).

**Not selected as main results:**
- Study 2 (morphology adaptation) and its compact extension;
- Study 3 (reflected contact mechanics);
- Study 4 (joint bridge, a negative screen);
- the remaining E-studies.

They may be cited where they answer a reviewer's question. Examples: E09's
hidden transmission change motivates holding morphology fixed, and Study 2
shows the cost of not doing so.

## Study limitations (shared by all mechanisms)
- **Nominal zero clearance.** Touching counts as feasible.
  - Under a uniform clearance, two prototype contacts decide CoDe's verdict:
    the A pin against the PIP servo, and the PIP servo against the MCP servo
    (E06).
  - With the pin housings exempt, CoDe's Study 1 counts hold to 1 mm.
  - Joint-servo references lose pairs gradually.
  - The mounting policy does not apply `body_clearance_mm`. This model
    inconsistency is irrelevant at zero clearance.
- **MCP sampled at 0° only.**
  - The servo–servo check covers the MCP-extended pose alone.
  - For the prototype, the PIP and MCP servos would collide below about −9°
    of MCP extension.
  - So results hold for MCP 0° and flexion. Any claim about the full MCP
    workspace, including hyperextension, would need the check extended for
    every mechanism.
- **Idealized planar packaging.**
  - Pin housings are side-view disks; the modelled 6 mm does not include
    bearing walls.
  - Links are centre lines, and Y layers are names.
  - Bearings, shafts, retention, bracket walls and fasteners are not
    modelled.
  - One 90 mm matched morphology; 2D side view.

## Deferred to the hardware workstream
These do not invalidate the kinematic comparison (`prototype_hardware.md`):
- **A bearing vs. the servo housing.** A flanged bearing does not fit in the
  servo's plane at A. Fix it with a layer choice or a bushing.
- **Finger width.** The width budget is ≥ 27.2 mm with the modelled parts,
  against a ~25 mm target.
- **Joint construction.** Shaft and bearing support, overhang and
  retention.
- **Module attachment.** The carrier, dowels, palmar fasteners and access.
- **Tolerances.** Manufacturing tolerances and a real clearance per pair.

## Next phase
**Contact Morphology Experiment Design.** Define how contact interfaces are
varied and compared on the fixed embodiment H (GOAL.md §5.6–5.7). Hardware
realization proceeds in parallel to serve those experiments.
