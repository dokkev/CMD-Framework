# CLEAR Hand Morphology — Source of Truth

This document defines the canonical morphology design space, coordinate
conventions, fixed hardware geometry, and derived geometry for CLEAR Hand.

If implementation details disagree with this document, this document is the
intended design unless explicitly updated.

---

## 1. Design Principle

CLEAR Hand morphology is separated into three levels:

1. **Fingertip morphology**
2. **Finger linkage morphology**
3. **Hand morphology**

The optimizer should expose only independent morphology variables.

Geometry that is fixed by the actuator, CAD architecture, or mechanical
assembly is **not** a design variable.

Geometry that can be deterministically computed from independent variables is
**derived**, not independently parameterized.

Conceptually:

\[
d =
[
d_{\mathrm{fingertip}},
d_{\mathrm{tip\ length}},
d_{\mathrm{linkage}},
d_{\mathrm{hand}}
]
\]

---

# 2. Coordinate Conventions

## 2.1 Hand / Global Frame

The canonical hand frame is:

- `+X_H`: hand longitudinal direction, distal toward the fingers
- `+Y_H`: ordinary finger flexion/extension rotation-axis direction
- `+Z_H`: dorsal direction

The hand morphology origin is the **index MCP-AA joint center**.

Therefore:

\[
{}^H T_{\mathrm{index\_mcp\_aa}} = I
\]

The wrist is not the morphology origin.

---

## 2.2 Joint Frames

Joint frames describe hand morphology and kinematic architecture.

A joint frame does **not** need to align its local `+Z` axis with its physical
revolute axis.

Physical joint axes are defined separately from frame orientation.

For ordinary fingers:

- MCP-AA axis: `+Z_H`
- MCP-FE axis: `+Y_H`

Ordinary MCP-AA and MCP-FE reference frames remain parallel to the hand frame.

---

## 2.3 XM335 Actuator Local Frame

The actuator-local frame is distinct from the hand/global and joint frames.

Canonical XM335 local frame:

- origin: output shaft center
- `+X_A`: actuator body-width direction
- `+Y_A`: actuator longitudinal direction toward the rounded cap
- `+Z_A`: output shaft / revolute axis

Therefore:

\[
a_{\mathrm{shaft}}^A = (0,0,1)
\]

Actuator geometry is attached to a joint through a fixed mount transform:

\[
{}^H T_A
=
{}^H T_J
{}^J T_A
\]

The hand frame must never be redefined to match the actuator-local frame.

---

## 2.4 Fingertip Local Frame

The fingertip frame shares the hand and linkage axis convention:

- origin: proximal end of the carrier, on its centreline, at the carrier
  shoulder plane `z = 0`
- `+X_tip`: distal (longitudinal); the fingertip occupies `0 <= x <= L`
- `+Y_tip`: lateral, parallel to the flexion axis
- `+Z_tip`: dorsal; the contact pad faces `-Z_tip`

The frame is right-handed and the geometry is symmetric in `y`. With
straight-body length \(L_s\) and nose semi-axes \(a\) (lateral), \(b\)
(longitudinal), \(c\) (vertical), the distal pad surface is

\[
z = -h - c\sqrt{1 - (y/a)^2 - ((x - L_s)/b)^2}
\]

`Fingertip` queries take `(x, y)` in this frame: `half_width_at_x_mm(x)`,
`dorsal_plate_half_width_at_x_mm(x)`, `lower_surface_z_mm(x, y)`,
`upper_surface_z_mm(x, y)` and `is_inside_pad_footprint(x, y)`.

Because the axes match the linkage plane, the fingertip attaches to the distal
link with a translation and a lateral offset only:

\[
{}^H T_{\mathrm{tip}} = {}^H T_L\, {}^L T_{BC}(q)\, {}^{BC} T_{\mathrm{tip}}
\]

---

# 3. Fingertip Morphology

All five fingertips use the same **structural morphology**.

The fingertip morphology family is shared across:

- index
- middle
- ring
- little
- thumb

Shared parameters describe properties such as:

- carrier geometry
- stem geometry
- vertical pad geometry
- elliptical pad geometry
- tip shape
- internal phalanx/support geometry

The shared independent structural parameters are owned by
`FingertipFamilyParameters`. A concrete `FingertipParameters` combines that
family with one digit's total length.

---

## 3.1 Per-Digit Fingertip Length

The structural morphology is shared, but total fingertip length may vary by
digit.

Per-digit fingertip length is independently variable:

\[
L_{\mathrm{tip}}
=
[
L_I,
L_M,
L_R,
L_L,
L_T
]
\]

where:

- `L_I`: index fingertip length
- `L_M`: middle fingertip length
- `L_R`: ring fingertip length
- `L_L`: little fingertip length
- `L_T`: thumb fingertip length

All other fingertip structural parameters remain shared unless this document
is explicitly revised.

The carrier distal inset is a shared fingertip-family morphology parameter.
Its current baseline value is 5 mm; it is not a fixed mechanical dimension:

\[
d_{\mathrm{carrier\ inset}} = 5\ \mathrm{mm}
\]

For each digit with total length \(L_i\):

\[
L_{\mathrm{pad},i}=L_i, \qquad
L_{\mathrm{carrier},i}=L_i-d_{\mathrm{carrier\ inset}}
\]

Thus changing a digit's total length changes only the proximal straight
section; the tip shape and carrier penetration into the distal tip remain
identical across all five digits when the shared family parameters are held
constant. These values are materialized by `FingertipParameters` and
`Fingertip`.

---

# 4. Ordinary Finger Linkage Morphology

Index, middle, ring, and little fingers share one linkage morphology, specified
by `FingerLinkageParameters`. Its inputs are direct reference-pose translations
in the canonical XZ plane; every point has `Y = 0`. The reference origin is
`O = (0, 0)`, where the tuple components are `(X, Z)`, and input translations
have no hidden frame rotation:

| Input | Meaning |
| --- | --- |
| `op_from_o` | `O' = O + op_from_o` |
| `a_from_o` | `A = O + a_from_o` |
| `ap_from_o` | `A' = O + ap_from_o` |
| `b_from_a` | `B = A + b_from_a` |
| `bp_from_a` | `B' = A + bp_from_a` |
| `bc_length_mm` | `C = B + (bc_length_mm, 0)`; positive distal `+X` length |
| `cp_from_b` | `C' = B + cp_from_b` |

Every vector is an XZ translation in the fully extended reference coordinates,
expressed relative to the named point in its field. There are no user-specified
reference rotation angles. Runtime rotations act on these stored vectors as
changes from the reference pose.

The fully extended reference is defined only by `BC` being parallel to the
canonical `+X` axis, which holds by construction. `B` and `C` may sit at any
`Z`, and `OC` need not be parallel to `BC`. The linkage plane is the
hand-frame XZ plane, and its physical flexion axis is canonical `+Y`; `q1 = 0`
is fully extended and `q1 < 0` is flexion. `OA` and `OA'` need not be
horizontal, and neither `A'` nor `B'` is required to lie on `O-B-C`. No
coupling-anchor collinearity condition is imposed.

The public planar point type is `PointXZ(x_mm, z_mm)`. It has no Y coordinate.

The main-chain lengths `|OA|` and `|AB|`, coupling lengths `|O'B'|` and
`|A'C'|`, and all reference point positions are derived from these inputs.
Do not independently optimize the shared linkage geometry for each ordinary
finger.

---

# 5. Hand Morphology

Hand morphology determines the placement and orientation of the finger
architectures relative to the hand frame.

The index MCP-AA is fixed as the root.

---

## 5.1 Ordinary Finger Design Variables

### Index

Index MCP-AA is fixed:

\[
{}^H T_{\mathrm{index\_mcp\_aa}} = I
\]

It has no placement design variable.

### Middle

Independent planar MCP-AA placement:

\[
(x_M, y_M)
\]

### Ring

Independent planar MCP-AA placement:

\[
(x_R, y_R)
\]

### Little

Independent planar MCP-AA placement:

\[
(x_L, y_L)
\]

No independent ordinary-finger MCP-AA `Z` variable is used initially.

No ordinary-finger MCP-FE position is independently parameterized.

---

# 6. Fixed Ordinary-Finger Architecture

The MCP-AA to MCP-FE geometry is fixed by the actuator/mechanical architecture.

Canonical directed transform:

\[
{}^{MCP-AA}T_{MCP-FE}
=
T(23.5,\ 0,\ 16.75)\ \mathrm{mm}
\]

with identity reference rotation.

Therefore:

- `ΔX = 23.5 mm`
- `ΔY = 0 mm`
- `ΔZ = 16.75 mm`

For each ordinary finger `f`:

\[
{}^H T_{f,\mathrm{MCP-FE}}
=
{}^H T_{f,\mathrm{MCP-AA}}
{}^{MCP-AA}T_{MCP-FE}
\]

This transform is fixed and is not part of the morphology design vector.

## 6.1 Ordinary MCP-FE to Linkage Frame

The ordinary-finger linkage frame is aligned with MCP-FE and has zero relative
translation and identity relative rotation:

\[
{}^{MCP-FE}T_L=I
\]

Thus linkage-frame `+X`, `+Y`, and `+Z` remain aligned with the corresponding
MCP-FE/hand-frame axes.

For each ordinary finger, `HandLayout.linkage_transforms` derives its hand-frame
pose:

\[
{}^H T_L={}^H T_{MCP-FE}{}^{MCP-FE}T_L
\]

The canonical fixed transform (`ORDINARY_MCP_FE_TO_LINKAGE`) is owned by
`cmd/morphology/hand_layout/model.py`, not by the URDF serializer.

---

# 7. Thumb Morphology

The thumb is represented as a transform chain rather than independent global
XYZ joint positions.

Reference chain:

```text
Hand
  |
  +-- CMC-RO
        |
        +-- CMC-RAA
              |
              +-- MCP-FE
                    |
                    +-- IP-FE
```

## 7.1 Thumb Independent Design Variables

The independent thumb layout variables are:

- CMC-RO planar placement \((x_T,y_T)\)
- CMC-RO orientation \(\theta\)
- CMC-RO to CMC-RAA distance \(L_{RO\rightarrow RAA}\)
- MCP-FE orientation \(\phi\)

Therefore the thumb design vector is:
\[
d_{\mathrm{thumb}}
=
[
x_T,
y_T,
\theta,
L_{RO\rightarrow RAA},
\phi
]
\]

## 7.2 CMC-RO

At theta = 0, the CMC-RO reference frame is aligned with the hand frame.
The reference transform is:
\[
{}^H T_{CMC-RO}
=
T(x_T,y_T,0)R_z(\theta)
\]

`theta` is a morphology parameter, not a runtime joint configuration. The
CMC-RO physical revolute axis is local `+Y`.

## 7.3 CMC-RO to CMC-RAA

CMC-RAA is placed along the CMC-RO local +X direction.
\[
{}^{CMC-RO}T_{CMC-RAA}
=
T_x(L_{RO\rightarrow RAA})
\]

`L_RO→RAA` is an independent morphology variable. CMC-RAA is not assigned an
independent global position. Its physical revolute axis is local `+Z`.

## 7.4 CMC-RAA to MCP-FE

The CMC-RAA to MCP-FE planar translation distance is fixed:
\[
L_{RAA\rightarrow MCP-FE}
=
39.5\ \mathrm{mm}
\]

Therefore:
\[
{}^{CMC-RAA}T_{MCP-FE}
=
T_x(39.5)R_z(\phi)
\]

39.5 mm is fixed architecture geometry and is not an optimization variable.
`phi` remains an independent morphology angle. The MCP-FE physical revolute
axis is local `+Y`.

## 7.5 MCP-FE to IP-FE

The MCP-FE to IP-FE planar distance is fixed:
\[
L_{MCP-FE\rightarrow IP-FE}
=
35.5\ \mathrm{mm}
\]

Therefore:
\[
{}^{MCP-FE}T_{IP-FE}
=
T_x(35.5)
\]

The IP-FE position is derived and must not be independently parameterized.

# 8. Current Hand-Level Design Vector

The current independent hand-layout variables are:
\[
d_{\mathrm{hand}}
=
[
x_M,y_M,
x_R,y_R,
x_L,y_L,
x_T,y_T,
\theta,
L_{RO\rightarrow RAA},
\phi
]
\]

This is 11 scalar variables and excludes fingertip and linkage morphology.

# 9. Fixed Mechanical Geometry

These architecture constants are not optimizer-controlled parameters:

| Quantity | Value |
| --- | ---: |
| Ordinary MCP-AA → MCP-FE ΔX | 23.5 mm |
| Ordinary MCP-AA → MCP-FE ΔY | 0 mm |
| Ordinary MCP-AA → MCP-FE ΔZ | 16.75 mm |
| Ordinary MCP-FE → linkage frame | identity rotation, zero translation |
| Thumb CMC-RAA → MCP-FE distance | 39.5 mm |
| Thumb MCP-FE → IP-FE distance | 35.5 mm |

Each value has one canonical definition in morphology code. Do not duplicate
them across optimizer, URDF, visualization, and constraint modules.

# 10. XM335 Geometry

All actuated joints use the XM335 actuator model. Canonical geometry:

- Radius: 9.5 mm
- Body width: 19.0 mm
- Rear extent from shaft: 25.5 mm
- Total longitudinal extent: 35.0 mm
- Depth along shaft axis: 22.0 mm

The shaft center is the actuator-local origin. Local bounds are:

\[
X_A\in[-9.5,9.5],\qquad
Y_A\in[-25.5,9.5],\qquad
Z_A\in[-11,11]
\]

Actuator size is a physical packaging constraint.

# 11. Actuator Packaging

Actuator body poses are derived from joint poses:
\[
{}^H T_A
=
{}^H T_J
{}^J T_A
\]

The constraint evaluator uses transformed XM335 body geometry, not arbitrary
center-distance constraints. The current implementation uses a conservative
transformed OBB test, which may produce false positives relative to the exact
D-shaped actuator geometry. This approximation must be identified as such.
Reference-pose packaging and swept-volume/runtime collision are separate
problems; current checks concern only the reference design assembly.

# 12. Independent vs Fixed vs Derived

## Independent design variables

- **Fingertip:** one shared family morphology, including the shared
  `carrier_distal_inset_mm` (baseline 5 mm), and one total length per digit.
- **Linkage:** one shared ordinary-finger linkage morphology.
- **Hand layout:** middle, ring, and little MCP-AA `(x, y)`; thumb CMC-RO
  `(x, y)`; `theta`; CMC-RO to CMC-RAA distance; and `phi`.

## Fixed architecture

- Index MCP-AA hand root and XM335 actuator geometry.
- Ordinary MCP-AA to MCP-FE transform.
- Ordinary MCP-FE to linkage-frame transform.
- Thumb CMC-RAA to MCP-FE distance: 39.5 mm.
- Thumb MCP-FE to IP-FE distance: 35.5 mm.
- Joint-axis definitions.
- Actuator mounting transforms once CAD clocking is finalized.

## Derived quantities

- Ordinary MCP-FE and linkage-frame poses.
- Thumb CMC-RAA, MCP-FE, and IP-FE poses.
- Hand-frame actuator poses.
- Linkage coupler lengths derived from reference geometry.
- Concrete fingertip geometry for each digit.
- URDF origins/orientations and packaging geometry in the hand frame.

Derived quantities are not independent optimizer variables.

# 13. Software Ownership

- `cmd/geometry`: shared geometry primitives and rigid transforms.
- `cmd/morphology`: morphology parameters, geometry, reference transform
  chains, and linkage kinematics.
  - `pad/`: `FingertipFamilyParameters`, `FingertipParameters` and the
    `Fingertip` surface model.
  - `linkage/`: `FingerLinkageParameters`, the reference `FingerLinkage`, its
    closed-form kinematics, sampled motion and workspace diagnostics, and the
    named designs `BASELINE_LINKAGE_V0` and `SELECTED_LINKAGE_V1`.
  - `hand_layout/parameters.py` owns `HandLayoutParameters`,
    `FingertipLengths`, and the whole-hand `HandMorphologyParameters`
    aggregate.
  - `hand_layout/model.py` materializes `HandLayout` and the composed
    `HandMorphology`. `HandLayout.linkage_transforms` owns ordinary
    MCP-FE-to-linkage placement.
- `cmd/hardware`: actuator-local geometry and mechanical properties.
- `cmd/design`: consumes morphology for feasibility constraints and URDF
  serialization; it does not reconstruct morphology transforms.
- `cmd/optimization/stage0`: the ordinary-finger linkage optimization
  (see `docs/optimization/stage0.md`).
- `cmd/visualization`: visualization only.

`cmd/morphology` is the source of truth for morphology geometry, and
`cmd/hardware` is the source of truth for actuator-local geometry.

# 14. Current Morphology Hierarchy

The hand-level design is composed from a shared fingertip family, one shared
ordinary-finger linkage, and hand layout:

```text
HandMorphologyParameters
├── fingertip: FingertipFamilyParameters
│   ├── shared structural morphology
│   └── carrier_distal_inset_mm (shared; baseline 5 mm)
├── fingertip_lengths: FingertipLengths (index, middle, ring, little, thumb)
├── linkage: FingerLinkageParameters (shared by ordinary fingers)
└── layout: HandLayoutParameters
    ├── index MCP-AA: fixed root
    ├── middle/ring/little MCP-AA: x, y
    └── thumb: CMC-RO placement (x, y) and orientation theta,
        RO→RAA distance, MCP-FE orientation phi

HandMorphology
├── fingertips[5]
├── linkage
└── layout
    ├── joint transforms
    ├── actuator transforms
    └── ordinary linkage-base transforms
```

The corresponding physical arrangement is:

```text
CLEAR Hand Morphology
├── Fingertip morphology
│   ├── shared structural morphology
│   └── per-digit total lengths: index, middle, ring, little, thumb
├── Ordinary-finger linkage morphology
│   └── one shared linkage design
└── Hand layout
    ├── index MCP-AA: fixed root
    ├── middle/ring/little MCP-AA: x, y
    └── thumb CMC-RO: x, y, theta
        └── CMC-RAA: variable RO-to-RAA distance
            └── MCP-FE: fixed 39.5 mm, orientation phi
                └── IP-FE: fixed 35.5 mm
```

# 15. Open / Not Yet Canonicalized

The following must not be guessed silently:

- Exact actuator housing clocking about each revolute shaft axis.
- The fixed transform attaching the distal linkage to each fingertip.
- Future Z-direction hand-layout design variables.
- Swept-volume actuator collision constraints.
- Runtime hand joint kinematics.
- Per-digit linkage variation.
- Per-digit fingertip structural-morphology variation.

These require explicit design decisions before they become part of the
canonical morphology model.
