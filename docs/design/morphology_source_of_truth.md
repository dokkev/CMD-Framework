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

The carrier's distal inset is also shared and fixed:

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
identical across all five digits. These values are materialized by
`FingertipParameters` and `Fingertip`, not independently optimized per digit.

---

# 4. Ordinary Finger Linkage Morphology

Index, middle, ring, and little fingers use the same linkage morphology.

There is one shared:

\[
d_{\mathrm{linkage}}
\]

for all four ordinary fingers.

The linkage morphology includes the independent parameters defined by
`FingerLinkageParameters`, including:

- proximal link geometry
- middle link geometry
- distal link geometry
- coupling anchor locations

Coupler lengths and reference closure geometry that can be deterministically
computed from the fully extended configuration are derived quantities.

Do not independently optimize the same linkage geometry for each finger.

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

---

# 7. Thumb Morphology

The thumb is represented as a transform chain rather than independent global
XYZ joint positions.

Reference chain:


Hand
  |
  +-- CMC-RO
        |
        +-- CMC-RAA
              |
              +-- MCP-FE
                    |
                    +-- IP-FE
7.1 Thumb Independent Design Variables
The current thumb morphology design variables are:
CMC-RO planar placement
\[
(x_T, y_T)
\]CMC-RO orientation
\[
\theta
\]CMC-RO to CMC-RAA inter-axis distance
\[
L_{RO\rightarrow RAA}
\]MCP-FE orientation
\[
\phi
\]Therefore the thumb design vector is:
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
\]7.2 CMC-RO
At theta = 0, the CMC-RO reference frame is aligned with the hand frame.
The reference transform is:
\[
{}^H T_{CMC-RO}
=
T(x_T,y_T,0)R_z(\theta)
\]theta is a morphology parameter, not a runtime joint configuration.
The CMC-RO physical revolute axis is local +Y.
7.3 CMC-RO to CMC-RAA
CMC-RAA is placed along the CMC-RO local +X direction.
\[
{}^{CMC-RO}T_{CMC-RAA}
=
T_x(L_{RO\rightarrow RAA})
\]L_RO→RAA is an independent morphology variable.
CMC-RAA is not assigned an independent global position.
The CMC-RAA physical revolute axis is local +Z.
7.4 CMC-RAA to MCP-FE
The CMC-RAA to MCP-FE planar translation distance is fixed:
\[
L_{RAA\rightarrow MCP-FE}
=
39.5\ \mathrm{mm}
\]Therefore:
\[
{}^{CMC-RAA}T_{MCP-FE}
=
T_x(39.5)R_z(\phi)
\]39.5 mm is fixed architecture geometry and is not an optimization variable.
phi remains an independent morphology angle.
The MCP-FE physical revolute axis is local +Y.
7.5 MCP-FE to IP-FE
The MCP-FE to IP-FE planar distance is fixed:
\[
L_{MCP-FE\rightarrow IP-FE}
=
35.5\ \mathrm{mm}
\]Therefore:
\[
{}^{MCP-FE}T_{IP-FE}
=
T_x(35.5)
\]The IP-FE position is derived and must not be independently parameterized.
8. Current Hand-Level Design Vector
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
\]Therefore the current hand-layout design space has:
\[
11
\]independent scalar variables.
This count excludes fingertip and linkage morphology variables.
9. Fixed Mechanical Geometry
The following values are architecture constants and must not appear as
optimizer-controlled morphology parameters.
Quantity	Value
Ordinary MCP-AA → MCP-FE ΔX	23.5 mm
Ordinary MCP-AA → MCP-FE ΔY	0 mm
Ordinary MCP-AA → MCP-FE ΔZ	16.75 mm
Thumb CMC-RAA → MCP-FE distance	39.5 mm
Thumb MCP-FE → IP-FE distance	35.5 mm


These values should have one canonical definition in code.
Do not duplicate them across optimizer, URDF, visualization, and constraint
modules.
10. XM335 Geometry
All relevant actuated joints use the XM335 actuator model.
Canonical actuator geometry:
- radius: 9.5 mm
- body width: 19.0 mm
- rear extent from shaft: 25.5 mm
- total longitudinal extent: 35.0 mm
- depth along shaft axis: 22.0 mm
Local bounds:
\[
X_A \in [-9.5,\ 9.5]
\]\[
Y_A \in [-25.5,\ 9.5]
\]\[
Z_A \in [-11,\ 11]
\]The shaft center is the actuator-local origin.
Actuator size is a hard physical packaging constraint.
11. Actuator Packaging
Hand morphology must satisfy actuator packaging constraints.
Actuator body positions are derived from joint poses:
\[
{}^H T_A
=
{}^H T_J
{}^J T_A
\]The constraint evaluator must use transformed XM335 body geometry.
Do not use arbitrary center-distance constraints when actuator geometry is
available.
The current implementation may use a conservative transformed OBB test.
A conservative OBB collision test may produce false positives relative to the
exact D-shaped actuator geometry, but it must be identified explicitly as an
approximation.
Reference-pose packaging and swept-volume/runtime collision are separate
problems.
Current hand morphology constraints concern the reference design assembly.
12. Independent vs Fixed vs Derived
Independent design variables
Fingertip
- shared fingertip structural morphology parameters
- one total fingertip length per digit
Linkage
- one shared ordinary-finger linkage morphology
Hand
- middle MCP-AA (x, y)
- ring MCP-AA (x, y)
- little MCP-AA (x, y)
- thumb CMC-RO (x, y)
- thumb theta
- thumb CMC-RO → CMC-RAA distance
- thumb phi
Fixed architecture
- index MCP-AA hand root
- XM335 actuator geometry
- ordinary MCP-AA → MCP-FE transform
- thumb CMC-RAA → MCP-FE distance = 39.5 mm
- thumb MCP-FE → IP-FE distance = 35.5 mm
- joint-axis definitions
- actuator mounting transforms once CAD clocking is finalized
Derived quantities
Examples include:
- all ordinary MCP-FE poses
- thumb CMC-RAA pose
- thumb MCP-FE pose
- thumb IP-FE pose
- all actuator world/hand transforms
- linkage coupler lengths derived from the reference geometry
- final fingertip geometry
- URDF origins and orientations
- packaging geometry in the hand frame
Derived quantities must not be duplicated as independent optimizer variables.
13. Software Ownership
Canonical ownership is:
cmd/geometry/
    shared geometry and rigid-transform primitives

cmd/morphology/
    morphology parameters
    morphology geometry
    reference transform chains
    linkage kinematics
    `hand_parameter.py` owns `HandLayoutParameters`, `FingertipLengths`, and
    the whole-hand `HandMorphologyParameters` aggregate
    `hand.py` materializes `HandLayout` and the composed `HandMorphology`

cmd/hardware/
    actuator-local hardware geometry and mechanical properties

cmd/design/
    morphology feasibility constraints
    URDF / design serialization

cmd/visualization/
    visualization only
cmd/morphology is the source of truth for morphology geometry.
cmd/hardware is the source of truth for actuator-local geometry.
cmd/design must consume these models rather than reconstructing geometry
independently.
14. Current Morphology Hierarchy
CLEAR Hand Morphology
|
+-- Fingertip Morphology
|   |
|   +-- shared structural morphology
|   +-- index length
|   +-- middle length
|   +-- ring length
|   +-- little length
|   +-- thumb length
|
+-- Ordinary Finger Linkage Morphology
|   |
|   +-- one shared linkage design
|
+-- Hand Morphology
    |
    +-- index MCP-AA
    |     fixed root
    |
    +-- middle MCP-AA
    |     x, y
    |
    +-- ring MCP-AA
    |     x, y
    |
    +-- little MCP-AA
    |     x, y
    |
    +-- thumb CMC-RO
          x, y
          theta
          |
          +-- CMC-RAA
                variable RO->RAA distance
                |
                +-- MCP-FE
                      fixed 39.5 mm
                      phi
                      |
                      +-- IP-FE
                            fixed 35.5 mm
15. Open / Not Yet Canonicalized
The following should not be guessed silently:
- exact actuator housing clocking about each revolute shaft axis
- any future Z-direction hand-layout design variables
- swept-volume actuator collision constraints
- runtime hand joint kinematics
- per-digit linkage variation
- per-digit fingertip structural morphology variation
These require an explicit design decision before becoming part of the canonical
morphology model.
