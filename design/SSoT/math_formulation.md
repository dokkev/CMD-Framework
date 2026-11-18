# CoDe Hand — Canonical 2D Finger Morphology Formulation (SSoT)

> **Status & Scope.** This document is the authoritative **Mathematical Source of Truth (SSoT)** for the 2D planar kinematic, packaging, collision, and contact-coverage formulation of 2-DoF robotic fingers in `design/`. It unifies `CoDe` (crossed four-bar), `2R` (joint-direct-drive revolute), `2R-remote` (remote belt/parallel-driven revolute), and `RCJ` (symmetric 1:1 rolling-contact joint) in a single $SE(2)$ geometric language while separating canonical morphology from study-specific requirements, mounting policies, and numerical optimization encodings.
>
> Out-of-plane 3D CAD details (bracket wall thickness, fasteners, bearing flanges, cable routing, and 3D hand-level $SE(3)$ digit placement in `design/core/morphology/code_hand/`) lie outside the scope of this 2D planar model except where explicit coordinate-chart equivalences are stated.

---

## 1. Fundamental Definition & Five-Layer Architecture

### 1.1 Canonical Finger Morphology

For a discrete mechanism architecture
$$
m \in \mathcal{A} = \{\mathrm{CoDe},\; \mathrm{2R},\; \mathrm{2R\text{-}remote},\; \mathrm{RCJ}\},
$$
a finger morphology $M$ is a structured 4-tuple of geometric parameter groups:
$$
\boxed{
M = \left(
\mathbf{p}_{\mathrm{mech}},\,
\mathbf{p}_{\mathrm{act}},\,
\mathbf{p}_{\mathrm{struct}},\,
\mathbf{p}_{\mathrm{contact}}
\right) \in \mathcal{M}(m)
}
$$
where $\mathcal{M}(m)$ is the **admissible morphology space** of architecture $m$.

$M$ is a composite of geometric objects (frames in $SE(2)$, compact planar footprints in $\mathbb{R}^2$, and positive metric dimensions), **not** an unstructured flat vector in $\mathbb{R}^n$.

| Group | Symbol | Representation Space | Physical Meaning |
|---|---|---|---|
| **Mechanism** | $\mathbf{p}_{\mathrm{mech}}$ | $\mathcal{P}_{\mathrm{mech}}(m)$ | Architecture-specific joint / linkage geometry |
| **Actuation** | $\mathbf{p}_{\mathrm{act}}$ | $\big(\mathcal{G} \times SE(2)\big)^2 \times \mathcal{P}_{\mathrm{trans}}(m)$ | 2D housing footprints $G_i \subseteq \mathbb{R}^2$, poses $T_i \in SE(2)$, and auxiliary transmission geometry |
| **Structure** | $\mathbf{p}_{\mathrm{struct}}$ | $\mathcal{P}_{\mathrm{struct}} \subseteq SE(2)^3$ | Segment boundary reference frames $(T_{p,\mathrm{end}}^{\{P\}},\, T_{d,\mathrm{start}}^{\{D\}},\, T_{d,\mathrm{end}}^{\{D\}})$ |
| **Contact** | $\mathbf{p}_{\mathrm{contact}}$ | $\big(\mathbb{R}_{>0}^2 \times \mathcal{P}_{c}\big)_{i \in \{p, d\}}$ | Contact dimensions $(\ell_i, h_i)$ and body-referenced poses $T_{c,i}^{\{b_i\}} \in SE(2)$ |

> **Idealized upper-bound ablations.** Declared upper-bound baselines (`revolute_idealized`, `rcj_idealized`) are represented within the identical canonical tuple by setting the distal actuator footprint to the empty set ($G_2 = \varnothing$) while keeping $\mathbf{p}_{\mathrm{mech}}$ unchanged.

### 1.2 Five Architectural Layers of the SSoT

The framework strictly separates **what a finger is** ($\mathcal{M}(m)$: Layers 1–2) from **what follows deterministically from it** (Layer 3), **what a specific study or hardware build asks of it** (Layer 4), and **how numerical code represents and searches it** (Layer 5):

| Layer | Name | Responsibility | Examples |
|---|---|---|---|
| **Layer 1** | **Canonical Morphology** | Geometric primitives and coordinate charts per group | $m, \mathbf{p}_{\mathrm{mech}}, \mathbf{p}_{\mathrm{act}}, \mathbf{p}_{\mathrm{struct}}, \mathbf{p}_{\mathrm{contact}}$ |
| **Layer 2** | **Parameter Dependencies** | Structural well-formedness and cross-group physical compatibility defining $\mathcal{M}(m)$ | Loop closure at $\varphi=0$, shaft–pivot alignment $\operatorname{pos}(T_2^{\{P\}}) = \operatorname{pos}(T_{A'}^{\{P\}})$, structural support containment |
| **Layer 3** | **Derived Properties** | Deterministic geometric and kinematic maps on $\mathcal{M}(m)$ | Common geometry $\mathbf{L}(M)$, forward kinematics $T_{PD}(\varphi)$, ICR $\mathbf{r}_{\mathrm{ICR}}(\varphi)$, transmission ratio $k(M,\varphi)$, clearances, accessible contact sets $\mathcal{U}_i(M,\varphi;\eta_L)$ |
| **Layer 4** | **Design Requirements & Policies** | Study- or prototype-specific constraints, design policies, and objectives | Required ROM $\varphi_{\min}^{\mathrm{des}}$, size bounds $\mathbf{L}_{\min} \le \mathbf{L}(M) \le \mathbf{L}_{\max}$, nonpenetration, context profile $\eta_L$, outer-end pad pinning, `shallowest_supported` mounting policy, $z_A = z_{B,0} = 0$ restriction |
| **Layer 5** | **Implementation** | Computational classes, flat search vectors, sampling grids, and solvers | `FingerMorphology`, `Finger`, `VARIABLE_NAMES`, `servo_step_deg`, `prototyping_resolution_mm`, NSGA-II / SMS-EMOA / AGE-MOEA |

A morphology $M \in \mathcal{M}(m)$ that fails a $120^\circ$ ROM requirement or incurs a servo–pad overlap in Layer 4 is **infeasible for that optimization problem**, not mathematically undefined.

### 1.3 Prototype Formulation vs. Study Formulation

Two strictly separated Layer 4 formulations share Layers 1–3 (`README.md`, *"Two optimizations: prototype and study"*):

| Attribute | Prototype Formulation (`profile: prototype`) | Study Formulation (`profile: study`) |
|---|---|---|
| **Scientific role** | Physical hardware realization of the CoDe finger (`scripts/optimize/linkage/config.yaml` and extensions) | Controlled cross-architecture scientific comparison across $m \in \mathcal{A}$ (`scripts/study/baseline.yaml`, `base.yaml`, Studies 1–4) |
| **Envelope & pivot sizing** | Tracks physical build hardware (e.g., 8 mm pivot housings in `config_pivot8.yaml`) | Frozen shared benchmark envelopes (`A: 6, A': 10, B: 6, B': 6 mm`) |
| **Pad mounting depth $d_i$** | Searched (`mount_policy: searched`, optional `align_mount_offsets`) | Policy-determined (`mount_policy: shallowest_supported`, no spacers) |
| **Default `include_linkage_occlusion` ($\eta_L$)** | **`True`** ($\eta_L = 1$): crossing bars occlude contact faces | **`False`** ($\eta_L = 0$): pure geometric fold baseline (sensitivity runs override explicitly) |

### 1.4 Representation Space vs. Minimal Design Space

Every pose in Layer 1 is written in $SE(2)$ so all architectures compose under one coordinate-free algebra. However, each constrained frame varies only over an admissible submanifold parameterized by minimal coordinates:
$$
\iota_k : \mathcal{Q}_k \subseteq \mathbb{R}^{d_k} \hookrightarrow SE(2), \qquad d_k = \dim \mathcal{Q}_k \le 3.
$$
Physical dependencies are satisfied **by construction** through $\iota_k$ rather than redundant equality constraints in the optimizer.

---

## 2. Reference Frames & $SE(2)$ Conventions

### 2.1 Planar Coordinates and $SE(2)$ Operators

All planar geometry is expressed in coordinates $\mathbf{r} = [x, z]^\top \in \mathbb{R}^2$ (in millimetres):
- $+X$: **distal** (along the extended finger from MCP base toward fingertip).
- $+Z$: **dorsal** (toward the back of the finger); hence $-Z$ is **palmar** (outward from the contact surface).

An element $T = (\mathbf{r}, \alpha) \in SE(2)$ acts on homogeneous planar points $[x, z, 1]^\top$ via:
$$
T = \begin{bmatrix} R(\alpha) & \mathbf{r} \\ \mathbf{0}^\top & 1 \end{bmatrix}, \qquad
R(\alpha) = \begin{bmatrix} \cos\alpha & -\sin\alpha \\ \sin\alpha & \cos\alpha \end{bmatrix} \in SO(2),
$$
where $\alpha \in (-\pi, \pi]$ is active counter-clockwise (CCW) rotation from $+X$ toward $+Z$. We write $\operatorname{pos}(T) = \mathbf{r}$, $\operatorname{ori}(T) = \alpha$, $\operatorname{Trans}(x, z) = ([x, z]^\top, 0)$, $\operatorname{Trans}_x(s) = \operatorname{Trans}(s, 0)$, $\operatorname{Trans}_z(w) = \operatorname{Trans}(0, w)$, and $\operatorname{Rot}(\alpha) = (\mathbf{0}, \alpha)$.

### 2.2 Palmar Flexion Sign Convention

Joint flexion turns $+X$ toward the palmar direction $-Z$, which is clockwise in the $(x, z)$ plane. Any flexion angle $\vartheta \ge 0$ (MCP flexion $q_{\mathrm{MCP}}$ or relative distal flexion $\varphi$) enters $SE(2)$ via:
$$
R_{\mathrm{flex}}(\vartheta) \triangleq R(-\vartheta) = \begin{bmatrix} \cos\vartheta & \sin\vartheta \\ -\sin\vartheta & \cos\vartheta \end{bmatrix}, \qquad
R_{\mathrm{flex}}(\vartheta)\begin{bmatrix} 1 \\ 0 \end{bmatrix} = \begin{bmatrix} \cos\vartheta \\ -\sin\vartheta \end{bmatrix}.
$$

### 2.3 Rigid-Body Frame Hierarchy $\{W\} \to \{P\} \to \{D\}$

Three rigid-body reference frames govern all components of $M$:

1. **World / Palm MCP Frame $\{W\}$**: Fixed on the palm at the MCP flexion axis $O = [0, 0]^\top$, with $+X_W$ distal and $+Z_W$ dorsal.
2. **Proximal Body Frame $\{P\}$**: Rigidly attached to the proximal phalanx at origin $O$, with $T_{WP}(q_{\mathrm{MCP}}) = \operatorname{Rot}(-q_{\mathrm{MCP}})$. At the morphological reference pose ($q_{\mathrm{MCP}} = 0$), $T_{WP}(0) = I$.
3. **Distal Body Frame $\{D\}$**: Rigidly attached to the distal phalanx. Its pose in $\{P\}$ at relative distal flexion $\varphi$ ($\varphi = 0$ extended, $+X_D \parallel +X_P$) is:
   $$
   T_{PD}(\varphi; \mathbf{p}_{\mathrm{mech}}) = \operatorname{Trans}\!\big(\mathbf{r}_D(\varphi; \mathbf{p}_{\mathrm{mech}})\big)\,\operatorname{Rot}(-\varphi), \qquad
   T_{PD}(0; \mathbf{p}_{\mathrm{mech}}) = \operatorname{Trans}(\mathbf{r}_{D,0}),
   $$
   where $\mathbf{r}_{D,0} \triangleq \mathbf{r}_D(0; \mathbf{p}_{\mathrm{mech}}) \in \mathbb{R}^2$ is the extended-pose origin of $\{D\}$ in $\{P\}$.

Every local transform $T^{\{b\}}$ attached to rigid body $b \in \{W, P, D\}$ is invariant in $\{b\}$ and maps to $\{W\}$ via:
$$
T^{\{W\}}(q_{\mathrm{MCP}}, \varphi) =
\begin{cases}
T^{\{W\}}, & b = W, \\[3pt]
T_{WP}(q_{\mathrm{MCP}})\, T^{\{P\}}, & b = P, \\[3pt]
T_{WP}(q_{\mathrm{MCP}})\, T_{PD}(\varphi; \mathbf{p}_{\mathrm{mech}})\, T^{\{D\}}, & b = D.
\end{cases}
$$

---

## 3. Canonical Parameterization (Layer 1)

### 3.1 Mechanism Group $\mathbf{p}_{\mathrm{mech}} \in \mathcal{P}_{\mathrm{mech}}(m)$

Circular revolute pins carry zero rotational orientation in 2D ($\operatorname{ori}(T_{\mathrm{pin}}^{\{\mathrm{parent}\}}) = 0$).

#### (a) `CoDe` — Crossed Four-Bar Linkage ($m = \mathrm{CoDe}$)

Four revolute pivots $(A, A', B, B')$ form two rigid bars: driver $A\text{–}B'$ and coupler $A'\text{–}B$. Pivots $A, A'$ belong to $\{P\}$; pivots $B, B'$ belong to $\{D\}$ with $\{D\}$ anchored at $B$ ($T_B^{\{D\}} = I$):
$$
\boxed{
\mathbf{p}_{\mathrm{mech}}^{\mathrm{CoDe}}
= \big(T_A^{\{P\}},\, T_{A'}^{\{P\}},\, T_B^{\{P\}}(0),\, T_{B'}^{\{D\}}\big) \in \mathcal{P}_{\mathrm{mech}}(\mathrm{CoDe}) \subseteq SE(2)^4
}
$$
with minimal 8D coordinate chart ($\dim \mathcal{P}_{\mathrm{mech}}(\mathrm{CoDe}) = 8$):
$$
\mathbf{q}_{\mathrm{mech}}^{\mathrm{CoDe}} = (x_A, z_A, x_{A'}, z_{A'}, x_{B,0}, z_{B,0}, \Delta x_{B'}, \Delta z_{B'}) \in \mathbb{R}^8,
$$
where $T_A^{\{P\}} = \operatorname{Trans}(x_A, z_A)$, $T_{A'}^{\{P\}} = \operatorname{Trans}(x_{A'}, z_{A'})$, $T_B^{\{P\}}(0) = \operatorname{Trans}(x_{B,0}, z_{B,0})$, and $T_{B'}^{\{D\}} = \operatorname{Trans}(\Delta x_{B'}, \Delta z_{B'})$.

- **Extended-pose `{P}` chart:** $(x_{B',0}, z_{B',0}) = (x_{B,0} + \Delta x_{B'},\, z_{B,0} + \Delta z_{B'})$ gives $T_{B'}^{\{P\}}(0) = T_{PD}(0)\,T_{B'}^{\{D\}}$.
- **Intrinsic bar-length chart:** Replaces $(x_{B,0}, z_{B,0})$ with machined bar lengths
  $$
  \ell_{AB'} = \|\mathbf{r}_{B',0} - \mathbf{r}_A\|_2, \qquad
  \ell_{A'B} = \|\mathbf{r}_{B,0} - \mathbf{r}_{A'}\|_2,
  $$
  plus the discrete assembly branch $\sigma_{\mathrm{branch}} \in \{-1, +1\}$, recovering $\mathbf{r}_{B,0}$ as the circle intersection $\mathcal{C}(\mathbf{r}_{A'}, \ell_{A'B}) \cap \mathcal{C}(\mathbf{r}_A - \Delta\mathbf{r}_{B'}, \ell_{AB'})$.
- **Axis-aligned design subspace (Layer 4):** Setting $z_A = 0$ and $z_{B,0} = 0$ restricts $\mathcal{P}_{\mathrm{mech}}(\mathrm{CoDe})$ to a 6D subspace.

#### (b) `2R` and `2R-remote` — Single Revolute Joint ($m \in \{\mathrm{2R}, \mathrm{2R\text{-}remote}\}$)

Both architectures share identical single-hinge articulation at $A$ on $+X_P$, with $\{D\}$ anchored at $A$ ($\mathbf{r}_{D,0} = [x_A, 0]^\top$, $\dim \mathcal{P}_{\mathrm{mech}} = 1$):
$$
\boxed{
\mathbf{p}_{\mathrm{mech}}^{\mathrm{2R}} = \mathbf{p}_{\mathrm{mech}}^{\mathrm{2R\text{-}remote}}
= \big(T_A^{\{P\}}\big), \qquad T_A^{\{P\}}(x_A) = \operatorname{Trans}_x(x_A),\quad x_A > 0
}
$$

#### (c) `RCJ` — Symmetric 1:1 No-Slip Rolling Contact Joint ($m = \mathrm{RCJ}$)

Two circular rollers of equal radius $r > 0$ roll without slip: proximal roller fixed to $\{P\}$ at $A = [x_A, 0]^\top$, distal roller fixed to $\{D\}$ at $B$ ($T_B^{\{D\}} = I$), joined by carrier link $A\text{–}B$ of length $2r$ ($\dim \mathcal{P}_{\mathrm{mech}}(\mathrm{RCJ}) = 2$):
$$
\boxed{
\mathbf{p}_{\mathrm{mech}}^{\mathrm{RCJ}}
= \big(T_A^{\{P\}},\, r\big), \qquad T_A^{\{P\}}(x_A) = \operatorname{Trans}_x(x_A),\quad x_A > 0,\quad r > 0
}
$$

### 3.2 Actuation Group $\mathbf{p}_{\mathrm{act}}$

For a 2-DoF finger ($i=1$: MCP in $\{W\}$; $i=2$: PIP/distal drive in $\{P\}$):
$$
\boxed{
\mathbf{p}_{\mathrm{act}} = \big((G_1, T_1^{\{W\}}),\, (G_2, T_2^{\{P\}});\; \mathbf{p}_{\mathrm{trans}}(m)\big)
}
$$
In local actuator frame $\{A_i\}$, the origin is the output shaft, $+X_{A_i}$ points toward the semicircular front cap, and $-X_{A_i}$ points along the rear housing:
$$
G_{\mathrm{D}}(r_s, e_{\mathrm{rear}}) =
\big\{(u, v) \in \mathbb{R}^2 : u \ge 0,\; u^2 + v^2 \le r_s^2\big\}
\,\cup\,
\big([-e_{\mathrm{rear}}, 0] \times [-r_s, r_s]\big).
$$
A dorsal rear-body tilt $\psi_i \in (-\pi, \pi]$ measured from $-X_{\mathrm{parent}}$ toward $+Z_{\mathrm{parent}}$ places the rear axis along $[-\cos\psi_i, \sin\psi_i]^\top$ and $+X_{A_i}$ along $[\cos\psi_i, -\sin\psi_i]^\top$, yielding $\operatorname{ori}(T_i) = -\psi_i$:
$$
T_i = \operatorname{Trans}(\mathbf{r}_{\mathrm{shaft},i})\,\operatorname{Rot}(-\psi_i).
$$
For $m = \mathrm{2R\text{-}remote}$, $\mathbf{p}_{\mathrm{trans}}(\mathrm{2R\text{-}remote}) = (r_{\mathrm{pulley}})$ with $r_{\mathrm{pulley}} > 0$ (1:1 belt connecting equal pulleys of radius $r_{\mathrm{pulley}}$ at $\mathbf{r}_{A'}$ and $\mathbf{r}_A$); for all other $m$, $\mathbf{p}_{\mathrm{trans}}(m) = ()$.

### 3.3 Structure Group $\mathbf{p}_{\mathrm{struct}} \in \mathcal{P}_{\mathrm{struct}}$

With proximal start fixed at MCP origin $O$ ($T_{p,\mathrm{start}}^{\{P\}} \triangleq I$), the three structural boundary frames ($A''$ on $\{P\}$, $B''$ and fingertip $C$ on $\{D\}$) translate along their parent body's $+X$ reference axis ($\dim \mathcal{P}_{\mathrm{struct}} = 3$):
$$
\boxed{
\mathbf{p}_{\mathrm{struct}} =
\big(\operatorname{Trans}_x(s_{p,\mathrm{end}}),\, \operatorname{Trans}_x(s_{d,\mathrm{start}}),\, \operatorname{Trans}_x(s_{d,\mathrm{end}})\big) \in \mathcal{P}_{\mathrm{struct}} \subseteq SE(2)^3
}
$$
with $s_{p,\mathrm{end}} > 0$, $s_{d,\mathrm{end}} > 0$, and $s_{d,\mathrm{start}} < s_{d,\mathrm{end}}$ ($s_{d,\mathrm{start}} < 0$ represents proximal overhang behind $\{D\}$'s origin).

### 3.4 Contact Group $\mathbf{p}_{\mathrm{contact}}$

Each segment $i \in \{p, d\}$ ($b_p = P, b_d = D$) carries a rectangular contact module of length $\ell_i > 0$ and thickness $h_i > 0$:
$$
\boxed{
\mathbf{p}_{\mathrm{contact}} = \big((\ell_p, h_p, T_{c,p}^{\{P\}}),\, (\ell_d, h_d, T_{c,d}^{\{D\}})\big), \qquad
T_{c,i}^{\{b_i\}}(s_{c,i}, d_i) = \operatorname{Trans}(s_{c,i},\, -d_i)
}
$$
with axial start $s_{c,i} \in \mathbb{R}$ and palmar mounting offset $d_i \ge 0$ ($\dim \mathcal{P}_{c,i} = 2$). In local contact frame $\{C_i\}$, both pads share the identical canonical block:
$$
B_{\mathrm{pad}}(\ell_i, h_i) = [0, \ell_i] \times [-h_i, 0],
$$
whose top mounting face is $[0, \ell_i] \times \{0\}$ ($z_{\mathrm{local}} = -d_i$ in $\{b_i\}$) and palmar contact face is $[0, \ell_i] \times \{-h_i\}$ ($z_{\mathrm{local}} = -(d_i + h_i)$ in $\{b_i\}$).

### 3.5 Architecture & DOF Summary Table

| Group | Component / Architecture | Parent | Canonical Representation | Minimal Coordinates | Independent DOFs |
|---|---|---|---|---|---:|
| **Mechanism** | `CoDe` | $\{P\}, \{D\}$ | $(T_A^{\{P\}}, T_{A'}^{\{P\}}, T_B^{\{P\}}(0), T_{B'}^{\{D\}})$ | $(x_A, z_A, x_{A'}, z_{A'}, x_{B,0}, z_{B,0}, \Delta x_{B'}, \Delta z_{B'})$ | $8$ |
| | `2R`, `2R-remote` | $\{P\}$ | $(T_A^{\{P\}})$ | $(x_A)$ | $1$ |
| | `RCJ` | $\{P\}$ | $(T_A^{\{P\}}, r)$ | $(x_A, r)$ | $2$ |
| **Actuation** | MCP ($i=1$), all $m$ | $\{W\}$ | $(G_1, T_1^{\{W\}})$ | $(\psi_{\mathrm{MCP}})$ ($\operatorname{pos}(T_1) \equiv \mathbf{0}$) | $1$ |
| | PIP ($i=2$), `CoDe`/`2R`/`RCJ` | $\{P\}$ | $(G_2, T_2^{\{P\}})$ | $(\psi_{\mathrm{tilt}})$ ($\operatorname{pos}(T_2)$ derived from $\mathbf{p}_{\mathrm{mech}}$) | $1$ |
| | PIP ($i=2$), `2R-remote` | $\{P\}$ | $(G_2, T_2^{\{P\}}; r_{\mathrm{pulley}})$ | $(x_{A'}, z_{A'}, \psi_{\mathrm{tilt}}, r_{\mathrm{pulley}})$ | $3$ pose $+ 1$ pulley |
| **Structure** | $(A'', B'', C)$, all $m$ | $\{P\}, \{D\}$ | $(T_{p,\mathrm{end}}^{\{P\}}, T_{d,\mathrm{start}}^{\{D\}}, T_{d,\mathrm{end}}^{\{D\}})$ | $(s_{p,\mathrm{end}}, s_{d,\mathrm{start}}, s_{d,\mathrm{end}})$ | $3$ |
| **Contact** | Proximal & Distal ($i \in \{p,d\}$) | $\{P\}, \{D\}$ | $(\ell_i, h_i, T_{c,i}^{\{b_i\}})$ | $(\ell_i, h_i, s_{c,i}, d_i)$ per segment | $4 + 4 = 8$ |

---

## 4. Parameter Dependencies & Admissible Space $\mathcal{M}(m)$ (Layer 2)

### 4.1 Classification of Parameter Relationships

Every relationship among parameters is classified into one of five mathematical roles so physical definitions are never conflated with solver constraints:

| Relationship | Category | Layer | Mathematical Expression / Treatment |
|---|---|---|---|
| Architecture $m \mapsto \mathcal{P}_{\mathrm{mech}}(m)$ | **Structural definition** | Layer 1/2 | Selects mechanism manifold (`CoDe` 8D, `2R`/`2R-remote` 1D, `RCJ` 2D) |
| Non-degenerate pivots & reference closure | **Admissible subspace restriction** | Layer 2 | `CoDe`: $\mathbf{r}_A \neq \mathbf{r}_{A'}$, $\Delta\mathbf{r}_{B'} \neq \mathbf{0}$, $\ell_{AB'}, \ell_{A'B} > 0$, non-collinear closure at $\varphi=0$; `2R`: $x_A > 0$; `RCJ`: $x_A, r > 0$ |
| MCP shaft at $O$; PIP shaft alignment | **Derived geometry** (by construction) | Layer 2 | $\operatorname{pos}(T_1^{\{W\}}) = \mathbf{0}$; $\operatorname{pos}(T_2^{\{P\}}) = \mathbf{r}_{A'}$ (`CoDe`), $\mathbf{r}_A$ (`2R`, `RCJ`); for `2R-remote`, $\mathbf{r}_{A'} \neq \mathbf{r}_A$ and $r_{\mathrm{pulley}} > 0$ |
| Axial structural ordering | **Admissible subspace restriction** | Layer 2 | $s_{p,\mathrm{end}} > 0$, $s_{d,\mathrm{end}} > 0$, $s_{d,\mathrm{start}} < s_{d,\mathrm{end}}$ |
| Contact axial support containment | **Admissible subspace restriction** | Layer 2 | $[s_{c,p}, s_{c,p} + \ell_p] \subseteq [0, s_{p,\mathrm{end}}]$ and $[s_{c,d}, s_{c,d} + \ell_d] \subseteq [s_{d,\mathrm{start}}, s_{d,\mathrm{end}}]$ |
| Axis-aligned four-bar ($z_A = z_{B,0} = 0$) | **Study requirement** (subspace policy) | Layer 4 | Restricts `CoDe` search space to 6D without altering $\mathcal{M}(\mathrm{CoDe})$ |
| Outer-end pad pinning & `shallowest_supported` | **Study requirement** (design policy) | Layer 4 | Pins $s_{c,p} = s_{p,\mathrm{outer}}$, $s_{c,d} = s_{d,\mathrm{end}} - o_d - \ell_d$, and $d_i = \pi_{\mathrm{mount},i}(M)$ |
| ROM, collision clearances, transmission angle, size bounds | **Optimization constraint** ($g_j(M) \le 0$) | Layer 4/5 | Evaluated on admissible $M \in \mathcal{M}(m)$ during optimization (`Mechanism.evaluate`) |

### 4.2 Formal Definition of the Admissible Morphology Space $\mathcal{M}(m)$

Combining Layers 1 and 2, the admissible morphology space for each architecture $m \in \mathcal{A}$ is:
$$
\boxed{
\mathcal{M}(m) = \left\{
M = (\mathbf{p}_{\mathrm{mech}}, \mathbf{p}_{\mathrm{act}}, \mathbf{p}_{\mathrm{struct}}, \mathbf{p}_{\mathrm{contact}})
\;\middle|\;
\begin{aligned}
&\mathbf{p}_{\mathrm{mech}} \in \mathcal{P}_{\mathrm{mech}}^{\circ}(m),\quad
\mathbf{p}_{\mathrm{struct}} \in \mathcal{P}_{\mathrm{struct}},\\
&\mathcal{D}_{\mathrm{act}}(m, \mathbf{p}_{\mathrm{mech}}, \mathbf{p}_{\mathrm{act}}),\\
&\mathcal{D}_{\mathrm{sup}}(\mathbf{p}_{\mathrm{struct}}, \mathbf{p}_{\mathrm{contact}})
\end{aligned}
\right\}
}
$$
where:
1. **Non-degenerate mechanism space $\mathcal{P}_{\mathrm{mech}}^{\circ}(m)$:**
   - $\mathcal{P}_{\mathrm{mech}}^{\circ}(\mathrm{CoDe}) = \big\{\mathbf{q}_{\mathrm{mech}}^{\mathrm{CoDe}} \in \mathbb{R}^8 : \mathbf{r}_A \neq \mathbf{r}_{A'},\; \Delta\mathbf{r}_{B'} \neq \mathbf{0},\; |\ell_{A'B} - \ell_{AB'}| < \|\mathbf{r}_A - \Delta\mathbf{r}_{B'} - \mathbf{r}_{A'}\|_2 < \ell_{A'B} + \ell_{AB'}\big\}$.
   - $\mathcal{P}_{\mathrm{mech}}^{\circ}(\mathrm{2R}) = \mathcal{P}_{\mathrm{mech}}^{\circ}(\mathrm{2R\text{-}remote}) = \{\operatorname{Trans}_x(x_A) : x_A > 0\}$.
   - $\mathcal{P}_{\mathrm{mech}}^{\circ}(\mathrm{RCJ}) = \{(\operatorname{Trans}_x(x_A), r) : x_A > 0,\; r > 0\}$.
2. **Mechanism–Actuation compatibility $\mathcal{D}_{\mathrm{act}}(m, \mathbf{p}_{\mathrm{mech}}, \mathbf{p}_{\mathrm{act}})$:**
   $$
   \operatorname{pos}(T_1^{\{W\}}) = \mathbf{0} \;\land\;
   \begin{cases}
   \operatorname{pos}(T_2^{\{P\}}) = \operatorname{pos}(T_{A'}^{\{P\}}), & m = \mathrm{CoDe}, \\[3pt]
   \operatorname{pos}(T_2^{\{P\}}) = \operatorname{pos}(T_A^{\{P\}}), & m \in \{\mathrm{2R}, \mathrm{RCJ}\}, \\[3pt]
   \operatorname{pos}(T_2^{\{P\}}) \neq \operatorname{pos}(T_A^{\{P\}}) \;\land\; r_{\mathrm{pulley}} > 0, & m = \mathrm{2R\text{-}remote}.
   \end{cases}
   $$
3. **Structure–Contact axial support containment $\mathcal{D}_{\mathrm{sup}}(\mathbf{p}_{\mathrm{struct}}, \mathbf{p}_{\mathrm{contact}})$:**
   $$
   0 \le s_{c,p} < s_{c,p} + \ell_p \le s_{p,\mathrm{end}}, \qquad
   s_{d,\mathrm{start}} \le s_{c,d} < s_{c,d} + \ell_d \le s_{d,\mathrm{end}}, \qquad
   d_p \ge 0,\quad d_d \ge 0.
   $$
   *(Note: In Layer 4/5 module sweeps where a fixed skeleton is tested against oversized candidate modules, `FingerMorphology.validate(require_support_containment=False)` separates kinematic well-formedness from graded support-violation reporting `proximal_support_mm`, `distal_support_mm`.)*

---

## 5. Derived Properties (Layer 3)

Every quantity below is a deterministic function of $M \in \mathcal{M}(m)$ and contains **zero** additional canonical parameters.

### 5.1 Common Finger Geometry $\mathbf{L}(M)$

Evaluated at the extended reference pose ($q_{\mathrm{MCP}} = 0, \varphi = 0$):
$$
\boxed{
\mathbf{L}(M) =
\begin{bmatrix} L_f(M) \\ L_p(M) \\ L_d(M) \end{bmatrix}
=
\begin{bmatrix}
\|\mathbf{r}_C(0) - \mathbf{r}_O\|_2 \\
\|\mathbf{r}_{A''} - \mathbf{r}_O\|_2 \\
\|\mathbf{r}_C(0) - \mathbf{r}_{B''}(0)\|_2
\end{bmatrix}
=
\begin{bmatrix}
\big\|\mathbf{r}_{D,0} + [s_{d,\mathrm{end}},\, 0]^\top\big\|_2 \\
s_{p,\mathrm{end}} \\
s_{d,\mathrm{end}} - s_{d,\mathrm{start}}
\end{bmatrix}
}
$$
When $\mathbf{r}_{D,0}$ lies on $+X_P$ ($z_{B,0} = 0$ in `CoDe`, and identically in `2R`, `2R-remote`, `RCJ`):
- $L_f^{\mathrm{CoDe}} = x_{B,0} + s_{d,\mathrm{end}}$, $\quad L_f^{\mathrm{2R}} = L_f^{\mathrm{2R\text{-}remote}} = x_A + s_{d,\mathrm{end}}$, $\quad L_f^{\mathrm{RCJ}} = x_A + 2r + s_{d,\mathrm{end}}$.

### 5.2 Forward Kinematics, ICR, and Transmission Ratio

1. **Distal Body Pose $T_{PD}(\varphi; \mathbf{p}_{\mathrm{mech}})$:**
   - `2R` / `2R-remote`: $\mathbf{r}_D(\varphi) = [x_A, 0]^\top$ for all $\varphi \in \mathbb{R}$.
   - `RCJ`: $\mathbf{r}_D(\varphi) = [x_A + 2r\cos(\varphi/2),\; -2r\sin(\varphi/2)]^\top$ for all $\varphi \in (-\pi, \pi)$.
   - `CoDe`: For a target distal flexion $\varphi$, let $\mathbf{c}(\varphi) = \mathbf{r}_A - R(-\varphi)\Delta\mathbf{r}_{B'}$. Loop closure requires $\mathbf{r}_B(\varphi) \in \mathcal{C}(\mathbf{r}_{A'}, \ell_{A'B}) \cap \mathcal{C}(\mathbf{c}(\varphi), \ell_{AB'})$, defined on the connected domain $\Phi_{\mathrm{close}}(\mathbf{p}_{\mathrm{mech}}) \subseteq [-\pi, \pi]$ containing $\varphi = 0$ on the reference assembly branch.
2. **Instantaneous Center of Rotation $\mathbf{r}_{\mathrm{ICR}}(\varphi)$:**
   - `2R` / `2R-remote`: $\mathbf{r}_{\mathrm{ICR}}(\varphi) \equiv [x_A, 0]^\top$.
   - `RCJ`: $\mathbf{r}_{\mathrm{ICR}}(\varphi) = [x_A + r\cos(\varphi/2),\; -r\sin(\varphi/2)]^\top$ (rolling contact point); $x_{\mathrm{ICR},0} = x_A + r$.
   - `CoDe`: Line intersection $(A\text{–}B'(\varphi)) \cap (A'\text{–}B(\varphi))$ in $\{P\}$ (undefined/at infinity only when the two bars become parallel).
3. **Instantaneous Transmission Ratio $k(M, \varphi) = |\mathrm{d}\varphi / \mathrm{d}\theta_{\mathrm{act}}|$:**
   By virtual work ($\tau_{\mathrm{act}}\,\delta\theta_{\mathrm{act}} = \tau_{\mathrm{distal}}\,\delta\varphi$), $k(M, \varphi)$ is the actuator torque required per unit distal moment held:
   - `2R` and `2R-remote` (1:1 belt): $k(M, \varphi) \equiv 1$.
   - `RCJ` (symmetric 1:1 rollers): $k(M, \varphi) \equiv 2$.
   - `CoDe`: Differentiating the rigid-bar constraints $\|\mathbf{r}_B - \mathbf{r}_{A'}\|^2 = \ell_{A'B}^2$ and $\|\mathbf{r}_{B'} - \mathbf{r}_A\|^2 = \ell_{AB'}^2$ with respect to the servo crank angle $\theta_{\mathrm{act}} = -\operatorname{atan2}(z_B - z_{A'}, x_B - x_{A'})$ yields the exact closed-form analytical velocity ratio (`CoDeMechanismParams.transmission_ratio`):
     $$
     \frac{\mathrm{d}\mathbf{r}_B}{\mathrm{d}\theta_{\mathrm{act}}} = -(\mathbf{r}_B - \mathbf{r}_{A'})^\perp, \qquad
     \frac{\mathrm{d}\mathbf{r}_{B'}}{\mathrm{d}\theta_{\mathrm{act}}} = (\mathbf{r}_{B'} - \mathbf{r}_A)^\perp \frac{(\mathbf{r}_B - \mathbf{r}_{B'})^\top (\mathrm{d}\mathbf{r}_B/\mathrm{d}\theta_{\mathrm{act}})}{(\mathbf{r}_B - \mathbf{r}_{B'})^\top (\mathbf{r}_{B'} - \mathbf{r}_A)^\perp},
     $$
     $$
     k(M, \varphi) = \frac{\big|(\mathbf{r}_B - \mathbf{r}_{B'}) \times (\mathrm{d}\mathbf{r}_B/\mathrm{d}\theta_{\mathrm{act}} - \mathrm{d}\mathbf{r}_{B'}/\mathrm{d}\theta_{\mathrm{act}})\big|}{\|\mathbf{r}_B - \mathbf{r}_{B'}\|_2^2},
     $$
     where $\mathbf{u}^\perp = [-u_z, u_x]^\top$ and $\mathbf{u} \times \mathbf{v} = u_x v_z - u_z v_x$. Singularities occur when the **transmission angle** $\gamma_{B'}(\varphi) = \angle(A - B'(\varphi),\, B(\varphi) - B'(\varphi)) \in \{0, \pi\}$.

### 5.3 Master Specification of Layer 3 Derived Quantities

| Quantity | Symbol | Canonical Inputs | Reference Pose / Domain | Exact vs. Numerical |
|---|---|---|---|---|
| Common Finger Geometry | $\mathbf{L}(M) = [L_f, L_p, L_d]^\top$ | $\mathbf{p}_{\mathrm{mech}}, \mathbf{p}_{\mathrm{struct}}$ | Extended ($q_{\mathrm{MCP}}=0, \varphi=0$) | Exact closed-form |
| Forward Kinematics | $T_{PD}(\varphi), \mathbf{r}_C(\varphi)$ | $\mathbf{p}_{\mathrm{mech}}, s_{d,\mathrm{end}}, \varphi$ | $\varphi \in \Phi_{\mathrm{close}}(\mathbf{p}_{\mathrm{mech}})$ | Exact closed-form (circle–circle / trig) |
| Instantaneous Center of Rotation | $\mathbf{r}_{\mathrm{ICR}}(\varphi), x_{\mathrm{ICR},0}$ | $\mathbf{p}_{\mathrm{mech}}, \varphi$ | $\varphi \in \Phi_{\mathrm{close}}$, non-parallel bars | Exact closed-form |
| External Descriptor (Study 2) | $E(M) = (L_f, L_p, L_d, x_{\mathrm{ICR},0})$ | $\mathbf{p}_{\mathrm{mech}}, \mathbf{p}_{\mathrm{struct}}$ | Extended ($q_{\mathrm{MCP}}=0, \varphi=0$) | Exact closed-form |
| Transmission Ratio | $k(M, \varphi)$ | $\mathbf{p}_{\mathrm{mech}}, \varphi$ | $\gamma_{B'}(\varphi) \notin \{0, \pi\}$ | Exact analytical (`canonical.py`); finite-diff over grid in `motion.py` |
| Peak Torque Ratio | $k_{\max}(M)$ | $\mathbf{p}_{\mathrm{mech}}, \varphi_{\min}^{\mathrm{des}}$ | $\varphi \in [0, \varphi_{\min}^{\mathrm{des}}]$ | Numerical max over $\Delta\theta_{\mathrm{act}}$ grid |
| Axial Support Rooms | $(S_p, S_d)$ | $\mathbf{p}_{\mathrm{struct}}, s_{c,p}, o_d$ | Body frames $\{P\}, \{D\}$ | Exact closed-form: $S_p = s_{p,\mathrm{end}} - s_{c,p}$, $S_d = s_{d,\mathrm{end}} - o_d - s_{d,\mathrm{start}}$ |

---

## 6. Static & Dynamic Collision Formulation

Static and dynamic collision checks are mathematically and computationally separated (`core/morphology/collision.py`). Under the actual-envelope convention, **touching is admissible ($c \ge 0$) and strict penetration is infeasible ($c < 0$)**; zero artificial body-clearance margin is added.

### 6.1 Analytical Signed-Clearance Primitives

1. **Axis-aligned rectangle $B = [u_0, u_1] \times [v_0, v_1]$ vs. disk $(\mathbf{c}, r)$** (pad vs. pivot pin or roller in the pad's parent frame):
   $$
   \operatorname{dist}(\mathbf{c}, B) = \sqrt{
   \max(u_0 - c_x,\, 0,\, c_x - u_1)^2 +
   \max(v_0 - c_z,\, 0,\, c_z - v_1)^2
   }, \qquad
   c_{\mathrm{rect\text{-}disk}}(B, \mathbf{c}, r) = \operatorname{dist}(\mathbf{c}, B) - r.
   $$
   Note that when $\mathbf{c} \in B$, $\operatorname{dist}(\mathbf{c}, B) = 0$ and $c_{\mathrm{rect\text{-}disk}} = -r < 0$.
2. **Disk $(\mathbf{c}_1, r_1)$ vs. disk $(\mathbf{c}_2, r_2)$** (pin–pin or roller–roller):
   $$
   c_{\mathrm{disk\text{-}disk}} = \|\mathbf{c}_1 - \mathbf{c}_2\|_2 - (r_1 + r_2).
   $$
3. **Disk $(\mathbf{c}, r)$ vs. D-shaped servo footprint $G_{\mathrm{D}}(r_s, e_{\mathrm{rear}})$** at pose $T_s = (\mathbf{s}, \alpha_s)$:
   Let $(u, v)^\top = R(-\alpha_s)(\mathbf{c} - \mathbf{s})$ be the disk center in the local servo frame $\{A_s\}$. The point-to-servo distance is:
   $$
   \operatorname{dist}(\mathbf{c}, G_{\mathrm{D}}) =
   \begin{cases}
   \max\!\big(\sqrt{u^2 + v^2} - r_s,\, 0\big), & u \ge 0, \\[4pt]
   \sqrt{\max(-e_{\mathrm{rear}} - u,\, 0)^2 + \max(|v| - r_s,\, 0)^2}, & u < 0,
   \end{cases}
   \qquad
   c_{\mathrm{disk\text{-}servo}} = \operatorname{dist}(\mathbf{c}, G_{\mathrm{D}}) - r.
   $$
4. **General convex/polygonal clearances** ($c_{\mathrm{poly\text{-}poly}}$, $c_{\mathrm{poly\text{-}servo}}$, $c_{\mathrm{servo\text{-}servo}}$): Computed via exact signed edge-segment distance with interior ray-containment sign (`core/morphology/packaging/_kernels.py`).

### 6.2 Static Collision ($\mathbf{q} = \mathbf{0}$ and Configuration-Invariant Intra-Body Pairs)

Static collision evaluates $c_{\mathrm{pair}}(M) \ge 0$ once prior to any motion sweep:
- **Strictly configuration-invariant intra-body pairs** (rigidly co-moving on $\{P\}$ or on $\{D\}$, independent of $(q_{\mathrm{MCP}}, \varphi)$ for all motions):
  - On $\{P\}$: proximal pad vs. $A, A'$ (`CoDe`, `2R-remote`) or proximal roller (`RCJ`); proximal pad vs. PIP servo $G_2$; $A$ pin vs. PIP servo $G_2$ (`a_pin_and_pip_servo`).
  - On $\{D\}$: distal pad vs. $B, B'$ (`CoDe`), $A$ (`2R-remote`), or distal roller (`RCJ`); $B$ vs. $B'$.
- **Fixed-MCP ($q_{\mathrm{MCP}} = 0$) invariant pairs**: MCP servo $G_1$ vs. PIP servo $G_2$, proximal pad, and proximal roller.
- **Extended reference-pose ($\varphi = 0$) cross-body pairs**: distal pad vs. proximal pins/roller, servos, and proximal pad at rest.

Because intra-body and fixed-MCP pairs are invariant wrt $\varphi$, dynamic motion sweeps pass `include_static_pairs=False` rather than recomputing them at every flexion sample.

### 6.3 Dynamic Collision & Discrete Checkpoint Policy

As the finger flexes ($\varphi > 0$), cross-body relative motion brings the distal body $\{D\}$ toward $\{P\}$ and $\{W\}$. For canonical pad–pad nonpenetration screening up to required flexion $\varphi_{\min}^{\mathrm{des}} \ge 0$, the SSoT defines the discrete checkpoint set:
$$
\boxed{
\Phi_{\mathrm{check}}(\varphi_{\min}^{\mathrm{des}}) =
\big\{60^\circ + 30^\circ k : k \in \mathbb{Z}_{\ge 0},\; 60^\circ + 30^\circ k \le \varphi_{\min}^{\mathrm{des}}\big\}
\,\cup\,
\big\{\varphi_{\min}^{\mathrm{des}}\big\}
}
$$
requiring at every checkpoint:
$$
c_{pd}(M, \varphi) \ge 0 \qquad \forall\, \varphi \in \Phi_{\mathrm{check}}(\varphi_{\min}^{\mathrm{des}}).
$$
(For $\varphi_{\min}^{\mathrm{des}} < 60^\circ$, $\Phi_{\mathrm{check}} = \{\varphi_{\min}^{\mathrm{des}}\}$.)

> **Scientific distinction: Discrete Screening vs. Continuous Verification.** Satisfying $c_{pd}(M, \varphi) \ge 0$ on the finite set $\Phi_{\mathrm{check}}$ is a **discrete screening condition**; it does **not** mathematically guarantee $c_{pd}(M, \varphi) \ge 0$ over the continuous interval $[0, \varphi_{\min}^{\mathrm{des}}]$. Separate dense-grid validation (`verify_dynamic_collision_refined` at $\Delta\varphi = 0.05^\circ\text{–}0.5^\circ$, and bounded rotational velocity bisection `_between_samples` for `2R`/`RCJ`) is maintained as a distinct numerical verification layer.

---

## 7. Pad Coverage Formulation

### 7.1 Five Distinct Length & Coverage Quantities

The SSoT strictly distinguishes five quantities that must **never** be used interchangeably:

| # | Quantity | Symbol | Definition & Physical Meaning |
|---:|---|---|---|
| 1 | **Geometrically supported room** | $S_i(M)$ | Axial structural length available on segment $i$ from the outer datum to $A''$ or $B''$ ($S_p = s_{p,\mathrm{end}} - s_{c,p}$, $S_d = s_{d,\mathrm{end}} - o_d - s_{d,\mathrm{start}}$) |
| 2 | **Mounted contact length** | $\ell_i$ | Physical axial length of the contact module installed on segment $i$ (admissibility requires $\ell_i \le S_i(M)$) |
| 3 | **Effective usable contact length** | $L_i^{\mathrm{eff}}(M, \varphi; \eta_L)$ | 1D Lebesgue measure $\mu(\mathcal{U}_i(M, \varphi; \eta_L))$ of the unoccluded, fold-accessible contact face at flexion $\varphi$ |
| 4 | **Endpoint Coverage** *(primary)* | $C_{\mathrm{endpoint}}(M; \eta_L)$ | Normalized total effective usable face at the required flexion $\varphi = \varphi_{\min}^{\mathrm{des}}$ |
| 5 | **Motion-Robust Coverage** *(diagnostic)* | $C_{\mathrm{robust}}(M; \eta_L)$ | Normalized measure of material face points that remain continuously accessible across **all** $\varphi \in [0, \varphi_{\min}^{\mathrm{des}}]$ |

### 7.2 Material Coordinate & Set-Based Accessible Contact Face

Parameterize each pad's palmar contact face in its **body-fixed material coordinate** $\xi_i \in [0, \ell_i]$, measured from the **outer end** ($\xi_p = 0$ near MCP $O$ toward $A''$; $\xi_d = 0$ at the fingertip $C$ toward $B''$):
$$
\mathbf{p}_{\mathrm{face},p}^{\{P\}}(\xi_p) =
\begin{bmatrix} s_{c,p} + \xi_p \\ -(d_p + h_p) \end{bmatrix}, \qquad
\mathbf{p}_{\mathrm{face},d}^{\{D\}}(\xi_d) =
\begin{bmatrix} s_{c,d} + \ell_d - \xi_d \\ -(d_d + h_d) \end{bmatrix},
$$
each with local outward palmar unit normal $\mathbf{n}_i^{\{b_i\}} = [0, -1]^\top$. Because $\xi_i$ is a material coordinate fixed on rigid body $b_i$, a fixed value of $\xi_i$ tracks the exact same physical point on the pad across all postures $\varphi$.

At configuration $\varphi$, let $\mathbf{p}_{i,\varphi}(\xi_i)$ and $\mathbf{n}_{i,\varphi}$ denote the posed face point and outward unit normal in $\{P\}$.

1. **Base accessible set $\mathcal{U}_i^{\mathrm{base}}(M, \varphi) \subseteq [0, \ell_i]$ (Fold rule):**
   Let $\ell_{i,\mathrm{fold}}(M, \varphi) \ge 0$ be the distance from the outer end $\xi_i = 0$ to the V-fold intersection of the two posed infinite contact-face lines ($\ell_{i,\mathrm{fold}} = +\infty$ when $\varphi \le 0$ or parallel):
   $$
   \mathcal{U}_i^{\mathrm{base}}(M, \varphi) = \big[0,\, \min(\ell_i,\, \ell_{i,\mathrm{fold}}(M, \varphi))\big].
   $$
2. **Linkage / transmission occlusion set $\mathcal{O}_i^{\mathrm{link}}(M, \varphi) \subseteq [0, \ell_i]$:**
   A point $\mathbf{q} \in \mathbb{R}^2$ on a crossing transmission member $E$ lies **in front of** face $i$ iff its normal coordinate satisfies $v(\mathbf{q}) = (\mathbf{q} - \mathbf{p}_{i,\varphi}(0))^\top \mathbf{n}_{i,\varphi} > 0$. Projecting $E \cap \{v \ge 0\}$ orthogonally onto the tangent axis $\mathbf{t}_{i,\varphi} = (\mathbf{p}_{i,\varphi}(\ell_i) - \mathbf{p}_{i,\varphi}(0))/\ell_i$ yields an **exact closed-form 1D interval** (`core/morphology/intervals.py`):
   - For `CoDe`: $E \in \{A\text{–}B'(\varphi),\, A'\text{–}B(\varphi)\}$ against both faces ($i \in \{p, d\}$).
   - For `2R-remote`: belt tangent strands and pulleys at $(A, A')$ of radius $r_{\mathrm{pulley}}$ against $i = p$, and pulley $A$ against $i = d$.
   - For `2R` and `RCJ`: $\mathcal{O}_i^{\mathrm{link}}(M, \varphi) = \varnothing$.
3. **Configurable accessible contact-face set $\mathcal{U}_i(M, \varphi; \eta_L)$:**
   With indicator $\eta_L \in \{0, 1\}$ (`include_linkage_occlusion`):
   $$
   \boxed{
   \mathcal{U}_i(M, \varphi; \eta_L) =
   \begin{cases}
   \mathcal{U}_i^{\mathrm{base}}(M, \varphi) \setminus \mathcal{O}_i^{\mathrm{link}}(M, \varphi), & \eta_L = 1\;\;(\text{ON}), \\[4pt]
   \mathcal{U}_i^{\mathrm{base}}(M, \varphi), & \eta_L = 0\;\;(\text{OFF}),
   \end{cases}
   \qquad
   L_i^{\mathrm{eff}}(M, \varphi; \eta_L) = \mu\!\big(\mathcal{U}_i(M, \varphi; \eta_L)\big)
   }
   $$
   Because $\mathcal{U}_i(M, \varphi; 1)$ is represented as a finite union of disjoint closed intervals (`IntervalSet`), **disconnected usable regions are supported exactly** with zero spatial discretization error, and $\eta_L = 1 \implies L_i^{\mathrm{eff}}(M, \varphi; 1) \le L_i^{\mathrm{eff}}(M, \varphi; 0)$ holds identically.

> **Explicit Context Defaults & Orthogonality.** `include_linkage_occlusion` is resolved unambiguously from the explicit configuration `profile`:
> - `profile: study` $\implies$ default `include_linkage_occlusion = False` ($\eta_L = 0$).
> - `profile: prototype` $\implies$ default `include_linkage_occlusion = True` ($\eta_L = 1$).
>
> Toggling $\eta_L$ modifies **only** $\mathcal{O}_i^{\mathrm{link}}$; it never disables pad–pad collision checks, fold face loss $\mathcal{U}_i^{\mathrm{base}}$, or structural support constraints.

### 7.3 Endpoint Coverage vs. Motion-Robust Coverage

1. **Endpoint Coverage (Primary Optimization Metric):** Evaluated at $\varphi = \varphi_{\min}^{\mathrm{des}}$, normalized by extended finger length $L_f(M)$:
   $$
   \boxed{
   C_{\mathrm{endpoint}}(M; \eta_L) =
   \frac{L_p^{\mathrm{eff}}(M, \varphi_{\min}^{\mathrm{des}}; \eta_L) + L_d^{\mathrm{eff}}(M, \varphi_{\min}^{\mathrm{des}}; \eta_L)}{L_f(M)}
   }
   $$
2. **Motion-Robust Coverage (Secondary Diagnostic Metric):** Defined via material-coordinate set intersection across the flexion trajectory:
   $$
   \boxed{
   \mathcal{U}_i^{\mathrm{rob}}(M; \eta_L) = \bigcap_{\varphi \in [0, \varphi_{\min}^{\mathrm{des}}]} \mathcal{U}_i(M, \varphi; \eta_L), \qquad
   C_{\mathrm{robust}}(M; \eta_L) =
   \frac{\mu\!\big(\mathcal{U}_p^{\mathrm{rob}}(M; \eta_L)\big) + \mu\!\big(\mathcal{U}_d^{\mathrm{rob}}(M; \eta_L)\big)}{L_f(M)}
   }
   $$

**Mathematical vs. Empirical Relationship:**
- **Exact inequality:** Since $\varphi_{\min}^{\mathrm{des}} \in [0, \varphi_{\min}^{\mathrm{des}}]$, $\mathcal{U}_i^{\mathrm{rob}}(M; \eta_L) \subseteq \mathcal{U}_i(M, \varphi_{\min}^{\mathrm{des}}; \eta_L)$, so $0 \le C_{\mathrm{robust}}(M; \eta_L) \le C_{\mathrm{endpoint}}(M; \eta_L) \le (\ell_p + \ell_d)/L_f(M)$ holds universally.
- **Empirical regime dependence across optimized Pareto fronts:**
  - *Equal total pad depths ($D_p = D_d$, e.g. `2r_dd_tilt90` or `align_mount_offsets: true`):* The V-crossing distance decreases monotonically with flexion angle $\varphi \in (0, \pi)$, yielding $C_{\mathrm{robust}} = C_{\mathrm{endpoint}}$ on `2R` ($r = 1.000$) and near-perfect rank preservation on `CoDe` ($r = 0.96\text{–}0.99$, Spearman $\rho = 0.97\text{–}1.00$).
  - *Unequal total pad depths ($D_p \neq D_d$, e.g. `2r_dd_tilt45`, `rcj`, or unaligned `CoDe`):* Under infinite-line V-crossing, two non-collinear face lines at depths $D_p \neq D_d$ intersect at $x_{\times}(\varphi) \approx x_{\mathrm{hinge}} + (D_p - D_d)/\sin\varphi$ as $\varphi \to 0^+$. When $D_p > D_d$, $x_{\times}(\varphi) \to +\infty$ past the fingertip at small flexion angles ($\varphi \approx 1^\circ$), causing $C_{\mathrm{robust}}$ to drop substantially below $C_{\mathrm{endpoint}}$ and breaking rank correlation ($r = -0.32\text{ to }+0.30$). Therefore, correlation between $C_{\mathrm{endpoint}}$ and $C_{\mathrm{robust}}$ is **not** a mathematical identity and must not be assumed when $D_p \neq D_d$.

---

## 8. Study & Prototype Optimization Formulations (Layer 4)

All active studies are formulated as constrained optimization or evaluation problems over $M \in \mathcal{M}(m)$:

1. **Multi-Objective Finger Design (Prototype & Baseline Study Search):**
   Given context profile $\eta_L$, required flexion $\varphi_{\min}^{\mathrm{des}}$, hyperextension $\varphi_{\mathrm{hyp}}$, length bounds $[L_{f,\min}, L_{f,\max}]$, and minimum module lengths $(\ell_{p,\min}, \ell_{d,\min})$:
   $$
   \min_{M \in \mathcal{M}_{\mathrm{search}}(m)} \Big(\, k_{\max}(M),\; -C_{\mathrm{endpoint}}(M; \eta_L),\; \pi - \varphi_{\max}(M) \,\Big)
   $$
   subject to static & dynamic nonpenetration, loop closure on $[-\varphi_{\mathrm{hyp}}, \varphi_{\min}^{\mathrm{des}}]$, $\gamma_{B'}(\varphi) \in [\gamma_{\min}, \gamma_{\max}]$ (`CoDe`), $x_C(\varphi) \ge 0$, $L_f(M) \in [L_{f,\min}, L_{f,\max}]$, and $L_i^{\mathrm{eff}}(M, \varphi_{\min}^{\mathrm{des}}; \eta_L) \ge \ell_{i,\min}$.
2. **Study 1 — Fixed-Morphology Contact Accommodation (`study1_fixed_morphology`):**
   Hold $(m, \mathbf{p}_{\mathrm{mech}}^0, \mathbf{p}_{\mathrm{act}}^0, \mathbf{p}_{\mathrm{struct}}^0)$ frozen at matched external descriptor $E_0$. For each catalog pair $(\ell_p, h_p, \ell_d, h_d)$ and target $\varphi_{\min}^{\mathrm{des}} \in \{90^\circ, 120^\circ, 150^\circ\}$, set $d_i = \pi_{\mathrm{mount},i}(M)$ (`shallowest_supported`) and test Layer 2 support admissibility and Layer 4 feasibility under $\eta_L = 0$ (and sensitivity $\eta_L = 1$).
3. **Study 2 — Hierarchical Morphology Adaptation (`study2_morphology_adaptation`):**
   Partition $H = (E, Q, A)$ where $E(M) = (L_f, L_p, L_d, x_{\mathrm{ICR},0})$, $Q(M)$ holds independent internal mechanism coordinates ($\dim Q = 3$ for axis-aligned `CoDe`, $1$ for `RCJ`, $0$ for `2R`/`2R-remote`), and $A$ holds actuation fixed. For any target module pair infeasible at $(E_0, Q_0, A_0)$:
   - *Level 1 ($E = E_0$ fixed):* $\min_{Q} \varepsilon_Q(M) \triangleq \max_j |q_j - q_{0,j}| / L_{f,0}$ s.t. feasible.
   - *Level 2 (if Level 1 empty):* Lexicographically minimize $\varepsilon_E(M) \triangleq \max_k |e_k - e_{0,k}| / L_{f,0}$, then $\varepsilon_Q(M)$ subject to $\varepsilon_E(M) \le \varepsilon_E^\star + \delta$.
4. **Study 3 — Transmission-Reflected Contact Mechanics (`study3_transmission`):**
   Uses the contact-point Jacobian $J_c(M, \varphi, \xi_i) \in \mathbb{R}^{2 \times 2}$ derived on $\mathcal{M}(m)$ to map isotropic/anisotropic joint stiffness and damping into local palmar contact wrench/impedance space.
5. **Study 4 — Continuous Joint-Interface Bridge Screening (`study4_joint_interface`):**
   Evaluates elastica arc-length strain of a continuous palmar bridge anchored between $\{P\}$ and $\{D\}$ across $\varphi \in [0, \varphi_{\min}^{\mathrm{des}}]$.

---

## 9. Implementation Mapping & Master Notation Tables

### 9.1 Flat Optimization Vector (`VARIABLE_NAMES`) Mapping to SSoT

Implemented in `design/core/optimization/canonical_mapping.py` (`VARIABLE_SLOT_MAPPING`, `morphology_from_design`, `design_from_morphology`):

| Mechanism (`core/optimization/*`) | Slot Name | SSoT Group | Role | Canonical / Derived Relationship |
|---|---|---|---|---|
| **`linkage`** (`CoDe`, 14 slots) | `a_x_mm`, `a_z_mm` | $\mathbf{p}_{\mathrm{mech}}$ | Canonical | $\operatorname{pos}(T_A^{\{P\}}) = [x_A, z_A]^\top$ ($z_A=0$ in axis-aligned study subspace) |
| | `b_x_mm`, `b_z_mm` | $\mathbf{p}_{\mathrm{mech}}$ | Canonical | $\operatorname{pos}(T_B^{\{P\}}(0)) = [x_{B,0}, z_{B,0}]^\top$ ($z_{B,0}=0$ in study subspace) |
| | `ap_x_mm`, `ap_z_mm` | $\mathbf{p}_{\mathrm{mech}} \to \mathbf{p}_{\mathrm{act}}$ | Canonical + Induced | $\operatorname{pos}(T_{A'}^{\{P\}}) = [x_{A'}, z_{A'}]^\top$; induces $\operatorname{pos}(T_2^{\{P\}})$ |
| | `bp_x_mm`, `bp_z_mm` | $\mathbf{p}_{\mathrm{mech}}$ | Relative chart | Extended $B'$ in $\{P\}$: $\Delta\mathbf{r}_{B'} = [x_{B',0} - x_{B,0},\, z_{B',0} - z_{B,0}]^\top$ |
| | `servo_tilt_deg` | $\mathbf{p}_{\mathrm{act}}$ | Canonical | Dorsal tilt $\psi_{\mathrm{tilt}}$ [deg]; $\operatorname{ori}(T_2^{\{P\}}) = -\psi_{\mathrm{tilt}}$ |
| | `bc_length_mm` | $\mathbf{p}_{\mathrm{struct}}$ | Canonical | $s_{d,\mathrm{end}}$ in $T_{d,\mathrm{end}}^{\{D\}}$ |
| | `app_offset_mm` | $\mathbf{p}_{\mathrm{struct}}$ | Relative chart | $s_{p,\mathrm{end}} - x_{A'}$ ($s_{p,\mathrm{end}} = x_{A'} + \text{app\_offset}$) |
| | `bpp_offset_mm` | $\mathbf{p}_{\mathrm{struct}}$ | Relative chart | $x_{B',0} - (x_{B,0} + s_{d,\mathrm{start}})$ ($s_{d,\mathrm{start}} = x_{B',0} - \text{bpp\_offset} - x_{B,0}$) |
| | `proximal_mount_offset_mm`, `distal_mount_offset_mm` | $\mathbf{p}_{\mathrm{contact}}$ | Canonical or Policy | $d_p, d_d \ge 0$ in $T_{c,i}^{\{b_i\}} = \operatorname{Trans}(s_{c,i}, -d_i)$ |
| **`revolute`** (`2R`, 7 slots) | `a_x_mm` | $\mathbf{p}_{\mathrm{mech}} \to \mathbf{p}_{\mathrm{act}}$ | Canonical + Induced | $x_A$ in $T_A^{\{P\}}$; induces $\operatorname{pos}(T_2^{\{P\}}) = [x_A, 0]^\top$ |
| | `ac_length_mm`, `app_x_mm`, `bpp_from_a_mm` | $\mathbf{p}_{\mathrm{struct}}$ | Canonical | $(s_{d,\mathrm{end}},\, s_{p,\mathrm{end}},\, s_{d,\mathrm{start}})$ |
| | `distal_drop_mm`, `proximal_drop_mm` | $\mathbf{p}_{\mathrm{contact}}$ | Canonical or Policy | $(d_d, d_p)$ |
| | `servo_tilt_deg` | $\mathbf{p}_{\mathrm{act}}$ | Canonical | $\psi_{\mathrm{tilt}}$ [deg] ($0$ and unused when $G_2 = \varnothing$ in `revolute_idealized`) |
| **`revolute_parallel`** (`2R-remote`, 9 slots) | *(first 7 slots as `revolute`, except $T_2$ not at $A$)* | — | — | Same $\mathbf{p}_{\mathrm{mech}}, \mathbf{p}_{\mathrm{struct}}, \mathbf{p}_{\mathrm{contact}}$ |
| | `ap_from_a_x_mm`, `ap_from_a_z_mm` | $\mathbf{p}_{\mathrm{act}}$ | Relative / Canonical | Remote shaft $\operatorname{pos}(T_2^{\{P\}}) = [x_A + \Delta x_{A'},\, z_{A'}]^\top$ |
| **`rcj`** (`RCJ`, 8 slots) | `a_x_mm`, `roller_radius_mm` | $\mathbf{p}_{\mathrm{mech}} \to \mathbf{p}_{\mathrm{act}}$ | Canonical + Induced | $(x_A, r)$; $x_A$ induces $\operatorname{pos}(T_2^{\{P\}}) = [x_A, 0]^\top$ |
| | `bc_length_mm`, `app_x_mm`, `bpp_from_b_mm` | $\mathbf{p}_{\mathrm{struct}}$ | Canonical | $(s_{d,\mathrm{end}},\, s_{p,\mathrm{end}},\, s_{d,\mathrm{start}})$ |
| | `distal_drop_mm`, `proximal_drop_mm` | $\mathbf{p}_{\mathrm{contact}}$ | Canonical or Policy | $(d_d, d_p)$ |
| | `servo_tilt_deg` | $\mathbf{p}_{\mathrm{act}}$ | Canonical | $\psi_{\mathrm{tilt}}$ [deg] ($0$ and unused when $G_2 = \varnothing$ in `rcj_idealized`) |

### 9.2 SSoT Code Architecture Map

| Module | SSoT Layer & Role |
|---|---|
| `design/core/geometry/planar.py` | Planar $SE(2)$ group `Transform2D` (`rot`, `flexion`, `trans`, composition, inverse, homogeneous matrix) |
| `design/core/morphology/canonical.py` | Layers 1–3: `FingerMorphology`, `CoDeMechanismParams`, `RevoluteMechanismParams`, `RcjMechanismParams`, `ActuationParams`, `StructureParams`, `ContactParams`, `CommonGeometry` |
| `design/core/morphology/collision.py` | Closed-form static clearances (`evaluate_static_collision`), checkpoint screening (`dynamic_checkpoints_deg`, `evaluate_dynamic_collision`), and refined verification (`verify_dynamic_collision_refined`) |
| `design/core/morphology/intervals.py` | Exact 1D closed-interval algebra (`IntervalSet`) and analytical segment/disk normal-projection occlusion |
| `design/core/morphology/coverage.py` | Context-profile resolution (`resolve_linkage_occlusion`), `accessible_face_sets`, `evaluate_pad_coverage` ($C_{\mathrm{endpoint}}, C_{\mathrm{robust}}$) |
| `design/core/optimization/canonical_mapping.py` | Layer 5 $\leftrightarrow$ Layer 1 bijection (`VARIABLE_SLOT_MAPPING`, `morphology_from_design`, `design_from_morphology`) |

### 9.3 Master Notation Table

| Symbol | Domain / Units | Definition |
|---|---|---|
| $m \in \mathcal{A}$ | Discrete | Architecture in $\{\mathrm{CoDe}, \mathrm{2R}, \mathrm{2R\text{-}remote}, \mathrm{RCJ}\}$ |
| $M \in \mathcal{M}(m)$ | Tuple | Admissible finger morphology $(\mathbf{p}_{\mathrm{mech}}, \mathbf{p}_{\mathrm{act}}, \mathbf{p}_{\mathrm{struct}}, \mathbf{p}_{\mathrm{contact}})$ |
| $\{W\}, \{P\}, \{D\}$ | Frames in $\mathbb{R}^2$ | World/MCP palm frame, proximal body frame, distal body frame (`+X` distal, `+Z` dorsal, `-Z` palmar) |
| $R(\alpha), R_{\mathrm{flex}}(\vartheta)$ | $SO(2)$ | CCW rotation by $\alpha$ [rad]; palmar flexion rotation $R_{\mathrm{flex}}(\vartheta) = R(-\vartheta)$ |
| $q_{\mathrm{MCP}}, \varphi$ | $\mathbb{R}$ [rad] | MCP flexion angle and relative distal-to-proximal flexion angle ($\varphi = 0$ extended) |
| $T_{PD}(\varphi; \mathbf{p}_{\mathrm{mech}})$ | $SE(2)$ | Pose of distal frame $\{D\}$ relative to proximal frame $\{P\}$ |
| $G_i, T_i, \psi_i$ | $\subseteq \mathbb{R}^2, SE(2), \text{rad}$ | Actuator $i \in \{1,2\}$ footprint, pose, and dorsal rear-body tilt ($\operatorname{ori}(T_i) = -\psi_i$) |
| $s_{p,\mathrm{end}}, s_{d,\mathrm{start}}, s_{d,\mathrm{end}}$ | $\mathbb{R}$ [mm] | Axial coordinates of structural endpoints $(A'', B'', C)$ in their parent frames |
| $\ell_i, h_i, s_{c,i}, d_i$ | $\mathbb{R}$ [mm] | Contact module $i \in \{p,d\}$ length, thickness, axial start, and palmar mounting offset |
| $\mathbf{L}(M) = [L_f, L_p, L_d]^\top$ | $\mathbb{R}_{>0}^3$ [mm] | Extended finger length and proximal/distal structural spans |
| $\mathbf{r}_{\mathrm{ICR}}(\varphi), k(M, \varphi)$ | $\mathbb{R}^2$ [mm], $\mathbb{R}_{\ge 0}$ | Instantaneous center of rotation in $\{P\}$ and instantaneous transmission ratio $|\mathrm{d}\varphi/\mathrm{d}\theta_{\mathrm{act}}|$ |
| $\Phi_{\mathrm{check}}(\varphi_{\min}^{\mathrm{des}})$ | Finite $\subset \mathbb{R}_{\ge 0}$ | Dynamic collision checkpoint set $\{60^\circ + 30^\circ k \le \varphi_{\min}^{\mathrm{des}}\} \cup \{\varphi_{\min}^{\mathrm{des}}\}$ |
| $\xi_i \in [0, \ell_i]$ | $\mathbb{R}$ [mm] | Body-fixed material coordinate along contact face $i$ from its outer end |
| $\eta_L \in \{0, 1\}$ | Binary | Linkage-occlusion indicator (`include_linkage_occlusion`: Study default $0$, Prototype default $1$) |
| $\mathcal{U}_i(M, \varphi; \eta_L)$ | $\subseteq [0, \ell_i]$ | Accessible contact-face subset at flexion $\varphi$; $L_i^{\mathrm{eff}} = \mu(\mathcal{U}_i)$ |
| $C_{\mathrm{endpoint}}, C_{\mathrm{robust}}$ | $[0, \infty)$ [-] | Endpoint coverage at $\varphi_{\min}^{\mathrm{des}}$ and motion-robust intersection coverage over $[0, \varphi_{\min}^{\mathrm{des}}]$ |

---

## 10. Modeling Assumptions, Claim Boundaries & Open Questions

### 10.1 Explicit Modeling Assumptions & Numerical Approximations

1. **2D Planar Projection & Out-of-Plane Y-Layers:** All kinematics, clearances, and occlusions are evaluated in the sagittal $(x, z)$ plane. Crossing bars (`CoDe` $A\text{–}B'$, $A'\text{–}B$) and remote belts (`2R-remote`) are assumed to occupy lateral $Y$ planes routed clear of the pad blocks (`links_clear_pads`), so they do not physically collide with pad interiors in 2D but optionally occlude palmar access ($\eta_L = 1$).
2. **Rigid Links, Ideal No-Slip Rollers & Zero-Backlash Drives:** Structural deflection, bearing play, tendon/belt compliance, and RCJ tether thickness/wall retention are unmodeled; `rcj_idealized` and `revolute_idealized` ($G_2 = \varnothing$) serve strictly as theoretical upper bounds.
3. **Actual-Envelope Nonpenetration:** Hardware envelopes carry nominal CAD dimensions (`MARGIN_TOLERANCE_MM = 1e-9 mm`); manufacturing tolerances must be absorbed into declared envelope parameters (`pivot_diameter_mm`, $h_i$) when configuring Prototype runs.
4. **Numerical Approximations Kept Separate from Analytical Definitions:**
   - Optimizer motion sweeps discretize actuator input at `servo_step_deg` ($1.0^\circ$) and verify Pareto fronts at `verify_step_deg` ($0.05^\circ$). Because a $1^\circ$ crank step on high-$k$ four-bars can overshoot $\varphi_{\min}^{\mathrm{des}}$ by $\sim 0.5^\circ\text{–}1.0^\circ$, coarse-grid endpoint evaluations approximate exact analytical evaluation at $\varphi_{\min}^{\mathrm{des}}$ and converge as $\Delta\theta_{\mathrm{act}} \to 0$.
   - Prototype `CoDe` bar lengths are rounded to `prototyping_resolution_mm` ($0.1\text{ mm}$) and re-assembled before evaluation (`apply_grid_assembly=True`).

### 10.2 Scientific Claim Boundaries

- **Never claim continuous collision freedom from discrete checkpoint screening alone.** Checkpoint feasibility on $\Phi_{\mathrm{check}}$ is a screening filter; continuous nonpenetration requires either exact analytical interval bounds (`_between_samples`) or dense verification (`verify_step_deg`).
- **Never claim 3D grasp accessibility from 2D geometric coverage alone.** $C_{\mathrm{endpoint}}$ and $C_{\mathrm{robust}}$ quantify planar palmar line-of-sight/fold exposure under stated $\eta_L$ assumptions, not 3D object-specific contact wrench closure.
- **Never mix Prototype and Study results** or claim physical hardware validation from planar kinematic optimization runs.

### 10.3 Open Scientific Questions

1. **Infinite-line V-crossing vs. finite opposing-segment projection at unequal pad depths ($D_p \neq D_d$):** Under the established infinite-line V-crossing rule (`fold_mode="crossing"`), unequal total depths ($D_p > D_d$) produce a small-angle ($\varphi \to 0^+$) line intersection far distal to the physical proximal pad, strongly depressing $C_{\mathrm{robust}}$. Both `"crossing"` (default, preserving historical Endpoint Coverage semantics) and `"segment"` (finite opposing-pad projection) are implemented in `core/morphology/coverage.py` so future studies can evaluate finite-object grasp correlation directly.
2. **Asymmetric / geared RCJ ratios ($r_p \neq r_d$):** The current canonical `RCJ` model fixes $r_p = r_d = r$ ($k \equiv 2$) matching all benchmark studies. Generalizing $\mathbf{p}_{\mathrm{mech}}^{\mathrm{RCJ}}$ to $(T_A^{\{P\}}, r_p, r_d)$ ($k = 1 + r_p / r_d$) is a natural extension if non-1:1 rolling joints are studied.
3. **3D lateral ($Y$-axis) linkage occlusion:** In `CoDe` hardware, crossing bars sit laterally beside or around the contact module rather than across its full $Y$-width; quantifying partial 3D grasp occlusion vs. planar $\eta_L \in \{0, 1\}$ bounds requires 3D contact simulation (`sim/isaaclab/`).
