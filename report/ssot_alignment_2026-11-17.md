# Mathematical Source of Truth (SSoT) & Implementation Alignment Report

**Date:** 2026-11-17  
**Scope:** Governing mathematical formulation (`design/SSoT/math_formulation.md`) and shared `design/core/` implementation across both **Study** (`profile: study`) and **Prototype** (`profile: prototype`) tracks (numbers from the two optimization tracks are kept strictly separated).

---

## 1. What Was Changed

1. **Authoritative Mathematical SSoT Established (`design/SSoT/math_formulation.md`):**
   - Formalized the five-layer architecture separating **Layer 1 (Canonical Morphology)**, **Layer 2 (Parameter Dependencies & Admissible Space $\mathcal{M}(m)$)**, **Layer 3 (Derived Properties)**, **Layer 4 (Study / Prototype Requirements & Policies)**, and **Layer 5 (Numerical Implementation & Encodings)**.
   - Defined planar $SE(2)$ conventions (`+X` distal, `+Z` dorsal, `-Z` palmar; active CCW rotation $R(\alpha)$; palmar flexion $R_{\mathrm{flex}}(\vartheta) = R(-\vartheta)$) and the rigid-body hierarchy $\{W\} \to \{P\} \to \{D\}$.
   - Unified all four architectures $m \in \{\mathrm{CoDe}, \mathrm{2R}, \mathrm{2R\text{-}remote}, \mathrm{RCJ}\}$ (plus idealized upper-bound ablations $G_2 = \varnothing$) under $M = (\mathbf{p}_{\mathrm{mech}}, \mathbf{p}_{\mathrm{act}}, \mathbf{p}_{\mathrm{struct}}, \mathbf{p}_{\mathrm{contact}}) \in \mathcal{M}(m)$.
   - Consolidated `docs/formulation.md` into a pointer to `design/SSoT/math_formulation.md` so no competing definitions remain.
2. **Planar $SE(2)$ & Canonical Morphology Layer Implemented (`design/core/`):**
   - Added `design/core/geometry/planar.py` (`Transform2D`) implementing exact $SE(2)$ composition, inversion, matrix isomorphism, and `flexion(phi)`.
   - Added `design/core/morphology/mechanism/rcj/model.py` (`RollingJoint`, `RollingPose`, `RollingSweep`) and `revolute_parallel/model.py` so `Finger.kinematics()` operates uniformly across `CoDe`, `2R`, `2R-remote`, and `RCJ`.
   - Fixed a bug in `design/core/morphology/mechanism/rcj/solids.py` where `ContactPad.length_mm` was ignored when `max_length_mm` was `None`.
   - Added `design/core/morphology/canonical.py` (`FingerMorphology` and its four parameter groups, Layer 2 `validate()`, exact analytical `CoDeMechanismParams.transmission_ratio()`, and Layer 3 derived properties) with zero upward imports from `core.optimization`.
   - Added `design/core/optimization/canonical_mapping.py` (`VARIABLE_SLOT_MAPPING`, `morphology_from_design`, `design_from_morphology`).
3. **Static & Dynamic Collision Module (`design/core/morphology/collision.py`):**
   - Implemented closed-form analytical clearances (`point_rect_distance_mm`, `rect_disk_clearance_mm`, `disk_disk_clearance_mm`, `disk_servo_clearance_mm`) and replaced vertex-loop checks in `linkage/packaging.py` (`own_pin_gaps`) and `rcj/evaluate.py` (`roller_pad_mm`).
   - Separated static collision (`evaluate_static_collision` at $\mathbf{q} = \mathbf{0}$) from discrete checkpoint dynamic screening (`dynamic_checkpoints_deg`, `evaluate_dynamic_collision` on $\Phi_{\mathrm{check}}$) and separate dense-grid verification (`verify_dynamic_collision_refined`), while avoiding redundant recomputation of $\varphi$-invariant pairs during dynamic sweeps (`include_static_pairs=False`).
4. **Set-Based Pad Coverage & Context-Profile Occlusion Policy (`design/core/morphology/intervals.py`, `coverage.py`):**
   - Replaced `0.5 mm` midpoint discretization (`probe_blocked`) in `link_kept_lengths` and `belt_kept_lengths` with exact closed-form 1D interval-set algebra (`IntervalSet`, `segment_occluded_interval`, `disk_occluded_interval`, `face_occluded_set`), supporting arbitrary disconnected accessible regions with zero spatial rounding error.
   - Migrated `links_block_face` to `include_linkage_occlusion` resolved explicitly via `profile`:
     - `profile: study` (`scripts/study/baseline.yaml`, `base.yaml`) $\implies$ default `include_linkage_occlusion = False`.
     - `profile: prototype` (`scripts/optimize/linkage/config.yaml`) $\implies$ default `include_linkage_occlusion = True`.
   - Added `motion_robust_coverage` ($C_{\mathrm{robust}}$) and per-segment robust lengths (`proximal_robust_effective_mm`, `distal_robust_effective_mm`) across all mechanism evaluators alongside primary Endpoint Coverage (`pad_coverage`, $C_{\mathrm{endpoint}}$).

---

## 2. Why Each Change Was Necessary

- **Eliminating representation–optimization conflation:** Previously, canonical morphology parameters, relative search coordinates (`bp_x_mm`, `app_offset_mm`), and study-specific policies (`shallowest_supported`) lived side-by-side inside flat per-mechanism optimizer vectors, and `RCJ` lacked both a canonical mathematical definition and a `Joint` kinematic solver.
- **Eliminating discretization artifacts in coverage:** The legacy `0.5 mm` midpoint probe introduced up to $\pm 0.25\text{ mm}$ spatial quantization error. For example, on `belt_2r_matched` carrying `distal_30x3` at $\varphi = 90^\circ$, the $30\text{ mm} \times 3\text{ mm}$ distal pad reaches $4.9483\text{ mm}$ from joint $A$, while the $6\text{ mm}$ pulley protrudes in front of the $3\text{ mm}$-deep face up to $\sqrt{6^2 - 3^2} = 5.1962\text{ mm}$ from $A$ — an exact $0.24785\text{ mm}$ occlusion strip that the old $0.5\text{ mm}$ midpoint at $5.1983\text{ mm}$ missed by $2.15\;\mu\text{m}$.
- **Separating physical hardware occlusion from architectural comparison:** Setting `profile: prototype` default `True` enforces physical bar/face clearance for buildable CoDe fingers, whereas `profile: study` default `False` isolates intrinsic joint-kinematic fold accommodation across architectures while retaining `include_linkage_occlusion: true` for explicit sensitivity studies.

---

## 3. Mathematical Definitions Improved

| Topic | Previous State | Updated SSoT Formulation |
|---|---|---|
| **RCJ parameterization** | Marked *"미정"* in `docs/formulation.md`; geometry existed only inside `rcj/solids.py` | $\mathbf{p}_{\mathrm{mech}}^{\mathrm{RCJ}} = (T_A^{\{P\}}, r)$ ($\dim = 2$), closed-form $\mathbf{r}_B(\varphi)$, $\mathbf{r}_{\mathrm{ICR}}(\varphi)$, $k \equiv 2$ |
| **Contact placement** | Scalar axial offset $(\ell_i, h_i, o_i)$ in `docs/formulation.md` | Body-referenced $SE(2)$ transform $T_{c,i}^{\{b_i\}}(s_{c,i}, d_i) = \operatorname{Trans}(s_{c,i}, -d_i)$ on $[0, \ell_i] \times [-h_i, 0]$ |
| **Actuator orientation** | Implicit `servo_rear(tilt_deg)` sign in packaging helpers | Explicit local frame $\{A_i\}$ with $G_{\mathrm{D}}(r_s, e_{\mathrm{rear}})$ and $\operatorname{ori}(T_i) = -\psi_i$ |
| **CoDe transmission ratio** | Numerical finite difference along `sweep_servo` samples only | Exact analytical implicit loop-closure derivative $k(M, \varphi) = |\mathrm{d}\varphi/\mathrm{d}\theta_{\mathrm{act}}|$ verified against central FD to $< 10^{-7}$ |
| **Static pad–pin / roller clearance** | General polygon vertex-edge kernel loop | Exact closed-form $\operatorname{dist}(\mathbf{c}, [u_0, u_1] \times [v_0, v_1]) - r$ and point-to-$G_{\mathrm{D}}$ distance |
| **Dynamic collision screening** | Mixed with optimizer grid settings | Explicit $\Phi_{\mathrm{check}}(\varphi_{\min}^{\mathrm{des}}) = \{60^\circ + 30^\circ k \le \varphi_{\min}^{\mathrm{des}}\} \cup \{\varphi_{\min}^{\mathrm{des}}\}$, strictly distinguished from continuous verification |
| **Accessible contact face** | `0.5 mm` midpoint boolean mask (`probe_blocked`) | Exact 1D interval set $\mathcal{U}_i(M, \varphi; \eta_L) = \mathcal{U}_i^{\mathrm{base}} \setminus \eta_L \mathcal{O}_i^{\mathrm{link}} \subseteq [0, \ell_i]$ supporting disconnected intervals |

---

## 4. Legacy Features Migrated, Preserved, or Removed

| Legacy Item | Action | Rationale & Verification |
|---|---|---|
| `goal.loss.links_block_face` (YAML configs, evaluators, study scripts, tests) | **Migrated $\to$ `include_linkage_occlusion` + `profile`** | Active configs and scripts updated; `config_from_json` retains backward-compatible deserialization of `"links_block_face"` so all archived `design/output/**/run.json` files load cleanly |
| `probe_blocked` inside `link_kept_lengths` and `belt_kept_lengths` | **Replaced by exact `IntervalSet` projection** | Eliminates $0.5\text{ mm}$ discretization error; `probe_blocked` retained only in `contact/loss.py` for legacy `integrity_depth_mm` / `access_depth_mm` diagnostics |
| `DISTAL_DROP_MIN_MM` (`core/optimization/revolute/config.py`) | **Removed** | Unused dead constant superseded by `mount_offset_mm` / `shallowest_supported` |
| Informal draft `docs/formulation.md` | **Consolidated into pointer** | Eliminates contradictory `(l_i, h_i, o_i)` and *"RCJ 미정"* text; canonical definition lives solely in `design/SSoT/math_formulation.md` |
| `DEPRECATED_VARIABLES` (`pip_Stopper`, carrier heights, chamfers) in `config_from_json` | **Retained at JSON boundary only** | Required to deserialize historical October–November 2026 `run.json` portfolios without mutating archived outputs |

---

## 5. Test & Empirical Coverage Results

### 5.1 Automated Verification Suite

- **Total test suite:** **418 passed, 0 failed** (389 updated regression/study tests + 29 new SSoT mathematical verification tests across `test_ssot_parameterization.py`, `test_ssot_collision.py`, and `test_ssot_coverage.py`), verified identically from both repository root and `design/`.
- **Independent mathematical cross-checks verified:**
  1. $SE(2)$ group axioms (identity, inverse, associativity, $3\times 3$ matrix homomorphism) and flexion sign $R_{\mathrm{flex}}(\pi/2)[1,0]^\top = [0,-1]^\top$.
  2. Bijection between `CoDe` 8D extended-pivot chart and intrinsic bar-length chart $(\ell_{AB'}, \ell_{A'B}, \sigma_{\mathrm{branch}})$, plus flat-vector $\leftrightarrow$ `FingerMorphology` round-trips across all architectures.
  3. Exact analytical `CoDe` transmission ratio $k(M, \varphi)$ vs. central finite differences ($\text{atol} = 10^{-7}$), and analytical `RCJ` rolling kinematics vs. independent trigonometry ($\text{atol} = 10^{-12}$).
  4. Analytical `rect_disk_clearance_mm` and `disk_servo_clearance_mm` vs. independent dense boundary sampling ($4{,}000$ samples/edge), intra-body configuration invariance across arbitrary $(q_{\mathrm{MCP}}, \varphi)$, and exact touching ($c_{pd} = 0$) vs. overlapping ($c_{pd} < 0$) dynamic boundaries.
  5. Disconnected 3-interval `IntervalSet` measure vs. $400{,}000$-point Riemann midpoint quadrature ($\text{atol} = 2\times 10^{-4}\text{ mm}$), exact equality $\eta_L = 1 \equiv \eta_L = 0$ on unoccluded faces, monotonicity $C(\eta_L=1) \le C(\eta_L=0)$, and strict orthogonality to collision/ROM constraints.

### 5.2 Empirical Investigation: Endpoint vs. Motion-Robust Coverage

Evaluating $C_{\mathrm{endpoint}}$ and $C_{\mathrm{robust}}$ across archived Pareto fronts in `design/output/study/` and `design/output/linkage/` established two distinct regimes:

1. **Universal inequality:** $C_{\mathrm{robust}}(M; \eta_L) \le C_{\mathrm{endpoint}}(M; \eta_L)$ held across **100%** of evaluated Pareto designs (`min(C_endpoint - C_robust) >= 0`).
2. **Equal pad depths ($D_p = D_d$):** On `2r_dd_tilt90` ($D_p = D_d = 3\text{ mm}$), $C_{\mathrm{robust}} = C_{\mathrm{endpoint}}$ identically ($r = 1.000$). On Prototype aligned-offset fronts (`config_pivot8/search`, `align_mount_offsets: true`), Pearson $r = 0.961\text{–}0.983$ and Spearman $\rho = 0.998\text{–}1.000$.
3. **Unequal pad depths ($D_p \neq D_d$):** On `2r_dd_tilt45` ($r = -0.108, \rho = 0.240$, $\max(C_{\mathrm{end}} - C_{\mathrm{rob}}) = 0.401$) and unaligned `CoDe` fronts (`config/search` $\eta_L=1$: $r = -0.322, \rho = 0.328$), $C_{\mathrm{robust}}$ diverges sharply from $C_{\mathrm{endpoint}}$ because infinite-line V-crossing between non-collinear face lines at $D_p > D_d$ sends the line intersection $x_{\times}(\varphi) \approx x_{\mathrm{hinge}} + (D_p - D_d)/\sin\varphi \to +\infty$ past the fingertip as $\varphi \to 0^+$. Thus correlation between $C_{\mathrm{endpoint}}$ and $C_{\mathrm{robust}}$ is **not** a mathematical guarantee when $D_p \neq D_d$.

---

## 6. Remaining Scientific Uncertainties

1. **Infinite-line V-crossing vs. finite opposing-pad fold geometry at $D_p \neq D_d$:** While infinite-line V-crossing (`fold_mode="crossing"`) is well-behaved at large endpoint flexions ($\varphi_{\min}^{\mathrm{des}} \in [90^\circ, 150^\circ]$), its small-angle ($\varphi \to 0^+$) behavior at $D_p \neq D_d$ penalizes $C_{\mathrm{robust}}$ even when the opposing physical pad is far away.
2. **Planar vs. 3D lateral linkage occlusion:** In 2D projection, $\eta_L \in \{0, 1\}$ treats crossing links as either non-occluding ($\eta_L = 0$) or full-width occluding ($\eta_L = 1$); actual 3D grasp contact depends on object curvature relative to the lateral $Y$-offset of the bars.
3. **Unmodeled mechanical compliance & RCJ tether mechanics:** Planar kinematics idealize rigid bars, zero bearing clearance, and zero-thickness no-slip RCJ rolling bands.

---

## 7. Recommended Next Research Steps

1. **Study Reruns Under Explicit $\eta_L = 0$ Baseline + $\eta_L = 1$ Sensitivity Pair:** When generating final camera-ready Study 1 & Study 2 tables, report `profile: study` ($\eta_L = 0$) alongside `goal_overrides: {include_linkage_occlusion: true}` ($\eta_L = 1$) into dedicated output directories so architectural fold capacity and transmission-occlusion penalties are cleanly decomposed.
2. **Finite-Object Contact Validation in `sim/isaaclab/`:** Compare planar $C_{\mathrm{endpoint}}(\eta_L \in \{0,1\})$ and `fold_mode="segment"` against 3D multi-object grasp contact area and wrench metrics in Isaac Lab (`GOAL.md` Studies A–D).
3. **Dense-Grid Verification Gate on Candidate Selection:** Continue pairing discrete checkpoint / $1^\circ$-grid optimization searches with `verify_step_deg: 0.05` (`verify_dynamic_collision_refined`) before freezing any hardware or benchmark seed.
