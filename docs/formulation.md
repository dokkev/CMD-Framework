# Mathematical Formulation Index

> **Canonical Source of Truth.** The complete, publication-ready 2D mathematical Source of Truth (SSoT) for CoDe Hand finger morphology, parameter dependencies, derived properties, static/dynamic collision, set-based pad coverage, and optimization mappings lives in:
>
> - **[design/SSoT/math_formulation.md](../design/SSoT/math_formulation.md)**

To prevent competing or inconsistent definitions across documentation files, all equations, frame conventions, architecture parameterizations ($m \in \{\mathrm{CoDe}, \mathrm{2R}, \mathrm{2R\text{-}remote}, \mathrm{RCJ}\}$), collision rules, and coverage definitions are maintained exclusively in [design/SSoT/math_formulation.md](../design/SSoT/math_formulation.md).

## Quick Summary of the SSoT Structure

1. **Canonical Morphology (Layer 1) & Admissible Space (Layer 2):**
   $$
   M = (\mathbf{p}_{\mathrm{mech}},\, \mathbf{p}_{\mathrm{act}},\, \mathbf{p}_{\mathrm{struct}},\, \mathbf{p}_{\mathrm{contact}}) \in \mathcal{M}(m),
   \qquad m \in \{\mathrm{CoDe},\, \mathrm{2R},\, \mathrm{2R\text{-}remote},\, \mathrm{RCJ}\}.
   $$
2. **Derived Properties (Layer 3):**
   Common external finger geometry $\mathbf{L}(M) = [L_f, L_p, L_d]^\top$, forward kinematics $T_{PD}(\varphi)$, instantaneous center of rotation $\mathbf{r}_{\mathrm{ICR}}(\varphi)$, analytical transmission ratio $k(M, \varphi)$, and Study 2 partition $H = (E, Q, A)$.
3. **Design Requirements, Collision & Pad Coverage (Layer 4):**
   - Analytical static clearances vs. discrete checkpoint dynamic screening $\Phi_{\mathrm{check}}(\varphi_{\min}^{\mathrm{des}})$ vs. refined verification.
   - Exact 1D interval-set accessible contact face $\mathcal{U}_i(M, \varphi; \eta_L) \subseteq [0, \ell_i]$, Endpoint Coverage $C_{\mathrm{endpoint}}$, and Motion-Robust Coverage $C_{\mathrm{robust}}$.
   - Explicit context profile defaults (`profile: study` $\implies$ `include_linkage_occlusion = False`; `profile: prototype` $\implies$ `include_linkage_occlusion = True`).
4. **Implementation Mapping (Layer 5):**
   Slot-by-slot correspondence between flat solver `VARIABLE_NAMES` and `FingerMorphology` (`design/core/optimization/canonical_mapping.py`).
