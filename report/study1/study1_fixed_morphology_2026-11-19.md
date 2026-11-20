# Study 1: fixed morphology, rerun under the SSoT (2026-11-19)

**Track: study** (`design/scripts/study/`). Formulation: `design/SSoT/problem_formulation.md` A.6.2.

## Setup

| Item | Value |
|---|---|
| Config | `design/scripts/study/study1_fixed_morphology/config.yaml` |
| References | `references_nominal15.yaml`, kind `matched`: one frozen morphology ($L_f(0)=90$ mm), each architecture built to it |
| Modules | 71 contact-module pairs (`modules.yaml`: named, proximal-only, distal-only and combined groups) |
| $q_{\mathrm{req}}$ | 90°, 120°, 150° |
| PIP servo tilt sweep | 0°–90° in 15° steps, for CoDe, 2R-DD, 2R-remote and RCJ; the idealized RCJ at its nominal |
| Rules | $MA_{\min}=0.2$ (CFL only), $\gamma\in[30^\circ,150^\circ]$, $\bar C_{\mathrm{pad}}$ over $\{90^\circ,\tfrac{90^\circ+q_{\mathrm{req}}}{2},q_{\mathrm{req}}\}$ |
| Output | `design/output/study/study1_fixed_morphology_tilt_sweep/assembly_results.csv` (6177 assemblies) |

Nothing is optimized: each assembly is evaluated once and a failure is reported, not redesigned.

## Feasible modules at each architecture's nominal tilt

Each cell shows feasible / mounted modules out of 71, then the median and the best $\bar C_{\mathrm{pad}}$ of the
feasible ones.

| Architecture (tilt) | $q_{\mathrm{req}}=90^\circ$ | $120^\circ$ | $150^\circ$ |
|---|---|---|---|
| CoDe (15°) | 38/38 · 0.372 · 0.576 | 37/38 · 0.377 · 0.587 | 36/38 · 0.386 · 0.598 |
| 2R-DD (0°) | 32/32 · 0.201 · 0.334 | 23/32 · 0.191 · 0.313 | 0/32 · – · – |
| 2R-remote (15°) | 38/38 · 0.241 · 0.450 | 38/38 · 0.253 · 0.457 | 32/38 · 0.239 · 0.463 |
| RCJ (0°) | 32/38 · 0.168 · 0.387 | 30/38 · 0.176 · 0.393 | 19/38 · 0.144 · 0.289 |
| RCJ, idealized (upper bound) | 38/38 · 0.260 · 0.494 | 38/38 · 0.252 · 0.487 | 36/38 · 0.246 · 0.483 |

33 of the 71 modules mount on no architecture: the mounting policy needs a spacer for them, and a spacer is not
allowed. 2R-DD mounts 6 fewer than the others.

## Paired comparison

Median $\bar C_{\mathrm{pad}}$ over the modules feasible on every real architecture at that $q_{\mathrm{req}}$:

| $q_{\mathrm{req}}$ | Modules | CoDe | 2R-DD | 2R-remote | RCJ | RCJ, idealized |
|---|---|---|---|---|---|---|
| 90° | 32 | **0.370** | 0.201 | 0.218 | 0.168 | 0.214 |
| 120° | 23 | **0.351** | 0.191 | 0.221 | 0.176 | 0.216 |
| 150° | 19 (without 2R-DD) | **0.341** | – | 0.224 | 0.144 | 0.218 |

On the same modules, CoDe covers 1.5–2.4 times as much as each other architecture, and more than the idealized RCJ.

## Tilt

Feasible modules at $q_{\mathrm{req}}=150^\circ$ by PIP servo tilt:

| Architecture | 0° | 15° | 30° | 45° | 60° | 75° | 90° |
|---|---|---|---|---|---|---|---|
| CoDe | 0 | 36 | 36 | 36 | 36 | 36 | 36 |
| 2R-DD | 0 | 4 | 21 | 23 | 28 | 28 | 28 |
| 2R-remote | 32 | 32 | 32 | 32 | 32 | 32 | 32 |
| RCJ | 19 | 21 | 24 | 24 | 24 | 24 | 24 |

- CoDe fails every module at 0° on static checks (`static_pad_collision_mm`, `static_pivot_pin_mm`). From 15° up
  the results do not change with the tilt: CoDe's motion, $MA$ and coverage do not depend on the servo body's
  angle once it clears.
- 2R-DD reaches 150° with more modules as the servo is laid back (`f_motion_deg` failures at small tilts).
- The 2R-remote drive sits away from the joint, so its tilt changes nothing.

## Failures

The mounted assemblies that are not feasible fail these constraints. The counts cover every tilt and
$q_{\mathrm{req}}$, and an assembly may fail several.

| Mechanism | Constraint | Count |
|---|---|---|
| CoDe | `f_motion_deg`, `ma_min` | 132 each |
| CoDe | `static_pad_collision_mm`, `static_pivot_pin_mm`, `gamma_range_deg` | 114 each (tilt 0°) |
| RCJ | `f_motion_deg` | 194 |
| 2R-DD | `f_motion_deg` | 107 |
| 2R-remote | `f_motion_deg` | 42 |
| RCJ, idealized | `f_motion_deg` | 2 |
