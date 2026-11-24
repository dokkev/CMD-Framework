# Study 1: fixed morphology, native references (2026-11-20)

**Track: study** (`design/scripts/study/`). Formulation: `design/SSoT/problem_formulation.md` A.6.1–A.6.2.

목적함수는 원기둥 coverage $\bar C_{\mathrm{pad}}$ 입니다. 이 report의 그림과 표는 직관적인 비교를 위해
**pad coverage** $(\ell_p+\ell_d)/L_f$ (펴진 손가락 길이 중 pad가 차지하는 비율)로 보여 줍니다.

## Setup

| Item | Value |
|---|---|
| References | `references_native.yaml`: 6 architectures × $q_{\mathrm{req}}\in\{90,120,150\}$ = 18 native designs |
| Native search | `scripts/study/<f>.yaml` + `base.yaml`, preset `search` (GA + NSGA-II, seeds 0–2), objective $\bar C_{\mathrm{pad}}$ only, $MA_{\min}$ null, $L_f\in[89.95, 90.05]$ mm, pads 5 mm thick, tilt $\in[0^\circ,45^\circ]$ (CFL ideal 15° fixed) |
| Rules changed on 2026-11-20 | CFL의 A pin housing과 MCP servo의 충돌 검사 추가 (SSoT A.4대로; CFL q90, CFL ideal q90·q120 재search). 2R-remote의 PIP servo 자세 고정: $A' = A + (-10, 15)$ mm, tilt 15° (search된 설계에 고정 자세를 넣음: feasible, coverage 같음) |
| Evaluation | each reference at its own $q_{\mathrm{req}}$ only |
| Modules | 71 pairs (`modules.yaml`): combined / proximal-only / distal-only, 15–45 mm × 3/5/8 mm, each 21; named 8 |
| Tilt sweep | 12 physical references (CFL, 2R-remote, 2R-DD, RCJ) held at 0/15/30/45°, rest of the design unchanged |
| Output | `design/output/study/study1_fixed_morphology_native/assembly_results.csv` (4686 assemblies, 66 references) |

*Native designs of each architecture for $q_{\mathrm{req}}$ = 90°, 120° and 150° ($L_f$ = 90 mm, 5 mm pads), each
drawn at its $q_{\mathrm{req}}$; dotted: the distal segment's centre of rotation from extension. (a) The designs, with
their pad coverage. (b) Pad coverage against $q_{\mathrm{req}}$. \*Upper bounds without the joint (PIP) servo body.*

## Native designs (balanced seed)

목적이 하나라 representative seed는 `most_coverage;balanced` 하나입니다.

| $q_{\mathrm{req}}$ | Architecture | proximal + distal pad (mm) | pad coverage | $\bar C_{\mathrm{pad}}$ | $q_{\max}$ | $MA_{wc}$ | tilt | stop | bound hits |
|---|---|---|---|---|---|---|---|---|---|
| 90° | CFL | 32.5 + 32.5 | 0.722 | 0.757 | 149.6° | 0.56 | 5° | transmission | – |
| | 2R-remote | 35.3 + 35.2 | **0.783** | 0.809 | 90.0° | 1.00 | 15° (fixed) | collision | drops low |
| | 2R-DD | 31.2 + 31.2 | 0.693 | 0.726 | 90.0° | 1.00 | **45°** | collision | tilt high |
| | RCJ | 29.6 + 29.7 | 0.659 | 0.663 | 90.3° | 0.50 | 15° | collision | roller_radius low |
| | CFL ideal* | 36.4 + 36.4 | 0.809 | 0.778 | 138.8° | 0.51 | – | tip | – |
| | RCJ ideal* | 35.0 + 35.0 | 0.778 | 0.787 | 90.8° | 0.50 | – | collision | roller_radius low |
| 120° | CFL | 33.3 + 33.4 | **0.741** | 0.748 | 180° | 0.55 | **45°** | curl | tilt high |
| | 2R-remote | 31.6 + 31.5 | 0.701 | 0.758 | 120.2° | 1.00 | 15° (fixed) | collision | drops low |
| | 2R-DD | 28.8 + 29.0 | 0.642 | 0.700 | 120.1° | 1.00 | **45°** | collision | tilt high |
| | RCJ | 26.4 + 26.3 | 0.586 | 0.608 | 120.1° | 0.50 | 22° | collision | roller_radius low |
| | CFL ideal* | 35.4 + 35.5 | 0.788 | 0.758 | 180° | 0.42 | – | curl | – |
| | RCJ ideal* | 34.3 + 34.2 | 0.761 | 0.781 | 123.9° | 0.50 | – | collision | roller_radius, drops low |
| 150° | CFL | 34.6 + 34.6 | **0.769** | 0.736 | 180° | 0.45 | 40° | curl | – |
| | 2R-remote | 21.6 + 25.0 | 0.518 | 0.593 | 150.1° | 1.00 | 15° (fixed) | collision | drops low |
| | 2R-DD | 21.6 + 25.0 | 0.518 | 0.593 | 150.1° | 1.00 | 44° | collision | drops low |
| | RCJ | 23.3 + 25.0 | 0.537 | 0.560 | 150.1° | 0.50 | 44° | collision | roller_radius low |
| | CFL ideal* | 35.2 + 34.9 | 0.779 | 0.736 | 180° | 0.60 | – | curl | – |
| | RCJ ideal* | 32.5 + 32.4 | 0.721 | 0.755 | 150.1° | 0.50 | – | collision | drops low |

\* 이상화 상한 (joint/PIP servo body 없음, tilt 없음).

- 실제 구조 중 pad coverage 1위는 90°에서 **2R-remote** (0.783 vs CFL 0.722), 120°·150°에서 **CFL**
  (0.741, 0.769)입니다. CFL q90은 A pin이 MCP servo 밖에 있어야 하면서 A가 $x$ = 5.9 → 23.9 mm로 밀려났고,
  pad가 36 → 32.5 mm로 줄었습니다.
- q_req가 커질수록 2R과 RCJ는 pad가 급격히 줄고 150°에서 distal pad가 최소 지지 길이 25 mm에 붙습니다.
  CFL은 오히려 조금 늘어납니다 (moving ICR: 120°·150° 설계는 $q_{\max}$ = 180°까지 curl).
- 150°의 2R-DD와 2R-remote는 같은 관절·pad 설계에 수렴했습니다 (pad 충돌이 지배, servo는 묶이지 않음).
- CFL(120°)과 2R-DD(90°, 120°)는 tilt가 상한 45°에 붙었습니다.

## Acceptance (각 reference 자기 tilt)

각 칸: 받아들인 module 수 (combined / proximal / distal, 각 21개 중), 그리고 양쪽 pad에 같은 module을 달 때
받아들인 최대 pad coverage (가장 긴 module × 그 길이에서 가장 두꺼운 것).

| Architecture | 90° | 120° | 150° |
|---|---|---|---|
| CFL | 11/12/12 · 0.67 (30×5) | 11/12/12 · **0.67** (30×5) | 12/12/12 · **0.67** (30×8) |
| 2R-remote | 14/15/15 · **0.78** (35×5) | 11/12/12 · **0.67** (30×5) | 4/6/8 · 0.44 (20×5) |
| 2R-DD | 11/12/12 · 0.67 (30×5) | 9/9/9 · 0.56 (25×8) | 4/6/8 · 0.44 (20×5) |
| RCJ | 9/9/9 · 0.56 (25×8) | 8/9/9 · 0.56 (25×5) | 6/6/9 · 0.44 (20×8) |
| CFL ideal* | 14/15/15 · 0.78 (35×5) | 14/15/15 · 0.78 (35×5) | 12/15/12 · 0.67 (30×8) |
| RCJ ideal* | 14/15/15 · 0.78 (35×5) | 11/12/12 · 0.67 (30×5) | 10/12/12 · 0.67 (30×5) |

- 90°에서는 2R-remote가 35 mm module까지 받고 CFL은 30 mm까지입니다. 120°에서는 둘이 같고, 150°에서는 CFL만
  30 mm module을 유지합니다 (2R 20 mm, RCJ 20 mm).
- 실패 이유는 대부분 장착 단계입니다: module이 phalanx보다 길거나 (`bounds_*_pad_mm`), 2R은 관절 hardware 때문에
  장착 깊이가 없습니다 (`no_depth`, 2R-DD 22개, 2R-remote 14개). 운동 단계 실패(`f_motion_deg`)는 150°의 2R에서
  4개, 나머지는 0–2개입니다.
- 5 mm grid라 30 / 35 mm 경계에서 동률이 많습니다. 차이를 더 보려면 길이 간격을 2.5 mm로 줄여야 합니다.

## Tilt sensitivity (physical references)

받아들인 module 수 (combined + proximal + distal, 63개 중):

| Reference | own | 0° | 15° | 30° | 45° |
|---|---|---|---|---|---|
| CFL q90 (own 5°) | 35 | **0** | 35 | 35 | 35 |
| CFL q120 (own 45°) | 35 | **0** | **0** | 35 | 35 |
| CFL q150 (own 40°) | 36 | **0** | **0** | 36 | 36 |
| 2R-remote q90 / q120 / q150 (own 15°) | 44 / 35 / 18 | 같음 | 같음 | 같음 | 같음 |
| 2R-DD q120 (own 45°) | 27 | 24 | 26 | 11 | 27 |
| 2R-DD q150 (own 44°) | 18 | **0** | 10 | 18 | 18 |
| RCJ q90 / q120 / q150 | 27 / 26 / 21 | 27 / 26 / 20 | 27 / 26 / 20 | 27 / 26 / 21 | 27 / 26 / 21 |

- CFL이 0이 되는 tilt는 모두 `static_pad_collision_mm`입니다: 이 native 설계를 그대로 두고 servo만 돌리면
  servo 몸체가 pad와 겹칩니다. 다른 tilt는 그 tilt의 native 설계가 아니므로, 0은 "그 tilt가 불가능"이 아니라
  "이 설계에서 servo만 돌리면 안 맞는다"는 뜻입니다.
- 2R-remote와 RCJ는 tilt에 거의 둔감합니다. 2R-DD는 PIP servo가 관절에 있어 tilt에 민감합니다.

## Notes

- Native 설계는 원기둥 $\bar C_{\mathrm{pad}}$를 최대화한 것이므로 pad coverage 최대 설계는 아닙니다.
- Pad 두께는 5 mm로 고정해 찾았습니다. 3 / 8 / 10 mm native search는 진행 중입니다 (2026-11-20 큐).
- 가림(링크, belt)은 SSoT대로 보지 않습니다 (서로 다른 lateral layer). 민감도는 따로 봅니다.
- 이전 matched run (`study1_fixed_morphology_2026-11-19.md`)과는 reference가 달라 숫자를 직접 비교할 수 없습니다.
- 규칙 변경 전 결과는 `design/output/legacy/study1_fixed_morphology_native_pre_a_mcp_2026-11-20/` (추적 안 함).
- 18 references 모두 `shared.references` 로딩과 `check_comparable` (설계 $q_{\mathrm{req}}$별 그룹)을 통과했습니다.
