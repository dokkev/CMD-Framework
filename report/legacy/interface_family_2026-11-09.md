# Fixed-H interface-family accommodation — independent experiment

> **Recheck 2026-11-17.** A fold-rounding error (±0.25 mm usable length, CoDe and belt 2R) was fixed. The dense and fold-off results (C7) are unchanged. In Result 1, CoDe drops to 131/114/95 and belt 2R to 108/77/24 (22 sampled). In Result 3, CoDe drops to 299/236/152 and belt 2R to 247/186/90. Corrected tables: [recheck_fold_rounding_2026-11-17.md](recheck_fold_rounding_2026-11-17.md).

2026-11-09. [GOAL.md](../GOAL.md) Sections 5.2–5.3 and 5.7에 대응한다.

## Question and independence

**한 번 고정한 skeleton/actuation에서 proximal/distal 모듈을 독립적으로 바꿀 때, 얼마나 많은 조합을 유지할 수 있는가?** 기존 Study 2의 pad별 재설계 성공 개수와 다른 질문이다. 기존 catalog의 one-sided/diagonal 변화가 놓치는 off-diagonal 두께 조합을 검사한다.

실행 전 격자를 선언했고, 독립 에이전트가 읽기 전용으로 설계·구현을 검토하고 원본 CSV의 집계와 선택 geometry를 재검사했다. 기존 mechanism evaluator를 재사용하므로 새로운 독립 실험 프로토콜과 독립 검토이지, 독립적인 물리 모델 또는 hardware validation은 아니다. CoDe가 불리한 길이 조합과 model-rule sensitivity도 포함한다.

## Protocol

- Proximal/distal 길이 쌍(mm): (15,15), (20,20), (25,25), (30,30), (15,30), (30,15).
- 각 side의 두께(mm): 1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 18, 20. 독립 Cartesian product 144개/길이 쌍, 총 864개 interface 조합.
- 비교: CoDe15°, DD0°/90°, Belt15°, physical RCJ0°/90°; ideal RCJ는 별도 이상화 비교군.
- Primary reference: 이전 ROM-conditioned pilot이 baseline pad만 보고 선택한 90/120/150° baseline. Interface별 최적화나 tilt 선택은 하지 않는다. Original nominal reference는 별도 control로 저장한다.
- 목표 자세 판정과 sampled-posture conjunction을 구분한다. 첫 실험의 sampled postures는 15/30/60/90/120/150° 중 목표 ROM 이하의 자세다.
- 검사한 각 posture의 face-use 조건과 기존 motion collision/ROM 조건을 모두 통과해야 한다. Collision sweep의 원래 resolution을 개선한 실험은 아니다.
- 장착 깊이는 shallowest-supported 정책으로 interface별 재계산한다. 따라서 일반적인 claim은 fixed skeleton/actuation + interface-dependent seating이다. 모든 mounting reference가 자동으로 고정되는 것은 아니다.
- CoDe angle30–150°, hyperextension10°, 직접적인 k_max cap 없음. 실제 envelope 비관통과 기본 link/face 검사는 유지한다.
- 원본 62,208개 evaluation row는 nominal 및 변경된 ROM baseline variants를 포함한다. Raw-identical baseline은 module/ROM override 후 config 동등성을 확인하여 nominal evaluation을 재사용한다. 이 row 수를 독립 실험 반복 횟수로 세지 않는다.

## Result 1 — an overlooked fixed-H thickness-family advantage

길이 20/20 mm에서 지원한 두께 조합 수(/144):

| Reference | 90° endpoint | 120° endpoint | 150° endpoint | ≤150°의 초기 6개 sampled poses 모두 통과 |
|---|---:|---:|---:|---:|
| CoDe15° | 132 | 117 | 95 | 54 |
| DD0° | 108 | 48 | 23 | 20 |
| DD90° | 108 | 77 | 24 | 22 |
| Belt15° | 120 | 82 | 25 | 23 |
| Physical RCJ0° | 84 | 38 | 19 | 13 |
| Physical RCJ90° | 132 | 103 | 63 | 22 |
| Ideal RCJ | 120 | 96 | 50 | 36 |

**현재 reference에서는 CoDe가 20 mm 모듈의 두께 조합을 더 많이 수용했다.** 특히 120/150°에서 endpoint 기준과 초기 sampled-posture 기준 모두 물리적 비교군보다 많았다. 기존 8개 target의 재설계 성공 수에서는 이 조합별 fixed-H 능력이 드러나지 않았다.

![20 mm thickness maps](../design/output/study/interface_family_2026-11-09/thickness_maps.png)

각 격자 칸은 선언한 두께 값의 조합이다. 두께 간격은 균일하지 않으며 그림의 색 면적을 연속 parameter-space volume으로 해석하지 않는다. 회색=endpoint 실패, 주황=endpoint만 통과, 청록=초기 sampled-posture conjunction 통과. 숫자는 endpoint 통과 수이다.

## Result 2 — posture refinement and fold-rule sensitivity

독립 재검사에서 기존 fold-use 판정의 중요한 영향을 확인했다. Near-extension에서 깊이가 다른 두 face의 **연장 직선**이 교차하면 finite pad가 실제 충돌하지 않아도 사용 길이를 제한할 수 있다. 예를 들어 CoDe20 mm의 proximal3/distal10 mm는 15°에서 pad_usable_mm 조건을 위반하고, 10/10 mm는 통과한다. 해당3/10 후보는 links_block_face를 꺼도 실패하며 count_fold를 끄면 통과한다.

이를 확인한 뒤 두 가지 **post-hoc validation**을 모든 7개 reference와 전체144개 두께 쌍에 동일하게 수행했다. 모두 frozen150° baseline을 사용하며 재최적화하지 않았다.

1. **Dense:** 5°부터150°까지5° 간격, 30개 sampled postures의 conjunction.
2. **Fold-off control:** 같은 dense 검사에서 count_fold만 False. 실제 collision 및 나머지 제약은 유지한다. Primary result가 아니라 model-rule sensitivity 진단이다.

| Reference | Dense, 기본 규칙 /144 | Dense, fold-off control /144 |
|---|---:|---:|
| CoDe15° | 28 | 106 |
| DD0° | 12 | 26 |
| DD90° | 13 | 55 |
| Belt15° | 13 | 55 |
| Physical RCJ0° | 6 | 42 |
| Physical RCJ90° | 6 | 88 |
| Ideal RCJ | 20 | 81 |

각 validation은30,240개 evaluation을 저장했다. **CoDe의 지원 조합 수 우위는 더 촘촘한 자세 검사와 fold-off 대조군에서도 유지됐다. 그러나 절대 수용 범위는 자세 sampling과 fold 모델에 매우 민감했다.** 이 민감도는 핵심 결과이며 작은 보정 사항이 아니다.

![Posture and fold sensitivity](../design/output/study/interface_family_2026-11-09/posture_sensitivity.png)

초기 6개 자세에서는 CoDe의 각 side 두께 집합 {3,4,5,6,8} mm의25개 조합이 모두 통과했다. **이 전체 family는 dense 검사에서는 유지되지 않았다.** Dense 기본 규칙에서3 mm를 시작점으로 한 최대 sampled rectangle은 proximal{3,4,5,6} × distal{3,4} mm의8개 조합이다. 각 side 공통 square는{3,4} mm로, DD90/Belt와 상한이 같다. 그러므로 “모든 방향에서 두께 범위가 더 크다”라고 쓰지 않는다.

Fold-off에서는 CoDe의 anchored sampled rectangle이 proximal{3,4,5,6,8,10,12} × distal{3,4,5,6,8,10} mm의42개 조합이다. 이 결과 역시 연속적인3–12/3–10 mm 범위의 증명이 아니다.

독립 geometry readback에서 CoDe의 초기3..8 family는 장착 깊이도 proximal0/distal1.1 mm로 동일했다. 하지만 이것을 전체 격자와 모든 비교군에 일반화하지 않는다.

## Result 3 — length and asymmetry limits remain

전체 6개 길이 strata의 초기 sampled-posture 통과 조합 합계(/864):

| Reference | 90° | 120° | 150° |
|---|---:|---:|---:|
| CoDe15° | 301 | 238 | 154 |
| DD0° | 150 | 99 | 74 |
| DD90° | 220 | 180 | 90 |
| Belt15° | 273 | 194 | 93 |
| Physical RCJ0° | 131 | 79 | 34 |
| Physical RCJ90° | 166 | 138 | 108 |
| Ideal RCJ | 291 | 229 | 162 |

이는 선언한 격자의 case count이며 sampling distribution에 의존한다. CoDe가 모든 stratum에서 우월한 것은 아니다. 예를 들어 길이30/15 mm, ROM150°에서는 CoDe0개, DD90°23개, Belt25개, physical RCJ90°19개가 초기 sampled-posture 조건을 통과한다. 30/30 mm,150°는 모든 physical reference가0개였다. 이 실험의 길이 상한은30 mm이며 기존35–45 mm proximal-heavy target의 제한을 해소하거나 반박하지 않는다.

![Length sensitivity](../design/output/study/interface_family_2026-11-09/length_sensitivity.png)

## Safe claim and remaining evidence gap

> 평가한 ROM-conditioned reference와 선언한 길이·두께 격자에서 CoDe는 고정 skeleton/actuation으로 지원하는 interface 조합 수가 물리적 비교군보다 많았다. 길이20 mm·목표150°의 상대적 이점은5° 간격 자세 검사와 fold-use 규칙을 제거한 대조군에서도 유지됐다. 이 결과는 fixed-H interface-family accommodation의 조건부 계산적 근거이며, 모든 길이·형상 또는 실제 manipulation 접근성에 대한 우위는 아니다.

기존의 “특정 thick pad 하나를 유지한다”보다 넓은 조합을 확인했지만, **physical swap 독립성·개발 비용·실험 반복성·센서 통합·조작 성능은 아직 측정하지 않았다.** 별도 load/contact 모델이나 hardware 실험의 독립 검증도 아니다. 연속적인 두께 범위와 continuous-motion usable-face 보장은 하지 않는다. Near-extension fold 모델의 실제 유효성을 먼저 확인해야 한다.

## Reproduction and provenance

From `design/`, using the `cmd` environment:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python scripts/study/study1_fixed_morphology/interface_family.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python scripts/study/study1_fixed_morphology/validate_interface_family.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python scripts/study/study1_fixed_morphology/validate_interface_family.py --fold-off
python scripts/study/study1_fixed_morphology/summarize_interface_family.py
```

Runners refuse to overwrite existing output directories. Config: [interface_family.yaml](../design/scripts/study/study1_fixed_morphology/interface_family.yaml).

Artifacts under `design/output/study/interface_family_2026-11-09/`:

- `manifest.json`: declared domain, evaluation count, source commit/patch/untracked snapshot, baseline input hashes, result hash.
- `inputs/rom_*/…/baseline.json`: exact reused baseline geometries/configs. `references.csv` records evaluated variants.
- `assembly_results.csv`: all62,208 primary/nominal-control rows, seating depths and failed constraints.
- `summary.csv`, `failure_counts.csv`: conditional counts, sampled families and endpoint failure breakdown.
- `dense_postures/`, `dense_fold_off/`: separate manifests, source snapshots and30,240 raw rows each.
- `dense_summary.csv`: dense family/count results. `analysis_manifest.json` hashes all three input CSVs and saves analysis source provenance.
- `thickness_maps`, `length_sensitivity`, `posture_sensitivity`: PNG/PDF figures read only saved results.

Independent review recomputed primary counts/families from raw rows and reevaluated selected CoDe, DD and RCJ cases. Independent review is not an additional physical replicate. The output directory is not git-tracked; publishing the report requires separately retaining these artifacts.
