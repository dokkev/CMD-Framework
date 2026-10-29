# 메커니즘 비교 study — bound 분류 (Task 2)

> **상태:** Checkpoint 2 보고 (2026-10-29). formulation: linkage-v5, revolute-goal-v5,
> revolute-idealized-goal-v4, revolute-parallel-goal-v4, rcj-goal-v3.
> **설정:** `scripts/optimize/comparison/base.yaml`, 생성 `make_configs.py`.
> **bound hit 근거:** `output/comparison_quick` (seed 1 × 300세대, 이전 formulation, 수렴 전). 분류 근거로만 쓰며 결과로 인용하지 않는다.

## 원칙

- **Requirement**: 비교 조건 정의. 논문에 근거와 함께 명시. binding되어도 된다.
- **Hardware**: CAD·부품에서 나온 물리 한계(mm). binding되어도 된다.
- **Search box**: 탐색 범위. binding되면 안 된다. front 설계의 bound hit이 0%에 가까워야 한다.

비율은 모두 extension 자세 |OC| 대비. A_z = B_z = 0이므로 extension에서 OA + AB + BC = |OC| (테스트 `test_the_extended_lengths_add_up_to_the_finger`). 2R 계열은 AB = 0, RCJ는 AB = 2r, linkage는 four-bar span.

## 분류표

hit = quick run best 설계 중 그 bound에 붙은 칸 수 / 설계가 나온 칸 수.

| 키 | 값 | 종류 | 근거 | 대상 | hit |
|---|---|---|---|---|---|
| `goal.link.finger_length_mm` | [99.5, 100] | Requirement | 비교 기준 길이 | 전체 | 상한 2R 8/8, belt 7/8, ideal 8/8, RCJ 9/9 |
| `goal.link.proximal_length_ratio` | [0.35, 0.55] | Requirement | 사람 손가락 분절 비 범위 | 전체 | – |
| `goal.link.distal_length_ratio` | [0.25, 0.55] | Requirement | 같음 | 전체 | – |
| `goal.link.distal_length_mm` | ≥ 25 | Hardware | distal carrier + fingertip | 전체 | – |
| `goal.link.icr0_share` | 없음 (통제 run만 [0.45, 0.55]) | Requirement | 굽힘점 통제 | linkage, RCJ | – |
| \|AB\| (joint span) | 제약 없음 | Search box | ICR₀ 비율로 모니터 | linkage, RCJ | – |
| `goal.link.transmission_angle_deg` | [30, 150] | Hardware | four-bar 구동성 | linkage | – |
| `goal.motion.dip_flexion_deg` | 150 / 120 / 90 | Requirement | 조건 격자 | 전체 | – |
| `goal.motion.hyperextension_deg` | 10 | Requirement | 펴짐 여유 | 전체 | – |
| `goal.pads.*.length_mm` 하한 | 20 | Requirement | 분절마다 interface 하나 | 전체 | – |
| `goal.pads.*.height_mm` | 10 / 5 / 3 | Requirement | 조건 격자 (interface 두께) | 전체 | – |
| `goal.pads.*.drop_mm` | [0, 자기 pivot 반경] | Hardware | pad는 자기 link 선보다 dorsal로 못 가고, 자기 pivot housing 아래로 매달림 한계 | 전체 | linkage 상한 9/9·8/9; 2R 계열 하한 다수 |
| `goal.pads.*.overhang_max_mm` | 10 | Hardware | cantilever 길이 | 전체 | – |
| `goal.loss.integrity_depth_mm` | 3 | Requirement | 손실 판정 정의 (2026-10-29 결정) | 전체 | – |
| `goal.loss.access_depth_mm` | 20 | Requirement (report) | accessibility 정의 | report | – |
| PIP servo `tilt_angle_deg` | [0, 90] | **Search box인데 binding** | 아래 "tilt와 두께" | 2R, belt, linkage | 상한 90°: 2R 3/8, linkage 1/9 |
| servo model | XM335 (Ø19 × 35) | Hardware | 부품 | 전체 | – |
| `goal.hardware.pulley_radius_mm` | 6 | Hardware | GT2 16T pitch 반경 5.1 + belt | belt | – |
| pivot 지름 A / A′ / B / B′ | 6 / 10 / 6 / 6 | Hardware | 핀 housing | 전체 | – |
| `distal_carrier_height_mm` | 4 | Hardware | carrier plate | 전체 | – |
| `search_bounds.a_z_mm`, `b_z_mm` | [0, 0] | Requirement | 정의 (A, B는 link 선 위) | linkage | 고정 |
| `search_bounds.ap_from_a_x_mm` | [−20, 30] | Search box | | linkage, belt | 0 |
| `search_bounds.ap_from_a_z_mm` | [0, 25] | Search box | | linkage, belt | 0 |
| `search_bounds.bp_from_b_x_mm` | [−50, 20] | Search box | | linkage | 0 |
| `search_bounds.bp_from_b_z_mm` | [−60, 80] | Search box | | linkage | 0 |
| `search_bounds.app/bpp_offset_mm` | [−60, 60] | Search box | | linkage | 0 |
| 2R `bpp_from_a_mm` 하한 | joint 반경 | Hardware | distal pad가 joint housing을 피함 | 2R 계열 | 2R 5/8, ideal 3/8 |
| RCJ `roller_radius_mm` | [9.5, 20] | 하한 Hardware / 상한 Search box | servo가 roller 안에 (반경 9.5) | RCJ | 하한 8/9 |

## ICR₀ (extension 굽힘점) 보고

- **계산**
  - 함수는 `core/optimization/analysis/icr.py`에 있다. 2R 계열은 A, RCJ는 A + r, linkage는 AB′와 A′B의 교점이다.
  - 모든 front 행에 `icr0_share` 열이 생겼다. linkage는 만들어진 설계(링크 0.1 mm grid)로 계산한다.
- **quick run best (provisional)**

  | 메커니즘 | ICR₀ / \|OC\| |
  |---|---|
  | 2R | 0.45–0.54 |
  | belt | 0.54–0.55 |
  | ideal | 0.48–0.54 |
  | RCJ | 0.49–0.64 |
  | linkage | 0.44–0.60 |

  2R 계열은 비율 제약으로 0.45–0.55에 묶인다. RCJ와 linkage는 그 밖으로 나간다. 그래서 굽힘점 통제 run이 필요하다.

## tilt와 두께 (Task 2 추가)

XM335의 body는 반경 9.5 mm cap에 rear extent 25.5 mm이다. tilt 0°이면 body가 −X로 누워 proximal pad 자리를 차지한다. tilt 90°이면 body가 dorsal로 서서 pad 자리를 비우는 대신 dorsal 높이를 25.5 mm 쓴다. 지금은 두께 제한이 없으므로 **tilt 상한에 붙는 설계는 finger 두께를 공짜로 써서 pad 길이를 사는 것**이다.

quick run best 설계의 extension 자세 dorsal–palmar 두께(finger 자기 solid와 pad, MCP servo 제외; `analysis.loss.thickness_mm`):

| 메커니즘 | 두께 (mm) | dorsal 최고점 | tilt |
|---|---|---|---|
| 2R ideal | 7–14 | +3–4 (핀, carrier) | – |
| RCJ | 22–30 | +9.5–10.8 (roller) | – |
| 2R | 34–42 | +24.5–27.2 (servo body) | 44–90° |
| belt | 39–60 | +32–52 (A′ servo) | 6–87° |
| linkage | 44–53 | +37.5–39.7 (A′ servo) | 50–90° |

dorsal 높이가 메커니즘마다 13–50 mm 차이 난다. 특히 servo를 A′에 두는 메커니즘(linkage, belt)이 가장 두껍다. pad coverage 비교가 공정하려면 공통 최대 두께 requirement가 필요하다.

**제안 (결정 대기)**
- 정의: extension 자세에서 finger 자기 solid(pad, carrier, link, 핀 housing, PIP servo, pulley, roller; MCP servo 제외)의 z 범위가 `goal.link.thickness_max_mm` 이하.
- 후보 값:
  1. **35 mm**: servo를 joint에 둔 2R의 best 두께(34–42) 수준. linkage와 belt는 A′를 낮추고 tilt를 줄여야 한다.
  2. **45 mm**: linkage best의 하한 수준. belt 일부만 제약된다.
  3. **servo 지름 + 2 × interface 두께 + 여유**처럼 h에 비례하는 값.

## 검증 run (Checkpoint 2 이후 사용자 실행)

모든 run은 integrity 3 mm 규칙으로 돌고, 출력 폴더 guard가 걸려 있다.

```bash
# 기준 run: 전체 격자 5 메커니즘 × 9 조건 × seed 3 (+ variant 3개)
bash scripts/optimize/comparison/run_all.sh
# 범위 확장 run: 두 비율 [0.25, 0.65], h5 × 150/120/90, 5 메커니즘
bash scripts/optimize/comparison/run_ratio_wide.sh
# 굽힘점 통제 run: ICR₀ ∈ [0.45, 0.55], h5 × 150/120/90, RCJ와 linkage
bash scripts/optimize/comparison/run_icr_control.sh
# 예산 진단 (먼저): h5·h10 / 150°, 1500세대 → output/comparison_budget3
bash scripts/optimize/comparison/budget/run_budget.sh
```

세대 수는 진단 run의 수렴 곡선을 본 뒤 정한다. 지금 `make_configs.GENERATIONS`는 전부 600이다.
