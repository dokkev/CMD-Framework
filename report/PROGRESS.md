# CoDe Hand — Progress and Defensible Claims

기준일: 2026-11-09 (§7 갱신 2026-11-17). 연구 목표는 [GOAL.md](../GOAL.md)를 따른다. 이 문서는 study 쪽 결과만 다룬다. Prototype 최적화 결과([prototype.md](prototype.md))는 근거로 쓰지 않는다(저장소 README, "Two optimizations: prototype and study"). 이 문서는 현재 확보한 근거로 주장할 수 있는 범위와 아직 검증이 필요한 범위를 기록한다. 연구 목표의 선언과 목표 달성의 증거를 구분한다.

## 1. 현재 결론

**현재 기여의 중심은 contact-centric architecture의 제안, contact accommodation과 embodiment adaptation을 분리하는 평가 절차, 그리고 조건부 계산적 이점과 한계이다.**

CoDe가 모든 인터페이스에서 우월하거나, 더 넓은 manipulation-usable design space를 제공하거나, 물리적 연구 플랫폼으로 검증되었다고 주장할 단계는 아니다. 특정 pad·ROM 조건에서 underlying embodiment를 유지하는 성공 사례와, 다른 조건에서 재설계가 필요하거나 탐색이 해결하지 못한 사례를 함께 보고한다.

## 2. 근거의 범위와 우선순위

| 근거 | 현재 사용할 수 있는 범위 | 출처 |
|---|---|---|
| Independent fixed-H interface-family experiment | 선언한 길이·두께 조합 격자에서의 fixed-skeleton/actuation 수용성; dense-posture 및 fold-off 민감도 포함 | [보고서](interface_family_2026-11-09.md), [summary CSV](../design/output/study/interface_family_2026-11-09/summary.csv), [dense CSV](../design/output/study/interface_family_2026-11-09/dense_summary.csv) |
| ROM-conditioned Study 2 pilot | 90/120/150°별 baseline 준비와 interface adaptation을 분리한 현재 모델의 관찰 | [보고서](study2_rom_conditioned_2026-11-09.md), [baseline CSV](../design/output/study/study2_rom_conditioned_2026-11-09/pilot/baseline_best.csv), [interface CSV](../design/output/study/study2_rom_conditioned_2026-11-09/pilot/interface_best.csv) |
| Link-in-front occlusion ON/OFF | Study 1(active 15°)에서 occlusion 규칙이 CoDe 수용 결과에 미치는 영향; OFF는 검증되지 않은 이상화 조건 | [보고서](study1_faceoff_2026-11-17.md) |
| Transmission-angle sensitivity | CoDe proximal accommodation이 angle 및 face-use 제약에 의존하는 사례; 동일 geometry 재평가 | [보고서](transmission_sensitivity_2026-11-09.md) |
| 기존 Studies 1–2 | 고정 형상 수용성과 형상 적응 비용의 평가 방법 및 해당 source snapshot의 결과 | [보고서](study1_study2_2026-11-06.md) |
| Compact-accommodation extension | 해당 snapshot과 탐색 영역에서 찾은 크기·span·articulation trade-off | [보고서](study2_compact_2026-11-07.md) |
| Studies 3–4 | 해당 snapshot의 local reflected mechanics 및 prescribed bridge의 geometric screening | [보고서](study3_study4_2026-11-07.md) |

2026-11-17에 `links_block_face`의 반올림 오류를 고쳤다(`9115e3f`). 규칙 ON에서 usable 길이가 최대 0.25 mm 길게 읽혔고, 그 이전 run의 CoDe·belt 2R ON 결과에는 이 오차가 남아 있다. Study 1 active 조건에서는 belt 2R의 수용 수만 37/33/17에서 36/31/13으로 바뀌었다.

이전 보고서의 tilt, clearance 및 평가 규칙은 각 저장 source/config에 귀속된다. 현재 모델에서 모두 다시 실행한 결과로 취급하거나, 서로 다른 조건의 결과를 하나의 architecture ranking으로 합치지 않는다. 특히 과거 compact 결과를 현재 fixed-H 유지의 증거로 사용하지 않는다.

ROM-conditioned pilot은 7개 비교 조건 × 3개 ROM × 8개 interface이며, population 32, 20 generations, seed 0 하나를 사용했다. CoDe/belt tilt는 15°로 고정하고, DD 및 physical RCJ는 0°와 90°를 별도 조건으로 평가했다. Ideal RCJ는 이상화 비교군이다. CoDe의 transmission angle 범위는 30–150°, hyperextension 요구는 10°이며 직접적인 k_max 상한은 없다. 실제 envelope 비관통 및 기존 face-use 검사를 유지했다.

21개 ROM baseline을 모두 확보했고, interface 조건 168개 중 124개에서 성공 설계를 찾았다. 저장된 성공 baseline/interface 설계 145개는 해당 config로 재평가했다. 이는 모델 조건의 통과 확인이며, 전역 최적성이나 물리적 buildability의 검증이 아니다. 더 큰 예산의 ROM-conditioned 본 실험 설정은 준비되어 있지만 아직 실행하지 않았다.

## 3. 현재 주장 가능한 내용

| ID | Claim | 근거와 경계 |
|---|---|---|
| C1 | **Contact interfaces를 우선하는 hand architecture와 연구 접근을 제안한다.** | 설계 의도와 구조의 제안이다. 독창성·최초성 또는 비교 우위는 별도의 선행연구 검토와 증거가 필요하다. |
| C2 | **Contact accommodation과 이를 위한 embodiment 변경을 분리해 평가한다.** | 고정 reference의 수용성, ROM baseline 준비 비용, 각 interface의 추가 E/Q 변경량을 구분한다. 이를 engineering time이나 전체 integration cost와 동일시하지 않는다. |
| C3 | **평가한 CoDe reference는 baseline 및 thick pad를 E/Q 변경 없이 90/120/150°에서 수용했다.** | 현재 평면 모델과 명시한 검사 조건의 pilot 관찰이다. 모든 두께·길이·재료·센서 모듈에 대한 일반 명제가 아니다. |
| C4 | **150° thick-pad 조건에서 CoDe는 nominal E/Q를 유지했으며, 평가한 physical DD·RCJ 구성에는 추가 외형 적응이 필요했다.** | 아래 수치 참조. 특정 reference와 조건의 비교다. DD·RCJ 전체 설계군의 필연적인 재설계 하한이나 architecture-wide 우위는 아니다. |
| C5 | **일부 interface의 수용 가능성은 transmission 및 face-use 제약에 의존한다.** | angle 완화 후 발견한 후보와 동일 geometry 재평가로 뒷받침한다. 특정 후보에 대한 진단을 모든 실패의 원인으로 확대하지 않는다. |
| C6 | **Interface 수용성과 embodiment 유지에는 architecture-dependent trade-off가 있다.** | CoDe의 thick-pad 유지 사례와 proximal 중심 pad의 제한, DD·RCJ의 적응 성공을 함께 보고한다. 최종 ranking이나 통계적 우월성 주장은 아니다. |
| C7 | **선언한 interface-family 격자에서 CoDe의 고정 skeleton/actuation이 지원하는 조합 수가 물리적 비교군보다 많았다.** | 6개 길이 strata와 독립적인 proximal/distal 두께 격자의 조건부 관찰이다. 20/20 mm·150°에서 dense 기본 규칙28/144, physical 비교군6–13/144; fold-off control106/144 대26–88/144. Interface-dependent seating을 허용하며 모든 길이에서의 우위가 아니다. |

### 추가 관찰: fixed-H interface family

이전8종 pad의 개별 재설계 결과와 달리, 한 형상을 고정하고 독립적인 proximal/distal 두께 조합을 바꾸는 실험에서 추가 이점을 확인했다. [독립 실험 보고서](interface_family_2026-11-09.md)에 raw 결과·sampling·source provenance와 독립 검토 범위를 기록했다. 기존 evaluator 재사용이므로 독립 물리 모델 검증은 아니다.

초기6개 자세에서 보인 CoDe20 mm pad의3..8 mm sampled family는5° 간격 검사에서 유지되지 않았다. Dense 기본 규칙에서 보장된 sampled rectangle은 proximal{3,4,5,6} × distal{3,4} mm이다. “두께 범위 전체를 연속적으로 지원”하거나 “모든 길이에서 우월”하다고 일반화하지 않는다. Fold 직선 교차 모델에 절대 수용 범위가 크게 의존한다는 점도 결과의 일부다.

### 대표적인 조건부 이점: thick pad와 150° ROM

Baseline pad는 proximal/distal 각각 20 × 3 mm, thick pad는 각각 20 × 8 mm이다.

| Architecture condition | ROM baseline 준비 εE | Thick pad의 추가 εE | Thick pad의 추가 εQ |
|---|---:|---:|---:|
| CoDe, tilt 15° | 0 | 0 | 0 |
| DD, tilt 0° | 0.103333 | 0.104733 | 0 |
| DD, tilt 90° | 0 | 0.216667 | 0 |
| Physical RCJ, tilt 0° | 0.008889 | 0.029736 | 0 |
| Physical RCJ, tilt 90° | 0 | 0.005556 | 0 |

εE는 정규화한 external descriptor의 최대 변경량이며, 손가락 전체 길이 증가율이 아니다. ROM 준비 비용은 nominal 길이, 추가 interface 비용은 선택된 ROM baseline 길이로 정규화한다. 두 최대 norm을 합산하지 않는다. RCJ 90°의 추가 변경량은 작으므로 모든 비교군에 대해 큰 효과라고 표현하지 않는다.

이 결과에서 확인한 불변량은 평가 지표 E/Q와 config에 고정된 actuation 설정이다. 질량·관성, 실제 조립 공차, 센서 배선, 접촉 응답까지 불변임을 뜻하지 않는다.

### 함께 보고해야 할 제한

| Architecture condition | 90°: 성공 pad / 8 | 120°: 성공 pad / 8 | 150°: 성공 pad / 8 |
|---|---:|---:|---:|
| CoDe, tilt 15° | 8 | 5 | 2 |
| DD, tilt 0° | 8 | 7 | 2 |
| DD, tilt 90° | 8 | 8 | 4 |
| Belt, tilt 15° | 7 | 5 | 2 |
| Physical RCJ, tilt 0° | 7 | 5 | 4 |
| Physical RCJ, tilt 90° | 7 | 7 | 4 |
| Ideal RCJ | 8 | 8 | 8 |

성공 수에는 외형·기구 변경 후 성공도 포함된다. 이 표만으로 fixed-H 수용 범위나 확률적 성공률을 주장하지 않는다. Candidate와 pad 종류는 독립적인 통계적 replicate가 아니다.

120°의 asym_long_proximal 및 asym_proximal_heavy에서는 DD/physical RCJ의 두 tilt 조건 모두 성공 설계를 찾았지만, CoDe pilot은 찾지 못했다. 90°에서는 CoDe도 두 pad를 수용했으나 추가 Q 변경이 필요했다. 따라서 현재 결과는 CoDe의 보편적인 proximal accommodation 우위를 지지하지 않는다.

120° long_thick에서는 CoDe εE=0.163333, εQ=0.078889; DD 90° εE=0.194444, εQ=0이다. 이는 외형과 기구 변경 사이의 trade-off이며, CoDe의 fixed-H 유지 사례가 아니다.

Transmission sensitivity에서는 10–170°로 angle 범위를 완화한 main search가 asym_long_proximal의 성공 후보를 3 seeds 중 1개에서 찾았다(k_max≈5.063). 그 후보는 E/Q를 변경했으므로 fixed-H swap의 증거가 아니다. Face-rule-off control은 별도의 진단이며 primary comparison에 포함하지 않는다.

## 4. 아직 주장하지 않는 내용

| 보류하는 claim | 필요한 추가 근거 |
|---|---|
| CoDe가 전반적으로 더 넓은 contact-interface design space를 제공한다 | 더 충분한 탐색과 명시된 design domain에서 일관된 비교; 불리한 조건 포함 |
| 더 넓은 manipulation-usable design space를 제공한다 | 기계적 수용 외에 접근성, 하중, 실제 task 수행 검증 |
| Interface 교체 시간·개발 비용·통합 노력을 줄인다 | 신규 interface 지원과 지원된 interface swap을 분리한 시간·부품·분해 단계·software 변경 기록 |
| Proximal/distal 모듈을 독립적으로 쉽게 교체할 수 있다 | 실제 하드웨어의 교체 절차와 articulation 분해 여부, 재장착 반복성 |
| Interface 효과를 완전히 인과적으로 분리했다 | Fixed-H 조건의 접촉/조작 비교와 제어·관측·실험 절차 통제; 잔여 confound 보고 |
| 조작 성능, policy transfer 또는 learning 효율이 개선된다 | Studies A–D에 대응하는 학습 및 평가 결과 |
| 시뮬레이션이 실제 compliant contact를 정확히 예측한다 | 대표 interface의 물리 검증; sim-to-sim 일치만으로는 충분하지 않음 |
| Polycentric articulation이 일반적으로 bridge 변형을 줄인다 | 기존 Study 4는 이 주장을 지지하지 않음; 구체적인 bridge 구조와 loaded-contact 검증 필요 |
| 계산된 reflected stiffness가 실제 actuator response이다 | Study 3 모델의 preload·관성·마찰·구동 특성 등 가정과 실제 응답 비교 |
| 탐색 실패 조건은 기구적으로 불가능하다 | 현재 not_found는 budget/domain 내 미발견이며 불가능성 증명이 아님 |

E/Q/A와 장착 기준을 유지한 실험은 그 구조적 변경 요인을 통제할 수 있다. 그러나 contact location, contact-point Jacobian, friction, compliance 및 tactile observations는 interface 자체의 의도된 변화일 수 있다. 이를 고정해야 embodiment가 보존되는 것은 아니다.

## 5. 기존 연구 결과의 위치

Study 1은 frozen reference의 수용 범위를, Study 2는 interface 수용을 위한 재설계량을 다룬다. Compact extension은 작은 finger를 얻기 위한 공간·형상 비용을 다루므로, 작은 설계를 찾았다는 결과를 동일 embodiment 유지로 바꾸어 해석하지 않는다.

Study 3은 동일 local contact law도 transmission과 contact Jacobian에 따라 다른 actuator-side matrix로 반영될 수 있음을 모델 수준에서 보여준다. 이 결과는 통제·보고해야 할 embodiment dependence이며 CoDe의 성능 우위가 아니다. Study 4는 평가한 continuous bridge에서 shortening, self-intersection 및 routing 한계를 보고했다. 현재 성공적인 연속 joint skin의 증거로 사용하지 않는다. 두 연구 모두 저장 snapshot의 결과이며 현재 변경된 모델 전체의 재검증을 뜻하지 않는다.

GOAL.md Section 5.7의 네 번째 기준인 동일 물리 hand에서의 통제·반복 실험, 그리고 Section 8의 learning-infrastructure reuse 및 manipulation 연구는 현재 이 문서가 확인한 계산 결과만으로 달성했다고 표시하지 않는다.

## 6. 논문·발표에 사용할 수 있는 문장

**현재 중심 claim:**

> CoDe는 접촉 인터페이스를 우선하는 로봇 손 구조를 제안하고, 인터페이스 수용성과 이를 위한 embodiment 변경을 분리해 평가한다. 평면 모델 평가에서는 동일 embodiment를 유지하는 interface–ROM 조건과 재설계가 필요하거나 주어진 탐색 예산에서 해결되지 않은 조건을 함께 확인했다.

**조건부 비교 관찰:**

> 평가한 thick-interface 및 150° ROM 조건에서 CoDe는 nominal embodiment를 유지했으며, 평가한 물리적 DD·RCJ 구성에는 추가 외형 적응이 필요했다. 이 관찰은 해당 모델과 reference 조건에 한정된다.

**English working claim:**

> We propose CoDe, a contact-centric robotic hand architecture, and evaluate contact-interface accommodation separately from the embodiment changes required to achieve it. Planar evaluations identify interface–ROM conditions under which CoDe preserves its underlying embodiment, alongside conditions that require redesign or remain unresolved within the search budget.

이 문장들은 최종 architecture ranking이나 물리적 연구 플랫폼 검증 완료를 의미하지 않는다. 정량 비교를 제출용 claim으로 강화하기 전에는 현재 pilot의 더 큰 예산·복수 seed 탐색을 수행하고, 결과의 안정성과 한계를 갱신한다.

## 7. 다음에 확보할 증거

0. **Studies 1–2를 현재 active 조건으로 다시 실행한다** (2026-11-17 추가). 조건은 CAD 기준 nominal 15°와 0–90° tilt sweep이다(`references_nominal15.yaml`). [study1_study2_2026-11-06.md](study1_study2_2026-11-06.md)는 10° reference로 얻은 과거 결과다. 현재 조건으로는 Study 2 quick run(CoDe, tilt 15°)만 있고 보고서는 없다. 그 전까지 Studies 1–2의 수치는 해당 snapshot의 결과로만 인용한다.
1. ROM-conditioned 본 실험으로 best-found 비용과 미발견 조건의 탐색 의존성을 확인한다. Baseline 준비와 interface 적응 비용은 계속 분리한다.
2. CoDe의 proximal 제한을 실제 비관통, transmission 및 face-use 조건으로 진단한다. 특정 geometry의 counterfactual과 재최적화 결과를 구분한다.
3. 대표 interface를 물리적으로 교체하고 retained ROM, 장착 반복성, 분해 단계와 통합 변경을 기록한다.
4. 동일 H 및 공통 실험 절차로 interface-dependent contact 또는 manipulation 차이를 측정한다. 그 뒤 learning reuse와 transfer claim을 평가한다.

새 결과가 나오면 성공과 실패를 함께 갱신한다. 모든 정량 claim은 saved results, source commit/patch, config, input artifact 및 sampling provenance로 추적 가능해야 한다. `design/output/`은 git에 추적되지 않으므로 문서 링크만으로 결과 보존을 대체하지 않는다.
