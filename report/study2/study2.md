# Study 2: 더 두꺼운 interface로 다시 설계할 때 접촉면에서 본 운동을 지킬 수 있는가

정리: 2026-11-24. `study2_redesign.md`(2026-11-21)를 기준으로, `pole_law.md`의 Study 2 관련 부분과 그 뒤의 확인(e19, 미끄럼 하한, spacer)을 합쳤다. 정의는 `design/SSoT/problem_formulation.md` A.6.3, 주장은 `design/SSoT/research-objective.md` §6. 수학은 `report/study0/study0.md`.

## 1. 질문과 SSoT의 주장

질문: 더 두꺼운 contact interface를 쓰려고 손가락을 다시 설계할 때, proximal phalanx(contact face)에서 본 distal 접촉점의 경로, 곧 contact Jacobian을 얼마나 지킬 수 있고, 그때 coverage는 얼마나 남는가(→ C2.2). Study 0의 고정 축 하한(H0.4)이 결과를 예측한다.

- **H2.** 3 mm 설계에서 5, 7, 9 mm pad로 다시 최적화하면 CFL은 front의 coverage 끝에서 고정 축 하한보다 작게 접촉면 운동을 바꾸고, coverage는 Study 1의 최선에 가깝다. (실패 조건 없음: CFL의 pole은 언제나 하한 아래로 움직일 수 있어 진짜 검정이 아니므로, 결과를 크기(하한의 10–51%)와 다른 관절과의 비교로 보고한다. 2026-11-22 삭제.)
- **R2.** 2D 측면 모델에서, 각 front의 coverage 끝에서 CFL은 Δ_contact를 0.35–4.31 mm, 하한의 10–51% 바꾸고, C̄_pad는 Study 1 최선과 0.2 %p 이내이다. 9개 칸 모두에서 실제 구조 넷 중 가장 작다. 2R-DD와 RCJ의 시작 오프셋을 빼면 9개 중 7개에서 가장 작고, 두 예외는 coverage를 2.4, 17.8 %p 잃는다(e19). Front를 따라 coverage를 2 %p 내줘도 2R-DD와 RCJ는 CFL보다 크다. 값은 탐색이 찾은 것이며 증명된 최솟값이 아니다. 접촉점을 face를 따라 미끄러뜨릴 수 있는(c₀를 지키지 않는) 고정 축도 90°, 120°, 150°에서 하한의 0.586, 0.667, 0.794배 아래로 갈 수 없고, CFL의 가장 큰 변화(90°, 9 mm, 51%)는 모든 칸에서 이보다 작다.
- **H0.4.** 2R-remote의 다시 설계는 90°와 120°에서 하한에 놓인다(120°: 3.46, 6.93, 10.39 mm).
- Spacer는 요구조건으로 배제한다(4절).

## 2. 설정

| 항목 | 내용 |
|---|---|
| 기준 손가락 M₀ | 구조와 q_req마다 Study 1의 3 mm 최선 설계(CFL은 독립 run과 재탐색 중 최선) |
| 새 interface | 5, 7, 9 mm(두 pad 같은 두께). 얇은 쪽은 다루지 않음 |
| 기준 좌표계 F_p | Proximal phalanx에 고정. 원점은 MCP 축에서 proximal contact face에 내린 수선의 발, x는 face를 따라 손끝 쪽 |
| Δ_contact | max_{q∈[0,q_req]} ‖c(M, q) − c(M₀, q)‖, c: distal face 위 손끝 아래 점을 F_p에서 본 위치. 단위 mm(코드 `contact_change`는 L_f로 나눈 값) |
| Δ_skel | 같은 식을 F_p에서 본 손끝 경로로(`skeleton_change`) |
| 탐색 | 두께마다 NSGA-II 200 × 200, seed 0–1, 두 목적: Δ_contact 최소화, Study 1 목표 아래 C̄_pad 최대화. Coverage 하한 없음. Study 1의 설계 M*(h)가 초기해 |
| 제약 | Study 1과 같음(가장 얕은 장착, spacer 없음, L_f = 90 mm) |
| 비교 기준 | 고정 축 하한 2 sin(q_req/2) Δh (Study 0 (9)) |
| 보고 | Front의 coverage 끝(가장 높은 C̄_pad의 설계) |

## 3. 결과

### 3.1 Coverage 끝의 설계

칸: Δ_contact mm (하한 대비) / C̄_pad.

| 구조 | q_req | 3 → 5 mm | 3 → 7 mm | 3 → 9 mm |
|---|---|---|---|---|
| 고정 축 하한 | 90° | 2.83 | 5.66 | 8.49 |
| | 120° | 3.46 | 6.93 | 10.39 |
| | 150° | 3.86 | 7.73 | 11.59 |
| **CFL** | 90° | **1.22 (43%)** / 0.776 | **1.84 (32%)** / 0.749 | **4.31 (51%)** / 0.717 |
| | 120° | **0.35 (10%)** / 0.753 | **0.86 (12%)** / 0.725 | **2.32 (22%)** / 0.695 |
| | 150° | **0.77 (20%)** / 0.738 | **1.71 (22%)** / 0.707 | **3.65 (31%)** / 0.673 |
| 2R-remote | 90° | 2.83 (100%) / 0.809 | 5.66 (100%) / 0.773 | 8.49 (100%) / 0.736 |
| | 120° | 3.46 (100%) / 0.758 | 6.93 (100%) / 0.697 | 10.39 (100%) / 0.633 |
| | 150° | 3.86 (100%) / 0.593 | 13.77 (178%) / 0.209 | 22.86 (197%) / 0.000 |
| 2R-DD | 90° | 2.12 (75%) / 0.726 | 3.82 (67%) / 0.708 | 5.23 (62%) / 0.693 |
| | 120° | 2.00 (58%) / 0.700 | 4.40 (64%) / 0.682 | 5.86 (56%) / 0.633 |
| | 150° | 3.00 (78%) / 0.593 | 설계 없음 | 설계 없음 |
| RCJ | 90° | 2.83 (100%) / 0.663 | 5.16 (91%) / 0.634 | 7.14 (84%) / 0.606 |
| | 120° | 2.08 (60%) / 0.608 | 3.72 (54%) / 0.579 | 9.30 (89%) / 0.555 |
| | 150° | 1.90 (49%) / 0.560 | 7.10 (92%) / 0.514 | 설계 없음 |
| CFL ideal* | 90° | 0.46 (16%) / 0.779 | 5.70 (101%) / 0.756 | 3.10 (37%) / 0.731 |
| | 120° | 0.52 (15%) / 0.760 | 1.54 (22%) / 0.733 | 2.20 (21%) / 0.703 |
| | 150° | 1.40 (36%) / 0.741 | 2.23 (29%) / 0.712 | 4.76 (41%) / 0.679 |
| RCJ ideal* | 90° | 2.80 (99%) / 0.787 | 1.91 (34%) / 0.775 | 4.33 (51%) / 0.745 |
| | 120° | 0.35 (10%) / 0.781 | 2.81 (41%) / 0.738 | 5.07 (49%) / 0.694 |
| | 150° | 1.28 (33%) / 0.755 | 3.20 (41%) / 0.696 | 5.20 (45%) / 0.636 |

\* servo 몸체가 없는 상한, 따로 읽는다.

- **CFL이 9개 칸 모두에서 실제 구조 넷 중 가장 작다.** 0.35–4.31 mm, 하한의 10–51%(120°에서 10–22%, 150°에서 20–31%, 90°에서 32–51%).
- **Coverage를 내주고 얻은 것이 아니다.** CFL의 C̄_pad는 최종 Study 1 최선과 0.2 %p 이내(최대 차이 0.18 %p, 90°/5 mm 0.776 대 0.778). CFL ideal은 0.5 %p 이내. 2R, RCJ와 RCJ ideal은 Study 1 최선과 같다.
- **2R-remote는 하한에 정확히 놓인다**(90°, 120°; H0.4). Front가 한 점으로 모인다. 150°의 7, 9 mm에서 하한보다 큰 것은 접촉하지 못하는 퇴화 설계(C̄_pad 0.209, 0)이다.
- **2R-DD와 RCJ는 하한 아래로 내려가지만 CFL보다 크다.** 2R-DD는 proximal pad가 이미 관절 servo 밑에 달려 그 face 깊이가 두께에 따라 거의 변하지 않고 관절을 x로 옮길 수 있어서, RCJ는 pad를 roller 밑에서 덜 내려 일부를 상쇄해서이다. 두꺼운 쪽에서 RCJ는 하한에 가깝다(120°/9 mm 9.30 mm).
- **두께가 커지면 변화가 커진다.** CFL도 9 mm에서 2.3–4.3 mm이다. 접촉면 운동을 완전히 지키지 못하고, 고정 축보다 2–10배 적게 바꾼다.
- CFL ideal 90°/7 mm(5.70 mm)는 coverage 끝의 설계가 크게 바뀐 경우이고, 같은 front에 0.58 mm로 바꾼 설계가 있다(coverage 0.539).

### 3.2 시작 오프셋을 뺀 비교 (e19)

Δ_contact를 q = 0의 오프셋 ‖Δc(0)‖과 운동 자체의 변화 max_q ‖Δc(q) − R_q Δc(0)‖로 나눈 뒤, 운동 자체의 변화(mm, coverage 끝의 설계):

| q_req | 두께 | CFL | 2R-remote | 2R-DD | RCJ |
|---|---|---|---|---|---|
| 90° | 5 / 7 / 9 mm | 1.20 / 1.69 / 4.21 | 2.83 / 5.66 / 8.49 | 1.58 / 2.70 / **3.81** | 2.83 / 5.09 / 7.07 |
| 120° | 5 / 7 / 9 mm | 0.33 / 0.88 / 2.18 | 3.46 / 6.93 / 10.39 | 1.39 / 3.02 / 3.67 | 1.91 / 3.14 / 9.38 |
| 150° | 5 / 7 / 9 mm | 0.87 / 1.61 / 3.51 | 3.86 / 13.77 / 22.86 | 2.16 / – / – | **0.70** / 6.86 / – |

CFL은 9개 중 7개에서 가장 작다. 예외 둘(2R-DD 90°/9 mm 3.81 대 4.21 mm, RCJ 150°/5 mm 0.70 대 0.87 mm)은 CFL보다 coverage가 각각 2.4, 17.8 %p 낮다.

### 3.3 Coverage를 2 %p 내줄 때

각 front에서 coverage 끝보다 2 %p 이내인 설계 가운데 Δ_contact 최솟값(mm). CFL은 coverage 끝의 값.

| q_req | 두께 | CFL | 2R-DD | RCJ |
|---|---|---|---|---|
| 90° | 5 / 7 / 9 mm | 1.22 / 1.84 / 4.31 | 1.70 / 3.32 / 5.00 | 2.55 / 4.67 / 6.29 |
| 120° | 5 / 7 / 9 mm | 0.35 / 0.86 / 2.32 | 1.90 / 3.72 / 5.80 | 0.78 / 2.94 / 5.28 |
| 150° | 5 / 7 / 9 mm | 0.77 / 1.71 / 3.65 | 2.99 / – / – | 1.50 / 3.85 / – |

모든 칸에서 2R-DD와 RCJ가 CFL보다 크다.

### 3.4 접촉점이 미끄러질 수 있는 고정 축

Δ_contact는 c₀의 이동을 q = 0의 값으로만 벌한다. 그래서 고정 축이 축 위치와 face를 따른 c₀ 이동 α를 함께 고를 수 있다고 하면 하한은 min_{a,α} max_q ‖(I − R_q)Δp + R_q (α, 0)‖이다(Δp = (a, Δt)). 하한 2 sin(q_req/2)|Δt|에 대한 비:

| q_req | 90° | 120° | 150° |
|---|---|---|---|
| 미끄럼을 허용한 고정 축의 최소 / 하한 | 0.586 | 0.667 | 0.794 |
| CFL의 가장 큰 비 (그 q_req) | 0.51 (9 mm) | 0.22 (9 mm) | 0.31 (9 mm) |

일반적으로 0.5 이상이다(q = 0과 q_req 항의 최대). CFL은 모든 칸에서 이 값보다 작으므로 CFL의 이점은 c₀ 이동이 아니라 pole의 이동이다. (v23 review r3이 제기, 2026-11-22 수치 확인; v28 r3, 이 정리 때 다시 계산해 같은 값.)

## 4. 한계와 범위

- 2D 측면 모델, 두 pad 같은 두께, 장착 규칙(가장 얕은 깊이, spacer 없음) 안에서의 비교이다.
- Δ_contact^min과 coverage 끝의 값은 탐색이 찾은 것이며 증명된 최솟값이 아니다. "설계 없음"은 탐색 범위와 예산 안에서 못 찾은 것이다.
- 2R-DD와 RCJ가 하한 아래로 가는 것의 일부는 시작 오프셋(face를 따른 c₀ 이동)이다(3.2, 3.4).
- 고정 축 하한은 Δ_contact만 묶는다. Δ_contact를 지키면 뼈대 운동 Δ_skel이 바뀐다(합 하한, Study 0 (7)). CFL은 뼈대 운동을 바꿔 접촉면 운동을 지킨다.
- **Spacer 배제.** 얇은 interface를 spacer로 가장 두꺼운 interface의 깊이까지 올리면 어느 구조에서나 접촉면 운동이 유지된다. 그러나 interface가 쓸 공간을 기능 없는 받침으로 채우고, 얇은 interface마다 깊은 설계의 coverage만 남기며(120°에서 3 mm를 9 mm 설계에 달면 CFL 8.6 %p, 2R-remote 18.2 %p 손실), 가장 두꺼운 interface를 미리 정해야 한다. 그래서 비교 대상이 아니라 요구조건으로 배제한다.
- Ideal* 구조는 상한이며 순위 비교에 넣지 않는다.

## 5. 데이터와 스크립트

| 내용 | 경로 |
|---|---|
| 조건별 요약 (커밋된 사본) | `report/study2/data/redesign_summary.csv` (원본 `design/output/study/study2_redesign_2026-11-21/summary.csv`). `change_at_most_c_pad` × 90 mm = 표의 Δ_contact, `fixed_axis_change` × 90 = 하한 |
| 모든 front 점 | `report/study2/data/redesign_fronts.csv` (`front.csv`) |
| Coverage 끝 설계의 q별 ‖Δc(q)‖ | `report/study2/data/redesign_profiles.csv` |
| Raw 표와 metadata | `report/study2/data/raw/redesign.csv`, `report/study2/data/raw/metadata.json` (`report/make_raw_tables.py`) |
| Study 0 대조용 51개 설계 | `report/study2/data/pole_law_designs.csv` (`report/study0/study0.md` 5절) |
| 다시 설계 탐색 | `design/scripts/study/study2_morphology_adaptation/redesign.py` (`one_finger.yaml`) |
| 3.2 시작 오프셋 | `design/scripts/study/exploratory/e19_aligned_contact_change.py` → `report/study2/data/e19_aligned_contact_change.csv` (원본 `design/output/study/exploratory/e19_aligned_contact_change/summary.csv`에서 복사) |
| 3.3 | `report/study2/data/redesign_fronts.csv`에서 계산 |
| 3.4 미끄럼 하한 | `paper/orchestra/reviews/v23_r3.md`, `v28_r3.md` (스크립트 없음, 수치 최소화) |
| Spacer 손실 | `paper/orchestra/experimental_log.md` |

대체된 것:
- `study2_redesign.md` 표의 괄호 값(탐색 시점의 Study 1 최선, 예: CFL 90°/5 mm 0.757)은 Study 1 재탐색 전 값이다. 기준 손가락(3 mm)은 바뀌지 않았으므로 Δ_contact와 그 coverage는 그대로이다.
- `pole_law.md`(2026-11-20)의 인접 두께 설계끼리의 비교(예: CFL 3 → 5 mm 0.84 mm)와 `derivation.md` 5절의 CFL 대조 값(0.5, 0.65, 1.73 mm)은 Study 1만의 설계(접촉면 운동을 목표로 주지 않음)이거나 이전 run의 값이므로 이 문서의 결과로 대체한다.
