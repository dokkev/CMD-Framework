# Study 1: pad 두께와 요구 flexion마다 최적화한 손가락의 coverage

정리: 2026-11-24. `study1_thickness_coverage.md`(2026-11-21)를 기준으로, `study1_fixed_morphology_native_2026-11-20.md`의 설정과 보조 결과를 합쳤다. 정의는 `design/SSoT/problem_formulation.md` A.6.1–A.6.2, 주장은 `design/SSoT/research-objective.md` §6.

## 1. 질문과 SSoT의 주장

질문: contact interface의 두께와 요구 flexion이 주어질 때, 구조마다 최적화한 손가락은 원기둥 기준 coverage C̄_pad를 얼마나 확보하는가(→ C1).

- **H1.** 두 구조 모두 설계가 있는 조건에서, 모든 pad 두께와 요구 flexion에서 CFL이 2R-DD보다 coverage가 높다. *실패 조건:* 어느 한 조건에서라도 2R-DD가 같거나 높다. 2R-remote와 ideal 구조에 대한 순위는 가설이 아니라 조건별로 보고한다.
- **R1.** 2D 측면 모델(L_f 90 mm, pad 3–9 mm, q_req 90–150°)에서 CFL의 최선 coverage는 둘 다 설계가 있는 10개 조건 모두에서 2R-DD보다 2.4–14.6 %p 높고, CFL 재탐색 없이도 그렇다. 2R-DD는 150°의 7, 9 mm에서 설계가 없다. 2R-remote와의 순위는 조건에 따른다: 90°에서 2R-remote가 높고, 120°에서 5와 7 mm 사이에서 교차하며, 150°에서 CFL이 높다. 3 → 9 mm에서 CFL은 C̄_pad를 0.085–0.096 잃는다. 차이는 탐색의 흩어짐보다 크다: CFL의 여섯 run 중 가장 나쁜 것도 10개 중 9개 조건에서 2R-DD의 최선보다 높다(예외: 90°, 9 mm, −0.6 %p). CFL 최적 설계의 최악 MA는 0.41–0.58(대칭 RCJ 0.5, revolute 1; 탐색에서 제약 없음).
- Study 0의 H0.1–H0.3을 이 설계들로 확인한다(`report/study0/study0.md` 5절).

## 2. 설정

| 항목 | 내용 |
|---|---|
| 모델 | 2D 측면 모델, L_f = 90 mm(탐색 범위 89.95–90.05 mm), 두 pad 같은 두께 |
| 조건 | 두께 h ∈ {3, 5, 7, 9} mm × q_req ∈ {90°, 120°, 150°} × 구조 6개(CFL, 2R-DD, 2R-remote, RCJ, 이상화 상한 CFL ideal*, RCJ ideal*) |
| 목적 | C̄_pad(A.5.1)를 최대화, q ∈ {90°, (90° + q_req)/2, q_req}의 평균. 바꾸지 않음 |
| 설계 변수 | morphology 전체(pad 길이 포함). 두께만 조건. Pad 깊이는 가장 얕은 장착, spacer 없음 |
| 규칙 | MA 하한 없음(MA_wc는 보고만). Transmission angle γ ∈ [30°, 150°]는 CFL에만. CFL의 A pin housing과 MCP servo 충돌 검사(A.4). 2R-remote의 PIP servo는 A′ = A + (−10, 15) mm, 15°로 고정. 가림(link, belt)은 보지 않음(다른 lateral layer). Ideal*은 joint actuator 몸체가 없는 상한 |
| 탐색 | 조건마다 GA·NSGA-II 독립 run(seed 0–2). 독립 run이 서로 다른 설계로 끝나는 CFL·CFL ideal은 같은 q_req의 이웃 두께 설계에서 위·아래로 한 번씩 다시 탐색(`run_continuation.sh`), 마지막으로 그 조건의 최선과 Study 2가 같은 두께에서 찾은 설계에서 한 번 더(`run_from_study2.sh`). 최선이 150세대 동안 나아지지 않으면 멈추고 fine grid에서 검증. 조건의 값은 모든 run 중 최선 |
| 5 mm | h = 5 mm의 결과가 A.6.1의 native 기준 설계 M₀^K(q_req)이다 |

## 3. 결과

### 3.1 최선 C̄_pad

| 구조 | q_req | 3 mm | 5 mm | 7 mm | 9 mm |
|---|---|---|---|---|---|
| CFL | 90° | 0.802 | 0.778 | 0.750 | 0.717 |
| | 120° | 0.781 | 0.753 | 0.725 | 0.695 |
| | 150° | 0.770 | 0.739 | 0.707 | 0.674 |
| 2R-remote | 90° | 0.843 | 0.809 | 0.773 | 0.736 |
| | 120° | 0.815 | 0.758 | 0.697 | 0.633 |
| | 150° | 0.740 | 0.593 | 0.209 | 0.000 |
| 2R-DD | 90° | 0.749 | 0.726 | 0.708 | 0.693 |
| | 120° | 0.717 | 0.700 | 0.682 | 0.633 |
| | 150° | 0.710 | 0.593 | 설계 없음 | 설계 없음 |
| RCJ | 90° | 0.698 | 0.663 | 0.634 | 0.606 |
| | 120° | 0.644 | 0.608 | 0.579 | 0.555 |
| | 150° | 0.607 | 0.560 | 0.514 | 설계 없음 |
| CFL ideal* | 90° | 0.803 | 0.779 | 0.756 | 0.733 |
| | 120° | 0.788 | 0.761 | 0.733 | 0.704 |
| | 150° | 0.779 | 0.746 | 0.713 | 0.679 |
| RCJ ideal* | 90° | 0.803 | 0.787 | 0.775 | 0.745 |
| | 120° | 0.790 | 0.781 | 0.738 | 0.694 |
| | 150° | 0.788 | 0.755 | 0.696 | 0.636 |

\* servo 몸체가 없는 상한. 2R-remote 150°의 7, 9 mm 설계는 제약을 만족하지만 pad가 원기둥에 닿지 못한다(0.209, 0).

### 3.2 H1: CFL 대 2R-DD (%p)

| q_req | 두께 | 최선 차이 | CFL 독립 run만(재탐색 없음) | CFL 최악 run − 2R-DD 최선 |
|---|---|---|---|---|
| 90° | 3 / 5 / 7 / 9 mm | +5.4 / +5.2 / +4.1 / **+2.4** | +5.4 / +3.1 / +2.3 / +0.5 | +2.8 / +3.5 / +2.6 / **−0.6** |
| 120° | 3 / 5 / 7 / 9 mm | +6.4 / +5.3 / +4.3 / +6.3 | +6.1 / +4.8 / +2.9 / +5.5 | +5.9 / +4.2 / +3.9 / +5.5 |
| 150° | 3 / 5 mm | +5.9 / **+14.6** | +5.6 / +14.3 | +5.4 / +13.6 |
| 150° | 7 / 9 mm | 2R-DD 설계 없음 | | |

H1은 10개 조건 모두에서 성립한다(2.4–14.6 %p). 재탐색 없이도 모두 양수이고, 최악 run으로도 10개 중 9개에서 양수이다.

### 3.3 CFL 대 2R-remote, 두께와 flexion에 따른 손실

- **90°:** 모든 두께에서 2R-remote가 높다. 차이 4.1 / 3.1 / 2.3 / 1.9 %p로 두께와 함께 준다.
- **120°:** 5 mm에서 2R-remote가 0.5 %p 높고 7 mm에서 CFL이 2.8 %p 높다. 선형 보간 교차점은 약 5.3 mm. 6 mm에서 두 구조를 실제로 최적화해 확인: 2R-remote 0.729, CFL 0.740(보간 0.728, 0.739와 0.002 이내). 교차는 CFL run의 흩어짐 안에 있어 근사로 말한다.
- **150°:** 모든 두께에서 CFL이 높다.
- **3 → 9 mm 손실:** CFL 0.085 / 0.086 / 0.096(90 / 120 / 150°), 2R-remote 0.107 / 0.182 / 0.740. 5 mm에서 q_req 90° → 150°의 손실은 CFL 0.039, 2R-remote 0.216.
- **RCJ:** 실제 hardware로는 가장 낮고 ideal은 높다. 손실이 구조의 운동이 아니라 packaging(roller와 servo)에서 온다.

### 3.4 CFL 최적 설계

| q_req | 두께 | proximal + distal pad (mm) | pad coverage (ℓ_p + ℓ_d)/L_f | q_max | MA_wc | 멈춘 이유 |
|---|---|---|---|---|---|---|
| 90° | 3 / 5 / 7 / 9 mm | 36.6+36.6 / 36.5+36.5 / 35.4+35.4 / 33.6+33.6 | 0.814 / 0.811 / 0.786 / 0.746 | 115.7 / 91.1 / 120.3 / 90.0° | 0.45 / 0.41 / 0.46 / 0.53 | transmission (9 mm: collision) |
| 120° | 3 / 5 / 7 / 9 mm | 34.5+34.6 / 34.4+34.4 / 34.4+34.5 / 33.1+33.2 | 0.767 / 0.764 / 0.765 / 0.736 | 180° (curl) | 0.58 / 0.55 / 0.51 / 0.53 | curl |
| 150° | 3 / 5 / 7 / 9 mm | 35.4+35.4 / 35.1+35.1 / 34.5+34.6 / 33.0+33.0 | 0.786 / 0.780 / 0.767 / 0.733 | 180° (curl) | 0.43 / 0.42 / 0.43 / 0.47 | curl |

MA_wc 범위 0.41–0.58. 다른 구조의 MA는 구조가 정하는 상수(2R-DD, 2R-remote 1, 대칭 RCJ 0.5).

### 3.5 탐색의 수렴

- 2R과 RCJ의 독립 run은 조건마다 같은 값에 도달했다(run 간 차이 0, RCJ 150°/7 mm만 0.9 %p).
- CFL의 독립 run은 1.3–3.4 %p 흩어졌고, 재탐색으로 최대 2.1 %p(90°/5 mm: 0.757 → 0.778) 올랐다.
- 마지막 단계 후 CFL·CFL ideal의 모든 5/7/9 mm 칸은 Study 2가 그 두께에서 찾은 최선 이상이다. 알려진 설계 중 더 높은 것은 없지만 전역 최적의 증명은 아니다.

### 3.6 보조: 고정한 손가락에 module을 다는 평가 (2026-11-20, 5 mm native 설계)

A.6.2의 장착 규칙(바깥 끝에 맞춤, 가장 얕은 깊이, spacer 없음)으로 71개 module(combined / proximal-only / distal-only 각 21개: 15–45 mm × 3/5/8 mm, named 8개)을 각 구조의 native 설계에 달았다. 칸: 받아들인 module 수(combined / proximal / distal), 양쪽에 같은 module을 달 때 받아들인 최대 pad coverage(module).

| 구조 | 90° | 120° | 150° |
|---|---|---|---|
| CFL | 11/12/12 · 0.67 (30×5) | 11/12/12 · 0.67 (30×5) | 12/12/12 · 0.67 (30×8) |
| 2R-remote | 14/15/15 · 0.78 (35×5) | 11/12/12 · 0.67 (30×5) | 4/6/8 · 0.44 (20×5) |
| 2R-DD | 11/12/12 · 0.67 (30×5) | 9/9/9 · 0.56 (25×8) | 4/6/8 · 0.44 (20×5) |
| RCJ | 9/9/9 · 0.56 (25×8) | 8/9/9 · 0.56 (25×5) | 6/6/9 · 0.44 (20×8) |
| CFL ideal* | 14/15/15 · 0.78 (35×5) | 14/15/15 · 0.78 (35×5) | 12/15/12 · 0.67 (30×8) |
| RCJ ideal* | 14/15/15 · 0.78 (35×5) | 11/12/12 · 0.67 (30×5) | 10/12/12 · 0.67 (30×5) |

실패는 대부분 장착 단계(module이 phalanx보다 김, 2R은 관절 hardware 때문에 장착 깊이 없음)이다. 5 mm 격자라 30/35 mm 경계에 동률이 많다. 이 native 설계 값은 그 뒤 재탐색으로 바뀐 칸이 있다(3.5).

## 4. 한계와 범위

- 2D 측면 모델, L_f 90 mm, 두 pad 같은 두께, 원기둥 기준 C̄_pad라는 coverage 정의 안에서의 비교이다.
- 각 점은 그 조건만을 위해 설계한 손가락의 상한이다. 손가락 하나로 여러 두께를 쓰는 경우는 Study 2.
- "설계 없음"은 탐색 범위와 예산 안에서 찾지 못했다는 뜻이고, 존재하지 않는다는 증명이 아니다. 값은 찾은 최선이며 전역 최적의 증명이 아니다.
- Ideal* 구조는 만들 수 없는 상한이며 순위 비교에 넣지 않는다.
- 가림과 MA는 목적에 없다. MA_wc는 결과로만 보고한다.

## 5. 데이터와 스크립트

| 내용 | 경로 |
|---|---|
| 조건별 최선 설계 요약 (커밋된 사본) | `report/study1/data/thickness_coverage_summary.csv` (원본 `design/output/study/study1_thickness_coverage/summary.csv`). 열: `c_pad_mean`(최선), `c_pad_independent`(독립 run만), `c_pad_run_low`(최악 run), `runs`, `source`, pad 길이, `q_max_deg`, `ma_wc` |
| Raw 표와 metadata | `report/study1/data/raw/coverage.csv`, `report/study1/data/raw/metadata.json` (`report/make_raw_tables.py`가 생성) |
| 탐색 | `design/scripts/study/study1_thickness_coverage/run_all.sh`, `run_continuation.sh`, `run_from_study2.sh` → `design/scripts/optimize/run.py`, 구조별 설정 `design/scripts/study/{cfl,2r_dd,2r_remote,rcj,cfl_ideal,rcj_ideal}.yaml` + `base.yaml` |
| 요약 생성 | `design/scripts/study/study1_thickness_coverage/figure.py` |
| 3.6 module 평가 | `design/scripts/study/study1_fixed_morphology/run.py`, `modules.yaml`, 기준 `design/scripts/study/study2_morphology_adaptation/references_native.yaml` → `design/output/study/study1_fixed_morphology_native/assembly_results.csv` (4686 assemblies, 추적 안 함) |
| 6 mm 교차 확인 | `paper/orchestra/experimental_log.md` (데이터는 `report/study1/data`에 없음) |

대체된 것:
- `study1_fixed_morphology_2026-11-19.md`(matched): 한 frozen morphology(`references_nominal15.yaml`)에 모든 구조를 맞춘 비교. A.6.1이 외형 맞추기를 금하므로 쓰지 않는다. 출력 `design/output/legacy/study1_fixed_morphology_tilt_sweep/`.
- `study1_fixed_morphology_native_2026-11-20.md`의 5 mm 값(예: CFL 90° 0.757, 120° 0.748)은 5 mm 재탐색 결과를 읽지 못하던 폴더 이름 오류와 탐색 부족으로 낮았다. 이 문서의 5 mm 칸(0.778, 0.753)으로 대체한다. 같은 문서의 tilt 민감도 결과는 쓰지 않는다.
