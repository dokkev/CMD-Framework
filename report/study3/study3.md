# Study 3: 손가락 형태가 바뀔 때 interface 효과 측정은 무엇을 지켜야 하는가 (시뮬레이션)

2026-11-24에 `experiments.md`(실험 3–29의 사전 기록 로그), `regrasp_geometry.md`, `study3_pilot.md`를 이 파일 하나로 줄였다. 계획 전문, 소규모 샘플, 수정 이력은 git에 남아 있다(4절의 commit). 주장과 범위의 SSoT는 `design/SSoT/research-objective.md` §6(H3/R3, H4/R4)과 §8, 정의는 `problem_formulation.md` A.6.4–A.6.5이다. 숫자가 이 파일과 SSoT에서 다르면 SSoT가 맞다.

## 1. 질문과 주장

**질문 (SSoT §8.1).** 기준 손 H₀를 더 두꺼운 interface에 맞춰 다시 설계하면, 측정한 interface 효과 Δ(H) = M(H, c₁) − M(H, c₀)에 재설계가 오차를 얹는다. 오차는 E(H) = |Δ(H) − Δ(H₀)| / |Δ(H₀)|이다.

> When the finger's morphology must change to carry a thicker interface, what must the change keep first so that the same commands still handle the object the same way, and the interface's measured effect stays the one measured on H₀: the contact-surface motion (Δ_contact), the skeleton's (Δ_skel), or another candidate?

morphology는 skeleton(링크와 관절)과 그것이 싣는 접촉면을 함께 말한다. 답은 명령이 접촉을 감지하는지에 따라 달라질 것으로 보고, 양쪽을 보이는 프로토콜을 골랐다.

**H3 / R3.** 서보 손이 쓰는 임피던스 setpoint(한 굽힘 명령, 같은 강성; 조건 A)를 재설계 사이에 재사용하면 E는 Δ_contact를 따른다. 반증: Δ_contact가 예측하지 못하거나, 다른 후보(Δ_skel, coverage, pad 두께, 전달)가 모든 설정에서 예측한다(손 bootstrap 95% 구간이 모두 0을 제외). 결과: 마찰 ρ 0.83(15개 손), pad 강성 0.80(14), 손과 무관한 격자 0.86(45), torque 재사용 0.75(16). 모든 설정에서 구간이 0을 제외하는 후보는 Δ_contact뿐이다. 두 번째 엔진·접촉 모델(Drake point, hydroelastic)과 다른 두 과제(grasp 획득, 손끝 원판)에서도 성립하고, in-hand 굴리기에서는 성립하지 않는다. 접촉을 감지해 닫으면(조건 E) 남는 오차는 Δ_skel을 따르고, torque를 재사용하면 skeleton도 지켜야 한다.

**H4 / R4 (Study 3의 마지막 부분).** O의 딱딱한 3 mm pad를 두께와 함께 부드러워지는 pad로 바꾸는 interface 연구에서, 재설계 손이 잰 pull-out 하중 변화는 Δ_contact가 클수록 진짜 변화(재설계 없이 O에 그 pad를 단 반사실 기준)에서 멀어진다. (1) 상대 오차와 (2) 결론이 뒤집힌 배치 비율 모두 Δ_contact와 양의 상관(단측 순열 p < 0.05). 확인 실험(19차)에서 (1)은 성립, (2)는 실패.

**범위 (SSoT §8.3).** 시뮬레이션, 두 엔진, 두 접촉 모델. CFL 한 메커니즘(고정축 대조 계열 하나), 요구 굽힘 120° 하나, 열린 루프 과제 셋(pull-out, grasp 획득, 손끝 원판; 굴리기는 실패), interface 속성 셋(마찰, compliance, 형상). 주장은 접촉을 감지하지 않는 명령(재사용 관절 명령, 기구학으로 다시 정한 setpoint, skeleton도 지킨 경우의 재사용 torque)에 한정한다. 손 형태 차이 일반이 접촉면에서 온다거나, 정책 성능이 보존된다거나, skeleton을 많이 바꾸는 것이 좋다고 주장하지 않는다.

## 2. 공통 설정

**손.** 모두 CFL, q_req 120°. O = H₀ = 3 mm 설계. 재설계는 각 두께에서 R(Study 2의 최대 coverage 재설계, Δ_contact 최소), U(Study 1의 첫 독립 설계, coverage만 봄), 그리고 설계 지표만 보고 고른 e18 재설계(Δ_contact 목표마다 coverage 최대, Δ_skel 최소/최대)다. 이 18개가 주 집합이다. 고정 장착 깊이 정책상 `d1p5_maxDs_s0`, `d3_maxDs_s0`의 장착 깊이는 proximal/distal 4.6/3.0, 5.9/3.0 mm다(spacer 아님).

| 손 | h | Δ_contact | Δ_skel | C̄_pad | | 손 | h | Δ_contact | Δ_skel | C̄_pad |
|---|---|---|---|---|---|---|---|---|---|---|
| `h3to5_hr` (R₅) | 5 | 0.35 | 3.55 | 0.753 | | `d2p25_maxDs_s0` | 7 | 2.30 | 9.17 | 0.712 |
| `h3to5_hu` (U₅) | 5 | 1.58 | 2.02 | 0.748 | | `d2p25_minDs_s0` (H_M) | 7 | 2.33 | 4.62 | 0.700 |
| `h3to5_d3_s0` | 5 | 2.92 | 6.38 | 0.750 | | `d3_maxDs_s0` | 7 | 3.04 | 11.55 | 0.700 |
| `h3to5_d3_minDs_s0` | 5 | 3.09 | 0.38 | 0.728 | | `d3_minDs_s0` | 7 | 3.09 | 3.83 | 0.713 |
| R (`hr`) | 7 | 0.86 | 7.67 | 0.725 | | `d4p5_s0` | 7 | 4.41 | 11.33 | 0.716 |
| `d1p5_maxDs_s0` | 7 | 1.46 | 9.67 | 0.707 | | `d5p5_s0` | 7 | 5.40 | 12.17 | 0.703 |
| `d1p5_minDs_s0` | 7 | 1.60 | 5.37 | 0.706 | | `h3to9_hr` (R₉) | 9 | 2.32 | 9.37 | 0.695 |
| U (`hu`) | 7 | 3.91 | 3.38 | 0.711 | | `h3to9_hu` (U₉) | 9 | 3.89 | 7.17 | 0.688 |
| | | | | | | `h3to9_d3p5_maxDs_s0` | 9 | 3.52 | 12.72 | 0.672 |
| | | | | | | `h3to9_d5_minDs_s0` | 9 | 5.08 | 5.32 | 0.673 |

(mm. 이름 규칙: dN = 목표 Δ_contact, p = 소수점, h3toN = 3 → N mm, maxDs/minDs = 그 목표에서 skeleton 변화 최대/최소, _s0/_s1 = 탐색 seed.)

- **격자 손 27개 (13·18·19차).** e18 seed 1, 7 mm, Δ_contact 목표 9개(1.0–4.0 mm) × skeleton 조건 3개(none/min/max). 주 집합과 합쳐 45개(`dose_analysis.EXPAND`, `designs_expand.csv`).
- **고정축 대조 계열 (14차).** O의 joint gap 중앙, 링크선 위 고정 PIP 하나, θ = q. pad 3/5/7/9 mm, 관절 쪽 끝은 clearance law의 gap(20.8/27.7/34.6/41.6 mm). Δ_skel = 0, Δ_contact = √3·Δh = 3.46/6.93/10.39 mm(bound와 같음). 자기 3 mm 손이 기준이다.
- **naive swap.** O의 skeleton에 두꺼운 pad를 끼우면 pad가 47°에서 충돌한다(요구 120°). 모든 두께에서 기계적 불가능으로 기록하고 측정하지 않았다.

**두 경로 변화.** Δ_contact = proximal face 좌표계(proximal phalanx와 함께 도는 좌표계, 원점은 MCP 축이 face에 내린 발)에서 본 distal face 기준점(손끝 아래 face 위의 점) c(q)의 변화, max_q ‖c_H(q) − c_O(q)‖ (0–q_req). Δ_skel = 손끝 경로(링크선)의 같은 변화. 둘은 contact Jacobian J(q) = [c′(q); 1]의 같은 pole을 face와 링크선에서 본 것이다: ‖Δc‖ = 2 sin(q/2)‖Δp‖, Δp_face = Δp_link − Δd e_z(R에서 0.01 mm 안으로 확인). 회전 성분은 모든 설계에서 1이므로 관절 명령으로 Δ_contact를 되돌릴 수 없다(φ = q). 전달비 변화는 각 손의 θ(q) 표로 보상한다. 로그의 옛 이름 RCT, D_c, D_s는 각각 contact-surface motion, Δ_contact, Δ_skel이다.

**측정 (pull-out, Isaac Lab 3, Newton + MuJoCo-Warp, point contact).**
- 원기둥(ρ 15, 20 mm)이 평면 3자유도로 자유롭고 중력은 끈다. O의 두 pad가 굽힘 q0에서 닿는 자리에, 각 손의 proximal face 위 같은 높이에 둔다(q0 5° 간격; 유효 배치는 사실상 q0 85–115°). 손가락 셋이 같은 원기둥을 쥐고 MCP는 고정.
- 절차: 원기둥을 잡은 채 PIP를 setpoint로 닫음(40 + 30 step) → 30 step에 걸쳐 놓고 90 step 안정화 → Gate 0 → 네 방향(+x 손끝, +x−z, −z 면에서 뗌, −x MCP 쪽) 외력을 2 N/s로 80 N까지. 한계 하중 F_limit은 첫 사건: slip(접선 속도 10 mm/s), contact loss(0.2 N 미만), collapse(5 mm 이동).
- 마찰 μ 1.3 → 2.0(O에서 미리 검증: 미끄러질 때 |f_t|/|f_n| = 1.30/2.00), impratio 1000(기본 10에서는 creep으로 Gate 0 0/12, O smoke 뒤 수정), 접촉 강성 ke 2.5 kN/m, substep 2. 외력 한 step 0.067 N = Δ(O)의 2.5–5.6%.
- **D와 E.** D(H, d) = F_limit(μ 2.0) − F_limit(μ 1.3). 쌍은 같은 배치·방향·반지름에서 O와 H가 두 μ 모두 Gate 0을 통과하고 한계가 있는 것. E(H) = 쌍들의 |D(H) − D(O)| / |D(O)| 중앙값. 보조로 대칭 지표 |ln(D_H/D_O)|.
- **Gate 0 (환경별, 외력 전).** (i) 모든 손가락에서 proximal, distal 수직력 각 0.2 N 이상, (ii) 마지막 10 step 물체 속도 < 2 mm/s, 각속도 < 0.1 rad/s, (iii) 두 법선 사이 각 = 측정 q* (±2°; hydro 4°). 조건 B는 distal 힘이 목표의 ±5% 안. 실패는 손·μ별로 세고 점수에서 뺀다(손마다 대략 36–89%, 대부분 q0 ≤ 80°의 release).
- **과제 gate.** O의 효과가 분명한 쌍만 쓴다(§8.1-5): |D(O)| ≥ 0.3 mm(획득) 또는 0.3°(다이얼), 굴리기 0.5°. 획득에서 H가 끝에 쥐지 못하면 가장 큰 오차로 둔다. 형상 interface는 O의 효과가 두 방향 이상에서 3 step(0.2 N) 이상이어야 했다.
- **관통 gate (28·29차).** point contact의 pad 관통 = distal 힘 / 2.5 kN/m. O의 최대 관통 1.5 mm 초과 배치는 빼고, H가 넘으면 over-pressed로 세어 점수에서 뺀다. 26차 다이얼의 문제(3(d))에서 나온 규칙이다.
- **잡음 바닥.** O를 명령 +0.05°로 다시 돌린 run과의 E(중앙값, 75분위).
- **통계.** 손 단위 Spearman ρ, 단측 순열 p. 손 ≤ 8개(Isaac) 또는 ≤ 9개(Drake)는 정확 순열, 그보다 많으면 무작위 순열이다: Isaac `dose_analysis.py`·`padk_matched.py` 100,000회(seed 0), Drake `spearman_perm` 20,000회(seed 0). 손 bootstrap 95% 구간, partial rank 상관, R 대 U는 배치·방향 쌍의 단측 Wilcoxon.

**Drake 복제 (21차부터, `sim/drake/two_pad_drake.py`, Drake 1.58 SAP).** 평면 CoDe 손가락 하나, MCP 0° 고정, four-bar는 거리 제약으로 닫음. 서보 관절 implicit PD, 이득 K_Q/(dθ/dq)², 조건 A, 같은 gate와 하중 램프. 하중·질량·임계값은 Isaac의 1/3이고 F_limit은 3배로 보고. time step 1 ms, solver 실패 배치는 0.5 ms로 한 번 재시도(`retried_time_step`).

**프로토콜.**

| | 재사용하는 것 | 설명 |
|---|---|---|
| A | O의 굽힘 setpoint q0 + 5°, 같은 굽힘 강성 K₀ 5 N·m/rad | 주 조건. 서보 손의 임피던스 setpoint 재사용 |
| B | O의 distal 접촉력 | 시뮬레이터 정답 힘으로 되먹임. 현실 불가능한 보상의 상한(oracle) |
| C | O의 굽힘 torque τ_O | 닫은 뒤 강성 0, feedforward τ_O (각 손의 전달비로 servo torque) |
| D | q_ref = q*_O, τ_ff = τ_O, K_q/K₀ ∈ {0.1, 0.3, 1, 3, 10} | C(K→0)와 A(K = K₀) 사이. D_q = 0.08 √(K_q/K₀) |
| E | 없음 | 각 손가락이 distal 수직력 0.05 N에서 q_touch를 재고 q_touch + 5°로 닫음 |
| 27차 | 기구학으로 다시 정한 setpoint | q_cmd,H = argmin_q ‖c_H(q) − c_O(q_cmd,O)‖. 런타임 접촉 없음, 임피던스는 A와 같음 |

## 3. 결과

모든 행에서 E_R / E_U는 같은 두께의 R과 U, ρ는 점수를 받은 재설계 손 전체. p는 단측 순열 p. [ ]는 손 bootstrap 95% 구간.

### (a) interface 속성 (Isaac, point, 조건 A, O 기준 배치)

| 설정 | 차 | 손 | ρ(E, Δ_contact), p | ρ(E, Δ_skel) | E_R / E_U (5, 7, 9 mm) | 판정 |
|---|---|---|---|---|---|---|
| 마찰 μ 1.3 → 2.0, 주 (`dose_A_all`) | 6 | 15 | 0.83 [0.41, 1.00], 0.0001; partial \| Δ_skel 0.88 | −0.40 [−0.78, 0.12] | 3 / 52, 21 / 124, 41 / 106% | 통과 |
| pad 강성 ke 2.5e3 → 1e3 N/m (`comp`) | 10 | 14 | 0.80 [0.33, 0.99], 0.0004; partial 0.91 [0.68, 0.99] | −0.51 [−0.85, 0.04] | 3 / 75, 25 / 173, 61 / 150% | 통과 |
| 형상 flat → 2 mm crowned, μ 1.3 | 8 | 6 (7 mm) | 0.94, 정확 p 0.008 | −0.89 | 27 / 157% (7 mm) | 통과, 그러나 둘을 가르지 못함 |
| 두께와 함께 부드러워지는 pad(ke 2.5 × 3/h kN/m), 기준 O_h(같은 pad를 단 O) (`padk_matched`) | 17 | 14 | 0.81 [0.33, 0.99], 0.0003 | −0.46 [−0.82, 0.08] | 4.7 / 54.9, 13.6 / 128.6, 50.0 / 124.0% (SSoT 5, 14, 50 / 55, 129, 124%; Wilcoxon p ≤ 1.1e-7, R<U 88%) | 통과 |

- 마찰: 두께별 ρ 1.00 (n 3), 0.93 (8), 0.80 (4). 표준화 회귀 E ~ +0.84 Δ_contact − 0.45 Δ_skel + 0.08 Δh(두께 자체는 거의 설명하지 않음). 9 mm의 `h3to9_d5_minDs_s0`는 157%이고, R₉(41%)는 가장 작은 `h3to9_d3p5_maxDs_s0`(Δ_contact 3.52, skeleton 가장 많이 움직임; 40%)와 1점 안이다. 쥐지 못한 손: `d4p5_s0`, `d5p5_s0`, `h3to5_d3_s0`(놓을 때 원기둥이 4–9 mm 밀려남; SSoT는 distal face가 원기둥에서 1.5–2.6 mm 물러나는 세 재설계라고 쓴다). 같은 Δ_contact에서 Δ_skel을 두 배로 해도(d2.25 쌍 4.6 대 9.2 mm) E 72 대 70%.
- 강성: Δ_contact를 고정한 Δ_skel의 partial은 −0.79 [−0.95, −0.37]. "skeleton 변화가 오차를 더하지 않는다"까지만 말한다.
- 형상: crowned에서 유효 쌍이 1–14개뿐이고 이 6개 손에서 Δ_skel이 Δ_contact와 반대로 움직이므로 순서를 반복할 뿐 둘을 가르지 못한다.
- 두께와 함께 부드러워지는 pad: 기준은 같은 pad를 단 O_h다. interface 연구는 같은 interface를 단 기준과 비교해야 차이를 interface에 돌릴 수 있다. 재설계 손 run은 15차 run을 쓰고, 기준과 판정은 기준 run 전에 정했다. 딱딱한 O를 기준으로 한 15차 판정은 기준이 pad 자신의 효과를 담지 않아 Study 3에서 제외했다. O_h 검증: Gate 0 52–60/132, 유효 마찰 1.30/2.00, Δ(O_h) 1.43, 1.20, 1.00 N.
- 못 쥐는 손을 가장 큰 오차로 넣으면 ρ 0.81(마찰), 0.67(강성). 두께 안에서만 bootstrap하면 [0.41, 0.99], [0.35, 0.98].

### (b) 시뮬레이터 (조건 A, 마찰, pull-out)

| 엔진 | 차 | 손 | ρ(E, Δ_contact), p | ρ(E, Δ_skel) | E_R / E_U (7 mm) | 비고 |
|---|---|---|---|---|---|---|
| Isaac Lab (Newton, MuJoCo-Warp), 3 손가락 | 6 | 15 | 0.83, 0.0001 | −0.40 | 21.0 / 123.6% | 주 결과 |
| Drake SAP, 1 손가락 | 21 | 16 | 0.67, 0.0027 (SSoT 0.003) | −0.37 | 19.4 / 71.7% | O의 Gate 0 28, 28, 28, 32 / 64–68 (Isaac 28, 28, 28, 29), 통과는 q0 ≥ 85° |

- U의 크기 차이는 collapse 하중(서보 모델)의 차이로 보인다. R < U와 순서는 같다.
- Isaac Lab의 hydroelastic 경로는 쓸 수 없었다: 같은 설정의 O 두 run의 E가 30%(point에서 명령 0.05° 변화는 2.2%), 결정적 모드는 MuJoCo-Warp에서 돌지 않음. 16차(Isaac hydro, 1 MPa, 같은 pad)는 O의 마찰 검증 실패(유효 마찰 0.77 / 1.80, 비 2.35)로 재설계 데이터 전에 중단했다. 원인 하나는 Newton 기본 contact gap 10 mm(곡면 hydro 접촉의 법선이 43–88° 기울어 마찰을 부풀림)였고, gap 0으로 Gate 0은 회복했지만 손 수준 마찰은 여전히 1.00–1.12로 설정을 따르지 않았다.

### (c) 접촉 모델 (Drake, 조건 A, 마찰, pull-out)

| 접촉 모델 | 차 | 손 | ρ(E, Δ_contact), p | ρ(E, Δ_skel) | E_R / E_U | 판정 |
|---|---|---|---|---|---|---|
| point 2.5 kN/m | 21 | 16 | 0.67, 0.0027 | −0.37 | 19.4 / 71.7% | 통과 |
| hydroelastic, 모든 손에 O의 pad(0.1 MPa / 3 mm 기초) | 21 | 16 | 0.86, < 10⁻⁴ | 0.09 (p 0.38) | 16.7 / 253.8% (SSoT 17 / 254%) | 통과 |
| hydroelastic, pad가 자기 두께만큼 부드러움(0.1 MPa / h), 기준 O_h | 25 | 15 | 0.78, 0.0003 (못 쥐는 3개 포함 0.87, < 10⁻⁴) | −0.10 (0.65) | 5.9 / 109.4, 16.3 / 347.2, 104.2 / 370.4% (5, 7, 9 mm) | 통과 |
| Isaac point, ke ∝ 1/h, 기준 O_h | 17 | 14 | 0.81, 0.0003 | −0.46 | 4.7 / 54.9, 13.6 / 128.6, 50.0 / 124.0% | 통과 ((a) 참조) |

- 21차 pad 강성 0.1 MPa는 O만으로 정했다(1 MPa에서는 O의 안정화 힘이 point의 약 10배). Drake 마찰 검증: point 1.33 / 2.06, hydro 1.45 / 2.28(비 1.57, 설정 1.54).
- 두 모델의 손별 E 순위는 약하게만 일치한다: ρ(E_hydro, E_point) = 0.39, p 0.069. 순서는 Δ_contact를 따르지만 크기는 모델에 달려 있다.
- solver: 5,016 배치 중 150개 재시도, 117개(2.3%) 최종 실패로 제외. `d4p5_s0`, `d5p5_s0`는 두 모델 모두 쌍 없음.
- 25차: 못 쥔 손 `d4p5_s0`, `d5p5_s0`, `h3to9_d5_minDs_s0`(μ 1.3에서 Gate 0 0–4/64–68). Δ_contact 1 mm 이하에서는 point와 같고(R 6, 16%), 그 위에서는 2–3배 크다(R₉ 104%). 순서는 같고 허용 범위가 좁아진다. Δ(O_h) 2.67, 2.13, 1.83 N. 잡음 바닥(O_7) 2.4%(75분위 3.4%).

### (d) 과제 (Drake point, 열린 루프, 재사용 명령, 손 18개)

| 과제 | 차 | 손 | ρ(E, Δ_contact), p | ρ(E, Δ_skel) | E_R / E_U | 잡음 바닥 (중앙 / 75분위) | 판정 |
|---|---|---|---|---|---|---|---|
| pull-out | 21 | 16 | 0.67, 0.0027 | −0.37 | 19.4 / 71.7% | — | 통과 |
| grasp 획득(펴진 손가락이 하중을 거슬러 원기둥을 끌어들임) | 23 | 18 | 0.87, < 10⁻⁴ | 0.11 (0.33) | 21.3 / 74.1% | 0.2 / 0.3% | 통과 |
| **손끝 원판 z (대표 구성)** | 28z | 18 | 0.78, 0.0001 | 0.20 (0.21) | 17.0 / 46.8% | 0.9 / 1.4% | 통과 |
| 손끝 원판 z, 무작위 시작 64개 | 29z | 18 | 0.68, 0.0009 | −0.00 (0.51) | 19.3 / 54.7% | 1.24 / 2.49% | 통과 |
| 손끝 동전 끌기(긴 끌기, 무작위 시작 64개) | 30b | 18 | 0.55, 0.0098 | −0.48 (0.98) | 2.2 / 16.3% | 0.60 / 1.05% | 통과 |
| 엄지–검지 집기 회전(무작위 시작 64개) | 33 | 18 | 0.51, 0.016 | 0.16 (0.27) | 1.7 / 11.4% | 0.99 / 1.94% | 통과 |
| in-hand 굴리기(굽힘 sweep) | 22 | 15 | 0.28, 0.16 | −0.25 | 26.8 / 34.8% | 0.8 / 1.7% | **실패** |
| 두 pad 다이얼, 5–10° | 24 | 18 | 0.82, < 10⁻⁴ | 0.18 (0.24) | 27.1 / 65.8% | 0.2 / 1.1% | 통과, **대체됨** |
| 두 pad 다이얼, 30° | 26 | 18 | 0.81, 0.0001 | 0.09 (0.36) | 31.1 / 88.0% | 0.7 / 1.6% | 통과, **대체됨** |

- **획득 (23차).** q = 0에서 시작, 원기둥은 O의 접촉 위치보다 손끝 쪽 a = 3, 6 mm, 하중 face 쪽 1.5 N + 손끝 방향 1.5 N(세 손가락 기준), 2 s에 O의 명령까지 닫고 2 s 유지, 회전 감쇠 2.0 N·m·s/rad. 지표는 MCP 쪽으로 끌려온 거리. E 3.2%(`h3to5_hr`)–101.6%(`d5p5_s0`). 끝에 못 쥔 쌍은 Δ_contact가 큰 손에 몰린다(`d4p5_s0` 15/25, `d5p5_s0` 16/25).
- **손끝 원판 (28z).** 반지름 15 mm, 두께 4 mm 원판. 축은 proximal face에 수직(z), 손가락 평면에서 y_off 옆. distal pad가 윗면을 누르며 MCP 쪽으로 끌어 비틀림 스프링(0.01 N·m/rad)을 거슬러 돌린다. proximal pad는 닿지 않는다(최소 간격 14.9 mm). 축은 z 방향 60 N/m 스프링. 명령 0 → q_t(1 s) → q_t + 40°(3 s) → 유지 1 s. 배치 12개(q_t 15/25/35°, x_b 10/14 mm, y_off 8/12 mm), O의 회전 12.1–38.4°(12–38°). 손별 최대 관통 0.78–1.47 mm, lost·over-pressed 없음.
- **손끝 동전 끌기 (30b차).** distal pad 하나가 탁자 위 동전(두께 2 mm, 3 g, 지름 28–32 mm)을 눌러 MCP 쪽으로 끈다. 동전은 시작 위치에 끈(45 N/m)으로 매여 있고(끈이 없으면 pad 마찰이 늘 탁자 마찰을 이겨 μ 효과가 0), 탁자는 30 N/m 스프링 위에 있어 누름이 순응으로 제한된다. 동전–탁자 μ 0.2. 동전은 y 이동을 막았다. 명령 0 → q_t(1 s) → q_t + s(4 s, s 45–55°) → 유지 1 s. 지표는 끝 이동, D = 이동(μ 2.0) − 이동(μ 1.3). 무작위 시작 64개(seed 31: q_t U(25, 38°), 손끝이 닿는 점 edge U(2, 5) mm, 탁자 높이 ±0.5 mm, 지름 U(28, 32) mm, sweep U(45, 55°)), 모든 손에 같은 뽑기. O는 64개 모두 남고 8.5–13.1 mm를 끈다(μ 2.0에서 13.9–21.3 mm, D(O) 5.3–8.2 mm, 모두 양수). 관통 최대 O 0.89 mm, 재설계 손 0.86–1.14 mm, lost·over-pressed·proximal 접촉 없음. time step 0.25 ms(1 ms에서는 미끄럼 접촉이 떨려 잡음 바닥 4.6%). 사후 관찰(판정에 쓰지 않음): 순서를 거스르는 손은 다이얼과 같은 `d4p5_s0`, `d5p5_s0`, `h3to5_d3_s0`이다. 끌린 거리는 누름, 즉 손끝 경로의 수직 성분을 따르는데 Δ_contact는 방향 없이 경로 변화를 잰다. 같은 과제의 짧은 끌기(30차, O 1.8–4.3 mm)는 판정 2가 성립하지 않았다(ρ 0.36, p 0.073; E_R 7.2%, E_U 22.5%). 그 기록은 `docs/experimental/drake_coin.md`에 남긴다.
- **엄지–검지 집기 회전 (33차).** 검지 distal pad와 엄지 pad 사이에 구(지름 16–24 mm)를 쥐고, 검지를 O의 열린 루프로 더 굽혀 구를 비틀림 스프링(0.25–0.40 N·m/rad)에 맞서 조금 돌린다. 재설계되는 것은 검지뿐이다. 엄지는 모든 손에서 같은 pad(같은 point stiffness와 μ)로, 구의 맞은편에서 법선 방향 40 N/m 스프링이 preload 0.4–0.8 N으로 민다. 쥐는 힘은 이 스프링이 정한다. 구는 손가락 평면에서 움직이고 돈다. 명령은 구를 잡은 채 0 → q_s(1 s), 유지 0.5 s, 놓고 0.5 s, 그 뒤 q_s → q_s + sweep(2 s, sweep 18–28°), 유지 0.5 s이다. 지표는 sweep 시작부터의 구 회전, D = 회전(μ 2.0) − 회전(μ 1.3)이다. 무작위 시작 64개(seed 33: q_s U(30, 55°), 접촉점이 O의 pad 끝에서 3–8 mm 뒤, 반지름, preload, sweep, k_r)는 모든 손에 같다. 보정은 O만으로 했다. 스프링 없는 구는 미끄러지지 않고 굴러 D = 0이었고, 단단한 엄지(100 N/m)는 관통 1.5 mm를 넘었다. O는 64개 모두 남고 2.3–7.1°를 돌린다(D(O) 1.3–4.5°, 모두 양수). 모든 손에서 lost(어느 pad든 끝에서 또는 0.1 s 동안 0.067 N 미만)와 over-pressed는 0이고, 관통 최대 0.96 mm, proximal 접촉 없음. time step 0.25 ms(1 ms에서는 미끄럼 떨림으로 D가 중앙 9% 달라짐). **효과는 작다.** 부드러운 엄지 스프링이 검지 경로의 차이를 흡수하므로, 궤적으로 보면 R − O는 sweep 내내 ±0.05°(중앙값) 안이고 U − O도 최대 +0.56°(μ 1.3), +0.89°(μ 2.0), 끝에서 +0.15°, +0.31°다(O의 끝 회전 중앙 4.1°, 6.6°). 순서는 Δ_contact를 따르지만 E 자체는 작고(1.1–16.3%), 18개 중 5개 손(R 포함)의 E는 잡음 바닥의 75분위보다 작다. 사후 관찰(판정에 쓰지 않음): `d4p5_s0`, `d5p5_s0`가 다이얼·동전과 같이 순서를 거스른다(E 2.2%). 기록은 `docs/experimental/drake_pinch.md`.
- **무작위 시작 (29차).** 미리 정한 균등 분포에서 64개(seed 29, `dial_placements.py --random 64 --seed 29`), 모든 손에 같은 뽑기: q_t U(12, 40°), ρ U(13, 17) mm, x_b U(8, 16), y_off U(6, 14), k_r U(0.005, 0.015). O만으로 정한 제외 뒤 59개(5개는 O의 관통 > 1.5 mm). 결론: 손끝 원판 결과는 손으로 고른 배치에 의존하지 않는다. (함께 돌린 y축 다이얼 28y·29y는 Study 3에서 제외했다; git `5c1a1ff`.)
- **잡음 바닥.** SSoT는 과제별 잡음 바닥을 0.2–0.8%로 쓰는데, 이는 획득·굴리기·두 pad 다이얼(22–24, 26차) 값이다. 손끝 원판은 0.9%(29z 1.24%), 동전 끌기 0.6%, 집기 회전 1.0%, 25차 hydro는 2.4%다.
- **굴리기 (22차).** Gate 0 뒤 O의 명령에서 시작하는 sweep(+8°, +15°, 3 s)을 재생, 회전 감쇠 2.0. 재설계가 10–35%를 얹지만 Δ_contact도 Δ_skel도 순서를 정하지 않는다(`d2p25_maxDs_s0` 94%, 3쌍). 과제의 경계로 보고한다.
- **두 pad 다이얼 (24·26차)이 대체된 이유.** 다이얼이 두 pad에 눌려 grasp에 가깝고, 26차에서 O가 점수를 받은 26개 배치 모두 끝 distal 관통이 2.0–11.0 mm(중앙 6.5 mm), 그중 22개가 3 mm pad보다 깊다. pad compliance 모델의 범위 밖이므로 28z로 대체했다(a2707db). 24차는 회전이 5–10°뿐이어서 26차(30°)로 먼저 대체되었다. 그 전에 "다이얼은 과제가 안 됨"으로 뺀 기록(34d5fce)은 브레이크를 건 구성에만 해당해 24차에서 정정했다. 24·26차 데이터는 Study 3에서 제외했다(git에 남음).

### (e) 명령 프로토콜 (pull-out)

| 프로토콜 | 차 | 엔진 | 손 | ρ(E, Δ_contact), p | ρ(E, Δ_skel) | E 범위, E_R / E_U | 판정 |
|---|---|---|---|---|---|---|---|
| A 재사용 setpoint | 6 | Isaac | 15 | 0.83, 0.0001 | −0.40 | 3–157%; 21 / 124% (7 mm) | 통과 |
| A, 7 mm만 (확장 포함) | 4 | Isaac | 8 | 0.93, 정확 0.0011 (확장 전 6개: 1.00, 0.0014) | −0.69 | 21 / 124% | 통과 |
| B 힘 맞춤(oracle) | 4 | Isaac | 6 | 0.83, 0.029 | — | 2.8–6.8%; 2.8 / 6.8% | 기술 |
| C 재사용 torque | 6 | Isaac | 16 | 0.75, 0.0006 (SSoT Holm 0.00055) | 대칭 지표 0.51 | 3–40% | 통과 |
| C, 7 mm만 | 5 | Isaac | 8 | 0.64, 정확 0.048 | 0.02 | 10–28%; 17 / 28%, Wilcoxon p 0.56 | 겨우 통과 |
| C, 격자, 새 손 | 18 | Isaac | 27 | 0.69, 4×10⁻⁵; partial \| 힘 편차 0.79 [0.54, 0.89] | 0.71 [0.41, 0.87] | 9–53% (중앙 20%) | 통과 |
| D 임피던스 sweep | 7 | Isaac | O, R, U | — | — | R 14–26%, U 44–131% | 기술 |
| E 접촉 기준 닫기 | 9 | Isaac | 18 | 0.44 [−0.09, 0.82], 0.034 | **0.88**; partial \| Δ_contact 0.90 [0.69, 0.97] | 5.6–61% (SSoT 6–61%, 중앙 약 20%); 6/23/22 / 9/20/20% | 형식상 통과, 남은 오차는 Δ_skel |
| 27차 접촉 기준 setpoint (손끝 점) | 27 | Drake | 18 | 0.73, 0.0007 (A와 같은 16개: 0.62, 0.006) | 0.18 (0.24) | 5.3–60%, 중앙 41.7 → 14.0% (A 대비); 13.8 / 25.9% | 통과 (판정 1–3) |

- **A의 경로.** E는 같은 명령에서의 distal 힘 어긋남 |N_d/N_d(O) − 1|을 따른다(7 mm: U 3.28배, R 0.75배). 부호는 distal face가 물체 쪽으로 오는가 물러나는가로 정해진다. Δ_contact ≥ 4.4 mm(Isaac 7 mm)에서는 같은 배치를 쥐지 못한다.
- **B.** 완벽한 힘 되먹임으로 3–7%까지 줄고 대부분 하중 한 step 수준이지만, 남은 작은 어긋남도 Δ_contact 순서다. 3차(O, R, U)에서도 B는 3–27%, A는 R 19–33%, U 99–258%. B의 사전 가설 "네 방향 모두 E_R < E_U"는 −z 동률로 엄밀히는 실패했다.
- **C.** 같은 torque에서 힘은 O의 0.84–1.45배(SSoT; 7 mm 0.89–1.26), 크기는 A의 4–5분의 1. 16개 손에서 ρ(E, C̄_pad) = −0.52의 coverage 교란이 있다. 18차(사전 기록, 독립 표본)에서 Δ_contact와 Δ_skel의 상관은 0.22로 낮고 둘 다 예측한다(서로 고정 0.78, 0.79). 힘 편차는 Δ_skel을 따르고(ρ 0.67) Δ_contact는 따르지 않는다(0.10): 같은 torque가 접촉에서 얼마의 힘이 되는지는 skeleton(lever)이 정하고, Δ_contact는 힘과 다른 기하 경로로 오차를 정한다. 그래서 torque 재사용에서는 skeleton도 지켜야 한다.
- **D (7 mm, K/K₀ 0.1, 0.3, 1, 3, 10).** E_R 25, 14, 20, 24, 26%; E_U 44, 87, 119, 125, 131%. 모든 강성에서 R < U(Wilcoxon p 0.007 ~ 7×10⁻¹⁵). K₀의 D는 A를 재현(힘 1.6% 안). 높은 강성에서 U의 간섭은 접촉 순응이 받아 힘이 약 17.5 N에서 포화한다(O 4.3–4.6 N). 정지 torque 초과 env는 U에서 3K₀ 4%, 10K₀ 11%.
- **E.** 18개 손 모두 쥔다. 힘 비 0.55–0.94. skeleton을 크게 바꾼 설계가 가장 크게 어긋난다(`d4p5_s0`, `d5p5_s0` 61%). torque 재사용(C)에서도 Δ_skel의 partial이 +0.65 [0.10, 0.90].
- **27차.** 두 조건 모두 점수를 받은 16개 손 중 15개에서 오차가 줄었다(중앙 41.7% → 14.0%; 예외 `h3to5_hr` 4.2 → 5.3%, 잡음 안). A에서 쌍이 없던 `d4p5_s0`, `d5p5_s0`도 ρ 20 mm에서 쥔다(55, 60%). 잡음 바닥 중앙 0%, 75분위 3.7%. 사상 잔차와 E의 ρ는 0.57로 Δ_contact보다 약하다. 한 점을 다시 맞추면 그 점의 예하중 차이는 없어지지만 다른 접촉점의 경로 차이는 남는다. 런타임 접촉(E)만 남은 오차를 skeleton 쪽으로 옮긴다.
- **robustness (8차, 조건 A, O·R·U, ρ 15).** 기준 E_R/E_U 26/170%; ke 1e3 24/168, ke 1e4 32/127, impratio 300 28/132, 3000 25/184, substep 4 29/177%. 모두 R < U(Wilcoxon p ≤ 3×10⁻⁴). E_U의 크기(127–184%)는 설정에 달려 있다.
- **전류 제한.** 조건 A의 1,482개 grasp에서 servo torque K₀(q_cmd − q*)/MA는 최대 0.25 N·m, XM335 stall 1.03 N·m의 약 25%이고 초과 0개. 그보다 높은 전류 제한의 위치 제어는 조건 A와 같다.

### (f) 고정축 계열, 후보 불변량, 허용 범위, 매개

**고정축 대조 (14차, Isaac, A, 유효 쌍 54–56).** Δ_skel = 0인데 E는 5, 7, 9 mm에서 95% (IQR 68–118), 179% (137–216), 228% (199–312). distal 힘은 기준의 2.2/3.2/4.2배(깊은 face가 일찍 닿음). 같은 두께의 CFL R은 3/21/41%, U는 52–124%. skeleton을 지켜도 측정은 지켜지지 않는다. 기준 손이 다르므로 쌍 비교는 아니다.

**손과 무관한 격자 (12·13차; 222c111에서 키 수정 후).** 원통 중심 x_c ∈ {21, 24, 27, 30} mm(ρ 15) 또는 {14, 18, 22, 26} mm(ρ 20) × q0 85–115° 7개.

| 표본 | ρ(E, Δ_contact) | ρ(E, Δ_skel) | partial E–Δ_contact \| Δ_skel | 손당 쌍 (중앙) |
|---|---|---|---|---|
| 18개 손 | 0.88 [0.63, 0.99], p < 10⁻⁵ | — | — | — |
| 45개 손 | 0.86 [0.72, 0.94], p < 10⁻⁵ | 0.22 [−0.12, 0.51] | 0.86 | 106 |
| (수정 전, 쓰지 않음) 45개 손 | 0.60 [0.36, 0.76] | 0.46 [0.17, 0.69] | 0.58 | 42 |

수정 전 분석은 env를 (q0, 방향)으로만 구분해 한 q0의 배치 4곳 중 하나만 남겼다(격자의 1/4). 수정 전에 쓴 "격자에서는 Δ_skel도 배치를 잃는 경로로 오차를 키운다"와 80% 부분집합 분석은 폐기했다. 수정 전 파일은 `sim/isaaclab/logs/study3/two_pad/pre_keyfix/`.

**후보 불변량 (11차 이후, `invariants.py`, `invariants.json`).** 조건 A의 ρ와 손 bootstrap 구간:

| 후보 | 마찰 (15) | pad 강성 (14) | 격자 (45) |
|---|---|---|---|
| Δ_contact | 0.83 [0.41, 1.00] | 0.80 [0.33, 0.99] | 0.86 [0.72, 0.94] |
| Δ_skel | −0.40 [−0.78, 0.12] | −0.51 [−0.85, 0.04] | 0.22 [−0.12, 0.51] |
| coverage | −0.26 [−0.76, 0.38] | −0.21 [−0.74, 0.43] | 구간 0 포함 |
| 두께 변화 | +0.20 [−0.42, 0.71] | +0.17 [−0.43, 0.69] | 구간 0 포함 |
| 전달 변화 max \|r_H/r_O − 1\| | −0.05 [−0.57, 0.48] | −0.02 [−0.58, 0.55] | 구간 0 포함 |

- 모든 설정에서 구간이 0을 넘는 후보는 Δ_contact뿐이다. 다섯 단측 p(0.0001, 0.00044, 0.00055, < 10⁻⁵, 0.00034; `dose_A_all`, `comp`, `dose_C_all`, `dose_A-grid_expanded`, `padk_matched`)는 각각 0.05/25보다 작아, 다섯 후보 × 다섯 설정 25개 검정의 Holm 보정을 다른 p와 무관하게 통과한다.
- 직접 비교 ρ(E, Δ_contact) − |ρ(E, X)|의 구간: 45개 격자에서 네 후보 모두 0을 넘는다(Δ_skel [0.35, 0.86], coverage [0.41, 0.84], 두께 [0.46, 0.87], 전달 [0.29, 0.85]). 네 설정의 16개 비교 중 9개에서 넘는다(SSoT). 14–16개 손에서는 Δ_skel과의 구간이 0을 포함한다(sum rule이 둘을 묶음). 둘을 가르는 것은 고정축 계열(Δ_skel = 0)이다.
- 45개 손에서 Δ_contact와 다른 후보의 상관은 |ρ| ≤ 0.40(SSoT).

**Δ_contact의 분해 (e20, 탐색).** distal face 법선 성분과 face 방향(미끄러짐) 성분: ρ(E, 전체/법선/미끄러짐) 마찰 0.83/0.76/0.79, 강성 0.80/0.67/0.86, 격자 0.86/0.88/0.47, 부드러운 pad 0.82/0.67/0.89, torque 0.76/0.82/0.51. 두 성분 모두 예측하지만 어느 쪽이 강한지는 설정마다 바뀌고(0.47–0.89), 전체가 가장 일관된다(0.76–0.86). Δ_contact는 모든 설정에서 1위다.

**사후 재분석 (리뷰 대응, 판정 아님).**
- 부호 있는 face 법선 오프셋(v18): |오프셋|의 ρ 0.76, 0.82, 0.71(마찰, 강성, 격자; 0.71–0.82). 오프셋을 고정한 Δ_contact partial 0.50, 0.21, 0.69; Δ_contact를 고정한 오프셋 0.09, 0.37, 0.06. 오프셋의 부호가 오차의 방향을 정한다.
- 예하중(v19): 격자 45개 손에서 setpoint 힘 편차를 고정한 partial ρ(E, Δ_contact) 0.64 [0.34, 0.86], 힘은 Δ_contact 고정 시 0.25. Δ_contact의 q에 대한 최대값은 grasp 자세의 평균만큼 또는 더 잘 예측한다(SSoT: 0.83–0.86 대 0.68–0.86). 로그 값은 평균 0.82, 0.86, 0.68, 최대 0.83, 0.80, 0.86(마찰, 강성, 격자)으로, 강성에서는 최대값(0.80)이 평균(0.86)보다 낮다.
- pad 압축(v24): 조건 A grasp 4,446개 접촉에서 distal 압축 = 법선력 / 2.5 kN/m, 손별 중앙값 R 0.47, O 0.65, U 2.27 mm(전체 중앙 1.2, 90분위 2.4, 최대 3.6 mm). 허용 오차 크기의 압축이 이미 들어 있는 상태에서 Δ_contact가 예측했다.
- spacer 대조(Δ_contact = 0)는 돌리지 않았다. 이 접촉 모델에서는 오차 0이 구조적으로 정해진다. 하드웨어의 몫이다.

**매개 (`mediation.py`, `mediation.json`; 수치는 SSoT).** A에서 O 대비 distal 힘 편차가 오차를 따르고(ρ 0.94), Δ_contact가 그 힘을 정한다(0.62). 힘을 고정한 Δ_contact partial 0.55 [−0.02, 0.93](마찰, 경계선). C에서는 힘이 O의 0.84–1.45배이고 힘을 고정한 partial 0.80 [0.37, 0.95]: 같은 torque에서 기하 자체가 중요하다(18차가 독립 표본으로 확인).

**허용 범위 (`tolerance.py`, E를 Δ_contact에 단조 증가로 맞춤; 수치는 SSoT).** 재사용 setpoint에서 E ≤ 20%는 Δ_contact 0.35–0.96 mm까지, E ≤ 50%는 1.46–3.89 mm까지(설정에 따라). 같은 torque에서 20%는 2.33 mm까지. CFL의 Study 2 재설계(120°: 0.35, 0.86, 2.32 mm)는 7 mm까지 20% 안이다. 9 mm(2.32 mm)에서는 41%. 두께만큼 부드러운 pad(25차)는 이 범위를 좁힌다.

### (g) interface 연구의 결론 (H4/R4)

| | 손 | 진짜 변화 | 결과 |
|---|---|---|---|
| 예비 (15·17차 run 사후, O 기준 배치) | 14 | −26, −41, −51% (5, 7, 9 mm) | U +21, +48, +25%, 뒤집힘 77–79%. R −26, −55%(뒤집힘 4%), 9 mm −15%(34%). ρ(뒤집힌 비율, Δ_contact) 0.80 [0.34, 0.98], p 0.0004; ρ(상대 오차, Δ_contact) 0.61, p 0.011. face가 물러나는 재설계는 부호는 지키고 손실을 과장(−58 ~ −69%) |
| 확인, 19차 (사전 기록; 격자, 새 7 mm 손, ke 2.5 × 3/7 kN/m) | 27 | −37% (−34 ~ −40%) | (1) ρ(상대 오차, Δ_contact) 0.94 [0.84, 0.97], p < 10⁻⁵: **성립**. (2) ρ(뒤집힌 비율, Δ_contact) −0.04 [−0.50, 0.42], p 0.58: **실패**. 측정된 변화 −4 ~ −75% |

- 사후: 격자 굽힘 범위(85–115°)에서 distal face의 부호 있는 법선 오프셋이 방향을 정한다(측정 − 진짜와 ρ −0.99, 뒤집힌 비율과 −0.88). 물체 쪽으로 나오는 face(minDs 계열, U)는 더 세게 눌러 손실을 줄이거나 뒤집고(최대 40% 배치), 물러나는 face는 과장한다.
- 안전한 주장: 재설계는 자신을 강제한 바로 그 interface의 측정 효과를 Δ_contact를 따르는 만큼 왜곡하고, 방향은 distal face가 움직인 쪽이 정한다. 뒤집힘은 앞으로 나오는 face의 사례이지 Δ_contact의 법칙이 아니다.

## 4. 실험 색인

계획 commit은 계획 제목이 처음 들어간 commit, 결과 commit은 결과가 들어간 commit이다(`git log -S`). 8d71987(2026-11-24) 전의 기록은 `docs/experimental/separability.md`에 있었다(`git show <hash>:docs/experimental/separability.md`). 28차는 `docs/experimental/drake_tasks.md`, 29차는 `docs/experimental/drake_dial_random.md`에서 계획·실행한 뒤 옮겼다. **계획과 결과가 같은 commit인 실험(3B, 4, 9, 10, 12, 13, 14)은 git만으로 사전 기록을 확인할 수 없다.**

| 차 | 목적 | 상태 | 계획 | 결과 |
|---|---|---|---|---|
| 1–2 | 준정적 형상(고정 원기둥), 축에 끼운 원기둥의 마찰 T_slip, O·R·U | 탐색 (3차의 전단계) | 25a096e, 7e5e061 | 같음 |
| 3 | 두 pad 자유 원기둥 pull-out, O·R·U, 조건 A·B; μ 검증, impratio 수정 | 실패 (B의 "네 방향 모두 E_R < E_U", −z 동률); A는 R < U | 6be3d93 (Stage A), d974ef3 (Stage B) | d974ef3 |
| 4 | 7 mm dose-response 8개 손, 조건 A(B 보조); 확장 2개 손 | 통과 (ρ 1.00 → 0.93) | 4b3f362; 확장 1f4b6f6 | 4b3f362; 1f4b6f6 |
| 5 | 조건 C(재사용 torque), 7 mm | 겨우 통과 (p 0.048), R 대 U 차이 없음 | 1f4b6f6 | af54b04 |
| 6 | 세 두께 15–16개 손, A와 C | 통과 | 1f4b6f6; 손 선정 af54b04 | 74d3ed5 |
| 7 | 조건 D 강성 sweep, O·R·U | 통과 (기술 통계) | 74d3ed5 | ae45aa6 |
| 8 | robustness 5설정, 형상 interface 6개 손 | 통과 | 17b55f8 | 47d3572, 02b4fe8 |
| 9 | 조건 E, 접촉 기준 닫기, 18개 손 | 형식상 통과(약함); 남은 오차는 Δ_skel | f777b82 | f777b82 |
| 10 | pad compliance interface, 18개 손 | 통과 (gate 값은 기록 전에 봄) | f777b82 | f777b82 |
| 11 | 다섯 후보 불변량 비교 | 탐색 (사후 분석) | — | e56d406 |
| 12 | 손과 무관한 격자, 18개 손 | 통과; 정정 | 58fcdcd | 58fcdcd; 정정 222c111 |
| 13 | 격자 45개 손, 상관 차이 | 통과 (차이 판정은 정정 후 네 후보 모두) | a4c6a56 | a4c6a56; 정정 222c111 |
| 14 | 고정축 대조 계열 | 통과 | 29c45bd | 29c45bd |
| 15 | 두께와 함께 부드러워지는 pad, 딱딱한 O 기준 | Study 3에서 제외 (기준이 pad 자신의 효과를 담지 않음; 재설계 run은 17차에서 사용) | ed8d53a | 1fd3f9a |
| 16 | Isaac hydroelastic, 같은 pad | 중단 (O 마찰 검증 실패, 재설계 데이터 없음) | a19ffda | 11ffe35; 원인 2ea7160 |
| 17 | 두께와 함께 부드러워지는 pad, 기준 O_h | 통과 | 2d449e9 | dae404c |
| 18 | 조건 C, 격자, 새 27개 손, 힘 편차 고정 | 통과 | c51e92d | 3af6625 |
| 19 | 결론 확인(H4), 격자, 새 27개 손 | (1) 통과, (2) 실패 | 6bc25cf | 6167aa7 |
| 20 | 물체 pose 오차와의 비교 | 취소 (run 전; 계통 오차와 잡음은 비교 불가) | — | 3af6625 |
| 21 | Drake 복제, point와 hydroelastic | 통과 | 2060ab5 | 251c28c (소규모), b3bf4ce |
| 22 | in-hand 굴리기 | 실패 (ρ 0.28) | e8f71dc | 8590c36 |
| 23 | grasp 획득 | 통과 | 6ad895e | d09b5b5 |
| 24 | 두 pad 다이얼 5–10° (34d5fce의 "다이얼 제외"를 정정) | 통과, 대체됨 (26) | 4db9b3c | 032860f |
| 25 | 두께만큼 부드러운 hydro pad, O_h 기준 | 통과 | a280bef | 425b3d4 (소규모), 54faaa9; 잡음 aca7665 |
| 26 | 두 pad 다이얼 30° | 통과, 대체됨 (28z; 관통 2–11 mm) | eb615e4 | 3b38fe7; 대체 a2707db |
| 27 | 접촉 기준 관절 setpoint (손끝 점) | 통과 | d34be24 | 18be958 |
| 28 | 손끝 다이얼 y, 손끝 원판 z | 통과; z가 Study 3 다이얼 과제 | 1b166a5 (fa177e0에서 이동) | 77b0c9d; 채택 a2707db |
| 29 | 28차를 미리 정한 무작위 시작 64개로 확인 | 통과; 규칙이 z를 고름 | 01af200 (O만의 결과 포함) | 90f2556; 이동 5c1a1ff |
| 30 | 손끝 동전 끌기, 짧은 끌기 (experimental) | 판정 2 실패 (ρ 0.36), `docs/experimental/drake_coin.md`에 남김 | 695d360 | 5fcf277 |
| 30b | 손끝 동전 끌기, 긴 끌기 | 통과 | 1d59c5e (O만의 결과 포함) | b73f616 |
| 33 | 엄지–검지 집기 회전 | 통과 (효과 작음) | 3ae7026 (O만의 결과 포함) | fae4d3f |

설계 쪽 탐색 스크립트 e16–e20(`design/scripts/study/exploratory/`)은 실험 번호와 별개다: e16 원기둥 재파지 기하(아래 regrasp 노트), e17 두 pad wrench 예측(3차 Stage A), e18 Δ_contact 목표 재설계(4·6·13차의 손), e19 Study 2 Δ_contact의 시작 오프셋 분리(R2에 반영), e20 face 오프셋·미끄러짐 분해.

## 5. 한계와 범위

- **point contact에만 있는 결과.** 형상, 고정축, 격자, 후보 비교, 조건 B–E, 획득, 원판, 27차는 point contact다. hydroelastic은 Drake pull-out(21, 25차)뿐이다. 두께만큼 부드러운 pad에서는 Δ_contact > 1 mm의 오차가 2–3배 커진다.
- **평면 손가락.** Drake는 손가락 하나, Isaac은 같은 세 손가락이 한 평면 원기둥을 쥔다. MCP는 고정이고 손 전체 효과는 없다.
- **MCP level은 지키지 않는다.** 지키는 것은 손가락 안쪽, proximal phalanx 좌표계의 PIP 열이다. 두꺼운 pad는 proximal 접촉면을 MCP 축에서 멀리 두므로 MCP 열은 모든 재설계에서 두께 변화만큼 바뀐다. 지키려면 spacer(제외) 또는 pad 안의 축(만들 수 없음)이 필요하다. 다심 MCP는 향후 과제다.
- **관통의 교훈.** point contact의 관통 = 힘 / 접촉 강성이 pad 두께를 넘으면 pad 모델의 범위 밖이다. 26차 다이얼은 2.0–11.0 mm로 대체되었고, 28차부터 1.5 mm gate를 미리 둔다. pull-out의 압축은 0.5–2.3 mm(손별 중앙)다.
- **순열 p의 표현.** 18개 손(Drake)의 p는 20,000번 무작위 순열(seed 0)이지 정확 p가 아니다. Isaac의 9개 이상 손은 100,000번(seed 0)이다. 정확 p는 Isaac 8개 이하(4차 8개 손, 형상 6개 손), Drake 9개 이하뿐이다. 로그의 15–27차 계획 일부가 "정확 단측 순열 p"라고 썼지만 실제로는 무작위 순열이다. 29차 계획부터 "순열 p"로 부른다.
- **사후 수정은 탐색으로 보고한다.** impratio 1000(3차), 손 재선정과 확장 2개 손(4차), 대칭 오차 지표(4차 확장), 격자 분석 키(12·13차), 동률 규칙(6차 손 선정, 사전에 없음), 10차 gate 값(기록 전에 봄), 21차 solver 재시도 규칙(O·R·U 소규모 샘플 뒤).
- **예측된 결과가 아닌 것.** 9차에서 접촉 기준 닫기의 남은 오차가 Δ_skel을 따르는 원인(같은 5°가 다른 lever로 힘이 됨)과 4·5차에서 큰 Δ_contact 손이 못 쥐는 원인(pad 모서리 접촉)은 확인하지 않았다.
- 하드웨어 Stage 2(SSoT §8.4)는 아직 하지 않았다. 계획한 H_R, H_M(`d2p25_minDs_s0`), H_U는 조건 A 마찰에서 E 21, 72, 124%(Δ_contact 0.86, 2.33, 3.91 mm; Δ_skel 7.67, 4.62, 3.38 mm; coverage 0.70–0.73, 전달 변화 0.20–0.23)이고, R < M < U는 48개 배치·방향 블록 중 43개에서 성립한다(SSoT; 전달 변화와 블록 수는 로그에 없음).

## 6. 보관한 문서

- **3A/3B RL pilot (`study3_pilot.md`, 2026-11-21).** pivot8 시제품 손(crossed four-bar 셋)에서 flat 대 2 mm crowned pad로 cube 수직축 재배향 PPO를 형상마다 seed 5개 학습했다. flat 정책은 crowned 손에서 첫 goal 성공을 잃었고(+0.18 ± 0.09, 부호 검정 p 0.0625, paired t p 0.010; overshoot 가설은 기각, 실제로는 undershoot), 형상이 학습 결과를 바꾼다는 것은 지지되지 않았다. 3B(MuJoCo-Warp → VBD)는 VBD가 정책을 안정하게 평가하지 못해 해석하지 않았다. SSoT상 증거가 아니다. 계획은 `docs/experimental/study3_interface_platform.md`, 재현 스크립트는 `sim/isaaclab/scripts/study3/run_pilot.sh`.
- **regrasp 기하 노트 (`regrasp_geometry.md`, 2026-11-21, e16).** 학습 없이, O에 접하는 원기둥에 같은 굽힘각을 R과 U에 지령할 때 distal face의 법선 어긋남을 계산했다. q_req 120°, 3 → 7 mm에서 중앙값 R 0.12 mm 대 U 2.01 mm(최대 0.33 대 3.10 mm)로 굽힘 범위 전체에서 약 10배 차이가 나고, 전달비와 MA는 두 손이 비슷했다. servo 각 지령은 두 손 모두 약 21° 어긋나므로 굽힘각 지령이 필수다. 이 계산으로 Study 3의 조건을 120°, 3 → 7 mm로 정했다. rigid pad에서는 1 mm 어긋남도 서보를 포화시키고, compliant pad(1–2 N/mm)에서 R과 U가 갈린다는 추정도 담았다.
- 두 문서와 `experiments.md` 전문은 git에 있다: `git show 5c1a1ff:report/study3/experiments.md`, `git show 5c1a1ff:report/study3/study3_pilot.md`, `git show 5c1a1ff:report/study3/regrasp_geometry.md`.

## 7. 데이터와 스크립트

- **요약 데이터 `report/study3/data/`.** Isaac: `isaac_dose_A_all.json`(마찰, 6차), `isaac_comp.json`(강성, 10차), `isaac_dose_A-grid_expanded.json`(격자 45, 13차), `isaac_conclusion_study3.json`(H4 예비). Drake: `drake_e21_point.json`, `drake_e21_point_pairs_RU.json`, `drake_e21_hydro.json`, `drake_e22_point.json`(굴리기), `drake_e23_point.json`(획득), `drake_e25_hydro_own.json`, `drake_e27_point.json`(사상 전체 `mapping_all`), `drake_e28z_point.json`, `drake_e29z_point.json`, `drake_e30b_coin.json`(동전 끌기), `drake_e33_pinch.json`(집기 회전, 궤적 편차 `deviation`). 2026-11-24에 제외한 것(git에 남음): 3A/3B pilot 표 `tables/`, 24·26차 두 pad 다이얼, 27차 pad 가운데 변형, 28·29차 y축 다이얼, 22차 사후 분석, 옛 렌더용 상태 기록.
- **raw 표 `report/study3/data/raw/`** (`report/make_raw_tables.py`, `metadata.json`에 열·사건·오차 정의, git commit 90f2556). run 단위: `runs_isaac_pullout.csv`(25,624), `runs_drake_pullout.csv`(20,592), `runs_drake_acquisition.csv`(2,508), `runs_drake_rolling.csv`(2,508), `runs_drake_tipdial.csv`(2,888, 28z·29z), `runs_drake_coin.csv`(2,432, 30b), `runs_drake_pinch.csv`(2,432, 33). 손 단위 E: `errors_by_hand.csv`(230 행).
- **렌더 `report/study3/data/renders/`.** `render_hold{0,1,2}.png`(power grasp pull-out, 손바닥 위 isometric), `render_tipdial{0,1,2}.png`(손끝 원판), `render_coin{0,1,2}.png`(동전 끌기), `render_pinch{0,1,2}.png`(집기 회전). 그림 `report/study3/figures/`: `power_grasp_hold_iso.png`, `fingertip_disk.png`, `fingertip_coin.png`, `fingertip_pinch.png`.
- **Drake `sim/drake/`.** `two_pad_drake.py`(모든 과제, `--task pullin|roll|dial|tipdial`), `analyze.py`(pull-out, `spearman_perm`), `analyze_roll.py`(굴리기, 다이얼, 손끝 다이얼, `--random`), `analyze_e25.py`, `dial_placements.py`, `coin_drag.py`(동전), `pinch_rotate.py`(집기 회전), `friction_probe.py`, `load_hand.py`, `run_e21.sh`, `run_roll.sh`, `run_pullin.sh`, `run_dial.sh`, `run_dial30.sh`, `run_e25.sh`, `run_e27.sh`, `run_e28.sh`, `run_e29.sh`, 렌더 `render_*.py`. 로그 `sim/drake/logs/`.
- **Isaac Lab `sim/isaaclab/scripts/study3_measurement/`.** 실행 `two_pad_wrench.py`, `fixed_axis_hands.py`, `run_padk.sh`, `run_padk_ref.sh`, `run_hydro.sh`, `run_torque_grid.sh`, `run_conclusion_grid.sh`; 분석 `two_pad_analysis.py`, `dose_analysis.py`, `impedance_analysis.py`, `robust_analysis.py`, `invariants.py`, `fa_analysis.py`, `padk_analysis.py`, `padk_matched.py`, `torque_grid_analysis.py`, `conclusion_analysis.py`, `face_offset_analysis.py`, `mediation.py`, `tolerance.py`; 1차 `separability_quasistatic.py`, `separability_analysis.py`. 로그 `sim/isaaclab/logs/study3/two_pad/`. hydro 마찰 진단은 `sim/isaaclab/scripts/experimental/hydro_friction_probe.py`.
- **손.** `design/scripts/study/study3_trajectory/make_hands.py`(O, R, U), e18 `design/scripts/study/exploratory/e18_rct_dose.py`, 배치 `design/output/sim/study3/{two_pad_grasps,grid_grasps,fa_grasps}.json`.
