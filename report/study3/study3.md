# Study 3: 손가락 형태가 바뀔 때 interface 효과 측정은 무엇을 지켜야 하는가 (시뮬레이션)

이 파일 하나가 Study 3의 압축 보고다(2026-11-24에 `experiments.md`, `regrasp_geometry.md`, `study3_pilot.md`를 합침; 2026-11-26에 과제를 point·hydro 둘 다에서 성립하는 다섯 개로 줄이고 분석 → 논문 결과 순으로 다시 짬). 주장과 범위의 SSoT는 `design/SSoT/research-objective.md` §6(H3/R3, H4/R4)과 §8, 정의는 `problem_formulation.md` A.6.4–A.6.5다. 숫자가 이 파일과 SSoT에서 다르면 SSoT가 맞다. Study 3에서 뺀 과제와 시도(동전 30b, 33/33b, 36, 39–42, 긴 손가락, RL 전이)는 `report/experimental/study3_excluded.md`와 `findings.md`에 있다.

## 1. 질문과 주장

**질문 (SSoT §8.1).** 기준 손 H₀(= O)를 더 두꺼운 interface에 맞춰 다시 설계하면, 측정한 interface 효과 Δ(H) = M(H, c₁) − M(H, c₀)에 재설계가 오차를 얹는다. E(H) = |Δ(H) − Δ(H₀)| / |Δ(H₀)|. 재설계가 무엇을 지켜야 E가 작은가: 접촉면 운동(Δ_contact), skeleton 운동(Δ_skel), 다른 후보?

**H3 / R3.** 서보 손이 쓰는 임피던스 setpoint(한 굽힘 명령, 같은 강성; 조건 A)를 재사용하면 E는 Δ_contact를 따른다. 반증: Δ_contact가 예측하지 못하거나, 다른 후보(Δ_skel, coverage, pad 두께, 전달)가 모든 설정에서 예측한다(손 bootstrap 95% 구간이 모두 0을 제외). 접촉을 감지해 닫으면(조건 E) 남는 오차는 Δ_skel을 따르고, torque를 재사용하면 skeleton도 지켜야 한다.

**H4 / R4.** O의 딱딱한 3 mm pad를 두께와 함께 부드러워지는 pad로 바꾸는 interface 연구에서, 재설계 손이 잰 pull-out 하중 변화는 Δ_contact가 클수록 진짜 변화에서 멀어진다. 확인 실험(19차)에서 (1) 상대 오차는 성립, (2) 결론이 뒤집힌 비율은 실패.

## 2. 무엇을 어떻게 쟀나

**손.** 모두 CFL, q_req 120°. O = 3 mm 설계. 재설계는 각 두께의 R(Study 2의 최대 coverage 재설계, Δ_contact 최소), U(Study 1의 첫 독립 설계, coverage만), 그리고 설계 지표만 보고 고른 e18 재설계(Δ_contact 목표마다 coverage 최대, Δ_skel 최소/최대)다. 이 18개가 주 집합이다. `d1p5_maxDs_s0`, `d3_maxDs_s0`의 장착 깊이는 proximal/distal 4.6/3.0, 5.9/3.0 mm다(spacer 아님).

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

(mm. dN = 목표 Δ_contact, p = 소수점, h3toN = 3 → N mm, maxDs/minDs = 그 목표에서 skeleton 변화 최대/최소, _s0/_s1 = seed.) 격자 손 27개(13·18·19차; e18 seed 1, 7 mm, Δ_contact 목표 9개 × skeleton 조건 3개)를 더하면 45개다. 고정축 대조 계열(14차)은 O의 joint gap 중앙에 PIP 하나, pad 3/5/7/9 mm, Δ_skel = 0, Δ_contact = √3·Δh다. naive swap(O에 두꺼운 pad)은 47°에서 충돌해 측정하지 않았다.

**두 경로 변화.** Δ_contact = proximal face 좌표계(MCP 축이 face에 내린 발이 원점)에서 본 distal face 기준점 c(q)의 변화 max_q ‖c_H(q) − c_O(q)‖ (0–q_req). Δ_skel = 손끝 경로(링크선)의 같은 변화. 둘은 contact Jacobian J(q) = [c′(q); 1]의 같은 pole을 face와 링크선에서 본 것이다. 회전 성분이 모든 설계에서 1이라 관절 명령으로 Δ_contact를 되돌릴 수 없다. 로그의 RCT, D_c, D_s는 contact-surface motion, Δ_contact, Δ_skel이다.

**측정의 공통 틀.** 모든 과제에서 손마다 같은 배치를 pad 마찰 μ 1.3과 2.0으로 한 번씩 돌린다(형상·강성 interface는 그 쌍). D(H) = 지표(2.0) − 지표(1.3), E(H) = O와 H가 모두 유효한 쌍들의 |D(H) − D(O)| / |D(O)| 중앙값. O의 효과가 작은 쌍(과제별 |D(O)| 하한)과 O가 실패한 쌍은 O만 보고 뺀다. 잡음 바닥 = O를 명령 +0.05°로 다시 돌린 run의 E. 통계는 손 단위 Spearman ρ와 단측 순열 p(Isaac 100,000회, Drake 20,000회, seed 0; 손 ≤ 8–9개만 정확 p), 손 bootstrap 95% 구간, R 대 U는 배치 쌍의 단측 Wilcoxon. 판정 1 = E_R < E_U, 판정 2 = ρ(E, Δ_contact) > 0, p < 0.05.

**Isaac Lab pull-out (주 실험, Isaac Lab 3, Newton + MuJoCo-Warp, point).** 같은 세 손가락이 원기둥(ρ 15, 20 mm, 평면 3자유도, 중력 없음)을 O의 두 pad가 q0에서 닿는 자리(각 손의 proximal face 좌표)에서 쥔다. PIP를 q0 + 5°로 닫고 → 놓고 안정화 → Gate 0(두 pad 0.2 N 이상, 물체 정지, 법선 각) → 네 방향 외력을 2 N/s로 80 N까지. 한계 하중 F_limit = 첫 slip(10 mm/s), contact loss(0.2 N), collapse(5 mm). μ 1.3 → 2.0(O에서 검증: 1.30/2.00), impratio 1000, ke 2.5 kN/m. 외력 한 step 0.067 N = Δ(O)의 2.5–5.6%.

**Drake 복제 (21차부터, `sim/drake/two_pad_drake.py`, Drake 1.58 SAP).** 평면 손가락 하나, MCP 0° 고정, four-bar는 거리 제약. 서보 관절 implicit PD, 이득 K_Q/(dθ/dq)², 조건 A. 하중·질량·임계값은 Isaac의 1/3, F_limit은 3배로 보고. 1 ms, solver 실패 배치는 0.5 ms로 한 번 재시도.

**Hydroelastic (21차 규칙, 사전 기록 `docs/experimental/drake_hydro.md`, 0535fdb).** 모든 손에 O의 pad 기초 강성(modulus 0.1 MPa / 3 mm, 21차에서 O만으로 정함), 물체·탁자는 rigid hydro, 1 ms. 무작위 과제(29z, 35)는 뽑기 앞 32개를 쓰고 point도 같은 32개로 다시 분석해 짝짓는다. point의 관통 gate(힘 / 2.5 N/mm > 1.5 mm)는 hydro에서 3.75 N pad-force gate가 된다. O의 제외는 O만의 hydro run으로 다시 정한다(8063f49). 21차 hydro는 지금 코드로 비트 단위 재현된다.

**다섯 과제 (Drake, 열린 루프, O의 명령 재사용, 손 18개).**

| 과제 | 차 | 무엇을 하나 | 지표, |D(O)| 하한 | 배치 |
|---|---|---|---|---|
| 감싸 쥐고 당겨 빼기 (power-grasp pull-out) | 21 | 위의 pull-out을 손가락 하나로 | F_limit, — | ρ 15/20 mm × q0, 네 방향 |
| grasp 획득 | 23 | q = 0에서 시작, 원기둥은 O의 접촉 위치보다 손끝 쪽 a = 3, 6 mm, 하중 face 쪽 1.5 N + 손끝 쪽 1.5 N(세 손가락 기준). 2 s에 O의 명령까지 닫고 1 s 유지, 회전 감쇠 2.0 N·m·s/rad | MCP 쪽으로 끌려온 거리, 0.3 mm. O가 끝에 쥐어야 하고, H가 놓치면 lost로 따로 셈(점수 없음) | ρ 15/20 × a 3/6 × q0 |
| 고리로 걸어 당기기 (hook) | 38 | 굽힌 손가락이 서랍 손잡이 막대(ρ 5–8 mm)를 proximal·distal pad 사이에 걸고 q0 + 10°로 닫은 뒤 놓고(Gate 0), 하중이 막대를 갈고리 밖(−z)과 대각(+x−z)으로 당김 | F_limit, 0.2 N; 관통 1.5 mm gate | ρ 5–8 × q0 80–115°, 54쌍 |
| 손끝 원판 | 28z/29z | 반지름 15 mm 원판, 축은 proximal face에 수직. distal pad만 윗면을 누르며 MCP 쪽으로 끌어 비틀림 스프링을 거슬러 돌림(proximal pad는 닿지 않음). 명령 0 → q_t(1 s) → q_t + 40°(3 s) → 유지 1 s | 회전각, 0.3°; 관통 1.5 mm gate | 28z: 12개 대표 구성; 29z: 미리 정한 분포의 무작위 64개(seed 29) |
| 멈춤쇠에 기대 상자 세우기 (box pivot) | 35 | distal pad가 블록 윗면을 손바닥 쪽으로 끌고, 탁자 위 3 mm 멈춤쇠가 블록 아래 모서리를 막아 블록이 기욺. 비틀림 복원 스프링이 기울기를 거스름. 비파지, 외부 접촉 | 끝의 기운 각; lost(넘어짐, 멈춤쇠 이탈, pad 놓침) | 무작위 64개(seed 35) |

**명령 프로토콜.** A: O의 굽힘 setpoint q0 + 5°, 같은 굽힘 강성 K₀ 5 N·m/rad(주 조건). B: O의 distal 접촉력(oracle). C: O의 굽힘 torque. D: q_ref = q*_O, τ_ff = τ_O, K_q/K₀ 0.1–10. E: 접촉 기준 닫기(0.05 N에서 q_touch, q_touch + 5°). 27차: 기구학으로 다시 정한 setpoint(손끝 점이 O의 것에 가장 가까운 굽힘, 런타임 접촉 없음).

## 3. 결과

ρ는 점수를 받은 재설계 손 전체, p는 단측 순열 p, [ ]는 손 bootstrap 95% 구간. E_R / E_U는 7 mm R과 U(따로 적으면 5, 7, 9 mm).

### 3.1 interface 속성과 후보 (Isaac pull-out, point, 조건 A)

| 설정 | 차 | 손 | ρ(E, Δ_contact), p | ρ(E, Δ_skel) | E_R / E_U (5, 7, 9 mm) |
|---|---|---|---|---|---|
| 마찰 1.3 → 2.0 (`dose_A_all`) | 6 | 15 | 0.83 [0.41, 1.00], 0.0001; partial \| Δ_skel 0.88 | −0.40 [−0.78, 0.12] | 3 / 52, 21 / 124, 41 / 106% |
| pad 강성 2.5e3 → 1e3 N/m (`comp`) | 10 | 14 | 0.80 [0.33, 0.99], 0.0004; partial 0.91 | −0.51 [−0.85, 0.04] | 3 / 75, 25 / 173, 61 / 150% |
| 손과 무관한 격자 (`dose_A-grid_expanded`) | 13 | 45 | 0.86 [0.72, 0.94], < 10⁻⁵; partial 0.86 | 0.22 [−0.12, 0.51] | — |
| 두께만큼 부드러운 pad(ke 2.5 × 3/h kN/m), 기준 O_h (`padk_matched`) | 17 | 14 | 0.81 [0.33, 0.99], 0.0003 | −0.46 [−0.82, 0.08] | 5 / 55, 14 / 129, 50 / 124% (Wilcoxon p ≤ 1.1e-7) |
| 형상 flat → 2 mm crowned, 형상마다 O의 접촉으로 배치 (Drake point, μ 1.3) | 34 | 16 | 0.69, 0.0027 | −0.22 | 27.0 / 50.0, 54.5 / 122.7, 56.0 / 100.0% |

- **후보 비교 (`invariants.json`).** 마찰·강성·격자 모두에서 구간이 0을 넘는 후보는 Δ_contact뿐이다(coverage −0.26/−0.21, 두께 +0.20/+0.17, 전달 −0.05/−0.02, 모두 구간이 0 포함). 다섯 p(0.0001, 0.00044, 0.00055, < 10⁻⁵, 0.00034)는 각각 0.05/25 아래라 25개 검정의 Holm 보정을 통과한다. 45개 격자에서는 ρ(E, Δ_contact) − |ρ(E, X)|의 구간이 네 후보 모두 0을 넘는다. 14–16개 손에서는 Δ_skel과의 차이 구간이 0을 포함한다(sum rule이 둘을 묶음). 둘을 가르는 것은 고정축 계열이다.
- **고정축 대조 (14차).** Δ_skel = 0인데 E는 5, 7, 9 mm에서 95, 179, 228%(distal 힘 기준의 2.2–4.2배). 같은 두께의 R 3/21/41%, U 52–124%. skeleton을 지켜도 측정은 지켜지지 않는다.
- **형상 (34차).** 8차(Isaac, flat의 배치 그대로)는 crowned에서 Gate 0이 크게 줄고 6개 손에서 Δ_skel이 Δ_contact와 반대로 움직여 대체했다. 34차는 형상마다 원기둥을 O의 pad가 닿는 곳에 둔다(crowned 중심은 0.7–1.7 mm 링크 쪽). O는 59쌍, D(O)는 부호가 바뀐다(−11.6 ~ +33.4 N). E 자체가 크다(27–123%). hydro에서는 O가 2 배치만 남아 분석할 수 없다(drake_hydro.md).
- **사후 재분석 (판정 아님).** 부호 있는 face 법선 오프셋을 고정해도 Δ_contact partial 0.50/0.21/0.69(마찰/강성/격자), 오프셋의 부호가 오차의 방향을 정한다. 격자에서 setpoint 힘 편차를 고정한 partial 0.64 [0.34, 0.86]. Δ_contact를 법선·미끄러짐 성분으로 나누면 어느 쪽이 강한지 설정마다 바뀌고(0.47–0.89) 전체가 가장 일관된다(0.76–0.86). grasp의 distal 압축은 R 0.47, O 0.65, U 2.27 mm(손별 중앙)이고 그 상태에서 Δ_contact가 예측한다. servo torque 최대 0.25 N·m(XM335 stall의 약 25%). 못 쥐는 손을 가장 큰 오차로 넣으면 ρ 0.81(마찰), 0.67(강성).
- **허용 범위 (`tolerance.py`).** 재사용 setpoint에서 E ≤ 20%는 Δ_contact 0.35–0.96 mm까지, ≤ 50%는 1.46–3.89 mm까지. CFL 재설계(0.35, 0.86, 2.32 mm)는 7 mm까지 20% 안, 9 mm에서 41%.

### 3.2 다섯 과제, point와 hydro (Drake, 조건 A, 마찰 1.3 → 2.0)

| 과제 | 차 | point: 손, ρ(E, Δ_contact), p | ρ(E, Δ_skel) | E_R / E_U | 잡음 | hydro: 손, ρ, p | ρ(E, Δ_skel) | E_R / E_U | 잡음 |
|---|---|---|---|---|---|---|---|---|---|
| power-grasp pull-out | 21 | 16, 0.67, 0.0027 | −0.37 | 19.4 / 71.7% | — | 16, 0.86, < 10⁻⁴ | 0.09 | 16.7 / 253.8% | — |
| grasp 획득 | 23 | 18, 0.87, < 10⁻⁴ | 0.11 | 21.3 / 74.1% | 0.24% | 18, 0.85, < 10⁻⁴ | −0.03 | 15.8 / 65.9% | 5.8% |
| hook | 38 | 14, 0.65, 0.0070 | −0.56 | 28.6 / 150.0% | 0.00% (75분위 9.1%) | 11, 0.55, 0.043 | −0.46 | 42.5% / — (U 쌍 0) | 5.5% |
| 손끝 원판, 앞 32개 | 29z | 18, 0.69, 0.0008 | 0.11 | 17.0 / 47.9% | 0.95% | 18, 0.44, 0.037 | −0.26 | 4.1 / 12.4% | 0.16% |
| 상자 세우기, 앞 32개 | 35 | 18, 0.44, 0.037 | −0.13 | 5.2 / 20.3% | 0.49% | 18, 0.42, 0.043 | −0.19 | 4.8 / 18.2% | 0.06% |

- **전체 뽑기 point (사전 판정 값).** 29z 64개(O 유지 59): ρ 0.68, p 0.0009, Δ_skel −0.00, R 19.3 / U 54.7%, 잡음 1.24%. 대표 구성 28z(12개): 0.78, p 0.0001, R 17.0 / U 46.8%, 잡음 0.9%. 35 64개(O 유지 63): ρ 0.41, p 0.04995(기준 바로 아래), Δ_skel −0.21, R 5.4 / U 22.2%, 잡음 0.52%. 위 표의 point는 hydro와 같은 앞 32개다.
- **판정.** 다섯 과제 모두 point에서 판정 1·2가 성립한다. hydro에서는 21, 23, 29z, 35가 판정 1·2 모두 성립하고(35는 경계), 38은 판정 2만 성립한다(U가 쌍 0이라 판정 1을 확인할 수 없음). **E_R < E_U가 뒤집힌 과제는 없다.**
- **38 hydro의 gate (§4.2).** 사전 기록의 18개 손 민감도 변형(실패를 가장 큰 오차로)은 ρ 0.76, p 0.0002(point 같은 변형 0.65, p 0.0022). 사후(판정 아님) hydro 등가 gate에서는 ρ 0.69, p 0.0058(13개 손), U 281.2%(4/54쌍), 18개 손 민감도 0.80.
- **과제별 세부.**
  - 21: O Gate 0 28/28/28/32 of 64–68(Isaac 28/28/28/29). solver 5,016 배치 중 117개(2.3%) 최종 실패. 손별 E 순위는 point와 hydro가 약하게만 일치한다(ρ 0.39, p 0.069): 순서는 Δ_contact를 따르지만 크기는 접촉 모델에 달려 있다. Drake 마찰 검증 point 1.33/2.06, hydro 1.45/2.28.
  - 23: E 3.2%(`h3to5_hr`)–101.6%(`d5p5_s0`). 끝에 못 쥔 쌍은 Δ_contact가 큰 손에 몰린다(`d4p5_s0` 15/25, `d5p5_s0` 16/25). hydro에서 O 유지 쌍은 13개이고 잡음 바닥이 point의 20배라 해상도가 낮다.
  - 38: 갈고리를 연 q0 < 85°와 +x(손가락 방향, 잡음 55%)는 O만 보고 뺐다. `d2p25_maxDs_s0`, `d4p5_s0`, `d5p5_s0`, `h3to5_d3_s0`는 point에서 Gate 0을 하나도 못 지나 점수가 없다. 5 mm 8.4 / 71.4%, 9 mm 33.3 / 100%. held load는 U가 O보다 3–4 N 더 버티고(face가 막대 쪽으로 나와 더 세게 누름) R은 0.3–0.5 N 덜 버틴다.
  - 29z: 프로토콜을 손으로 고른 배치에 의존하지 않게 하려고 미리 정한 분포(q_t U(12, 40°), ρ U(13, 17) mm, x_b U(8, 16), y_off U(6, 14), k_r U(0.005, 0.015))에서 뽑았다. 손별 최대 관통 0.78–1.47 mm, proximal 접촉 없음. hydro에서 모든 손의 E가 point의 약 1/4이다.
  - 35: lost 0, proximal과 pad–탁자 접촉 0, time step 0.25 ms(hydro 1 ms; 시험에서 같음). O의 D 중앙 6.3°. 끝 기운 각 편차는 μ 2.0에서 R −0.4°, U −1.3°.

### 3.3 접촉 모델과 부드러운 pad (pull-out, 마찰)

| 설정 | 차 | 손 | ρ(E, Δ_contact), p | ρ(E, Δ_skel) | E_R / E_U (5, 7, 9 mm) |
|---|---|---|---|---|---|
| Isaac point | 6 | 15 | 0.83, 0.0001 | −0.40 | 3 / 52, 21 / 124, 41 / 106% |
| Drake point | 21 | 16 | 0.67, 0.0027 | −0.37 | 19.4 / 71.7% (7 mm) |
| Drake hydro, 모든 손에 O의 pad | 21 | 16 | 0.86, < 10⁻⁴ | 0.09 | 16.7 / 253.8% (7 mm) |
| Drake hydro, pad가 자기 두께만큼 부드러움(0.1 MPa / h), 기준 O_h | 25 | 15 | 0.78, 0.0003 (못 쥐는 3개 포함 0.87) | −0.10 | 5.9 / 109.4, 16.3 / 347.2, 104.2 / 370.4% |
| Isaac point, ke ∝ 1/h, 기준 O_h | 17 | 14 | 0.81, 0.0003 | −0.46 | 4.7 / 54.9, 13.6 / 128.6, 50.0 / 124.0% |

- Isaac Lab의 hydro 경로는 쓸 수 없었다(같은 설정 두 run의 E 30%; 16차는 O 마찰 검증 실패로 중단). 그래서 Drake를 만들었다.
- 25차: Δ_contact 1 mm 이하에서는 point와 같고(R 6, 16%), 그 위에서는 2–3배 크다(R₉ 104%). 순서는 같고 허용 범위가 좁아진다. 잡음 2.4%.
- interface 연구는 같은 interface를 단 기준(O_h)과 비교해야 차이를 interface에 돌릴 수 있다. 딱딱한 O 기준의 15차 판정은 그래서 제외했다.

### 3.4 명령 프로토콜 (pull-out, 마찰)

| 프로토콜 | 차 | 엔진 | 손 | ρ(E, Δ_contact), p | ρ(E, Δ_skel) | E |
|---|---|---|---|---|---|---|
| A 재사용 setpoint | 6 | Isaac | 15 | 0.83, 0.0001 | −0.40 | 3–157%; R 21 / U 124% |
| B 힘 맞춤(oracle) | 4 | Isaac | 6 | 0.83, 0.029 | — | 2.8–6.8% |
| C 재사용 torque | 6 | Isaac | 16 | 0.75, 0.0006 | 대칭 지표 0.51 | 3–40% |
| C, 격자, 새 손 | 18 | Isaac | 27 | 0.69, 4×10⁻⁵; partial \| 힘 편차 0.79 | 0.71 [0.41, 0.87] | 9–53% (중앙 20%) |
| D 강성 sweep (7 mm, K/K₀ 0.1–10) | 7 | Isaac | O, R, U | — | — | R 14–26%, U 44–131% |
| E 접촉 기준 닫기 | 9 | Isaac | 18 | 0.44, 0.034 | **0.88** | 6–61% |
| 27차 접촉 기준 setpoint | 27 | Drake | 18 | 0.73, 0.0007 | 0.18 | 중앙 41.7 → 14.0%; R 13.8 / U 25.9% |

- A에서 E는 같은 명령에서의 distal 힘 어긋남 |N_d/N_d(O) − 1|을 따른다(7 mm: U 3.28배, R 0.75배). C에서는 힘이 Δ_skel을 따르고(0.67) Δ_contact는 따르지 않아(0.10) skeleton도 지켜야 한다. E에서는 skeleton을 크게 바꾼 설계가 가장 어긋난다(`d4p5_s0`, `d5p5_s0` 61%). 27차는 16개 중 15개 손에서 오차가 줄지만 남은 오차가 여전히 Δ_contact를 따른다: 한 점을 다시 맞춰도 다른 접촉점의 경로 차이는 남는다.
- robustness (8차): ke 1e3/1e4, impratio 300/3000, substep 4에서 모두 R < U(Wilcoxon p ≤ 3×10⁻⁴), E_U의 크기(127–184%)는 설정에 달려 있다.

### 3.5 interface 연구의 결론 (H4/R4)

| | 손 | 진짜 변화 | 결과 |
|---|---|---|---|
| 예비 (15·17차, O 기준 배치) | 14 | −26, −41, −51% (5, 7, 9 mm) | U +21, +48, +25%(뒤집힘 77–79%), R −26, −55, −15%. ρ(뒤집힌 비율, Δ_contact) 0.80, p 0.0004 |
| 확인, 19차 (사전 기록; 격자, 새 7 mm 손 27개, ke 2.5 × 3/7 kN/m) | 27 | −37% | (1) ρ(상대 오차, Δ_contact) 0.94 [0.84, 0.97], p < 10⁻⁵: 성립. (2) ρ(뒤집힌 비율, Δ_contact) −0.04: 실패 |

distal face의 부호 있는 법선 오프셋이 방향을 정한다(측정 − 진짜와 ρ −0.99). 물체 쪽으로 나오는 face(minDs 계열, U)는 손실을 줄이거나 뒤집고, 물러나는 face는 과장한다. 안전한 주장: 재설계는 interface의 측정 효과를 Δ_contact만큼 왜곡하고 방향은 face가 움직인 쪽이 정한다.

### 3.6 물체를 어디에 두었나, 그리고 proximal 면의 자리 (R3.1, 2026-11-27)

**SSoT:** research-objective R3.1. 사전 등록과 원자료는 `docs/experimental/pr_mr.md`에 있다. Drake, point contact, 조건 A.

**측정이 놓친 것.**
- 다섯 과제는 물체를 각 손의 proximal 면 좌표에 둔다.
- 그래서 재설계가 proximal 면을 손바닥에 대해 옮긴 양 Δd_p는 설정 때문에 보이지 않는다.

**손바닥 기준으로 다시 잰 결과.** 같은 물체를 O의 MCP 좌표(F₁)에 두고 다시 쟀다.

| 과제 (E) | 기준 | Study 2 재설계 | coverage만 | MO-Pad | 골격 끝 (Palm-Tip) |
|---|---|---|---|---|---|
| dial y | 면 | 9.3% | 26.5% | 8.0% | 13.9% |
| dial y | F₁ | 23.4% | 7.7% | 9.2% | 10.1% |
| pivot | 면 | 5.4% | 22.2% | 7.4% | 15.3% |
| pivot | F₁ | 11.7% | 7.8% | **3.0%** | 12.1% |
| pull-out (point) | 면 | 19.4% | 71.7% | **8.5%** | 90.4% |
| pull-out (hydro) | 면 | 16.7% | 254% | **12.8%** | |

- **F₁ 기준에서는 E가 Δ_MCP를 따른다.** 31개 손에서 dial y ρ 0.83, pivot 0.73이다.
- Δ_MCP와 Δ_skel을 떼어 놓은 손 세 개(e21)를 넣으면 pivot은 Δ_MCP만 따른다(partial 0.68, Δ_skel −0.05). 손끝 가장자리 접촉인 dial y는 둘 다 따른다.
- **MO-Pad는 두 기준 모두에서 오차가 9.2% 이하다.** pull-out에서는 Study 2 재설계보다 작다(9 mm에서도 8.0 대 14.8%).
- **골격 끝을 지킨 재설계는 pad 과제에서 무너진다.** 같은 탐색에서 Δ_skel 0.60 mm로 뽑은 손이고, pull-out 90.4%, F₁ pivot 12.1%다. 같은 CFL 안에서 "골격을 지키면 된다"의 반례다.
- 손끝 끝 접촉(RL relay dial, Study 4 참조)은 반대로 골격 끝을 지킨 손이 가장 잘 넘겨받는다. 범위 밖이다.

**아직 하지 않은 것.**
- 나머지 과제를 MO-Pad로 돌리지 않았다: acquisition(23), hook(38), 손끝 disk(28z/29z), Isaac friction/stiffness, R4 확인 실험.
- MO-Pad가 대표 재설계로 바뀔지는 research-objective §9 미결정 3에 있다.

### 3.7 Skeleton: 골격 변화를 최소화한 재설계 (2026-11-27)

**배경:** 사용자 결정으로 논문은 세 손을 비교한다.
- CoDe: R, Δ_contact 최소
- Skeleton: 같은 메커니즘과 같은 탐색에서 Δ_skel 최소
- Coverage: U

**Skeleton 손.** `design/scripts/study/exploratory/e25_skeleton_min.py`로 만들었고 손 이름은 `conv_e25`다. 선택 규칙은 coverage가 CoDe보다 0.05 넘게 낮지 않은 설계 중 Δ_skel이 가장 작은 것이다.

| 두께 | Δ_skel | Δ_contact | C̄_pad |
|---|---|---|---|
| 5 mm | 0.06 | 3.43 | 0.738 |
| 7 mm | 0.34 | 6.61 | 0.684 |
| 9 mm | 1.94 | 8.51 | 0.646 |

사전 정한 18개 손의 상관(ρ) 집합에는 넣지 않고 E만 따로 잰다. 사전 등록은 `docs/experimental/pr_mr.md`에 있다.

**Drake 다섯 과제 (7 mm, E):**

| 과제 | point (CoDe / Skeleton / Coverage) | hydro (CoDe / Skeleton / Coverage) |
|---|---|---|
| pull-out | 19 / 86 / 72% | 17 / 388 / 254% |
| 획득 | 21 / 107 / 74% | 16 / 92 / 66% |
| hook | 29 / 246 / 150% | 42 / — / — |
| 손끝 원판 | 17 / 46 / 48% | 4 / 15 / 12% |
| 상자 세우기 | 5 / 27 / 20% | 5 / 21 / 18% |

- hook의 Skeleton 값은 48쌍 중 2쌍만 버틴 결과다.
- 손끝 원판과 상자 세우기는 hydro 재실행과 같은 처음 32개 시작에서 잰 값이다.

**결과.**
- CoDe는 모든 과제와 접촉 모델에서 가장 작다.
- Skeleton은 잡기 과제에서 Coverage보다 크고, 손끝 과제에서는 Coverage와 비슷하거나 더 크다.
- 골격 경로를 0.34 mm 안으로 지켜도 접촉면이 6.6 mm 움직이면 측정이 틀어진다.

**Table I.** Drake 행:
- pad 형상: +181%
- point: +82%
- hydro: +388%
- 두께만큼 부드러운 pad: +490%
- 접촉 setpoint: +11%

Isaac 행은 GPU 대기열에 있다.

## 4. 통찰

### 4.1 왜 순서가 유지되나

- 재사용 setpoint는 같은 굽힘각을 같은 강성으로 명령한다. face가 O보다 물체 쪽으로 나와 있으면 더 세게 누르고(U: distal 힘 3.3배), 물러나 있으면 덜 누른다(R 0.75배). 그 예하중과 접촉점 경로가 과제의 결과(한계 하중, 끌려온 거리, 회전, 기운 각)를 바꾸고, 마찰 효과도 그 위에서 측정된다. 그래서 E는 face가 움직인 양, Δ_contact를 따른다.
- 이 경로는 접촉 모델에 묶여 있지 않다. 다섯 과제가 point와 hydro 둘 다에서 R < U이고 ρ > 0이다. pull-out은 hydro에서 오히려 강해진다(0.67 → 0.86; U의 오차가 72 → 254%).
- skeleton을 지키는 것은 답이 아니다. 고정축 계열(Δ_skel = 0)이 95–228% 어긋나고, 다섯 과제 모두 ρ(E, Δ_skel) ≤ 0.11이다.

### 4.2 어디서 약해지나

- **hydro는 손끝 과제의 효과를 줄인다.** 손끝 원판은 hydro에서 모든 손의 E가 point의 약 1/4이 되고(R 17.0 → 4.1%, U 47.9 → 12.4%) ρ도 0.69 → 0.44로 약해진다. 면 접촉이 접촉점을 넓게 퍼뜨려 face 위치 차이를 덜 드러내는 것으로 본다(확인 안 함). 상자 세우기는 point에서도 약했고(0.44) hydro에서 그대로다(0.42). 잡기 과제(21, 23)는 약해지지 않는다.
- **38의 힘 gate.** point의 1.5 mm 관통 gate를 3.75 N 힘 gate로 옮겼는데, hydro의 pad 힘은 point의 3–4배다(D(O) 중앙 1.80 대 0.50 N). 그래서 Δ_contact가 큰 손이 정확히 빠지고(U는 Gate 0 28, 힘 gate 24로 54쌍 모두), 판정 1을 확인할 수 없다. 18개 손 민감도(0.76)와 사후 hydro 등가 gate(0.69)는 순서가 남아 있음을 보이지만, 사후 gate는 판정이 아니다.
- **잡음.** 38의 point 잡음 75분위는 9%(하중 한 step이 |D(O)|의 13%)이고 23의 hydro 잡음은 5.8%다. 그 근처의 손(`h3to5_hr` 38차 8.4%)은 구별되지 않는다.

### 4.3 순서를 깨는 손

- `d4p5_s0`, `d5p5_s0`(Δ_contact 4.4, 5.4 mm, Δ_skel 11–12 mm)는 잡기 과제에서는 쥐지 못하거나(21·38 점수 없음, 23 lost 15–16/25) 가장 큰 오차(23: 99, 102%)를 내지만, 손끝 과제에서는 Δ_contact에 비해 오차가 작다(앞 32개 뽑기에서 35 point 2.5, 3.2%, hydro 2.0, 2.2%; 29z hydro 3.2, 3.4%; 29z point 26–27%로 R보다 크지만 U보다 작음; 35 전체 64개 point 2.3, 2.8%).
- skeleton을 크게 움직인 다른 손도 같은 쪽이다: `h3to5_d3_s0`(35 앞 32개 2.5%), `d2p25_maxDs_s0`(4.8%), 38의 `d3_maxDs_s0`(20%)와 `h3to9_d3p5_maxDs_s0`(32%).
- 손끝 과제의 결과는 누름, 즉 손끝 경로의 수직 성분을 크게 따르는데 Δ_contact는 방향 없이 경로 변화의 최대를 잰다. skeleton이 크게 바뀐 손은 그 수직 성분이 O에 가깝게 남을 수 있다. 원인은 확인하지 않았다(가설; SSoT에 넣지 않음). 이 손들이 순서를 거슬러도 R < U는 모든 과제에서 유지되고, ρ는 손끝 과제에서 약해진다(0.41–0.69).

## 5. 논문에 넣을 결과

논문은 "Study N", CFL을 쓰지 않는다(`paper/SSoT.md`). 아래가 논문이 쓰는 주장과 숫자 전부다.

1. **재사용 setpoint에서 E는 Δ_contact를 따르고 다른 후보는 아니다 (Isaac).** 마찰 0.83(15), 강성 0.80(14), 격자 0.86(45), torque 0.75(16); 모든 setpoint p ≤ 0.0005, Holm 통과; partial 0.86–0.91; coverage·두께·전달 |ρ| ≤ 0.26. CoDe 3, 21, 41%(5, 7, 9 mm), Conventional 124–173%(7 mm, 마찰·강성). 고정축 95–228%. Δ_contact ≲ 1 mm면 E 약 20%.
2. **다섯 과제, 두 접촉 모델 (Fig. 4).** 모든 과제·모델에서 CoDe < Conventional, E가 Δ_contact를 따름. point / hydro의 E_R / E_U: pull-out 19 / 72%, 17 / 254%; 획득 21 / 74%, 16 / 66%; hook 29 / 150%, 42% / —; 손끝 원판 17 / 48%, 4 / 12%; 상자 세우기 5 / 20%, 5 / 18%(원판과 상자는 같은 32개 시작). 한 줄 단서: hook의 hydro에서는 point에서 옮긴 힘 gate가 Conventional의 모든 쌍을 걸러내지만, 실패를 가장 큰 오차로 두면 순서가 남는다(ρ 0.76).
3. **Table I.** pad 변화(Isaac 마찰 −21 / +115%, 강성 −25 / +173%, 부드러운 pad −14 / +125%, 형상 −43 / +70%), 접촉 모델(Drake point −19 / +70%, hydro −15 / +254%, 두께만큼 부드러운 hydro −8 / +347%), 명령(setpoint, 접촉 setpoint, torque, touch, force). 값은 `table_signed.json`(부호 있는 편향의 중앙값).
4. **결론 뒤집힘 (H4).** O에서 −37%(격자; O의 배치 −41%), 측정 오차가 Δ_contact를 따름(ρ 0.94, 사전 기록); Conventional은 +48%, CoDe는 −55%.
5. **다른 프로토콜.** force 3–7%(양쪽 pad 힘 감지 필요), touch 6–61%(skeleton을 따름), torque 9–53%(둘 다 따름).
6. **범위.** 시뮬레이션, 두 엔진, 두 접촉 모델, CoDe 한 메커니즘, q_req 120°, 열린 루프 다섯 과제; in-hand 굴리기에서는 성립하지 않음(22차, ρ 0.28).

## 6. 한계와 범위

- **과제의 경계.** in-hand 굴리기(22차, sweep +8°, +15°): 재설계가 10–35%를 얹지만 Δ_contact도 Δ_skel도 순서를 정하지 않는다(ρ 0.28, p 0.16). 다섯 과제 밖이고 경계로만 보고한다. 두 pad 다이얼(24·26차)은 distal 관통 2–11 mm로 pad 모델 범위 밖이라 28z로 대체했다.
- **hydro가 없는 결과.** 형상(34), 고정축, 격자, 후보 비교, 조건 B–E, 27차는 point뿐이다. 34는 hydro에서 O가 2 배치만 남아 분석할 수 없다.
- **평면 손가락, MCP 고정.** Drake는 손가락 하나, Isaac은 같은 세 손가락이 한 평면 원기둥을 쥔다. MCP level은 지키지 않는다(두꺼운 pad는 proximal face를 MCP 축에서 멀리 둠; 지키려면 spacer나 pad 안의 축).
- **관통.** point의 관통 = 힘 / 접촉 강성이 pad 두께를 넘으면 모델 범위 밖이다. 28차부터 1.5 mm gate를 미리 둔다. hydro에서는 3.75 N 힘 gate가 되며 38에서 지나치게 엄했다(§4.2).
- **순열 p.** 18개 손 Drake p는 20,000번 무작위 순열이지 정확 p가 아니다(15–27차 계획 일부가 "정확"이라 썼음).
- **사후 수정은 탐색으로 보고한다.** impratio 1000(3차), 4차 손 재선정·확장, 대칭 지표, 격자 분석 키(12·13차, 222c111), 6차 동률 규칙, 10차 gate 값, 21차 solver 재시도, 38 hydro의 hydro 등가 gate.
- **기록 정정 (2026-11-26).** 23차 유지 시간은 1 s다(옛 문서의 "2 s 유지"는 run의 `hold_s` 1.0과 다름). 23차에서 H가 끝에 놓친 쌍은 lost로 따로 세고 점수에 넣지 않는다(옛 문서와 raw metadata의 "가장 큰 오차로 둔다"는 코드와 다름; `fig4_sync.py`가 E를 그대로 재현해 확인).
- 하드웨어 Stage 2(SSoT §8.4)는 아직이다. 계획한 H_R, H_M, H_U의 E는 21, 72, 124%(Δ_contact 0.86, 2.33, 3.91 mm)이고 R < M < U는 48개 블록 중 43개.

### 6.1 TODO

- **23b: distal pad에서 시작하는 grasp 획득.** Fig. 4의 획득 렌더(`sim/drake/render_acquire_tip.py`, `logs/render_acquire_tip_strip.png`)는 원기둥을 편 손가락의 distal pad 위(proximal-face 좌표 x 62 mm, pad의 proximal 끝에서 6.6 mm)에 두고 시작한다.
  - 잰 23차는 두 pad가 쥐는 위치에서 손끝 쪽으로 3·6 mm 옮긴 곳(편 손가락에서는 PIP 근처)에서 시작하므로, 지금 렌더는 실험과 시작 위치가 다르다.
  - 같은 시작으로 실험을 돌려 렌더와 맞춘다. 순서는 지금까지와 같다.
    1. 사전 기록을 쓰고 O를 보정한다.
    2. R과 U를 먼저 돌려 E_U < 10%이면 멈춘다.
    3. 통과하면 18개 손과 대체 손을 point와 hydro 둘 다 돌린다.
  - O 렌더 시뮬레이션에서는 x 64 mm까지의 시작이 두 pad 쥐기(중심 25, −21 mm)로 끝난다.
  - 66 mm부터는 닫히는 손끝이 원기둥을 자기 아래에 가둔다. 원기둥은 proximal pad에 닿지 않고, 중심이 쥐는 자리보다 약 20 mm 아래에 남는다(MCP 고정, 손가락 하나).
  - 23b의 시작 범위는 이 경계 안에서 정한다.
  - 렌더의 엄지는 물체를 감싸는 듯한 그림용 자세일 뿐이다. 시뮬레이션하지 않고 물체에 닿지 않는다.

## 7. 실험 색인

계획 commit은 계획 제목이 처음 들어간 commit, 결과 commit은 결과가 들어간 commit이다. 8d71987 전의 기록은 `git show <hash>:docs/experimental/separability.md`. 계획과 결과가 같은 commit인 실험(3B, 4, 9, 10, 12, 13, 14)은 git만으로 사전 기록을 확인할 수 없다.

| 차 | 목적 | 상태 | 계획 | 결과 |
|---|---|---|---|---|
| 1–2 | 준정적 형상, 축에 끼운 원기둥 마찰 | 탐색 | 25a096e, 7e5e061 | 같음 |
| 3 | 두 pad pull-out O·R·U, 조건 A·B | B 실패(−z 동률), A는 R < U | 6be3d93, d974ef3 | d974ef3 |
| 4 | 7 mm dose-response 8개 손 | 통과 | 4b3f362; 1f4b6f6 | 4b3f362; 1f4b6f6 |
| 5 | 조건 C, 7 mm | 겨우 통과 (p 0.048) | 1f4b6f6 | af54b04 |
| 6 | 세 두께 15–16개 손, A와 C | 통과 | 1f4b6f6; af54b04 | 74d3ed5 |
| 7 | 조건 D 강성 sweep | 통과 (기술) | 74d3ed5 | ae45aa6 |
| 8 | robustness, 형상 6개 손 | 통과; 형상은 34로 대체 | 17b55f8 | 47d3572, 02b4fe8 |
| 9 | 조건 E | 형식상 통과; 남은 오차는 Δ_skel | f777b82 | f777b82 |
| 10 | pad compliance | 통과 | f777b82 | f777b82 |
| 11 | 다섯 후보 비교 | 탐색 | — | e56d406 |
| 12, 13 | 손과 무관한 격자 18·45개 손 | 통과; 정정 222c111 | 58fcdcd, a4c6a56 | 같음; 222c111 |
| 14 | 고정축 대조 계열 | 통과 | 29c45bd | 29c45bd |
| 15 | 부드러운 pad, 딱딱한 O 기준 | 제외 (기준이 pad 효과를 담지 않음) | ed8d53a | 1fd3f9a |
| 16 | Isaac hydro | 중단 | a19ffda | 11ffe35; 2ea7160 |
| 17 | 부드러운 pad, 기준 O_h | 통과 | 2d449e9 | dae404c |
| 18 | 조건 C, 격자, 새 27개 손 | 통과 | c51e92d | 3af6625 |
| 19 | 결론 확인(H4) | (1) 통과, (2) 실패 | 6bc25cf | 6167aa7 |
| 20 | 물체 pose 오차와 비교 | 취소 | — | 3af6625 |
| 21 | Drake 복제, point와 hydro | 통과 | 2060ab5 | 251c28c, b3bf4ce |
| 22 | in-hand 굴리기 | 실패 (경계) | e8f71dc | 8590c36 |
| 23 | grasp 획득 | 통과 | 6ad895e | d09b5b5 |
| 24, 26 | 두 pad 다이얼 | 대체됨 (28z) | 4db9b3c, eb615e4 | 032860f, 3b38fe7; a2707db |
| 25 | 두께만큼 부드러운 hydro pad | 통과 | a280bef | 54faaa9; aca7665 |
| 27 | 접촉 기준 setpoint | 통과 | d34be24 | 18be958 |
| 28 | 손끝 원판 z (y는 제외) | 통과 | 1b166a5 | 77b0c9d; a2707db |
| 29 | 28차의 무작위 64개 | 통과 | 01af200 | 90f2556; 5c1a1ff |
| 34 | pad 형상, 형상마다 맞춘 배치 | 통과 | fa6b107 | c048965 |
| 35 | 상자 세우기 | 통과 (p 0.04995) | 6b3a1f8 | 2faa812 |
| 38 | hook | 통과 | 0e7bc80 | cba6ccb |
| hydro | 다섯 과제의 hydro 재실행 (21차 규칙) | 21·23·29z·35 판정 1·2 성립, 38 판정 2만 | 0535fdb; O만 8063f49 | 6b808e5 (23), 2a1e814 (29z), a02ae1e·183cb7b (38), 5ef5a6c (35) |

30, 30b(동전), 33, 33b, 36, 39–42는 Study 3 밖이다: `report/experimental/study3_excluded.md`.

설계 쪽 탐색 스크립트 e16–e20(`design/scripts/study/exploratory/`)은 실험 번호와 별개다: e16 재파지 기하, e17 두 pad wrench 예측, e18 Δ_contact 목표 재설계, e19 Study 2 시작 오프셋 분리, e20 face 오프셋·미끄러짐 분해. 보관 문서(3A/3B RL pilot, regrasp 기하 노트, experiments.md 전문)는 `git show 5c1a1ff:report/study3/<파일>`.

## 8. 데이터와 스크립트

- **요약 `report/study3/data/`.** Isaac: `isaac_dose_A_all.json`(6차), `isaac_comp.json`(10차), `isaac_dose_A-grid_expanded.json`(13차), `isaac_padk_matched.json`(17차), `isaac_dose_{B,C_all,E_all}.json`, `isaac_shape.json`, `isaac_conclusion_study3.json`(H4). Drake point: `drake_e21_point.json`, `drake_e21_point_pairs_RU.json`, `drake_e23_point.json`, `drake_e38_point.json`(궤적 편차 `deviation`), `drake_e28z_point.json`, `drake_e29z_point.json`, `drake_e29z_point_first32.json`, `drake_e35_pivot.json`, `drake_e35_point_first32.json`, `drake_e27_point.json`, `drake_e34_shape.json`, `drake_e22_point.json`(경계). Drake hydro: `drake_e21_hydro.json`, `drake_e23_hydro.json`, `drake_e38_hydro.json`(사전 판정, `sens18`, 사후 `posthoc_force_gate`), `drake_e29z_hydro.json`, `drake_e35_hydro.json`, `drake_e25_hydro_own.json`. 그림 데이터: `fig4_sync.json`(Fig. 4; 손별 E와 부호 있는 편향, point와 hydro, 궤적 편차), `table_signed.json`(Table I).
- **raw `report/study3/data/raw/`** (`report/make_raw_tables.py`, 정의는 `metadata.json`). run 단위 `runs_isaac_pullout.csv`, `runs_drake_{pullout,acquisition,hook,tipdial,pivot,shape,rolling}.csv`(hydro run은 `contact_model` = hydro), 손 단위 `errors_by_hand.csv`.
- **렌더.** `report/study3/figures/`: `power_grasp_hold_iso.png`(21), `grasp_acquisition.png`(23, `sim/drake/render_acquire.py`), `hook_pull.png`(38, `render_hook.py`, 손잡이를 따라 본 시점), `fingertip_disk.png`(28z), `fingertip_pivot.png`(35). 프레임은 `data/renders/`.
- **Drake `sim/drake/`.** `two_pad_drake.py`(pull-out, 획득, 원판; `--curve`는 궤적을 남김, 23차의 궤적 rerun은 tag e23c로 결과가 비트 단위로 같음), `hook_pull.py`, `box_pivot.py`, `pad_shape.py`, `analyze.py`, `analyze_roll.py`, `hydro.py`·`hydro_run.py`(hydro 패치), `hydro_subset.py`(앞 32개), `hydro_sens18.py`, `hydro_relax.py`·`hydro_gate_eq.py`(사후 gate), `run_hydro.sh`, 렌더 `render_*.py`. 로그 `sim/drake/logs/`.
- **Isaac Lab `sim/isaaclab/scripts/study3_measurement/`.** 실행 `two_pad_wrench.py`, `fixed_axis_hands.py`, `run_*.sh`; 분석 `dose_analysis.py`, `invariants.py`, `padk_matched.py`, `torque_grid_analysis.py`, `conclusion_analysis.py`, `mediation.py`, `tolerance.py` 등. 로그 `sim/isaaclab/logs/study3/two_pad/`.
- **손.** `design/scripts/study/study3_trajectory/make_hands.py`(O, R, U), `design/scripts/study/exploratory/e18_rct_dose.py`, 배치 `design/output/sim/study3/`.
