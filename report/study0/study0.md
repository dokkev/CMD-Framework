# Study 0: 관절이 pad에서 빼앗는 길이 (해석)

정리: 2026-11-24. `derivation.md`(2026-11-20, 2026-11-21 고침), `feasible_space.md`(2026-11-21), `report/study2/pole_law.md`(2026-11-20 작업 기록)를 하나로 줄였다. 정의는 `design/SSoT/problem_formulation.md` A.6.0, 주장은 `design/SSoT/research-objective.md` §6. 이 문서가 Study 0의 수학과 증명의 기준이다.

## 1. 질문과 SSoT의 주장

질문: 최적화 없이, 요구(pad 두께 h, 요구 flexion q_req)와 morphology 사이의 관계를 기하로 쓸 수 있는가. 관절은 pad에서 길이를 얼마나 빼앗고, 다시 설계할 때 접촉면에서 본 운동은 얼마나 바뀌는가.

| SSoT | 내용 | 이 문서 |
|---|---|---|
| R0.1 (→ C1) | 유한 pole로 유도한 clearance 법칙(S1, S2에서 정확)과 coverage 법칙. 가장 좋은 관절은 contact face 위의 점을 중심으로 돈다. 실제 pin은 pad 장착면 위에 있으므로 그런 점은 가상의 pole이어야 한다 | 3.3–3.5, 4.4 |
| R0.2 (→ C2.1) | 접촉면 운동 법칙 ‖Δc‖ = 2 sin(q/2)‖Δp‖, 합 하한 Δ_contact + Δ_skel ≥ 2 sin(q_req/2)\|Δt\|, 고정 축 하한(roller 반지름이 고정된 구름 접촉에서도 성립) | 3.6, 4.1, 4.2 |
| H0.1 | Coverage 최적에서 pad–pad 접촉이 자주 활성 제약 | 5절: 51개 중 47개가 0.1 mm 이내 |
| H0.2 | Pole 경로와 pad로 계산한 coverage = 평가기 | 5절: 51개 모두 10⁻¹³ 이내 |
| H0.3 | 고르게 놓인 고정 축 설계는 G = 2(d+h) tan(q_req/2) | 5절: G − G_sym 고정 축 −3.4~+0.2 mm, CFL −1.6~+0.2 mm |
| H0.4 | 고정 축의 어떤 다시 설계도 하한을 넘지 못함 | 5절, `report/study2/study2.md` |

R0.1, R0.2는 기하의 정리이다. H0.1–H0.4는 최적화 결과에 대한 가설이고 Study 1, 2의 설계로 확인한다.

## 2. 설정

- **좌표.** Proximal phalanx에 고정. x는 link 선을 따라 손끝 쪽, z는 dorsal 쪽이 +. Link 선은 기준 자세(q = 0)에서 관절 축들을 잇는 직선 z = 0.
- **굽힘.** Flexion q ∈ (0, π]에서 distal phalanx는 palmar 쪽으로 q만큼 돈다: y(q) = R_q y₀ + b(q), R_q = [[cos q, sin q], [−sin q, cos q]].
- **Pad.** Pad i (1 proximal, 2 distal)는 자기 phalanx에 고정된 직사각형. 장착 깊이 d_i(link 선에서 pad 윗면), 두께 h_i, contact face 깊이 t_i = d_i + h_i. 기준 자세에서 proximal pad는 [x₀, x_1e], distal pad는 [x_2s, L_f]. 관절 간격 G = x_2s − x_1e이고

  (ℓ₁ + ℓ₂)/L_f = 1 − (x₀ + G)/L_f.   (1)

- **겹침.** 양의 넓이로 겹치면 충돌, 경계 접촉은 허용. 설계가 만족할 조건은 한 자세가 아니라 (0, q_req]의 모든 자세에서 겹치지 않는 것.
- **가정.** (A1) 평면 운동. (A2) Pad는 강체 직사각형, contact face는 자기 link 선에 평행. (A3) 두 pad 사이의 충돌만 다룬다(다른 부품은 d_i와 pad 범위로 들어온다). (A4) Distal phalanx의 방향은 어느 구조에서나 q.

## 3. 정리

### 3.1 Pole

**보조정리 1 (finite pole).** q ∈ (0, 2π)이면 변위 y₀ ↦ R_q y₀ + b는 한 점 P를 중심으로 한 q 회전이고

  P(q) = (I − R_q)⁻¹ b(q) = ½ (b − cot(q/2) J b),  J = [[0, −1], [1, 0]].   (2)

det(I − R_q) = 4 sin²(q/2) > 0이므로 유일하다. 손끝 경로 r_C(q)만 알면 b(q) = r_C(q) − R_q r_C(0)이므로 구조와 무관하게 같은 식으로 계산한다. Pole의 깊이 z(q) = −P_z(q)(palmar 쪽 +), pole에서 본 face 깊이 τ_i(q) = t_i − z(q).

**Finite pole은 순간 회전 중심이 아니다.** y′(q) = R′_q (y₀ − P) + (I − R_q) P′(q)의 둘째 항 때문에, 둘은 pole이 고정된 관절에서만 같다. q → 0에서 P(q)는 기준 자세의 순간 회전 중심으로 수렴한다. 아래 명제는 유한 변위만 쓴다. 계산에서는 |q| < 10⁻⁶ rad에서 pole을 정의하지 않는다.

**보조정리 2 (반사).** P를 지나는 수직선에 대한 반사 F, P를 지나고 법선이 n_q = (cos(q/2), −sin(q/2))인 직선 m_q에 대한 반사 M_q에 대해 R_q ∘ F = M_q.

### 3.2 명제 1: 일반 clearance (임의의 pole, 유한한 두 pad)

Proximal pad 중심 c_A, 반길이 (a_x, a_z), distal pad 기준 자세 중심 c_B(0), 반길이 (b_x, b_z), c_B(q) = P + R_q (c_B(0) − P). 후보 축 U(q) = {e_x, e_z, R_q e_x, R_q e_z}에서

  m_u(q) = |uᵀ(c_B(q) − c_A)| − (a_x|uᵀe_x| + a_z|uᵀe_z|) − (b_x|uᵀR_q e_x| + b_z|uᵀR_q e_z|)

라 하면

  **자세 q에서 겹치지 않음 ⇔ m(q) := max_{u∈U(q)} m_u(q) ≥ 0**,  설계 조건: min_{0<q≤q_req} m(q) ≥ 0.   (3)

증명은 separating axis theorem(직사각형 변의 법선은 자기 두 축뿐). Pole, 깊이, 길이, 두께에 조건이 없고 **설계의 판정은 이것으로 한다**. 한 자세에서는 정확하고, 구간 조건은 q 격자에서 확인한다(대조 0.05°, study 평가기 1° 탐색 후 0.05° 재확인). 연속 구간의 인증은 아니다. m(q) > 0의 크기는 가장 좋은 축의 틈이며 유클리드 거리와 다르다.

### 3.3 명제 2: 대칭 clearance 법칙

q_req < π. (S1) 두 face 깊이가 같다(τ₁ = τ₂ = τ). (S2) 모든 자세에서 두 pad의 관절 쪽 아래 모서리가 pole에서 x 방향으로 같은 거리 s에 있다(pole의 x가 두 모서리의 가운데에 머문다). 그러면

  **(0, q_req] 전체에서 겹치지 않음 ⇔ s ≥ max_q [τ(q)]₊ tan(q/2)**,
  G_sym(h, q_req) = max_{0<q≤q_req} 2 [d + h − z(q)]₊ tan(q/2).   (4)

최댓값을 주는 q가 임계 flexion q*. 증명: 보조정리 2로 돌린 distal pad는 proximal pad K의 거울상 M_q K이고, 볼록 집합과 그 거울상의 내부가 겹치지 않을 필요충분조건은 K가 m_q의 한쪽 닫힌 반평면에 있는 것. n_q·u는 관절 쪽 아래 모서리에서 최대이고, q → 0⁺에서 K는 음의 쪽에 있으며 연속성 때문에 넘어가려면 중간 자세에서 m_q를 지나야 한다. 길이가 다르면 짧은 쪽을 늘려도 모서리 값은 같다.

이 법칙이 말하지 않는 것:
- 한 자세의 필요충분조건이 아니다(유한한 pad는 겹쳤다가 지나칠 수 있다).
- (S2)가 깨지면 추정값이다(pole의 x가 움직이거나 pad가 고르게 놓이지 않을 때).
- 모든 배치 중 최소가 아니다. 예: 고정 축, q_req = 90°에서 한 모서리를 축 바로 아래, 다른 모서리를 t 떨어뜨리면 간격 t로 충분하지만 (4)는 2t. 그래도 대칭 배치가 기준인 이유는 명제 3(더 붙인 부분은 coverage로 세지 않음).

**따름정리 2.1 (두께에 대한 볼록성).** z(q), d 고정이면 G_sym은 h에 대해 볼록(아핀 함수와 0의 상한). q*(h)가 유일하고 τ(q*) > 0인 곳에서 dG_sym/dh = 2 tan(q*/2) ≤ 2 tan(q_req/2). q*가 h에 따라 움직이므로 일반적으로 구간별 선형이 아니다(고정 축만 선형).

### 3.4 명제 3: 두 face가 만나는 점과 원기둥 기준 coverage

(S1)만 가정하면 자세 q에서 두 contact face 직선은 m_q에 대해 거울상이므로, pole을 원점으로 X(q) = (−τ tan(q/2), −τ)에서 만난다. 각 face 위에서 X는 pole의 수선의 발에서 τ tan(q/2) 떨어져 있다(pad 길이와 배치에 무관). A.5.1의 정의에서 pad i가 X로부터 덮는 거리는 T_i = [max(a_i, 0), a_i + ℓ_i], a_i = s_i − τ tan(q/2)이고 C_pad(q) = 2|T₁ ∩ T₂|/L_f(q). X보다 관절 쪽에 놓인 pad 부분은 세지 않으므로, 고르게 놓은 배치보다 더 붙여도 coverage는 늘지 않는다.

(S1), (S2), ℓ₁ = ℓ₂, G = 2s, phalanx 구조의 끝이 pad의 관절 쪽 끝과 같은 x에 있으면

  **C_pad(q) = (L_f − x₀ − G) / (L_f − G (1 − cos(q/2)) + 2 z(q) sin(q/2))**   (5)

(L_vp = 2(s cos(q/2) + z sin(q/2)), L₁ + L₂ = L_f − G 대입). 분모의 −G(1 − cos(q/2))는 관절 간격이 굽히면 짧아지는 것, +2z sin(q/2)는 pole이 palmar 쪽이면 두 phalanx 끝이 벌어지는 것이다. 조건 밖의 설계는 X, T_i, L_vp를 pole과 pad로 직접 계산한다(`pole_law.coverage_law`). 그 입력에는 phalanx 구조의 끝(CFL은 pivot housing일 수 있음, A.3)과 기준 자세의 distal link 선(CFL은 link 길이 반올림으로 최대 0.24 mm 어긋남)이 포함된다. C̄_pad는 q ∈ {90°, (90° + q_req)/2, q_req}의 평균.

**따름정리 3.1 (pole을 내리는 것).** Pole 깊이가 상수 z < t이고 G = 2(t − z) tan(q_req/2)인 대칭 모델에서 z를 늘리면 (5)는 q ≤ q_req의 모든 평가 자세에서 커진다. (T = tan(q_req/2): ∂N/∂z = 2T, ∂D/∂z = 2T(1 − cos(q/2)) + 2 sin(q/2) ≤ 2T, 등호는 q = q_req; N < D이므로 ∂(N/D)/∂z > 0.) 이득은 z = t(G = 0)까지이고, z > t는 분모만 키운다. **이 모델에서 가장 좋은 관절은 contact face 위의 점을 중심으로 도는 관절이다**(τ = 0, G = 0).

### 3.5 명제 4: 접촉면에서 본 운동

Proximal contact face를 기준으로 본 pole을 p(q)(두 성분), distal face 위 점의 기준 위치를 c₀라 하면 c(q) = p + R_q (c₀ − p). 같은 c₀의 두 손가락에서 pole이 Δp(q) 다르면

  **‖Δc(q)‖ = 2 sin(q/2) ‖Δp(q)‖**   (6)

(Δc = (I − R_q)Δp, (I − R_q)ᵀ(I − R_q) = 4 sin²(q/2) I).
- 따름정리 4.1: 기준 자세의 상대 위치가 같을 때 접촉면 운동이 같을 필요충분조건은 모든 q에서 p(q)가 같은 것. 깊이 τ만이 아니라 pole의 x도 같아야 한다. 단차 t₁ − t₂가 다르면 c₀가 달라져 (6)에 더해진다.
- 따름정리 4.2: p(q), pad 치수, face 위 위치가 같으면 (3)의 m(q)도 같다. 접촉면 운동을 지키는 다시 설계는 같은 pad를 같은 자리에 실을 수 있다.

### 3.6 합 하한 (A.6.3)

Face 깊이가 Δd(가장 얕은 장착에서 Δh = Δt) 바뀌면, contact face에서 본 pole 변화와 link 선(뼈대)에서 본 pole 변화는 Δp_face = Δp_link − Δd e_z이다. q = 0의 위치가 같으면 ‖Δc(q)‖ = 2 sin(q/2)‖Δp_face‖, ‖Δr_C(q)‖ = 2 sin(q/2)‖Δp_link‖이고 삼각부등식으로

  **Δ_contact + Δ_skel ≥ 2 sin(q_req/2) |Δt|**.   (7)

곧 pad가 깊어지면 접촉면 운동과 뼈대 운동을 함께 지킬 수 없다. 고정 축은 Δp_link ≈ 0이므로 Δ_contact가 하한 전부를 진다(4.1).

## 4. 구조별 결과

### 4.1 고정 축 (2R-DD, 2R-remote)

Pole = 관절 축: z ≡ 0, P_x 상수. 깊이가 같고 축에 대해 고르게 놓으면 (S1), (S2)가 성립하고 q* = q_req:

  G_sym = 2(d + h) tan(q_req/2),  ∂G_sym/∂h = 2 tan(q_req/2),  ∂G_sym/∂q_req = (d + h) sec²(q_req/2).   (8)

| q_req | 90° | 120° | 150° |
|---|---|---|---|
| 두께 1 mm당 G_sym 증가 | 2.00 mm | 3.46 mm | 7.46 mm |

깊이가 다르거나(2R-DD) 길이 하한 때문에 고르게 놓지 못하면(큰 q_req) (8)은 추정값이고 명제 1로 판정한다.

**고정 축 하한.** Contact face가 축에서 δ = Δh − Δd 멀어지도록 다시 설계하면(Δd: 장착 깊이를 줄인 양), 축이 어디에 있든 Δp는 q에 무관한 상수이고 ‖Δp‖ ≥ |δ|. (6)에서

  max_{q≤q_req} ‖Δc(q)‖ ≥ 2 sin(q_req/2) |Δh − Δd|,   (9)

등호는 축이 face에 대해 x로 움직이지 않을 때. 다시 설계의 내용과 무관한 하한이다. 2R-remote는 d = 0이므로 Δd = 0. c₀를 face를 따라 미끄러뜨리는 것까지 허용한 하한은 `report/study2/study2.md` 참조(0.586, 0.667, 0.794배).

### 4.2 구름 접촉 (RCJ, 같은 반지름 r)

Proximal roller 중심 (a, 0)에서

  P(q) = (a + r, −r tan(q/4)),  z(q) = r tan(q/4).   (10)

(같은 반지름의 구름에서 distal은 proximal을 공통 접선에 대해 반사한 것이고, 변위는 두 반사의 합성.) P_x가 상수이므로 깊이가 같고 x = a + r에 대해 고르게 놓으면 (S1), (S2) 성립. f(q) = (t − r tan(q/4)) tan(q/2), f′(q) = ½ sec²(q/2)(t − r sin(q/2)):
- t ≥ r sin(q_req/2)이면 q* = q_req, G_sym = 2[t tan(q_req/2) − r(sec(q_req/2) − 1)], 두께 기울기는 고정 축과 같은 2 tan(q_req/2).
- t < r sin(q_req/2)이면 q* = 2 arcsin(t/r)(구간 안쪽).

r이 관절 actuator로 정해져 있으면 pole을 옮길 수 없으므로 (9)가 같은 형태로 성립한다.

### 4.3 Crossed four-bar (CFL)

Pivot A, A′(proximal), B, B′(distal), link AB′, A′B. Finite pole은 (2)로 계산하고 z(q), P_x(q)는 linkage 치수가 정한다. **Crossed four-bar 일반에 대한 주장은 없다.** Study의 최적 CFL 설계에서 관찰한 것: z(q)는 기준 자세 근처에서 음수(dorsal)이고 굽힐수록 커진다. q* < q_req(구간 안쪽, 예: 5 mm에서 87–99°)이므로 q_req를 올려도 G_sym이 커지지 않는다. P_x(q)가 움직이는 설계가 있고, 그때 (4)는 추정값이다.

### 4.4 Pole이 놓일 수 있는 자리 (가상의 pole)

복소수로 P(q) = b/(1 − e^{iq}) = (b/2)(1 + i cot(q/2)). Proximal phalanx 좌표(원점: 관절 쪽 끝의 축 위)에서 한 자세의 P가 설계에 따라 차지하는 집합:
- **고정 축:** 한 점(P = 0).
- **같은 roller:** P = r − i r tan(q/4), r 하나에 따른 선분. r ∈ [9.5, 20] mm, 120°에서 (9.5, −5.5)–(20, −11.5) mm. 반지름이 다른 roller(설계 공간 밖, 비교용)도 깊이 5.5–11.5 mm로, 5 mm보다 얕은 pole은 얻지 못한다.
- **Crossed four-bar:** pivot 위치에 따라 영역. 해석적 sweep(최적화 아님): 설계 공간 경계에서 pivot 200,000개, 교차 49,960개, [0, 120°]에서 같은 조립 형태로 닫히는 44,909개. P(120°) − A의 95%가 phalanx 끝에서 약 −23–49 mm, 축에서 약 −11–21 mm. Study 1 CFL(3/5/7/9 mm)의 P − A = (23.7, −3.2), (26.1, −5.3), (28.4, −7.6), (28.4, −8.8) mm로 영역의 palmar 쪽 끝. 닫힘식은 평가기와 소수 둘째 자리까지 같다(2R은 pin, RCJ는 (10)과 정확히 일치). 각 각도까지 닫히는 수: 60/90/120/150°에서 48,877/47,361/44,901/41,393개.

구조마다 허용하는 face 위 pole 높이 τ = t − z:
- Link 선 위의 pin: τ = t.
- Roller: pole을 face에 두려면 r = t cot(q/4) > t(120°에서 r ≈ 1.73t)로 roller가 face 밖으로 나온다. Roller가 face 안(r ≤ t)이면 τ ≥ t(1 − tan(q/4)), 120°에서 0.42t.
- 가상의 pole(four-bar): τ = 0 가능.

Study 1 설계(120°)의 τ: 2R-remote 3/5/7/9 mm(= h), 2R-DD 7.9–9.0, RCJ 6.9–13.0, RCJ ideal 3.1–6.0, CFL 2.4–3.2, CFL ideal 1.4–2.5 mm. 모두 위 하한을 지킨다. CFL만 두께와 무관하게 거의 일정하고, 남은 2–3 mm는 pad가 pivot housing(3 mm) 아래에 달리기 때문이다. Pin과 roller는 pad 장착면 위에 있어야 하므로 깊은 pole을 얻으려면 roller가 커져야 한다. Four-bar의 pole은 가상의 점이라 그 제약이 없다(R0.1).

### 4.5 요약

| 구조 | z(q) | P_x(q) | (4)의 적용 | 접촉면 운동 (face가 δ 멀어질 때) |
|---|---|---|---|---|
| 고정 축 | 0 | 상수 | 깊이 같고 고르게 놓으면 정확, q* = q_req | 하한 (9) |
| 구름 접촉, r 고정 | r tan(q/4) | 상수 | 같은 조건에서 정확, q* = q_req 또는 2 arcsin(t/r) | 하한 (9) |
| Crossed four-bar | linkage가 정함 | linkage가 정함 | P_x가 움직이면 추정값 | p(q)를 유지하는 만큼 작아짐 |

## 5. 대조 (H0.1–H0.4)

Study 1의 72개 조건 가운데 2026-11-21 시점에 끝난 51개 설계(`pole_law.py`, `report/study2/data/pole_law_designs.csv`).

| 가설 | 대조 | 결과 |
|---|---|---|
| H0.1 | 설계 pad로 계산한 min_q m(q) | **51개 중 47개가 0.1 mm 이내.** 예외 넷: RCJ 120°/9 mm 1.68 mm, RCJ ideal 120°/5 mm 0.29 mm, RCJ ideal 90°/7 mm 0.24 mm, CFL 150°/5 mm 0.25 mm |
| H0.2 | Pole과 pad로 계산한 C̄_pad 대 평가기 | **51개 모두 10⁻¹³ 이내** (최대 3.5 × 10⁻¹⁴) |
| H0.3 | G − G_sym | **고정 축 −3.4~+0.2 mm**(크게 벗어나는 것은 pad가 고르게 놓이지 않은 150° 설계), **CFL −1.6~+0.2 mm, CFL 9개 설계의 \|G − G_sym\| 중앙값 0.05 mm**, RCJ ideal 0~+0.7 mm. 추정으로서의 정확도이고, clearance 판정은 명제 1로 한다 |
| H0.4 | Study 2의 2R-remote 다시 설계, q_req 120° | **3.46, 6.93, 10.39 mm**(5/7/9 mm)로 하한과 같다. 90°에서도 하한과 같다(`report/study2/study2.md`) |
| 명제 1 구현 | 네 축 clearance 대 study의 공용 다각형 계산 | 무작위 400개 배치에서 10⁻⁹ 이내. 움직이는 pole, 다른 두께·깊이의 첫 접촉 각도도 일치(단위 테스트) |
| 고정 손가락 | 다른 두께 pad를 단 경우 첫 접촉 각도 대 평가기 q_max | 99개 경우 −0.25°~+2.1°(평가기 1° 격자 수준). 예: 2R-remote 3 mm용 설계에 5/7/9 mm → 식 92.2/73.2/60.1°, 평가기 92/73/60° |
| (10), CFL pole | 닫힌 식 대 수치; CFL pole을 독립 four-bar 구현으로 재계산 | 단위 테스트 6개 통과; 9개 설계에서 pole 깊이 차이 0.13 mm 이하(link 길이 0.1 mm 반올림) |

**C̄_pad 일치에 필요했던 입력.** 처음 계산은 CFL에서 0.0045, CFL ideal에서 0.018까지 달랐다. 원인은 기하가 아니라 입력 두 가지였다. (1) CFL의 phalanx 구조 끝은 pad의 관절 쪽 끝과 pivot housing(반지름 3 mm) 중 관절에 가까운 곳(A.3). (2) Link 길이를 0.1 mm로 반올림해 닫으면 기준 자세의 distal link 선이 최대 0.24 mm 아래에 평행하게 놓인다. 둘을 넣으면 차이가 없어진다. 따라서 가정 (A2)는 CFL에서 0.2 mm 수준으로만 성립하며, 명제 2와 대조 스크립트의 clearance 계산은 이 어긋남을 넣지 않는다.

## 6. 부록: CFL의 가능한 설계 공간 (탐색 보조)

Study 1의 CFL 탐색을 위한 분석이다. SSoT의 R 주장에는 쓰이지 않는다.

- **닫힘은 닫힌 식이다.** u = A − A′(길이 e, 각 α), w = B′ − B(길이 c, 각 β), δ = α − β, link 길이 r₁ = |A′B|, r₂ = |AB′|. B(q)는 중심 A′(r₁)와 Â(q) = A − R_q w(r₂)의 두 원의 교점이고 D(q)² = e² + c² − 2ec cos(δ + q). 기준 조립 형태로 q_req까지 이어질 필요충분조건은 |r₁ − r₂| < D(q) < r₁ + r₂ (∀q ∈ [0, q_req]), 곧 cos(δ + q)가 k_lo = (e² + c² − (r₁ + r₂)²)/(2ec)와 k_hi = (e² + c² − (r₁ − r₂)²)/(2ec) 사이에 머무는 것. 운동 한계 q_max^kin은 cos(δ + q)가 처음 k_lo 또는 k_hi에 닿는 q. 무작위 four-bar 552개에서 실제 굽힘 한계와 0.25° 이내로 일치.
- **Closure 좌표.** (a, e, c, δ, σ, ρ)와 조립 형태, r₁ + r₂ = D_max + σ (σ > 0), r₁ − r₂ = ρ D_min (|ρ| < 1)로 두면 닫힘 조건이 상자가 된다(`core/geometry/fourbar.py`). 평가기가 받아들인 CFL·CFL ideal 623개를 모두 받아들이고(한계까지 여유 최소 2.99 mm), 좌표 변환 후 coverage가 같다(재구성 오차 3 × 10⁻¹⁴ mm, 반올림 회전 제외).
- **좁히는 것은 pad 충돌이다.** 표본 200,000개(120°, 7 mm): 가능 비율은 기존 좌표 0.017%, closure 좌표 0.009%로 오르지 않았고(closure 표본의 98%가 탐색 범위 밖), 가능 설계당 CPU 시간은 18.6 s → 4.4 s. Closure와 transmission angle을 보장한 뒤에도 운동 단계에 닿은 174개 중 160개(92%)가 떨어지고, 원인은 굽히는 중 충돌(첫 충돌 쌍: pad–pad 62%, A의 pin–distal pad 27%). H0.1과 같은 방향의 관찰이다.
- **Linkage 먼저, pad는 그 뒤에.** 같은 linkage 1,066개에서 pad를 명제 1로 조건부로 뽑으면 가능한 linkage 14 → 78개(5.6배), 차지한 칸 14 → 51(3.6배), pad끼리 충돌 115 → 2개, coverage 최고 0.532 → 0.603.
- 이 좌표가 GA를 개선하는지(e11 비교)는 결론 내지 않았다. 2R, RCJ에는 적용하지 않는다.

## 7. 한계와 범위

- 2차원 측면 모델, 강체 직사각형 pad, 두 pad 사이의 충돌만. 가림, MA, 3차원 형상, pad 변형은 다루지 않는다.
- 명제 1의 구간 조건은 격자 확인이며 연속 구간의 인증이 아니다. 대칭 법칙 (4), (5)는 (S1), (S2) 밖에서 추정값이다.
- Crossed four-bar가 p(q)를 얼마나 옮기거나 유지할 수 있는지는 식으로 쓰지 않았다(Study 2가 탐색으로 잰다). 무작위 four-bar 40만 개로 한계를 구해 보았으나 실제 최적보다 나쁜 값만 나와 한계로 쓸 수 없었다.
- 장착 깊이 d는 최적화 결과에서 읽는다(2R-DD, RCJ에서는 pad를 관절 hardware 밑으로 넣는 절충).
- 주어진 pole 경로에서 coverage에 가장 좋은 pad 배치는 주지 않는다. 명제 3은 고르게 놓은 배치보다 더 붙여도 얻는 것이 없음을 말할 뿐이다.
- H0.1–H0.3의 대조는 2026-11-21 시점의 51개 설계이다. 그 뒤 Study 1의 CFL은 재탐색으로 일부 칸이 올랐다(예: 90°/5 mm 0.757 → 0.778, `report/study1/study1.md`). 51개 표는 다시 돌리지 않았다.

## 8. 데이터와 스크립트

| 내용 | 경로 |
|---|---|
| Pole, clearance, coverage 계산 | `design/core/geometry/pole.py` |
| 대조 (H0.1–H0.3, 고정 손가락, 인접 두께 비교) | `design/scripts/study/study2_morphology_adaptation/pole_law.py` → `design/output/study/study2_pole_law/` (`designs.csv`, `poles.csv`, `changes.csv`, `fixed_finger.csv`). 51개 설계 표의 커밋된 사본: `report/study2/data/pole_law_designs.csv` |
| 고정 손가락에 다른 두께 pad | `design/scripts/study/study2_morphology_adaptation/one_finger.py` |
| 단위 테스트 | `design/tests/test_study2_pole_law.py` |
| Four-bar 닫힘, closure 좌표 | `design/core/geometry/fourbar.py`, `design/core/optimization/analysis/closure_search.py`, `design/core/optimization/analysis/placement.py` |
| 6절 표본 실험 | `design/scripts/study/exploratory/e12_closure_sampling.py`, `e14_conditional_pads.py`, `e11_placement_ablation.py` |
| 4.4 pole 영역 (원고 Fig. 2) | `paper/figures/fig_mechanisms.py`; 입력 `report/study0/data/linkage_samples_seeds.csv`(linkage 표본), `report/study0/data/prototype_seeds.csv`와 `prototype_run_003/run.json`(CoDe prototype 손가락), `report/study0/data/fig2_cfl_pad_gap150.json`(150°에서 pad가 0.3 mm로 닫히는 CFL, `paper/figures/fig2_pad_search.py`, `fig2_pad_refine.py`) |

대체된 것: `report/study2/pole_law.md`(2026-11-20)의 24개 설계 표, "모서리 식", "pole보다 아래만 다가온다"(순간 속도에 대한 진술로, pole이 움직이는 CFL에서는 성립하지 않음)는 이 문서의 명제 1–2와 51개 대조로 대체했다. 같은 문서의 인접 두께 비교 표와 `derivation.md` 5절의 CFL (9) 대조 값(0.5, 0.65, 1.73 mm)은 Study 2의 정식 결과(`report/study2/study2.md`)로 대체했다.
