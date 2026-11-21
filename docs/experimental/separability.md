# Playground: 측정 분리 가능성 실험

SSoT가 아니다. 진행 중인 가설과 실험이며, 결과에 따라 바뀐다.

## 질문

Does minimizing changes in the relative contact trajectory reduce embodiment-dependent variation in the measured effect of a contact interface?

## 손

모두 CFL이고, 요구 flexion 120°, pad 3 → 7 mm이다. 만드는 스크립트는 `design/scripts/study/study3_trajectory/make_hands.py`이다.

| 손 | pad | 재설계 | D_c |
|---|---|---|---|
| O (= H₀) | 3 mm | 기준 | 0 |
| R (= H_R) | 7 mm | RCT와 coverage | 0.86 mm |
| U (= H_U) | 7 mm | coverage만 | 3.91 mm |

q90, 3 → 5 mm 조건(D_c 1.22 대 5.84 mm)도 `make_hands.py --flexion 90 --to 5`로 만들 수 있다.

## 1차: pad 형상 (flat 대 crowned), 준정적, 2026-11-21

- 코드는 `sim/isaaclab/scripts/experimental/separability_quasistatic.py`와 `separability_analysis.py`이다. 결과는 `sim/isaaclab/logs/experimental/separability/`에 있다.
- 원기둥을 kinematic으로 고정한다. 손마다 자기 proximal 접촉면 위 같은 높이에 둔다. 반지름은 12, 15, 18 mm이고, 위치는 O가 95, 105, 115°에서 두 pad로 닿는 곳이다.
- PIP 굽힘각을 60°에서 120°까지 0.5°씩 지령한다. 굽힘 강성과 감쇠는 세 손에서 같게 맞추고, 토크 포화는 끈다.
- 검증: O에서 distal pad의 첫 접촉각이 설계 예측과 같았다(0.5° 해상도). loop 오차는 0.18 mm 이하였다.

**결과** (조건 9개의 중앙값):

| | R − O | U − O |
|---|---|---|
| 첫 접촉각 (flat; identical surface-profile negative control) | 0.5° | 9.5° |
| 110°에서의 distal 힘 (flat) | 1.1 N | 14.8 N |
| 형상 효과 추정의 차이 (110°) | 0.033 N (O 효과의 14%) | 0.56 N (230%) |
| 형상 효과 추정의 차이 (120°) | 0.105 N (10%) | 0.275 N (25%) |

- 한계: O에서 형상 효과 자체가 작다(110°에서 0.24 N, 120°에서 1.1 N).

## 2차: pad friction, T_slip, 2026-11-21

- interface 변수: pad 마찰 μ = 0.4 대 0.8, 표면은 flat. 판정 가설(데이터 전에 기록): Δμ(H) = T_slip(H, 0.8) − T_slip(H, 0.4), E(H) = |Δμ(H) − Δμ(O)|, E_R < E_U.
- 코드: `sim/isaaclab/scripts/experimental/friction_slip.py`, `friction_analysis.py`, 축 원기둥 `code_isaaclab/assets/roller.py`. 결과: `sim/isaaclab/logs/experimental/friction/` (`summary_r15.json`).
- 설정: 원기둥 ρ = 15 mm를 축에 끼운다(회전만 자유, roller μ 0.01이라 pad의 μ가 정한다: MuJoCo-Warp는 큰 쪽을 쓴다). 위치는 1차와 같다(O가 q0 = 95/105/115°에서 두 pad로 닿는 곳, 각 손의 proximal 접촉면 위 같은 높이). 축 토크를 0.1 N·m/s로 양 방향 키운다. 환경 6개 = q0 3 × 방향 2.
  - 조건 A(같은 지령): PIP 굽힘 지령 q0 + 5°.
  - 조건 B(같은 힘): A의 지령에서 출발해, 원기둥의 총 접촉력이 O의 조건 A 값과 같아지도록 지령을 힘 되먹임으로 맞춘다.
- 축이 고정이라 proximal pad에는 힘이 걸리지 않는다(distal pad가 축을 미는 한쪽 쥠). 1차와 같은 배치다.

**Gate와 판정 기준 변경 (O에서만, R·U 데이터 전).**
- 모델의 shape μ가 0.4/0.8(pad), 0.01(roller)인 것을 확인했다.
- 처음 기준(원기둥 1° 회전)은 μ와 무관하게 T_slip ≈ 0.05 N·m를 냈다. 미끄러지기 전에도 손가락 순응으로 pad가 붙은 채 끌려가 회전이 토크에 비례해 생긴다(O에서 약 27°/N·m, 두 μ에서 같음, 기본 속도에서 약 0.05 rad/s). 그래서 기준을 원기둥 각속도 0.5 rad/s로 바꿨다(0.25, 1.0도 기록).
- 바꾼 뒤 유효 마찰 T_slip/(ρN_slip)은 모든 손, 방향, q0에서 0.32–0.36(μ 0.4)과 0.57–0.61(μ 0.8)이다. 공칭보다 낮은 것은 MuJoCo soft contact 때문으로 보이며 손과 무관하다.

**결과** (0.5 rad/s, 환경 6개의 중앙값; 괄호는 Δμ(O) 대비):

| | E_R | E_U | 측정값 차이(μ 0.4) R − O / U − O |
|---|---|---|---|
| A: 같은 지령 | 0.040 N·m (23%) | 0.215 N·m (121%) | −0.028 / +0.167 N·m |
| B: 같은 힘 | 0.023 N·m (10%) | 0.007 N·m (7%) | −0.012 / −0.010 N·m |

- 기준 민감도: A에서 E_R/E_U는 22%/57%(0.25 rad/s), 26%/104%(1.0 rad/s). B에서 13%/7%, 4%/5%.
- A: 같은 지령에서 수직력이 O 17–22 N, R 13–15 N, U 50–57 N이다. U는 접촉이 일찍 시작돼(측정 굽힘 오차 약 4° 대 O 1.6°) 원기둥을 훨씬 세게 민다. friction 효과는 수직력에 곱해지므로(T_slip ≈ μ_eff ρ N) E_U가 크다. R도 q0 115°에서 O보다 9 N 약하다.
- B: 힘을 맞추면 세 손에서 Δμ가 거의 같다. 즉, 이 task에서 friction 효과의 손 의존성은 거의 전부 수직력을 통해 들어온다. 남는 차이는 − 방향(마찰이 pad를 굽힘 쪽으로 끄는 쐐기 효과)에서 R +9%, U −9..−13%로, 부호가 반대이고 크기는 비슷하다.
- B의 무효 환경: R의 q0 115°는 지령이 굽힘 한계 120°에 걸려 힘을 맞추지 못했다(13 N 대 목표 22 N). 이 두 환경을 빼면 E_R 중앙값은 0.015 N·m, E_U는 0.015 N·m로 같다.

**해석.**
- 같은 지령으로 재면, coverage만 본 재설계(U)는 friction 효과를 O의 두 배 이상으로 잰다. RCT를 지킨 재설계(R)는 23%로 어긋난다. 가설 E_R < E_U는 조건 A에서 성립한다.
- 같은 힘으로 재면 두 재설계 모두 O와 비슷하게 잰다. 이것은 사용자가 지적한 순환성과 같은 이야기다: 효과가 수직력에 곱해지는 측정에서는 힘을 맞추면 형상 차이가 거의 사라진다.
- 따라서 이 실험이 보이는 것은 "RCT를 지키면 같은 지령에서 잡는 힘이 보존되어 측정이 맞는다"이다. 힘을 직접 맞출 수 있는 실험에서는 RCT가 덜 중요하다. RCT가 꼭 필요한지 보이려면, 힘을 맞춰도 남는 효과(접촉 위치나 Jacobian에 의존하는 측정)가 필요하다.
- 한계: 축 고정이라 한쪽 쥠이다. 반지름 15 mm 한 가지다. 조건 B에서 R의 한 q0는 무효다.
