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

## 2차 (계획): friction

- interface 변수: μ = 0.4 대 0.8. 실제 값은 O에서 예비 분석한 뒤 고정한다. 표면은 flat이다.
- task: 원기둥을 축에 끼워 회전만 가능하게 두고, 두 pad로 잡은 뒤 외부 torque를 점점 키운다. 주 지표는 T_slip이다.
- 판정: Δμ(H) = T_slip(H, 0.8) − T_slip(H, 0.4)일 때 E_R = |Δμ(H_R) − Δμ(H₀)|, E_U = |Δμ(H_U) − Δμ(H₀)|를 계산한다. 가설은 E_R < E_U이다.
- 구조: 축에 고정된 원기둥이 돌려면 두 접촉점이 모두 미끄러져야 한다. 그래서 T_slip = ρμ(N_p + N_d)이고, friction 효과는 수직력에 곱해진다. 따라서 E는 같은 지령에서의 수직력 차이를 따라간다.
- 대조: 굽힘각에 따라 비교한다(q90에서 40–60°는 음성 대조, 70–90°는 주 조건).
- Stage A: numpy와 scipy로 준정적 평형과 미끄러짐 한계를 계산한다.
- Stage B: MuJoCo-Warp에서 torque를 키워 가며 잰다. 그 전에 friction 설정이 실제로 반영되는지 확인해야 한다. 막대 실험에서 마찰을 1.0으로 바꿔도 결과가 같았기 때문이다.
