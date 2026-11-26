# 시도 7: MCP를 잠그고 PIP만 쓰는 RL 과제 (사전 등록, R/U 수치를 보기 전, 2026-11-25)

## 왜 하는가
- 큐브 정책은 MCP AA/FE와 thumb를 쓴다. 두꺼운 pad는 R과 U 모두에서 MCP 축에 대한 면 위치를 바꾼다. 그래서 R이 PIP 열에서 지킨 접촉 경로의 이점이 가려진다 (`rl_transfer.md` 시도 1–6).
- Drake에서 효과가 분명했던 것은 MCP를 고정하고 PIP만 움직인 경우다 (Study 3, 손끝 원판 28z/29z).
- 그래서 MCP를 잠그고 PIP만 쓰게 한다. 다만 Drake를 되풀이하지 않도록, 열린 루프로 재사용한 명령으로는 할 수 없는 과제를 고른다. 접촉을 반복해서 만들고 끊는 순서(finger gaiting)가 필요한 과제다 (사용자 지시).

## 시도 7a': 손가락 릴레이 다이얼 (`CoDe-Finger-Relay-O`, `tasks/finger_relay/`)

### 손과 동작
- 손바닥이 위를 향한다.
- 고정하는 관절 (목표각 고정): 모든 MCP FE 0, AA 0, ring PIP 0. thumb는 ro/raa/mcp/ip 0으로, index 쪽에 납작하게 다이얼 아래로 놓인다.
- **action:** index와 middle의 PIP 굽힘 목표 두 개. [-1, 1]을 0–120°에 대응시키고, EMA α 0.5를 쓴다.
  - PIP 임피던스는 굽힘 공간에서 K 5, D 0.08 (큐브 과제와 같다).

### 다이얼
- 원판: 반지름 45 mm, 두께 4 mm, 30 g.
- **축:** middle 손가락의 proximal pad 면에 수직이다 (손바닥이 위이므로 세로).
  - 축은 축 방향으로 미끄러지고, 스프링이 받친다 (60 N/m, 감쇠 1, 이동 0–25 mm). Drake와 같은 축 스프링이다.
  - 그래서 굽히는 손끝이 원판의 손 쪽 면을 누르며 MCP 쪽으로 끌 수 있다.
- **회전 저항:** 비틀림 스프링 0.002 N·m/rad, 감쇠 0.001.
- **배치 (face frame, O에서 정하고 모든 손에 같은 좌표):**
  - 원판의 손 쪽 면 높이: O index 손끝(pad 끝)이 굽힘 30°일 때의 높이 (18.8 mm)
  - 중심: 그 손끝보다 10 mm 몸쪽 (x 69.6 mm), index 바깥쪽으로 12 mm (y −37.0 mm)
  - 각 손의 face frame은 그 손 자신의 pad 면에 있다. 그래서 Drake 손끝 원판처럼, 다이얼은 각 손의 pad 면에서 같은 위치에 놓인다.
- **두 손가락의 역할:**
  - index와 middle 손끝 모두 원판 위에 있다. ring은 닿지 않는다.
  - 둘 다 같은 방향(+)으로 돌린다. 지렛대는 index 12 mm, middle 37 mm다.
- **smoke 확인 (O):**
  - index만 0 → 90°로 굽히면 원판이 +73° 돈다 (접촉 1–1.6 N).
  - 펴면 끌어서 −23°까지 되돌린 뒤, 스프링이 0으로 되돌린다.
  - 따라서 계속 돌리려면 한 손가락이 떼고, 돌아와 다시 잡는 동안 다른 손가락이 잡고 있어야 한다.
- 다이얼 재질은 큐브와 같다 (μ 0.5). pad는 1.3이고 MuJoCo는 큰 쪽을 쓴다. pad는 dome 1×1이고, 접촉 모델은 `pad13_condim4_v1`이다.

### goal과 reward
- 45°가 아니라 30° 단위로 이어지는 goal이다.
- 성공: |θ − goal| < 8°를 3 step 유지.
- reward:
  - 다이얼 전진 Δθ: step마다 ±0.2 rad로 자르고, 1 rad당 10
  - goal 보너스: 2
  - action rate: −0.01
- episode는 10 s다. 시작 자세: index와 middle을 0–25° 균등(1° 격자)으로 놓아 닿지 않은 채 시작한다. 다이얼은 정지 상태다.

### 관측
- **closed loop (`CoDe-Finger-Relay-O`):** 직전 action (2), PIP 굽힘 (2), 다이얼 (sin θ, cos θ, goal 오차, 각속도; 4), distal pad 접촉 (index, middle의 힘과 flag; 4). 5-step history로 60차원이다.
- **접촉만 (`CoDe-Finger-Relay-O-Touch`):** 다이얼 관측을 뺀다 (40차원).
- 접촉 감지: 힘 0–5 N, ±10% gain 잡음, flag 5% 뒤집힘 (원기둥 과제와 같은 센서 모델).
- critic: 잡음 없는 같은 항목 + slider 이동 + goal 오차 (14차원).

### 학습
- PPO: entropy 0.001, std 상한 1.0, actor/critic [256, 128, 64], lr 3e-4 adaptive, γ 0.99. O, seed 20, 4096 env.
- segment는 200 iteration이다. segment마다 O eval (1024 env, seed 94000/95000, μ 1.3)을 한다.
- **정체 규칙:** 평균 회전의 상대 증가가 앞 segment 대비 5% 미만이면 멈춘다. 최대 5 segment (1000 iteration).
- 이전에 쓰는 checkpoint는 멈춘 segment의 마지막 것이다.

### 조기 gate
- O의 평균 다이얼 회전(두 seed 평균) ≥ 180°. 목표 360°/episode의 50%다.
- 못 미치면 변형을 중단하고 기록한다.
- touch-only 변형은 closed loop가 gate를 통과할 때만 한다.

### 이전과 지표
- 대상: O, R, U (7 mm)를 먼저, pad μ 1.3과 2.0, eval seed 94000/95000, 1024 env (`scripts/finger_relay/eval_relay.py`).
- eval seed는 시작 굽힘과 센서 잡음을 바꾼다.
- **주 지표:** μ 1.3 → 2.0 교체에 대한 회전 E (Study 3, `rl_transfer.md` 시도 1c의 정의).
  - E(H) = |Δrot(H) − Δrot(O)| / |Δrot(O)|, Δrot = rot(μ 2.0) − rot(μ 1.3), 두 seed 평균.
  - 예측: E(R) < E(U)이고, 그 차이가 잡음 합보다 크다. 잡음 합은 R과 U 각각 Δrot의 seed 차이를 |Δrot(O)|로 나눈 값의 합이다.
  - 반증: E(R) ≥ E(U)이고, 그 차이가 잡음 합보다 크다.
  - 판정 불가: |Δrot(O)|가 O의 Δrot seed 차이의 2배보다 작다.
- **보조 지표:** μ 1.3에서 O 대비 회전 비율. 예측은 ratio_R > ratio_U이고, 차이가 R과 U의 seed 차이 합보다 커야 한다.
  - goals/episode, 180° 넘긴 비율, 접촉 make 횟수(gaiting)도 보고한다.
- **7 mm 정지 규칙:** R과 U가 주 지표와 보조 지표 모두에서 잡음 안이면 기록하고 이 변형을 멈춘다. 아니면 5/9 mm (R5/U5, R9/U9)를 같은 방식으로 한다.

### 공개
- 시도 1–6의 결과(R/U가 갈리지 않음)를 본 뒤에 설계했다.
- 배치와 스프링 값은 O에서만 smoke로 확인했다. R/U에서 이 과제를 돌린 수치는 없다. 다만 코드 점검으로, 학습되지 않은 2-iteration checkpoint를 R(16 env)과 U(16 env)에서 한 번씩 돌렸다 (회전 약 4–5°).
- 학습 seed가 하나라 잡음은 eval seed 차이로만 잰다.

## 시도 7b': 원기둥을 손바닥 위로 걸어 옮기기
설계와 등록은 7a'가 학습되는 동안 이 문서에 따로 commit한다.
