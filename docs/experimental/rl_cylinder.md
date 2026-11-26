# 원기둥 회전 과제 (fingertip 영역, 손바닥 접촉 금지) — 설계와 사전 등록 (2026-11-25)

목적: Study 3가 R이 보존한다고 본 손가락 끝 접촉 경로에 결과가 직접 의존하는 과제에서, O로 학습한 정책을 R/U로 옮긴다.
R > U가 나타난다면 실제 이유가 있는 곳이다.

## 과제 (`CoDe-Rotate-Cylinder-O-Fingertip`)

| 항목 | 설정 |
|---|---|
| 손 | O, 손바닥 위로 (palm up) |
| 물체 | 원기둥, 지름 35 mm, 길이 55 mm (`CylinderCfg`, 축 body y) |
| 물체 질량 | 큐브와 같은 밀도 400 kg/m³ → 약 21 g |
| 물체 놓이는 자세 | 손가락을 가로지르게 눕힘. proximal pad 위, distal pad 벽에서 2 mm 앞, thumb pad가 위에서 누름 |
| grasp 기하 | `pinch_grasp(size_m=0.035)` |
| goal | 자기 축 둘레 45° 단위로 이어지는 goal |
| progress | 매 step 회전 증분을 그 순간의 원기둥 축에 투영해 누적 |
| 성공 | \|오차\| + 축 기울기 < 0.3 rad (축이 손가락을 가로지르는 rest 방향에서 기운 각, 부호 무관), 위치 20 mm 이내, 0.2 s 유지 |

**접촉 규칙**
- 원기둥이 손바닥(palm body)에 닿으면 episode를 끝낸다 (접촉력 > 0.05 N).
- penalty가 아니라 종료로 한 이유: 규칙이 단 하나이고 분명하다. 별도 weight를 튜닝하지 않아도 된다. 끝난 episode는 이후 goal 보너스를 잃으므로 그것으로 충분히 비싸다.
- 손가락 위의 다른 접촉(proximal pad, proximal phalanx 포함)은 모두 허용한다. 사용자 정정을 따랐다.

**Pad**
- 둥근 dome을 쓴다: 길이 방향 crown 2.0 mm, 폭 방향 0.5 mm (`PadProfile(crown_mm=2.0, crown_wid_mm=0.5)`).
- 원기둥은 pad의 길이 방향으로 굴러가므로 그 방향 곡률을 크게 했다.

| pad | 길이 방향 arc 반경 | 3 mm pad 모서리 두께 | 길이 끝 두께 |
|---|---|---|---|
| proximal (44 mm) | 약 122 mm | 0.5 mm | 1.0 mm |
| distal (34.6 mm) | 약 76 mm | 0.5 mm | 1.0 mm |
| thumb (25 mm) | 약 40 mm | 0.5 mm | 1.0 mm |

- R/U로 옮길 때도 같은 profile을 각 손 자신의 pad 두께 위에 쓴다.

**Recipe (v2c와 같다)**
- OmniReset reset 네 type: 이 물체와 grasp로 다시 만든다.
  - grasp type의 회전 샘플링은 원기둥 자기 축 둘레로 한다.
  - 손바닥에 닿은 상태는 버린다.
  - 정책이 생기면 mid-turn 상태를 harvest해서 추가한다.
- goal 보너스 5, entropy 0.001, std 상한 1.0, actor history 5 + target 관측, 비대칭 critic.
- NaN 복구, 10× 속도 검사, 접촉 모델 v1, PIP servo collider.
- W&B group `omnireset_fingertip`.

**Eval** (`CoDe-Rotate-Cylinder-<hand>-Fingertip-Eval[-Mapped]`)
- 손별 held-out grasp는 `logs/rotate_cube/heldout_cylinder_v1/<hand>.pt`. OmniReset grasp 생성(`gen_omnireset_resets.py --type ObjectAnywhereHandGrasped`)으로 학습 dataset과 다른 seed, 1024개를 만든다. 수율을 보고한다.
- 지표는 task 축 회전이다 (`eval_policy.py`의 `task_*`):

| 구분 | 지표 |
|---|---|
| **주 지표** | full turn (자기 축 360°) + clean time-out 비율 |
| 보조 | goals/episode |
| 보조 | 회전 평균 |
| 보조 | 축 기울기가 20° 이내인 시간 |

- 큐브용 upright gate는 이 과제에 맞지 않아 쓰지 않는다.
- eval에서 `object_tilted` 종료는 끈다. 원기둥이 자기 축으로 돌면 reset 때의 up 축이 설계상 기운다.

## 이전 예측 (O 정책이 생긴 뒤, R/U 수치를 보기 전에 확정한다)
- **P-C1:** 같은 두께에서 R이 U보다 O의 주 지표를 더 많이 유지한다: ratio_R > ratio_U. 대상은 7 mm, 그리고 가능하면 5/9 mm.
- **P-C2:** 그 차이는 두께가 커질수록 커진다.
- **반증:** 7 mm에서 ratio_U ≥ ratio_R이고 그 차이가 두 seed 차이의 2배보다 크다. 또는 차이가 두께에 대해 단조롭지 않다 (seed 잡음을 넘는 역전).
- **mapped 조건:** gain_U > gain_R일 것으로 예측한다. 큐브와 같은 정의다.
- O 정책이 주 지표에서 10% 미만이면 이전을 판정하지 않는다. 그 경우 과제를 고쳐서 다시 등록한다.

## 실행 계획과 조기 점검 (2026-11-25, 학습 전 commit)

**실행 전 수정 두 가지** (commit 3eee555):
- `cube_goal_error`의 원기둥 모드에서 축을 가로지르는 두 방향을 잘못 잘라 썼다 (SVD 기저 `Vh[0, 1:]` → `Vh[1:]`). 이 때문에 env가 시작하자마자 멈췄다. 따라서 이 과제로 학습된 정책은 아직 없다.
- reset 생성 중에 손바닥 접촉 종료가 world를 reset하며 새 상태를 만들고 있었다. 생성 중에는 이 종료를 끄고, `on_palm` 검사가 걸러 낸다.

**학습:**
- O, seed 20, 4096 env.
- segment는 250 iteration이다. segment마다 held-out eval을 한다 (1024 env, seed 94000/95000).
- reset 생성은 `--max_rounds 120`으로 한다. grasp type의 수율이 smoke에서 약 4%였기 때문이다.

**조기 점검:**
1. **250 iteration 뒤:** 학습 goals/episode > 0.5, 또는 뚜렷한 상승 추세. 상승 추세의 기준: 마지막 50 iteration 평균이 처음 50 iteration 평균의 2배 이상이고, 0.1 이상.
2. **500 iteration 뒤:** O의 held-out 회전(원기둥 축, `task_progress_deg_mean`, 두 seed 평균) ≥ 180°.

- 어느 점검이든 실패하면 고칠 시도를 한 번만 한다. 후보는 mid-turn harvest를 추가하거나, goal 단위를 줄이는 것(예: 22.5°)이다. 고친 뒤 250 iteration 더 학습하고 다시 점검한다.
- 그래도 실패하면 과제를 중단하고 기록한다.

**이전:** O가 두 점검을 통과한 뒤에 한다.
- 대상: R/U 5/7/9 mm, unmapped. μ 교체와 이전 비율을 본다.
- 등록은 R/U 수치를 보기 전에 이 문서에 따로 commit한다. `rl_transfer.md`의 "앞으로의 구조"를 따른다: 주 지표는 μ 교체에 대한 회전 E이고, 이전 비율은 보조 지표다.

## 관측 변경: 원기둥 pose 대신 pad 접촉 감지 (2026-11-25 23:20, 학습 결과를 보기 전)

**무엇을 했나:** 사용자 요청으로 actor가 원기둥 pose를 모르게 하고, 대신 접촉을 감지하게 했다. pose를 보던 첫 학습(`cyl_seg1`, 22:44 시작)은 iteration 약 185에서 멈췄다. 판정하지 않았고, run 폴더는 `logs/rsl_rl/code_rotate_cylinder_fingertip/stopped/`로 옮겼다. 학습 중 로그의 goals/episode만 봤다.

reset dataset과 held-out grasp는 그대로 쓴다. actor 입력과 무관하기 때문이다.
- dataset acceptance: reaching 15.2%, grasp 5.1%, near-object 7.6%, near-goal 77%
- held-out grasp: 1024개, acceptance 19.4% (생성 마찰 1.3)

**새 과제** (`CoDe-Rotate-Cylinder-O-FingertipTouch`, eval `CoDe-Rotate-Cylinder-<H>-FingertipTouch-Eval`):
- **Actor 입력:** 직전 action, 관절 위치, 관절 target, pad 접촉(`mdp.pad_touch`). 모두 5-step history다. 차원은 (13 + 13 + 13 + 14) × 5 = 265.
  - 원기둥 pose와 goal 오차는 뺀다. 그래서 actor는 goal을 모르고, 시도 4처럼 "+축으로 계속 돌린다"를 배운다. 방향 flag는 넣지 않았다. 방향이 하나뿐이라 필요 없다.
- **pad 접촉 (7개 link):** index/middle/ring의 proximal과 distal, thumb distal. 원기둥을 각 link에 대해 filter한 contact sensor로 잰다.
  - link마다 두 값:
    - 접촉력 크기: 0–10 N을 0–1로 자르고 나눈다.
    - 접촉 flag: 0.05 N 초과.
  - 힘은 link에 걸린 net 접촉력이다 (normal과 마찰 포함). 접촉 위치(pad 길이 방향)는 이 sensor가 싸게 주지 않아 넣지 않았다.
- **센서 잡음:** 매 읽기마다 힘에 1 ± 10% 균등 gain을 곱하고, flag는 5% 확률로 뒤집는다. 저가 FSR/정전식 촉각 센서의 gain 오차와 간헐적 오검출 정도로 잡았다. eval에서도 켠다 (센서 모델의 일부다).
- **Critic:** privileged 그대로다 (77차원: 참 pose, goal 오차, 속도, 물성).
- **reward와 goal 계산:** 참 pose로 한다. 바꾸지 않았다.
- 학습 recipe와 W&B: 위와 같다 (goal 보너스 5, entropy 0.001, std 상한 1.0, 비대칭 critic, 처음부터 학습, seed 20, 4096 env). W&B group은 `omnireset_fingertip_touch`다.

**smoke:**
- 3 iteration 학습이 통과했다.
- held-out O에서 0-action 10 step 뒤 접촉 읽기 (32 env):
  - thumb: 항상 닿음, 약 6.8 N
  - 손가락 link: 19–41%의 env에서 닿음
- eval이 통과했다.

**조기 점검은 그대로 둔다:**
- 250 iteration 뒤: 학습 goals/episode > 0.5, 또는 뚜렷한 상승.
- 500 iteration 뒤: O held-out 회전 ≥ 180°.
- 고칠 시도는 한 번만 허용한다. 그래도 실패하면 중단한다.

## 결과: 접촉만 보는 정책 — 중단 (gate 2 실패, 2026-11-26)

| 점검 | 기준 | 결과 | 판정 |
|---|---|---|---|
| 250 iteration | 학습 goals/episode > 0.5 | 8.9 (225–250 구간 평균, 0.12에서 상승) | 통과 |
| 500 iteration | O held-out 회전 ≥ 180° | 32.7° / 38.5° (seed 94000 / 95000) | **실패** |

**O held-out 지표** (1024 env):

| 지표 | 250 iteration | 500 iteration |
|---|---|---|
| 원기둥 축 회전 평균 | 142° / 143° | 33° / 38° |
| 회전 중앙값 | 158° / 164° | 32° / 36° |
| goals/episode | 2.07 / 2.12 | 0.38 / 0.38 |
| clean time-out | 86% | 84% |
| full turn + clean | 2.6% / 1.9% | 0% / 0% |
| thumb·손가락 link 접촉 비율 | 0.54 | 0.37 |

**관찰:**
- 250 → 500 사이에 학습 goals/episode는 8.9에서 12.3으로 올랐다.
- 같은 기간 held-out 회전은 142°에서 35° 안팎으로 떨어졌다.
- 학습 분포(네 reset type, 마찰 0.3으로 만든 상태, randomization)와 held-out grasp(마찰 1.3으로 만든 잡기) 사이에서 일반화가 무너진 것으로 보인다. 원인은 확인하지 않았다.

**처리:**
- 사용자 규칙("가망 없는 시도는 빨리 멈춘다")에 따라 고칠 시도 없이 중단했다. 이전은 하지 않았다.
- 250 iteration checkpoint는 180°에 미치지 못했으므로 이전에 쓰지 않는다.

**run:**
- `logs/rsl_rl/code_rotate_cylinder_fingertip_touch/*_cyl_seg1`, `*_cyl_seg2`
- eval: `logs/rotate_cube/cylinder_run/eval_seg{1,2}_seed*.json`
