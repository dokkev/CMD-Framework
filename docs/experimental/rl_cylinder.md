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
