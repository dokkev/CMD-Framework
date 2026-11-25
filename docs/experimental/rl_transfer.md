# 학습된 회전 정책의 zero-shot 이전: O → R, U (사전 등록, 2026-11-25)

Study 4가 원래 하려던 일이다. 가설은 하나다. 접촉면 움직임(contact Jacobian)을 지킨 재설계 R은 O에서 학습한 정책을 피복만 맞춘 재설계 U보다 더 잘 물려받고, 두께가 커질수록 그 차이가 커진다.

**이 문서는 R/U 수치를 하나도 보기 전에 commit한다.** 결과는 맨 아래 "결과"에 덧붙인다. 위의 등록 내용은 결과를 본 뒤 바꾸지 않는다.

## 정책
- `CoDe-Rotate-Cube-O-OmniResetV2` release policy: v2c segment 2 `model_1497` (`logs/rsl_rl/code_rotate_cube_omnireset_v2/2026-11-25_07-41-53_rotate_cube_O_OmniResetV2_s20_v2c_seg2/model_1497.pt`).
- O, 3 mm dome pad, 45° goal에서 학습했다. 설정은 `omnireset_recipe.md` §6.2.
- fine-tuning도, 손별 조정도 하지 않는다. evaluator 안에 적응 단계도 없다 (SSoT).

## 손
| key | export | pad 두께 | 비고 |
|---|---|---|---|
| O | `cfl_q120_h3to7_h0` | 3 mm | 기준. Study 3 Isaac run과 같은 export (`scripts/study3_measurement/common.py` `HANDS`) |
| R / U | `cfl_q120_h3to7_hr` / `_hu` | 7 mm | Study 3의 7 mm 쌍. 같은 export |
| R5 / U5 | `cfl_q120_h3to5_hr` / `_hu` | 5 mm | 지금 exporter로 다시 export했다 (`scripts/rotate_cube/export_transfer_hands.sh`) |
| R9 / U9 | `cfl_q120_h3to9_hr` / `_hu` | 9 mm | 위와 같다 |

5/9 mm 쌍을 다시 export한 이유:
- commit된 export에는 servo 모양과 visual 정보가 없다. 그래서 PIP servo collider를 만들 수 없다.
- 다시 export해도 관절, loop, four-bar table, pad, 충돌 쌍은 원본과 같다. 바뀐 것은 palm housing과 servo/visual 항목뿐이다.
- 7 mm 쌍은 지금 exporter로 다시 export하면 source hash 말고는 원본과 같다.

## 자산과 물리 (모든 손 동일)
- 접촉 모델: `pad13_condim4_v1`.
- 각 손의 pad 두께 위에 dome profile을 씌운다 (길이 방향 1 mm, 폭 방향 1 mm).
- thumb pad, PIP servo collider, 40 mm 큐브.
- eval 조건은 `CoDe-Rotate-Cube-<hand>-OmniResetV2-Eval`이다. O와 같다: 20 s, randomization 없음, rotate_cube 종료 조건, eval override는 `max_distance` 0.07, `min_dot` 0.0.

## 정책 입력
- 관찰과 action 공간은 바꾸지 않는다.
- 모든 손의 관절 이름 19개와 순서가 같다 (export에서 확인했다).
- 구동 관절 13개, action 13차원, actor 입력 255, critic 77로 같다.
- 관절 한계는 PIP 4절 링크 관절(pip_servo, a, b)만 다르다. 정책은 PIP를 flexion(0–120°)으로 보고 명령하므로 입력의 의미는 같다.
- 손마다 다른 것은 grasp 기하뿐이다: 쉬는 위치, thumb의 MCP/IP. 관찰에 쓰는 큐브 위치는 각 손의 쉬는 위치를 기준으로 잰다.
- 실행할 때 각 손에서 관찰 차원을 다시 확인한다.

## 손별 held-out grasp
- `heldout_omnireset_v1`과 같은 절차와 seed로 손마다 만든다:
  - `gen_grasps.py --hand <key> --target 1024 --num_envs 4096 --seed_start 76000` (noise 0.25 rad, hold 3 s)
  - `CODE_ROTATE_CUBE_CROWN_LAYOUT=dome`, `CODE_ROTATE_CUBE_CONTACT=pad13_condim4_v1`
- 저장 위치는 `logs/rotate_cube/heldout_transfer_v1/<key>.pt`다. O는 `heldout_omnireset_v1/O.pt`를 그대로 복사한다.
- **수율(acceptance)도 결과다.** O에서 성립하는 grasp가 R/U에는 없을 수 있다. 손마다 몇 개가 나오는지 보고한다. 같은 seed의 round 수가 모자라면 1024개를 채우지 못한 그대로 보고한다.

## 지표
`eval_policy.py`로 잰다. seed 94000과 95000, 1024 env.

| 구분 | 지표 |
|---|---|
| **주 지표** | upright ≤30° gate에서 full turn 비율 (40 mm, clean time-out) |
| 보조 | 회전 평균 |
| 보조 | upright ≤45° full turn 비율 |
| 보조 | goals/episode |
| 보조 | clean time-out |
| 보조 | 20° 안 시간 |

- 각 손의 값과 O 대비 비율을 보고한다.
- 두 seed의 차이를 잡음의 기준으로 쓴다.

## 예측 (Study 3에서)
- **P1:** 7 mm에서 R이 U보다 O의 성능을 더 많이 유지한다. 즉 ratio_R > ratio_U (주 지표, 두 seed 평균).
- **P2:** 그 차이(ratio_R − ratio_U)는 5 → 7 → 9 mm로 갈수록 커진다.

**반증 조건:**
- P1 반증: 7 mm에서 ratio_U ≥ ratio_R이고, 그 차이가 두 seed 차이의 2배보다 크다.
- P2 반증: 차이가 두께에 따라 단조롭게 커지지 않는다 (seed 잡음을 넘는 역전이 있다).
- 판정 불가: 어느 손에서든 grasp 수율이 낮아(< 256) eval이 불가능하면 그 두께는 판정하지 않는다. 회전 차이가 seed 잡음 안이면 "구별 안 됨"으로 쓴다.

## 2차: 인터페이스 교체 (시간이 되면)
- 손마다 pad μ를 1.3에서 2.0으로 바꾼다 (`CODE_CONTACT_PAD_MU=2.0`). 정책은 그대로다.
- 회전과 주 지표의 변화를 O의 변화와 비교한다. Study 3의 "인터페이스 교체 효과" 논리를 정책에 적용한 것이다.
- 사전 예측은 두지 않는다 (탐색).

## 영상
손마다 W&B group `omnireset_transfer`에 짧은 play 영상을 올린다.

## 결과
(아래에 덧붙인다.)
