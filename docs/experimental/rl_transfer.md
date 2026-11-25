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

## 추가 사전 등록 (2026-11-25, 해당 수치를 보기 전에 commit)

이 문서 전체에 적용하는 원칙:
- 모든 시도는 실패까지 포함해 보고한다.
- 시도마다 주 지표를 미리 정한다.
- 성공 정의 두 가지를 항상 같이 보고한다:
  - 기울기 gate 없이 full turn + clean time-out (`full_turn_and_clean_timeout_rate`)
  - goals/episode

비교는 언제나 같은 두께에서 O로 학습한 정책을 R과 U에 zero-shot으로 옮긴 결과다.

### 시도 1b: 인터페이스 교체 (pad μ 1.3 → 2.0)
μ 2.0 결과는 driver가 이미 계산했지만 열어 보지 않았다.

**지표:** Study 3와 같은 E. 작을수록 좋다.

E(H) = |Δ(H) − Δ(O)| / |Δ(O)|, Δ(H) = perf_H(μ 2.0) − perf_H(μ 1.3)

- perf: 주 지표 (upright ≤30° full turn, 두 seed 평균)
- 보조로 같은 E를 회전 평균에도 계산한다.

**예측:** 각 두께에서 E(R) < E(U).

**반증:** 7 mm에서 E(R) ≥ E(U)이고, 그 차이가 잡음보다 크다.
- 잡음: 각 손 Δ의 seed 차이를 |Δ(O)|로 나눈 값을 R과 U에 대해 더한 것

**판정 불가:** |Δ(O)|가 O의 Δ seed 차이의 2배보다 작으면, 그 지표에서는 E를 판정하지 않는다.

### 시도 1c: 인터페이스 교체, 5 mm와 9 mm (확증 시험)

이 등록은 5/9 mm의 μ 2.0 수치를 열어 보기 전에 commit한다.

**주 지표:** 평균 회전에 대한 E (각 두께에서 따로 계산).

E = |Δrot(H) − Δrot(O)| / |Δrot(O)|

Study 3의 정의를 그대로 쓴 것이다. 재설계 손이 측정한 인터페이스 효과를 O의 효과와 비교한다.

**예측:** 5 mm와 9 mm 각각에서 E(R) < E(U)이고, 그 차이가 seed 잡음 합보다 크다.
- seed 잡음 합: 각 손 Δ의 seed 차이를 |Δrot(O)|로 나눈 값을 R과 U에 대해 더한 것

**반증:** 어느 두께에서든 E(R) ≥ E(U)이고, 그 차이가 잡음 합보다 크다.
- 차이가 잡음 합 이내면 그 두께는 "구별 안 됨"으로 보고한다.

**보조 지표:** 같은 E를 goals/episode, full≤30°, full+clean (gate 없음)에 대해서도 계산한다.

**공개:** 지표 선택은 시도 1b의 7 mm 결과를 보고 정했다 (회전 E에서 R 0.03, U 0.27). 따라서 7 mm는 확증이 될 수 없다. 확증 시험은 5 mm와 9 mm뿐이다.

**앞으로의 구조** (시도 2 mapped와 원기둥 과제에도 적용한다):
- 주 질문은 "이 손이 O처럼 인터페이스 효과를 측정하는가"다. 주 지표는 μ 교체에 대한 회전 E다.
- 이전 비율(ratio)은 보조 지표로 둔다.

### 시도 2: contact 기준 관절 사상 ("mapped")
**사상** (`tasks/rotate_cube_omnireset/joint_transfer.py`):

| 관절 | 사상 |
|---|---|
| PIP | 실험 27의 contact setpoint. distal 면의 손끝 아래 점을 proximal 면 frame에서 맞춘다 |
| MCP FE | proximal pad 면 위 세 점의 최소제곱 |
| AA, thumb | identity. 축, anchor, thumb pad가 모든 손에서 같다 (확인함) |

- 정책 action은 O 관절각으로 해석해 사상하고, 관측은 역사상해서 정책에 O 좌표로 보인다.
- 정책, cache, seed, 지표는 unmapped와 같다.

**사전 점검** (손별 최대 편차와 잔차; 수치는 아래 결과에):
- O에서는 사상이 identity다.
- PIP 사상은 Drake 실험 27의 7 mm R/U setpoint와 0.05° 안에서 일치한다.

**주 지표:** upright ≤30° full turn (O 대비 비율).

**예측:** 사상이 U를 R보다 더 돕는다: gain_U > gain_R. gain_H = ratio_H(mapped) − ratio_H(unmapped).

**반증:** 7 mm에서 gain_U ≤ gain_R이고, 그 차이가 R·U의 seed 차이 합보다 크다.

(원래 예측 "unmapped에서 R ≥ U"는 7 mm에서 이미 반증됐다. 결과 1 참고.)

### 시도 2 보완 (1c와 같은 구조, mapped 수치를 보기 전에 commit)

- **주 지표:** mapped 조건에서 μ 1.3 → 2.0 교체에 대한 회전 E.
  - 대상: 5, 7, 9 mm. 그러려면 mapped μ 2.0 eval이 필요하다.
  - 예측: 각 두께에서 E(R) < E(U)이고, 그 차이가 seed 잡음 합보다 크다.
- **보조 지표:** 앞서 등록한 gain (mapped − unmapped 비율), full≤30°, full+clean, goals/episode.

## 결과
(아래에 덧붙인다.)

### 결과 1: unmapped 이전, μ 1.3 (P1, P2)

**grasp 수율:** 모든 손에서 1024/4096이다 (acceptance 0.361–0.388). 모든 손에 유효한 grasp가 있다.

| 두께 | R full≤30° | U full≤30° | 차이 R−U | seed 차이 R / U | R 회전 | U 회전 | R goals | U goals | full+clean R / U |
|---|---|---|---|---|---|---|---|---|---|
| 5 mm | 0.77 | 0.82 | −0.04 | 0.026 / 0.027 | 0.81 | 0.84 | 0.81 | 0.83 | 69.0 / 72.3% |
| 7 mm | 0.70 | 0.80 | −0.10 | 0.005 / 0.033 | 0.73 | 0.70 | 0.72 | 0.69 | 64.4 / 69.1% |
| 9 mm | 0.74 | 0.71 | +0.02 | 0.068 / 0.045 | 0.65 | 0.62 | 0.63 | 0.60 | 67.9 / 66.4% |

- full≤30°, 회전, goals 열은 O 대비 비율이다. full+clean은 절대값이다.
- O 자체: full≤30° 67.1 / 63.0%, 회전 933 / 883°, full+clean 79.7 / 75.1%.

**판정:**
- P1 반증. 7 mm 차이 −0.10이 seed 차이의 2배(0.066)보다 크다.
- P2 반증. 차이가 단조롭게 커지지 않는다.
- 보조 지표를 보면, 회전과 goals에서는 7/9 mm에서 R이 조금 앞서고 5 mm에서는 U가 앞선다. 기울기 관련 지표(gate, 20° 안 시간)는 U가 앞선다. 7 mm에서 기울기 정점 중앙값은 R 28.2°, U 25.6°다.

### 결과 1b: μ 교체, 7 mm

E (작을수록 좋다):

| 지표 | Δ(O) | E(R) | E(U) | 잡음 합 | 판정 |
|---|---|---|---|---|---|
| full≤30° (주 지표) | −7.5 pt | 0.54 | 0.37 | 0.18 | 구별 안 됨 |
| 회전 | −131° | 0.03 | 0.27 | 0.12 | R이 더 작다 |
| goals | −2.93 | 0.05 | 0.28 | 0.14 | R이 더 작다 |
| full+clean | −5.0 pt | 1.49 | 1.04 | 0.55 | 구별 안 됨 |

### 결과 1c: μ 교체, 5 mm와 9 mm (확증 시험, 주 지표는 회전 E)

| 두께 | Δrot(O) | Δrot(R) | Δrot(U) | E(R) | E(U) | 잡음 합 | 판정 |
|---|---|---|---|---|---|---|---|
| 5 mm | −131° | −163° | −155° | 0.244 | 0.184 | 0.263 | **구별 안 됨** (R이 0.06 더 큰데, 잡음 안) |
| 9 mm | −131° | −118° | −126° | 0.097 | 0.035 | 0.238 | **구별 안 됨** (R이 0.06 더 큰데, 잡음 안) |

**예측 E(R) < E(U)는 확증되지 않았다.** 두 두께 모두 차이가 잡음 안이고, 수치는 오히려 U가 작다. 등록한 반증 기준(차이가 잡음보다 커야 함)에는 미치지 않는다.

**보조 지표 E (R / U):**

| 두께 | goals | full≤30° | full+clean |
|---|---|---|---|
| 5 mm | 0.24 / 0.17 | 0.71 / 0.44 | 1.18 / 1.23 |
| 9 mm | 0.10 / 0.03 | 0.78 / 0.46 | 2.01 / 1.84 |

**정리:**
- 7 mm에서 회전 E가 R에 유리했던 것(1b)은 확증 두께인 5/9 mm에서 재현되지 않았다.
- 회전 E 자체가 모든 손에서 0.03–0.27로 작다. 곧 정책이 마찰 교체에 반응하는 방향과 크기가 손과 관계없이 비슷하다.

**영상** (W&B group `omnireset_transfer`):

| 손 | run |
|---|---|
| O | `jjduefpf` |
| R | `hhw0k00z` |
| U | `e357yw1q` |
| R5 | `lu0hubzx` |
| U5 | `ax773z1n` |
| R9 | `dpsfktpq` |
| U9 | `y6myezl8` |

## 시도 4: 물체를 보지 않는 정책 (사전 등록, 학습 전 commit, 2026-11-25)

**왜 하는가:** 지금까지 정책은 큐브 pose와 goal 오차를 본다. 그래서 손마다 접촉면이 다르게 움직여도 물체 피드백으로 보정할 수 있고, 이것이 R/U를 가르지 못한 이유일 수 있다. 피드백이 없으면 손이 만들어 내는 접촉면 움직임이 결과를 더 직접 정한다. Drake에서 같은 open-loop 명령을 다시 쓴 실험과 같은 논리다.

**정책** (`CoDe-Rotate-Cube-O-OmniResetV2Blind`, 변형 (a)):
- **Actor 입력:** 직전 action, 관절 위치, 관절 target만 보고, 5-step history를 쓴다 (39 × 5 = 195). 큐브 pose와 goal 오차는 빼서 HORA의 고유감각 정책과 같은 형태다.
- **Critic:** privileged 그대로다 (77).
- **Goal과 reward:**
  - env는 참 pose로 goal(45° 단위)을 센다. reward도 v2c와 같다 (goal 보너스 5).
  - actor는 goal을 모르므로 실제로는 "계속 +z로 돌린다"를 배운다.
- **학습:**
  - actor 입력 차원이 달라 release policy에서 warm start할 수 없다. 그래서 O에서 처음부터 학습한다.
  - recipe는 v2c와 같다: entropy 0.001, std 상한 1.0, 비대칭 critic, 네 OmniReset type + mid-turn reset, NaN 복구, 10× 속도 검사.
  - seed 20, 4096 env, segment는 500 iteration이다. 최대 4 segment이고, O의 held-out 지표가 정체하면 멈춘다.
- **이전에 쓸 checkpoint:** 마지막 segment의 것을 쓴다. 미리 정해 두어서, 결과를 보고 고르지 않는다.

**이전과 지표:**
- 손, cache, seed, eval 조건은 시도 1/1c와 같다 (`CoDe-Rotate-Cube-<hand>-OmniResetV2Blind-Eval`).
- 마찰 교체는 μ 1.3과 2.0 두 조건이다.
- **주 지표:** μ 1.3 → 2.0 교체에 대한 회전 E. 5, 7, 9 mm에서 계산한다.
  - **예측:** 각 두께에서 E(R) < E(U)이고, 그 차이가 seed 잡음 합보다 크다.
- **보조 지표:** 회전 이전 비율. 예측은 ratio_R > ratio_U다. full≤30°, full+clean (gate 없음), goals/episode의 비율도 같이 보고한다.

**반증:** 어느 두께에서든 E(R) ≥ E(U)이고, 그 차이가 잡음 합보다 크다. 차이가 잡음 안이면 "구별 안 됨"이다.

**판정 불가:** O에서 μ 1.3 평균 회전이 180° 미만이면 정책이 회전을 배우지 못한 것으로 보고, 판정하지 않는다.
