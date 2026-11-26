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
  - **수정 (2026-11-25 18:10, 어떤 R/U 수치도 보기 전):** 학습을 segment 3에서 끝낸다(segment 3 마지막 checkpoint를 이전에 쓴다). 이유는 시간이다(segment 4에 1.5 h). O 지표만 보고 정했다. segment 2에서 O 회전 642–669°, full≤30° 59–62%로 판정 조건(180°)을 넘었다.

**이전과 지표:**
- 손, cache, seed, eval 조건은 시도 1/1c와 같다 (`CoDe-Rotate-Cube-<hand>-OmniResetV2Blind-Eval`).
- 마찰 교체는 μ 1.3과 2.0 두 조건이다.
- **주 지표:** μ 1.3 → 2.0 교체에 대한 회전 E. 5, 7, 9 mm에서 계산한다.
  - **예측:** 각 두께에서 E(R) < E(U)이고, 그 차이가 seed 잡음 합보다 크다.
- **보조 지표:** 회전 이전 비율. 예측은 ratio_R > ratio_U다. full≤30°, full+clean (gate 없음), goals/episode의 비율도 같이 보고한다.

**반증:** 어느 두께에서든 E(R) ≥ E(U)이고, 그 차이가 잡음 합보다 크다. 차이가 잡음 안이면 "구별 안 됨"이다.

**판정 불가:** O에서 μ 1.3 평균 회전이 180° 미만이면 정책이 회전을 배우지 못한 것으로 보고, 판정하지 않는다.

### 결과 2: contact 기준 관절 사상 (mapped)

**사상 점검:**
- O에서는 identity다. mapped O와 unmapped O의 결과가 seed 잡음 안에서 같다:

| 지표 | unmapped O | mapped O |
|---|---|---|
| full≤30° | 67.1 / 63.0% | 67.9 / 63.1% |
| 회전 | 933 / 883° | 932 / 883° |

- R/U 사상의 최대 편차 (PIP / MCP FE)와 PIP 잔차:

| 손 | PIP 최대 편차 | PIP 잔차 중앙값 / 최대 | MCP FE 최대 편차 |
|---|---|---|---|
| R5 | 0.2° | 0.2 / 0.35 mm | 3.0° |
| U5 | 2.2° | 1.0 / 1.2 mm | 3.3° |
| R (7 mm) | 1.1° | 0.5 / 0.8 mm | 5.8° |
| U (7 mm) | 5.5° | 2.5 / 2.8 mm | 6.8° |
| R9 | 2.4° | 1.5 / 1.9 mm | 9.1° |
| U9 | 5.0° | 2.5 / 2.9 mm | 9.8° |

- MCP 사상은 −10° 하한 근처를 빼면 거의 일정한 offset이다. pad가 두꺼워진 만큼 면이 손바닥 쪽으로 밀려서, 굽힘을 줄이는 쪽으로 맞춘다. 남는 위치 잔차는 2.6–3.9 mm다.

**주 지표: mapped 조건에서 μ 교체에 대한 회전 E**

| 두께 | E(R) | E(U) | 잡음 합 | 판정 |
|---|---|---|---|---|
| 5 mm | 0.54 | 0.51 | 0.37 | 구별 안 됨 |
| 7 mm | 0.42 | 0.08 | 0.39 | 구별 안 됨 (U가 0.34 작다) |
| 9 mm | 0.31 | 0.02 | 0.28 | **U가 잡음 넘게 작다** → 예측 E(R) < E(U) 반증 |

**보조 지표: gain = 비율(mapped) − 비율(unmapped)**

| 두께 | full≤30° gain R / U | 회전 gain R / U | goals gain R / U |
|---|---|---|---|
| 5 mm | −0.02 / −0.03 | −0.05 / −0.09 | −0.05 / −0.09 |
| 7 mm | −0.05 / −0.13 | −0.09 / −0.16 | −0.09 / −0.16 |
| 9 mm | −0.16 / −0.12 | −0.15 / −0.20 | −0.15 / −0.21 |

- 모든 손, 모든 두께에서 gain이 0보다 작다. 사상은 오히려 성능을 떨어뜨렸다.
- 등록한 예측 "사상이 U를 R보다 더 돕는다"(gain_U > gain_R)는 맞지 않았다.
  - 7 mm 회전에서는 U가 더 많이 잃었다. 차이 0.07이 seed 잡음 합 0.056보다 크다. 반증이다.
  - full≤30°에서는 잡음 안이다.

**해석 (가설):** O에서 학습한 closed-loop 정책은 이미 물체 피드백으로 손 차이를 흡수하고 있다. 기구학적 사상은 그 보정을 오히려 흔든다. MCP offset 6–10°가 grasp 자세를 바꾸는 것이 원인 후보다.

## 시도 5: 긴 손가락 RL / UL로 zero-shot 이전 (사전 등록, RL/UL 수치를 보기 전에 commit, 2026-11-25)

**왜 하는가:** 5/7/9 mm 쌍에서는 R과 U의 접촉면 움직임 차이가 작아서 정책 수준에서 갈리지 않았을 수 있다. 손가락을 길게 하면 같은 pad 두께에서도 피복만 맞춘 재설계가 접촉면 움직임을 훨씬 크게 바꾼다. 그 차이가 정책 이전에서 보이는지 본다.

**손** (export는 `logs/rotate_cube/transfer/hands/`, 설계 파라미터는 `design/output/sim/study3/hands/`, commit 1ea240f, e1d8b65):

| key | export | L_f | pad | 설계 |
|---|---|---|---|---|
| O | `cfl_q120_h3to7_h0` | 90 mm | 3 mm | 기준, 바꾸지 않는다 |
| RL | `cfl_q120_L120_h3to7_hr` | 120 mm | 7 mm | O의 피복을 지키면서 contact Jacobian 변화(Δ_contact)를 최소로 |
| UL120 | `cfl_q120_L120_h3to7_hu` | 120 mm | 7 mm | 피복만 맞춤. **주 비교 상대** (commit 029e100, 9631ba4) |
| UL | `cfl_q120_L150_h3to7_hu` | 150 mm | 7 mm | 피복만 맞춤. 보조 비교 |

**수정 (f08a2ef 직후, RL/UL 수치를 보기 전):** RL(120 mm)과 UL(150 mm)은 길이가 달라 길이가 비교를 교란한다. 그래서 주 비교 쌍을 **RL vs UL120** (둘 다 L_f 120 mm, 7 mm pad)으로 바꾸고, UL(150 mm)은 보조 비교로 둔다. 아래 지표, 예측, 반증, 판정 불가 규칙에서 "UL"은 주 비교에서는 UL120을 뜻한다. 같은 규칙을 RL vs UL(150)에도 보조로 계산한다.

**공개 (수치를 보기 전에 알고 있는 것):**
- RL과 UL은 길이가 다르다 (120 / 150 mm). O(90 mm)와도 다르므로 큐브 grasp 기하가 많이 바뀐다.
- O 대비 접촉 경로 변화: RL 30.1 mm (길이 offset 제외 24.4 mm), UL 52.1 mm.
- RL의 Δ_J는 24 mm다. R7처럼 거의 0이 아니다. 이 길이에서는 피복을 지키면서 contact Jacobian을 유지할 수 없다. 따라서 RL은 "접촉을 지킨 손"이 아니라 "접촉 변화를 덜 만든 손"이다.
- RL은 굽힘 범위와 MA에서 경계에 있다: q_max 124° (120°보다 4° 큼), 최악 MA 0.37 (O 0.58), transmission angle 최저 32°, servo 기울기가 45° 한계에 닿아 있다.
- smoke에서 grasp acceptance는 RL 0.28, UL 0.44, UL120 0.34였다 (16 env, 참고용).
- O 대비 설계 수치 (Δ는 mm, R/U 수치를 보기 전에 받은 것):

| 손 | L_f | C_pad | Δ_contact | Δ_J | q_max | 최악 MA | γ |
|---|---|---|---|---|---|---|---|
| RL | 120.1 | 0.782 | 30.11 | 24.43 | 124° | 0.369 | 32–89° |
| UL120 | 120.0 | 0.786 | 30.03 | 25.95 | 180° | 0.615 | 66–115° |

- **주의:** 같은 길이에서 RL과 UL120의 Δ_J 차이는 1.5 mm뿐이다. UL120의 전달 특성(MA, γ)이 오히려 더 좋다. 따라서 RL vs UL120은 R/U(7 mm)만큼 접촉 보존을 대비시키지 못한다. 설계 쪽에서 더 낮은 피복 하한의 RL 후보를 검토 중이다. 주 비교 쌍을 바꾸게 되면 수치를 보기 전에 여기에 다시 등록한다. 그때까지 이 쌍의 결과는 등록한 규칙대로 판정하되, 위 주의와 함께 보고한다.
- 시도 1–2의 결과를 모두 본 뒤에 등록한다. 손은 그 뒤에 설계됐다.

**정책과 조건:**
- release policy `model_1497` (시도 1과 같다), fine-tuning 없음, unmapped (`CoDe-Rotate-Cube-<H>-OmniResetV2-Eval`).
- held-out grasp (UL120도 같다): `gen_grasps.py --hand RL|UL --target 1024 --num_envs 4096 --seed_start 76000`, `CODE_ROTATE_CUBE_CROWN_LAYOUT=dome`, `CODE_ROTATE_CUBE_CONTACT=pad13_condim4_v1` → `heldout_transfer_v1/RL.pt`, `UL.pt`. 수율을 보고한다.
- pad μ 1.3과 2.0, seed 94000과 95000, 1024 env. eval override는 시도 1과 같다.
- O의 수치는 시도 1b의 것을 그대로 쓴다 (같은 checkpoint, cache, seed, eval 조건).

**주 지표:** μ 1.3 → 2.0 교체에 대한 회전 E (시도 1c의 정의).

E(H) = |Δrot(H) − Δrot(O)| / |Δrot(O)|, Δrot(H) = rot_H(μ 2.0) − rot_H(μ 1.3), 두 seed 평균.

- **예측:** E(RL) < E(UL)이고, 그 차이가 seed 잡음 합보다 크다.
  - 잡음 합: RL과 UL 각각 Δrot의 seed 차이를 |Δrot(O)|로 나눈 값의 합.
- **반증:** E(RL) ≥ E(UL)이고, 그 차이가 잡음 합보다 크다.
- 차이가 잡음 합 이내면 "구별 안 됨"이다.
- **판정 불가:** RL 또는 UL의 grasp 수율이 256 미만이면 판정하지 않는다. μ 1.3에서 RL이나 UL의 평균 회전이 180° 미만이어도 (정책이 그 손에서 거의 돌리지 못함) E를 판정하지 않고 그대로 보고한다.

**보조 지표:**
- μ 1.3의 O 대비 이전 비율: 회전, full≤30°, full+clean (gate 없음), goals/episode. 예측은 ratio_RL > ratio_UL (seed 차이 합을 넘을 때만 판정).
- 같은 E를 goals, full≤30°, full+clean에 대해서도 계산한다.
- mapped 조건 (`-OmniResetV2-Eval-Mapped`, μ 1.3과 2.0): 같은 E와 gain을 보고한다. 예측은 두지 않는다 (탐색).

### 시도 5 보완: RL60 추가 (2026-11-25 14:57, RL·UL·UL120의 어떤 수치도 보기 전에 commit)

RL과 UL120은 같은 길이에서 Δ_J가 1.5 mm만 다르다. 이 쌍은 접촉 보존을 대비시키지 못한다. 그래서 피복 하한을 0.60으로 낮춘 접촉 보존 손 **RL60**(`cfl_q120_L120_h3to7_hr60`, `long_fingers.py --coverage-floor 0.60`)을 추가한다. 설계 수치(설계 쪽 보고, O 대비, mm)는 다음과 같다.

| 손 | L_f | C_pad | Δ_J | q_max | 최악 MA | γ | servo 기울기 |
|---|---|---|---|---|---|---|---|
| RL60 | ≈120 | 0.602 | 4.6 | 146° | 0.28 | 49–133° | 30° |
| UL120 | 120.0 | 0.786 | 25.95 | 180° | 0.615 | 66–115° | 35° |

- **접촉 보존 대비 쌍: RL60 vs UL120.** 지표, 예측, 반증, 판정 불가 규칙은 위와 같고 "RL"을 RL60으로 읽는다.
  - **예측:** E(RL60) < E(UL120), 그리고 ratio_RL60 > ratio_UL120 (보조 지표).
- 등록된 쌍 RL vs UL120도 그대로 판정해 보고한다. 두 쌍을 따로 판정하고, 하나만 골라 보고하지 않는다.
- **공개:**
  - RL60은 피복이 낮다(0.60, O 0.78). 짧은 pad는 정책이 쥐는 데 불리할 수 있으므로, 이전 비율에서는 RL60이 불리한 쪽으로 교란된다.
  - MA도 낮다(0.28).
  - 주 지표 E는 각 손의 μ 반응을 O와 비교하는 것이므로 이 교란에 덜 민감하다.
- held-out grasp와 평가 조건은 RL과 같다(`heldout_transfer_v1/RL60.pt`, seed 76000).

### 시도 5 중단 (2026-11-25 15:25, 사용자 결정)

사용자가 "긴 손가락에서 Δ_contact를 최소화하기 힘들면 그만두자"고 결정했다. 120 mm 이상에서는 시작 손끝이 O보다 (L_f − 90) mm 앞에 있다. 그래서 Δ_contact는 어떤 설계에서도 약 30 mm 아래로 내려가지 않는다(RL60도 30.08 mm). 시작점을 맞춘 Δ_J만 줄일 수 있다. 시도 5는 판정하지 않고 중단한다. RL60·UL120 평가와 μ 2.0 평가는 돌리지 않았다.

중단 시점에 이미 끝난 평가는 RL과 UL의 μ 1.3 평가 4개뿐이다. 판정 없이 그대로 남긴다(주 지표 E는 μ 2.0이 있어야 계산된다).

| 손 | seed | 회전 | full≤30° | full+clean | goals/ep |
|---|---|---|---|---|---|
| RL | 94000 | 53° | 0.0% | 0.8% | 0.62 |
| RL | 95000 | 51° | 0.1% | 0.5% | 0.59 |
| UL | 94000 | 17° | 0.0% | 0.0% | 0.02 |
| UL | 95000 | 17° | 0.0% | 0.0% | 0.01 |

hand 파일과 설계(RL, UL, UL120, RL60)는 기록으로 남긴다(commit 1ea240f, e1d8b65, 029e100, 9631ba4, f1f6062, c3862ab).

### 시도 4 보완: 이전 대상에 RL / UL 추가 (사전 등록, 같은 commit)

- 시도 4의 blind 정책(미리 정한 마지막 segment checkpoint)을 RL, UL120, UL에도 이전한다. 주 비교는 RL vs UL120, UL(150)은 보조다. `CoDe-Rotate-Cube-RL|UL-OmniResetV2Blind-Eval`, 시도 5와 같은 cache, μ, seed.
- 주 지표와 예측은 시도 4와 같다: 회전 E에서 E(RL) < E(UL)이고 차이가 잡음 합보다 크다.
- RL/UL 행은 따로 판정한다. 5/7/9 mm의 판정은 바꾸지 않는다.
- 시도 4의 판정 불가 조건(O의 μ 1.3 평균 회전 < 180°)과 시도 5의 수율 조건을 그대로 적용한다.
