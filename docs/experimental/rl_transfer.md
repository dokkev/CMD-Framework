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

## 시도 6: fine-tuning 비용 (사전 등록, fine-tuning 수치를 보기 전에 commit, 2026-11-25)

**왜 하는가:** 시도 1–5의 zero-shot 이전은 R과 U를 가르지 못했다. closed-loop 정책은 두 손에서 모두 15–35%를 잃었고(결과 1), 물체를 보지 않는 정책은 두 손에서 모두 무너졌다(시도 4). 그래서 질문을 "얼마나 물려받나"에서 **"O의 성능을 되찾는 데 학습이 얼마나 필요한가"**로 바꾼다. 접촉면 움직임을 지킨 R이 피복만 맞춘 U보다 적은 적응으로 회복한다면, 접촉 적응의 부담 일부가 제어에서 기구 설계로 옮겨 갔다는 뜻이다.

**손:**
- O: 대조군. 같은 예산으로 O 자신에서 이어서 학습한다.
- R, U: 7 mm 쌍 (주 비교).
- R5/U5, R9/U9: 7 mm가 끝난 뒤 시간이 되면 같은 절차로 한다. 각 두께를 따로 판정한다. 돌리지 못하면 "하지 않음"으로 적는다.

**시작점과 recipe:**
- 모든 손에서 release policy `model_1497`의 가중치로 시작한다 (actor, critic, optimizer 상태, 관측 normaliser 모두. v2c segment 사이의 resume과 같다).
- 단 하나 바꾸는 것: checkpoint의 iteration 카운터를 1497에서 1로 바꾼 사본(`logs/rotate_cube/finetune_run/start_model_1497_iter1.pt`)에서 시작한다. 그래야 `model_k`가 정확히 k번의 fine-tuning update 뒤의 가중치가 된다. rsl_rl에서 이 카운터는 checkpoint 이름과 로그에만 쓰인다.
- recipe는 v2c와 같다:
  - `CODE_OMNIRESET_ABNORMAL_FACTOR=10`, `CODE_OMNIRESET_GOAL_BONUS=5`
  - `agent.algorithm.entropy_coef=0.001`, `agent.actor.distribution_cfg.std_range=[0.001,1.0]`
  - NaN 복구, 다섯 reset type (0.25/0.25/0.25/0.125/0.125), 45° goal
- 손과 그 손의 reset dataset만 바뀐다. 학습 task는 `CoDe-Rotate-Cube-<H>-OmniResetV2`다 (R/U용 학습 task id를 O와 같은 방식으로 새로 등록했다).
- 학습 seed 20과 21, 손마다 둘.
- **예산:** run마다 300 iteration, 4096 env. 25 iteration마다 checkpoint (`agent.save_interval=25`).
- W&B group `omnireset_finetune`, experiment `code_rotate_cube_omnireset_v2_finetune`.
- 순서: seed 20의 O, R, U, 그다음 seed 21의 O, R, U.

**손별 reset dataset (R, U):**
- 네 type은 O와 같은 절차와 크기로 그 손에서 만든다: `gen_omnireset_resets.py --task CoDe-Rotate-Cube-<H>-OmniResetV2 --type all --count 10000 --num_envs 4096 --friction 0.3 --seed_start 0`.
  - O는 v1 task로 만들었다. v2 task로 만들어도 생성 과정은 같다. 다른 것은 저장된 goal offset뿐인데, v2 학습은 이 네 type의 goal을 reset 때 다시 정한다(`goal_offsets`).
- mid-turn 상태는 `model_1497`을 그 손에서 rollout해서 모은다 (`harvest_omnireset_midturn.py --four_types`, 1024 env, seed 11, turn 6000 / limit 4000, 15 step 간격). rollout은 그 손의 네 type reset에서 0.25씩 시작한다. 그다음 `gen_omnireset_resets.py --type ObjectMidTurnHandWound --hold_s 0.5`로 가공한다 (O와 같다).
- O는 release가 학습한 dataset(`O_40f4fc6626`)을 그대로 쓴다. O의 mid-turn 상태는 v1 정책에서 모았고, R/U의 것은 `model_1497`에서 모았다. 정책이 그 손에서 실제로 지나가는 상태를 쓰는 것이 공정한 손별 dataset이라고 보고 이렇게 정했다. 수율과 개수를 결과에 적는다.

**주 지표: 회복 iteration**
- 정의: held-out 평균 회전(μ 1.3)이 O release 값의 90%에 처음 닿는 checkpoint의 iteration.
  - 기준값: O release의 1024-env held-out 평균, seed 94000 값 933.2°. 따라서 문턱은 840°다. (eval seed가 94000이므로 그 seed의 값을 쓴다.)
- checkpoint 평가: 학습이 끝난 뒤, 그 손의 held-out cache(`heldout_transfer_v1/<H>.pt`), 256 env, eval seed 94000. iteration 25, 50, …, 300.
- iteration 0은 `model_1497`을 같은 조건(256 env, seed 94000)으로 그 손에서 잰 값이다. 두 학습 seed가 공유한다.
- 회복 곡선: 회전 비율(회전 / 933.2°) 대 iteration.
- **예측:** R이 U보다 적은 iteration에서 회복한다. 두 seed 평균 회복 iteration의 차이 (U − R)가 seed 잡음보다 크다.
  - seed 잡음 = |R_s20 − R_s21| + |U_s20 − U_s21| (이 문서의 다른 시도와 같은 "잡음 합" 규칙).
- **반증:** U의 평균 회복 iteration이 R보다 작고, 그 차이가 잡음보다 크다.
- 차이가 잡음 안이면 "구별 안 됨"이다.
- **회복하지 못하는 경우:** 어떤 run이 300 iteration 안에 90%에 닿지 못하면 그 run의 최고 비율을 보고한다. 이때 (R 또는 U의 run 중 하나라도 회복하지 못하면) 판정은 미리 정한 tie-breaker로 한다:
  - 회복 곡선 아래 넓이 AUC: iteration 0–300의 회전 비율을 사다리꼴로 적분하고 300으로 나눈다.
  - 예측 AUC_R > AUC_U, 차이가 같은 잡음 합 규칙(두 손 각각의 seed 차이 합)보다 커야 한다. 반증과 "구별 안 됨"도 같은 규칙이다.
- AUC는 어느 경우에나 보조로 함께 보고한다.

**보조 지표:**
- iteration 300의 held-out 비율 (1024 env, seed 94000/95000, μ 1.3): 회전, full≤30°, full+clean (gate 없음), goals/episode. O release 대비 비율(seed별 값으로)로 보고하고, 같은 seed로 fine-tuning한 O 대비 비율도 같이 적는다.
- μ 1.3 → 2.0 교체의 회전 E (시도 1c 정의), iteration 300 정책:
  - E(H) = |Δrot(H) − Δrot(O_ft)| / |Δrot(O_ft)|. O_ft는 같은 학습 seed로 fine-tuning한 O 정책이다. Δ는 두 eval seed 평균.
  - 질문: 적응한 R과 U도 O처럼 인터페이스를 측정하는가. 예측은 두지 않는다 (탐색). E(R)과 E(U)를 잡음 합(R, U 각각 Δrot의 eval seed 차이 / |Δrot(O_ft)|)과 함께 보고한다.
- O 대조군 곡선: fine-tuning 자체가 O의 성능을 얼마나 바꾸는지 보인다. O가 300 iteration 동안 90% 아래로 떨어지면 그 사실을 적고, 판정은 그대로 한다.

**공개:**
- 시도 1–5의 결과를 모두 본 뒤에 이 시도를 골랐다. 특히 zero-shot 회전 비율이 7 mm에서 R 0.73, U 0.70으로 가깝다는 것을 안다.
- 학습 seed가 손마다 둘뿐이므로, 잡음 추정이 거칠다.

### 시도 6 보완: 조기 중단 규칙과 학습 분할 (2026-11-25, fine-tuning 수치를 보기 전에 commit)

**조기 점검:** seed 20의 O, R, U를 먼저 iteration 100까지 학습한다. 그리고 checkpoint 25, 50, 75, 100을 평가한다 (256 env, seed 94000). 다음 중 하나이면 시도를 멈추고 "분리 불가"로 기록한다.
- R과 U가 둘 다 iteration ≤ 25에서 840°에 닿는다 (천장: 잴 것이 없다).
- iteration 100에서 R과 U의 회전 비율 차이가 3% 이내이고, 둘 다 50–100 구간에서 평평하다.
  - 평평함의 기준: iteration 50, 75, 100 값의 최댓값 − 최솟값이 0.03 이하.

아니면 300까지 이어 가고 seed 21을 돌린다.

**학습 분할:** 조기 점검 때문에 모든 run을 두 process로 나눈다. 모든 손과 seed에 똑같이 적용한다.
- 1부: iteration 1–100. 시작 checkpoint는 `start_model_1497_iter1.pt`다.
- 2부: iteration 101–300. 1부의 `model_100`에서 resume한다 (actor, critic, optimizer, normaliser). iteration 카운터만 101로 바꾼 사본에서 시작하므로 `model_k`는 여전히 k번 update 뒤의 가중치다.
- v2c segment 사이의 resume과 같은 방식이다. 경계에서 env가 모두 새로 reset되는 차이만 있다.

**R5/U5, R9/U9:** 시도 4b와 과제 B가 대기열에 있으므로, 7 mm 결과가 나온 뒤 할지 정한다. 하지 않으면 "하지 않음"으로 적는다.

### 결과 4: 물체를 보지 않는 정책 (segment 3 checkpoint, 2026-11-25)

O held-out(μ 1.3): 회전 782 / 747°, full+clean 72.7 / 70.1%, full≤30° 64.9 / 60.9%, goals 13.6 / 13.5.

**이전 비율 (μ 1.3, 회전):** R5 0.17, U5 0.17, R 0.18, U 0.19, R9 0.15, U9 0.15. 모든 재설계에서 정책이 거의 무너진다(full+clean 0–1%).

**주 지표, μ 1.3 → 2.0 회전 E** (Δrot(O) = −115°):

| 두께 | E(R) | E(U) | 잡음 합 | 판정 |
|---|---|---|---|---|
| 5 mm | 0.90 | 0.87 | 0.10 | 구별 안 됨 |
| 7 mm | 0.82 | 0.90 | 0.07 | R이 잡음 이상으로 작다 (규칙상 성립) |
| 9 mm | 0.93 | 0.92 | 0.03 | 구별 안 됨 |

- 7 mm만 규칙상 E(R) < E(U)가 성립한다. 그러나 두 손 모두 거의 돌지 않는다(O의 15–19%). 그래서 μ 반응이 O의 1/5–1/10이고 E가 1에 가깝다. 신호가 거의 없는 상태의 결과로 보고, 확증으로 세지 않는다. 5/9 mm에서는 반복되지 않았다.
- 보조 지표: goals E는 9 mm에서 R이 작았다(0.84 대 0.87). full≤30°와 full+clean E는 9 mm에서 U가 작았다. 나머지는 구별되지 않는다.
- **해석:** 물체 피드백이 없으면 정책은 손이 달라지자마자 무너진다. R도 U와 같다. 7 mm 재설계는 PIP의 접촉 경로 밖에서도(MCP 수준 기하, grasp 자세) 정책에 충분히 큰 변화를 준다.

## 시도 4b: 잡음과 지연이 있는 큐브 pose (사전 등록, 학습 전 commit, 2026-11-25)

**왜 하는가:** 시도 4의 blind 정책은 모든 손에서 무너졌다. 반대로 참 pose를 보는 정책(시도 1)은 물체 피드백으로 손 차이를 흡수한다. 그 중간을 본다. 실제 센서처럼 pose가 늦고 흔들리면, 피드백으로 보정할 수 있는 정도가 줄어든다. 그러면 손이 만들어 내는 접촉면 움직임이 결과에 더 많이 드러날 수 있다.

**정책** (`CoDe-Rotate-Cube-O-OmniResetV2Noisy`):
- actor 입력은 release policy와 같다 (255차원). 그래서 `model_1497`에서 warm start한다.
- actor의 `cube_pose`와 `cube_goal_error`를 같은 센서 하나에서 계산한다 (`mdp._PoseSensor`):
  - **지연:** episode마다 3–6 control step (100–200 ms, 30 Hz)을 정수 균등분포로 뽑는다. 보고하는 pose는 그만큼 이전의 참 pose다. episode 시작 직후에는 첫 pose를 쓴다.
  - **위치 잡음:** 매 step, 축마다 ±15 mm 균등분포.
  - **자세 잡음:** 매 step, world frame에서 균등 임의 축 둘레로 0–20° 균등분포만큼 돌린다.
  - **goal 오차:** 지금의 goal에서 지연된 progress를 빼고, 잡음 회전의 world-z 성분을 더 뺀다. 기울기와 위치 오차는 잡음 섞인 pose에서 계산한다.
- critic은 privileged 참 상태 그대로다.
- **학습:** recipe는 v2c와 같다 (goal 보너스 5, entropy 0.001, std 상한 1.0, abnormal 10×, NaN 복구, 다섯 reset type). O, seed 20, 4096 env, segment 250 iteration.
- **조기 점검** (segment마다, O held-out, 1024 env, seed 94000/95000 평균 회전, 센서 켬):
  - segment 1(전체 잡음) 뒤 ≥ 600°이면 그 checkpoint를 이전에 쓴다.
  - < 600°이면 한 번만 줄인다: 잡음을 절반으로 (±7.5 mm, 0–10°, 지연은 그대로; `...V2NoisyHalf`). segment 1의 마지막 checkpoint에서 250 iteration을 더 학습한다.
  - 그 뒤 ≥ 600°이면 그 checkpoint를 쓰고, 이전 eval은 절반 잡음 센서로 한다. 아니면 시도를 중단하고 "학습 실패"로 기록한다.
  - 이전에 쓸 checkpoint는 이 규칙으로만 정한다. R/U 결과를 보고 고르지 않는다.

**이전:**
- 대상: O, R, U, R5, U5, R9, U9. task는 `CoDe-Rotate-Cube-<H>-OmniResetV2Noisy[Half]-Eval`이다 (학습과 같은 센서를 켠 채로).
- 조건은 시도 1과 같다: cache `heldout_transfer_v1`, 1024 env, seed 94000/95000, pad μ 1.3과 2.0, eval override.
- 순서: 7 mm 쌍(O, R, U)의 μ 1.3/2.0을 먼저 한다. R과 U가 모든 지표에서 잡음 안이어도 5/9 mm까지 기록용으로 마친다.
- 센서 잡음은 torch 전역 RNG로 뽑는다. eval seed가 정해지면 재현된다.

**주 지표:** μ 1.3 → 2.0 교체에 대한 회전 E (시도 1c 정의). 5, 7, 9 mm에서 각각 계산한다. O의 Δrot은 같은 정책을 O에서 잰 값이다.
- **예측:** 각 두께에서 E(R) < E(U)이고, 그 차이가 seed 잡음 합보다 크다.
- **반증:** 어느 두께에서든 E(R) ≥ E(U)이고, 그 차이가 잡음 합보다 크다.
- 차이가 잡음 합 안이면 "구별 안 됨"이다.
- **판정 불가:** |Δrot(O)|가 O의 Δrot seed 차이의 2배보다 작으면 E를 판정하지 않는다.

**보조 지표:** μ 1.3의 O 대비 이전 비율 (회전, full≤30°, full+clean, goals/episode).
- 예측: ratio_R > ratio_U. 차이가 seed 차이 합을 넘을 때만 판정한다.

**공개:**
- 시도 1–5의 결과를 모두 본 뒤에 등록한다.
- 코드 점검(smoke) 때 `model_1497`(4b 정책이 아니다)을 이 센서를 켠 R에서 32 env로 한 번 돌렸다. 평균 회전은 432°였다.

### 시도 6 보완 2: iteration 200 점검 (2026-11-25 21:30, iteration 100 값을 본 뒤, checkpoint 200을 평가하기 전에 추가)

**공개:** 이 규칙은 seed 20의 iteration 100 값을 본 뒤 추가했다 (사용자 지시, "가망 없는 시도는 빨리 멈춘다"). 그때 R은 0.71 → 0.76, U는 0.69 → 0.79였다.

**규칙:** seed 20의 R과 U를 iteration 200까지 학습하고 checkpoint 125–200을 평가한다 (256 env, seed 94000). 다음 중 하나이면 시도 6을 멈춘다. 300과 seed 21은 돌리지 않고, "R은 적응 이점을 보이지 않는다 (seed 20 곡선, iteration 200까지)"로 기록한다.
- iteration 200에서 U ≥ R (회전 비율).
- R이 여전히 평평하다: iteration 100–200 값의 범위가 0.02 미만.

R이 200에서 U보다 0.03 넘게 앞설 때만 300과 seed 21로 간다. (0 < R − U ≤ 0.03이고 R이 평평하지 않은 경우는 규칙에 없다. 이때도 멈춘다. 이어 갈 조건은 "0.03 초과"뿐이다.)

**학습 분할 변경:** 이 점검을 위해 seed 20의 R과 U는 101–200과 201–300으로 나눈다. 둘째 부분은 `model_200`에서 resume하고, 카운터를 201로 바꾼 사본에서 시작한다. O seed 20의 2부(101–300)는 시작 직후 멈췄다. 시도를 이어 갈 때만 다시 돌린다.

### 결과 6: fine-tuning 비용 — 중단 (iteration 200 규칙)

**판정: R은 적응 이점을 보이지 않는다 (seed 20 곡선, iteration 200까지).** 보완 2의 규칙에 따라 멈췄다. iteration 200에서 U(0.774) ≥ R(0.753)이었다. 300 iteration과 seed 21은 돌리지 않았다. 따라서 주 지표(회복 iteration)와 보조 지표(iteration 300 비율, μ 교체 E)는 없다.

**reset dataset (손별, 각 type 10,000개; acceptance R / U / O):**

| type | R | U | O |
|---|---|---|---|
| reaching | 17.2% | 17.9% | 19.4% |
| grasp | 6.1% | 5.1% | 7.0% |
| near-object | 9.0% | 9.8% | 18.0% |
| near-goal | 80.5% | 75.3% | 81.5% |
| mid-turn | 92.3% | 92.1% | 88.8% |

- mid-turn harvest (`model_1497`, 그 손에서): R turn 6022 / limit 4016 (555 step), U 6020 / 4017 (572 step).
- 진행 각도 중앙값: R 42°, U 43°.

**회복 곡선:** 회전 / 933.2° (256 env, seed 94000, μ 1.3, 그 손의 held-out cache). 문턱은 0.90이다.

| iteration | 0 | 25 | 50 | 75 | 100 | 125 | 150 | 175 | 200 |
|---|---|---|---|---|---|---|---|---|---|
| O (s20) | 1.001 | 1.008 | 0.990 | 0.988 | 0.989 | – | – | – | – |
| R (s20) | 0.712 | 0.737 | 0.769 | 0.755 | 0.759 | 0.792 | 0.785 | 0.784 | 0.753 |
| U (s20) | 0.694 | 0.714 | 0.743 | 0.785 | 0.788 | 0.804 | 0.792 | 0.777 | 0.774 |

- 회전으로 보면 R은 664 → 최고 739°, U는 648 → 최고 751°다. 둘 다 문턱 840°에 닿지 않았다.
- 0–200 구간 AUC (등록한 AUC는 0–300이다. 이 값은 참고용): R 0.764, U 0.767.
- O 대조군: 100 iteration 동안 0.99–1.01로 유지됐다. fine-tuning 자체가 성능을 떨어뜨리지 않는다.

**조기 점검 기록:**
- iteration 100: R 0.759, U 0.788. 차이 0.029로 3% 안이었다. 그러나 U가 50–100에서 평평하지 않아(범위 0.045) 계속했다.
- iteration 200: U ≥ R이므로 멈췄다. R의 100–200 범위는 0.039로, 평평함 조건(< 0.02)에는 걸리지 않았다.

**정리:**
- 두 손 모두 200 iteration (약 26M transition) 동안 O 대비 약 0.71 → 0.75–0.80으로만 회복했고, 그 뒤 평평하다.
- 회복 속도와 수준에서 R이 U보다 낫다는 증거는 없다. 수치는 오히려 U가 조금 높다. seed 하나라서 잡음 판정은 하지 않는다.
- R5/U5, R9/U9 fine-tuning은 하지 않았다. 7 mm 결과를 보기 전(iteration 100 시점)에 대기열(과제 B, 시도 4b)을 이유로 정했다.

**run** (`logs/rsl_rl/code_rotate_cube_omnireset_v2_finetune/`, W&B group `omnireset_finetune`):
- `*_O_OmniResetV2_s20_ft1`
- `*_R_OmniResetV2_s20_ft1`, `_ft2`
- `*_U_OmniResetV2_s20_ft1`, `_ft2`
- O s20의 101–300 부분은 시작 직후 멈췄다 (`stopped/`).
