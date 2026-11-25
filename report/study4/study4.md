# Study 4: Morphology O 큐브 in-hand 회전 (RL)

**작성일:** 2026-11-25
**상태:** 완료(1차). 최종 정책은 OmniReset 계열 v2c 구간 2 `model_1497`이다. held-out grasp에서 20초 동안 평균 880–930°를 돌린다. 큐브를 수평 가까이 세운 채 한 바퀴 이상 돌린 에피소드가 64–66%(기울기 ≤ 30° 기준)다.

## 1. 설정

- **시뮬레이터:** Isaac Lab + Newton MuJoCo-Warp, RSL-RL PPO, 4,096개 환경, 30 Hz 정책, γ 0.997.
- **손:** Morphology O이고 dome pad(1 mm crown, 같은 높이)를 쓴다. PIP servo collider는 모든 USD에서 켠다(`CODE_ROTATE_CUBE_SERVO_COLLIDERS=0`이면 예전 asset).
- **접촉 모델:** `pad13_condim4_v1`. pad μ 1.3, condim 4, torsional 0.003, 그 밖의 면 μ 0.5, impratio 1000, solref 0.012.
- **골격:** OmniReset(Yin et al., ICLR 2026; UWLab 2b40afb).
  - 리셋 네 종류: reaching, 물체 근처, grasp, 목표 근처.
  - 보상: action² −1e-4, action rate −1e-3, 관절 속도 −1e-2, abnormal −100, reach 0.1, dense success 0.1, success 1.0.
  - 종료: time-out과 abnormal만.
  - 원 레시피와 우리 이식의 대응은 `docs/experimental/omnireset_recipe.md` §1–2에 있다.
- **평가:** held-out grasp cache `heldout_omnireset_v1`(seed 76000), 20초, 관측 noise 끔, 평가 seed 94000과 95000. 두 seed는 서로 다른 grasp를 뽑는다(같은 grasp 0.1%, 에피소드별 회전량 상관 약 0).
  - 바퀴 비율은 다음 세 조건을 모두 요구한다.
    - 깨끗한 time-out
    - 변위 40 mm 이내
    - 가장 위를 향한 면의 최대 기울기 ≤ 30°(또는 ≤ 45°). 최대 기울기는 큐브가 처음 20° 안으로 수평이 된 뒤부터 센다.
  - 예전의 "리셋 방향 60°" 기준은 이 손에 맞지 않는다. 시작 grasp에서 큐브가 기울어진 proximal pad 위에 있어, 수평으로 세우는 동작이 기울기로 읽힌다(recipe §6.3).

## 2. 결과

| 정책 | seed | 정상 종료 | 평균 회전 | 1바퀴 (≤30°) | 1바퀴 (≤45°) | 20° 안 시간 | 목표 달성/에피소드 (45°) |
|---|---|---|---|---|---|---|---|
| **v2c 구간 2 `model_1497` (최종)** | 94000 | 94.0% | **930°** | **66.4%** | 74.7% | 74.4% | 20.3 |
| **v2c 구간 2 `model_1497` (최종)** | 95000 | 93.1% | **880°** | **63.8%** | 70.7% | 71.1% | 19.3 |
| v2c 구간 1 (256개 환경) | 94000 / 95000 | 97.3 / 95.7% | 836 / 829° | 63.3 / 62.5% | 76.2 / 74.6% | 72.0 / 70.1% | 18.3 / 18.2 |
| v2e 구간 1 (수평 페널티 −1.0) | 94000 / 95000 | 92.4 / 90.8% | 909 / 866° | 62.7 / 59.2% | 73.4 / 68.5% | 72.6 / 69.0% | 19.9 / 18.9 |
| v1 구간 3 (OmniReset 그대로) | 94000 | 90.9% | 72° | – | – | – | 0.00 (90° 목표) |
| model_5595 (예전 asset, 256개 환경) | 94000 / 95000 | 80.5 / 83.2% | 56 / 65° | 0 / 0% | 0 / 0% | 10.7 / 12.3% | – |

- v2c 구간 1과 model_5595 행은 같은 seed, seed당 256개 환경으로 다시 잰 값이다. 나머지는 1,024개 환경이다.
- 최종 정책의 servo 속도는 p50 / p95 / p99가 0.25 / 1.06 / 1.39 rad/s다. servo 한계(5.55 rad/s)를 넘는 샘플은 0.014%다.
- 데이터: [최종 정책 평가](data/omnireset/eval_seg2_seed94000.json)([seed 95000](data/omnireset/eval_seg2_seed95000.json)), [v2c 구간 1 재평가](data/omnireset/rescore_v2c_seg1_seed94000.json), [model_5595 재평가](data/omnireset/rescore_5595_seed94000.json), [servo 속도](data/omnireset/speed_seg2.json).

## 3. 무엇이 정책을 돌게 했나 (run 계보)

| run | 바꾼 것 | 결과 |
|---|---|---|
| v1 | OmniReset 그대로, 90° 목표. 과속 종료 기준은 구간 3부터 servo 한계의 2배에서 10배로 바꿈. MJWarp는 implicit servo의 속도 한계를 적용하지 않아, 2배 기준이 평범한 빠른 동작에 걸려 에피소드의 8–16%를 끊고 있었다 | 약 60–70°에서 정체 |
| v2 | 회전 도중 리셋(v1 정책 roll-out에서 모은 10,000개, Khandate et al. RSS 2023), actor에 관절 목표와 5-step history, 비대칭 critic(UWLab v1.2.0·논문 설정), 45° 연쇄 목표, 성공 0.3 rad를 0.2 s 유지 | 학습 중 목표는 늘었지만 held-out 약 44°. 정책 std가 0.84에서 1.74로 커지며 목표가 줄어듦 |
| v2b | 엔트로피 0.006 → 0.001, std 상한 2.0 → 1.0 | noise 증가는 멈췄지만 목표는 계속 줄고 보상은 오름 |
| **v2c** | **목표를 넘길 때마다 보너스 5점**(Dactyl +5, AnyRotate 10, Isaac Lab Allegro 보너스 형태). 그 전에는 목표 하나가 약 0.2점이었고, 쥐고만 있어도 초당 0.1–0.2점이 쌓여 "조심스럽게 쥐기"가 최적이었다 | **연속 회전 시작.** 구간 1 약 820°, 구간 2 약 900° |
| v2d, v2e | 수평 유지 페널티 −0.1, −1.0 (15°까지는 0) | 20° 안 시간이 약 72%에서 오르지 않음. 정해 둔 기준에 따라 중단 |

모든 변경과 출처는 `docs/experimental/omnireset_recipe.md` §6과 `sim/isaaclab/logs/rotate_cube/omnireset_run/CHANGES.txt`에 있다.

## 4. 공학적 문제와 처리

- **NaN world:** 한 world가 한 번 NaN이 되면 리셋해도 계속 NaN이었다. Isaac Lab은 solver의 warm-start 버퍼만 지우기 때문이다. 이제 그 world의 상태를 0으로 바꾸고 solver를 mask 리셋한 뒤 리셋 상태를 쓴다. 발생과 복구 실패를 따로 세고, 발생할 때마다 버퍼를 `nan_onsets.jsonl`에 남긴다. 고친 뒤 발생은 10,000 env step당 0.0002–0.0005회, 복구 실패는 0이다.
- **리셋 데이터 검사:** 50,000개 상태를 학습 물리에서 10 step 돌렸다. 빠진 상태는 0개다(큐브 접촉력 최대 14 N).
- **servo 모델:** implicit servo의 속도 한계가 MJWarp에서 적용되지 않는다. 속도가 오를수록 토크가 줄어드는 dc_motor 모델은 아직 열린 확인 항목이다.

## 5. 한계

- 큐브가 20° 안에 있는 시간은 약 70–74%다. 돌리는 도중 최대 기울기는 중앙값 약 25°, p90 약 44°다.
- 최종 정책은 seed 하나로 학습했다(seed 20).
- 결과는 Newton MJWarp 시뮬레이션 평가다. 실물 하드웨어 성능을 뜻하지 않는다.

## 6. 이전 계열 (예전 asset과 접촉 모델, 기록)

OmniReset 이전에는 진행량 보상 기반 `rotate_cube` 과제(s15, posediff)를 썼다. 최고 정책 `posediff_seg1/model_5096`은 예전 asset에서 1바퀴 약 5%였다(예전 엄격 기준). 같은 계열의 model_5595는 새 asset과 접촉 모델에서 56–65°로 떨어진다. 예전 pad 관통은 p90 3.6 mm로, pad 모델 범위를 벗어났다. 정체 진단(관절 목표가 한계에 붙어 finger gaiting으로 이어지지 않음)과 시도 기록은 git `report/study4/in-hand.md`(이 파일의 이전 판)에 있다. 그 평가 JSON은 `data/`에 남긴다.

## 7. 재현

- **최종 체크포인트:** `sim/isaaclab/logs/rsl_rl/code_rotate_cube_omnireset_v2/2026-11-25_07-41-53_rotate_cube_O_OmniResetV2_s20_v2c_seg2/model_1497.pt`
- **과제:** `CoDe-Rotate-Cube-O-OmniResetV2`(평가 `-Eval`). 코드는 `sim/isaaclab/src/code_isaaclab/tasks/rotate_cube_omnireset/`에 있다.
- **설정:**
  - 환경 변수: `CODE_OMNIRESET_ABNORMAL_FACTOR=10`, `CODE_OMNIRESET_GOAL_BONUS=5`
  - `--override agent.algorithm.entropy_coef=0.001 --override agent.actor.distribution_cfg.std_range=[0.001,1.0]`
  - 4,096개 환경, seed 20
  - 이어받은 체크포인트: v2 구간 1 `model_499`
- **스크립트** (`sim/isaaclab/scripts/rotate_cube/`):
  - 리셋 데이터: `gen_omnireset_resets.py`, `harvest_omnireset_midturn.py`, `filter_omnireset_resets.py`
  - 평가와 진단: `eval_policy.py`, `servo_speed_probe.py`
  - 학습 영상 업로드: `video_watcher.py`
- **리셋 데이터:** `sim/isaaclab/logs/rotate_cube/omnireset/O_40f4fc6626/`
- **W&B** (project code-hand-inhand):
  - group: [omnireset_v2](https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/groups/omnireset_v2), [omnireset_v1](https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/groups/omnireset_v1)
  - 최종 정책 학습: https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/runs/obzohgn8
  - 최종 정책 영상: https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/runs/x57x39ic
