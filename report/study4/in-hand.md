# Morphology O 큐브 인핸드 회전 보고서

**작성일:** 2026-11-24  
**상태:** 시뮬레이션에서 회전 동작은 학습됐으나, 완전 회전 성공률은 아직 낮음

## 설정과 평가

- Isaac Lab + Newton MJWarp + RSL-RL PPO로 Morphology O 손의 큐브 회전을 학습했다.
- 선택한 정책은 s15 체크포인트 `model_4597.pt`이며, 1 mm crown 패드와 late-stage progress reward를 사용했다. s15 fine-tune은 1,024개 환경에서 200 iteration, 총 6.55M transition으로 수행했다.
- held-out grasp cache 1,024개를 seed 94000으로 평가했다. 관측 노이즈는 끄고 episode 길이는 20초로 설정했다.
- 생존 평가는 off-grasp 70 mm, tilt 기준 완화 설정을 사용했다. 회전 유지율은 별도로 엄격한 변위 40 mm 이하 및 tilt 60° 이하 기준으로 계산했다.

## 결과

| 지표 | 결과 |
|---|---:|
| 깨끗한 timeout 유지 | 828/1,024 (80.9%) |
| 평균 부호 있는 회전량 | 154.4° |
| 깨끗한 timeout 에피소드 평균 회전량 | 161.5° |
| 엄격 기준에서 1/4회전 이상 | 538/1,024 (52.5%) |
| 엄격 기준에서 반 바퀴 이상 | 249/1,024 (24.3%) |
| 엄격 기준에서 3/4회전 이상 | 74/1,024 (7.2%) |
| 엄격 기준에서 완전 회전 | 12/1,024 (1.17%) |
| 최대 유지 회전량 | 397.4° |

같은 held-out 평가에서 초기 s8 대비 1/4회전 유지율은 39.4%에서 52.5%, 반 바퀴는 14.6%에서 24.3%, 3/4회전은 0.2%에서 7.2%로 올랐다. 완전 회전도 관측됐지만 1.17%라 안정적인 과제 완수라고 보기는 어렵다. 깨끗한 timeout 에피소드의 후반부 추가 회전은 평균 19.8°여서 후반 regrasp와 회전 지속이 주요 개선 후보로 남는다.

## 패드 형상 메모

1 mm 둥근 crown의 초기 A/B 평가는 64회로 표본이 작다. 깨끗한 timeout은 50/64에서 54/64로 늘었지만, 평균 회전량은 72.1°에서 62.6°로, 1/4회전 유지율은 9.4%에서 6.25%로 낮아졌다. 현재 결과만으로 둥근 패드가 회전을 개선한다고 결론내리기는 어렵다.

## W&B 및 재현 자료

- [s15 학습 run](https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/runs/21hvawq2)
- [held-out 평가 run](https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/runs/57n41vcj)
- [평가 아티팩트와 체크포인트](https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/artifacts/policy-evaluation/rotate-cube-o-s15-late3-evaluation/v0)
- [최종 평가 JSON](data/s15_model_4597_eval_seed94000.json)
- 학습 설정 및 결과 코드는 `sim/isaaclab/src/code_isaaclab/tasks/rotate_cube/`와 `sim/isaaclab/scripts/rotate_cube/`에 있다.

**범위:** 위 결과는 Newton MJWarp 시뮬레이션 평가다. 실물 하드웨어 성능을 의미하지 않는다.
