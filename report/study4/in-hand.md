# Morphology O 큐브 인핸드 회전 보고서

**작성일:** 2026-11-24  
**상태:** 손 자세 패널티, 행동 표준편차 하한, 회전 중간 상태 reset을 더한 fine-tune으로 s15보다 모든 지표가 좋아졌다. 완전 회전 성공률은 아직 5% 안팎이다.

## 설정과 평가

- Isaac Lab + Newton MJWarp + RSL-RL PPO로 Morphology O 손의 큐브 회전을 학습했다.
- 기준 정책은 s15 체크포인트 `model_4597.pt`이며, 1 mm crown 패드와 late-stage progress reward를 사용했다. s15 fine-tune은 1,024개 환경에서 200 iteration, 총 6.55M transition으로 수행했다.
- 현재 최고 정책은 s15에서 이어 학습한 `posediff_seg1`의 `model_5096.pt`다. 4,096개 환경에서 500 iteration, 총 65.5M transition으로 수행했다.
- held-out grasp cache 1,024개를 seed 94000과 95000으로 평가했다. 관측 노이즈는 끄고 episode 길이는 20초로 설정했다. 두 정책 모두 같은 설정으로 평가했으며 학습용 reset pool은 평가에 쓰지 않았다.
- 생존 평가는 off-grasp 70 mm, tilt 기준 완화 설정을 사용했다. 회전 유지율은 별도로 엄격한 변위 40 mm 이하 및 tilt 60° 이하 기준으로 계산했다.
- 같은 s15 체크포인트를 다시 평가하면 seed 94000에서도 반 바퀴 유지율이 20.4–24.3%, 완전 회전이 1.0–1.4% 범위에서 흔들린다. 이 범위를 넘는 차이만 개선으로 본다.

## 결과

| 지표 | s15, seed 94000 | s15, seed 95000 | posediff_seg1, seed 94000 | posediff_seg1, seed 95000 |
|---|---:|---:|---:|---:|
| 깨끗한 timeout 유지 | 828/1,024 (80.9%) | 841/1,024 (82.1%) | 883/1,024 (86.2%) | 893/1,024 (87.2%) |
| 평균 부호 있는 회전량 | 154.4° | 156.8° | 175.1° | 172.6° |
| 깨끗한 timeout 에피소드 평균 회전량 | 161.5° | 163.1° | 183.6° | 179.1° |
| 깨끗한 timeout 에피소드 후반부 추가 회전 | 19.8° | 20.4° | 40.8° | 37.6° |
| 엄격 기준에서 1/4회전 이상 | 538 (52.5%) | 554 (54.1%) | 584 (57.0%) | 603 (58.9%) |
| 엄격 기준에서 반 바퀴 이상 | 249 (24.3%) | 239 (23.3%) | 335 (32.7%) | 318 (31.1%) |
| 엄격 기준에서 3/4회전 이상 | 74 (7.2%) | 80 (7.8%) | 158 (15.4%) | 147 (14.4%) |
| 엄격 기준에서 완전 회전 | 12 (1.17%) | 16 (1.56%) | 45 (4.39%) | 51 (4.98%) |
| 최대 유지 회전량 | 397.4° | 427.0° | 456.7° | 449.0° |

두 seed 모두에서 posediff_seg1이 s15의 재평가 변동 폭을 넘어섰다. 깨끗한 timeout은 5–6%p 늘었고, 3/4회전은 약 2배, 완전 회전은 약 3.5배가 됐다. 깨끗한 timeout 에피소드의 후반부 추가 회전이 약 20°에서 약 40°로 늘어, 후반부 정체가 일부 풀렸다. 다만 완전 회전은 아직 5% 안팎이라 안정적인 과제 완수라고 보기는 어렵다.

## 회전 정체 진단

s15와 GRU 정책을 held-out grasp 128개로 추적했다. 2초 창에서 15° 넘게 회전한 마지막 시점 이후를 정체 구간으로 보면, s15는 중앙값 9초, 약 163°에서 멈췄다. 정체 구간에서는 관절 목표가 범위 끝에 붙는다. s15의 index, middle, ring 외전 목표가 하한에 머무는 시간 비율은 회전 구간의 22%, 35%, 15%에서 정체 구간의 63%, 57%, 36%로 늘었다. thumb 회전 목표는 처음부터 약 80% 시간 동안 하한에 있다. 관절 목표의 움직임은 정체 구간에서 약 3분의 1로 줄었다. 즉 손가락이 관절 한계에 도달한 뒤 다시 잡기(finger gaiting)로 이어지지 않는다. 이 추적은 측정 관절각이 아니라 명령 목표를 기록한 것이다.

Hora, PenSpin, LEAP Hand는 이 문제에 gaiting 보상을 따로 두지 않고 손 자세 정규화 항을 쓴다. Hora는 에피소드 시작 grasp 쪽으로 −0.3, PenSpin은 −0.1, LEAP Hand Isaac Lab은 고정 기본 자세 쪽으로 관절 목표에 −0.3을 준다. s15 계열은 이 항을 0으로 꺼 두었다.

## s15 이후 바꾼 것

1. **손 자세 패널티 (−0.1):** 관절 목표와 고정 기준 자세의 차 제곱합. 기준 자세는 학습 grasp cache 1,280개의 평균 명령 목표다(LEAP 방식). 에피소드 시작 자세를 기준으로 하면 관절 한계 근처에서 시작하는 reset이 그 자세에 묶이므로 고정 기준을 썼다. 기준 자세는 [canonical pose JSON](data/posediff_seg1_canonical_pose.json)에 있다.
2. **행동 표준편차 하한 (0.07):** 4,096개 환경으로 규모만 키운 fine-tune에서 행동 표준편차가 0.07에서 0.05로 줄며 회전이 오히려 줄었다. RSL-RL의 `std_range`로 s15의 평균값 0.07을 하한으로 고정했다.
3. **회전 중간 상태 reset (50%):** OmniReset(Yin et al., ICLR 2026)처럼 reset의 절반을 s15 roll-out에서 모은 2,873개 상태에서 시작한다. 2–20초 사이 무작위 시점에 큐브를 쥐고 있고 변위 40 mm, tilt 60° 안인 상태만 모았으며, 약 48%는 관절 목표의 4분의 1 이상이 한계에 붙은 정체 상태다([pool 요약](data/reset_pool_s15_summary.json)). 관절 위치·속도·PD 목표, 정책의 관절 목표, 큐브 자세·속도를 그대로 복원하고, 마일스톤과 진행량 보상, 목표 관측, tilt 기준은 reset 시점의 큐브 자세부터 다시 센다. 64개 reset 점검에서 첫 스텝의 큐브 속도와 패드 접촉력에 튐이 없어 안정화 구간은 두지 않았다.

세 가지를 함께 바꿨으므로 각 항의 기여는 따로 분리하지 않았다.

## 효과가 없었던 시도

- **규모 확대만:** s15 설정 그대로 4,096개 환경, lr 5e-4 adaptive로 1,500 iteration을 더 학습했다. 깨끗한 timeout은 약 84%로 늘었지만 평균 회전량은 128–134°, 완전 회전은 0.2%로 s15보다 낮아졌다.
- **GRU 정책 처음부터 학습:** LEAP Hand 설정(GRU 256, lr 5e-4, KL 0.02)으로 1,500 iteration을 학습했다. 평균 회전량 116–120°, 깨끗한 timeout 66–69%, 반 바퀴 유지율 6–7%에서 정체했고 3/4회전은 없었다.

## 패드 형상 메모

1 mm 둥근 crown의 초기 A/B 평가는 64회로 표본이 작다. 깨끗한 timeout은 50/64에서 54/64로 늘었지만, 평균 회전량은 72.1°에서 62.6°로, 1/4회전 유지율은 9.4%에서 6.25%로 낮아졌다. 현재 결과만으로 둥근 패드가 회전을 개선한다고 결론내리기는 어렵다.

## W&B 및 재현 자료

- [posediff_seg1 학습 run과 play 영상](https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/runs/2f29fa86)
- [s15 학습 run](https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/runs/21hvawq2)
- [s15 held-out 평가 run](https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/runs/57n41vcj)
- [s15 평가 아티팩트와 체크포인트](https://forge.coreweave.com/wandb/dhk6869/code-hand-inhand/artifacts/policy-evaluation/rotate-cube-o-s15-late3-evaluation/v0)
- 평가 JSON: [posediff_seg1 seed 94000](data/posediff_seg1_model_5096_rotation_seed94000_17699c520a.json), [posediff_seg1 seed 95000](data/posediff_seg1_model_5096_rotation_seed95000_823fbc8f87.json), [s15 seed 94000](data/s15_model_4597_rotation_seed94000_17699c520a.json), [s15 seed 95000](data/s15_model_4597_rotation_seed95000_823fbc8f87.json)
- 학습 설정: `CODE_ROTATE_CUBE_POSE_DIFF=-0.1`, `CODE_ROTATE_CUBE_RESET_POOL=<pool 디렉터리>`, `agent.actor.distribution_cfg.std_range=[0.07,1000000.0]`, 4,096개 환경, lr 5e-4 adaptive, KL 0.01, seed 18. reset pool은 `scripts/rotate_cube/harvest_reset_pool.py`로 만든다.
- 학습 설정 및 결과 코드는 `sim/isaaclab/src/code_isaaclab/tasks/rotate_cube/`와 `sim/isaaclab/scripts/rotate_cube/`에 있다.

**범위:** 위 결과는 Newton MJWarp 시뮬레이션 평가다. 실물 하드웨어 성능을 의미하지 않는다.
