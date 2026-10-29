# 설계 비교 실험: 결정 사항과 TODO

`design-comparison-task.md`, `paper-figures-task.md` 진행 중 내린 결정과 남은 일. 결정이 바뀌면 이 파일을 고친다.

## 결정됨

### 비교 study (`scripts/optimize/comparison/`)
- production linkage study와 분리. `base.yaml`이 goal을 고정: nsga2, population 200, seeds [0, 1, 2], 출력 `output/comparison`, proximal pad 하한 20 mm. preset 의존 없음.
- warm start: 모든 메커니즘이 쓰지 않음 (linkage seed 상속 금지).
- linkage는 `on_links: false`. 기준 조건(h10, φ150)에 `on_links: true` 변형 run (`linkage_onlinks`).
- `joint_span_ratio` 제거. 대신 `proximal_length_ratio [0.35, 0.55]`, `distal_length_ratio [0.25, 0.55]`를 모든 메커니즘이 읽음 (baseline formulation 버전 업).
- 조건 grid: 두께 10/5/3 mm × 요구 flexion 150/120/90°.
- 예산: baseline을 linkage 이상으로 맞추고 모든 run의 수렴 곡선(`history.npz`, generation별 best) 저장. 세대 수는 미정 (아래 TODO).
- 4b(대칭성 통계)는 현재 production front로 계산하고, production 조건(목적 3개, `on_links: true`)의 통계임을 명시.

### 메커니즘
- 2R parallel → belt 구동(k = 1), servo는 A′. pulley 반경 `goal.hardware.pulley_radius_mm` (기본 6 mm). tendon(Task 3a)과 같은 pulley 모델을 쓰고 차이는 palmar routing 띠만.
- parallelogram 2R은 transmission [30, 150]에서 φ > 120° 불가 → 보고서에 기록.
- 1a 통일: tilt 탐색, A 핀 검사, 공통 distal carrier, drop [0, r], hyperextension −10°.
- overhang: `pads.*.overhang_max_mm` 기본 10 mm, 모든 메커니즘 공통. h10/φ150에 0 mm 민감도 run (`linkage_overhang0`, `rcj_overhang0`).
- formulation: linkage-v4, revolute-goal-v4, revolute-idealized-goal-v3, revolute-parallel-goal-v3 (belt), rcj-goal-v2.

### Y-layer
- central: pad, servo. outer: 링크, carrier 외 bar, belt pulley·strand. through: 핀.
- belt pulley: outer layer, 충돌 검사 없음, 가림 판정에는 포함.
- tendon palmar routing 띠: pad layer(central).

### 손실 판정
- 임시 규칙: v4 segment (상대 pad 면 segment 뒤·맞은편이면 손실). 모든 메커니즘 적용, 테스트: 깊이가 같을 때 기존 규칙과 일치, φ에 대해 연속.
- 구조(제안 수용): 두 판정으로 분리, 판정 함수가 규칙과 D를 인자로 받음 (`core/optimization/analysis/loss.py`).
  - integrity: 모든 자세 [0, φ_req], 작은 D (`loss.integrity_depth_mm`). objective와 coverage에 사용.
  - accessibility: 자세별 프로파일 A(φ), 큰 D (`loss.access_depth_mm`, 초기 20 mm). report와 Visual figure 자세 열에 사용.
  - 대상 solid: 자기 pad를 제외한 모든 solid (상대 pad, carrier, 링크, 핀 housing, servo, pulley, roller), Y-layer 무관.
- 손실 진단은 물리적 의미 확인용이며 규칙 선택에 쓰지 않음.

### 실행 / figure
- `scripts/optimize/run.py`는 run_*/portfolio_*가 있는 출력 폴더를 거부 (`--append`로만 추가).
- 논문 figure: heavy search보다 figure 생성 구조 먼저. 초안은 비교 study quick run(`output/comparison_quick`, seed 0, 300세대) + v4 segment, PROVISIONAL 표시. `python -m visualize.paper.build [source]`.

## 결정 대기 (사용자)
- [ ] **integrity D** 값 (`loss.integrity_depth_mm`). 근거: `visualize.study.loss_profile` 진단 (D ∈ {0, 2, 3, 5, 10, 20} mm).
- [ ] **예산**: 진단 run 결과 보고 후 결정. 기본안 전 메커니즘 1500세대.
- [ ] **메커니즘 공통 최대 두께 requirement** (Task 2 추가 항목 결과 보고 후).
- [ ] **palmar budget 정의** (Checkpoint B).
- [ ] **interface envelope 변수 표** (Checkpoint C, Scientific figure (c)).

## TODO (구현)
- [ ] integrity D 결정 후: `loss.integrity_depth_mm` / `loss.access_depth_mm` config 키, 모든 evaluator에 integrity 연결 (formulation 버전 업), 테스트.
- [ ] 진단 run 재실행 준비: h5/φ150, h10/φ150 (`scripts/optimize/comparison/budget/`). 출력 폴더 guard 때문에 새 폴더 사용. 보고: 수렴 곡선, run best vs 0.05° verified 차이 (메커니즘별).
- [ ] 동점 front 중복 제거 검토 (단일 목적 front의 0.05° 검증이 느림).
- [ ] Task 2: bound 분류 표, evaluator의 ICR₀ 보고, extended / ICR-control run, tilt 90° 상한 = 두께 공짜 사용 여부, best 설계의 dorsal–palmar 두께 → CP2.
- [ ] Task 4: 4a 대칭 모드 closed-form 테스트, 4b 대칭 통계(production 조건 명시), 4c free vs symmetric, 4d v0 후보(margin, CAD 좌표 0.1 mm grid) → CP4.
- [ ] Task 3: 3a revolute_tendon (r_pulley, t_route placeholder는 사용자가 정함), 3b 3R 결정 목록 → CP3.
- [ ] Task 5: 5a analytic overlay, 5b h_b(φ), 5c tolerance Monte Carlo (±0.1 mm × 500), 5d minimal-change 재최적화.
- [ ] Task 6 / paper figures: full run 결과로 `visualize.paper.build output/comparison`, PROVISIONAL 해제.
- [ ] 최종 보고서 `docs/figures/paper-figures.md`. 포함할 것: parallelogram φ > 120° 불가, v3 → v4 규칙 변경, production front v4 재검증에서 coverage 상승 (+0.96%p, 전부 endless line → segment 효과), belt pulley outer layer.

## 관찰 (provisional, 인용 금지)
- quick run: 설계가 나온 42칸 모두 CoDe coverage 최대. 2R belt = 2R ideal (모든 칸). 차이는 굽힘·두께가 클수록 큼 (h5: 90° +2.3, 120° +5.4, 150° +15.9 %p). 90°는 seed 편차 안일 수 있음.
- h5/φ150 CoDe는 링크가 φ ≈ 45°부터 proximal 면 앞 0–1 mm를 지남. production linkage는 하드웨어가 면에서 7 mm 이상 떨어짐.
- h10/φ150: 2R 계열 feasible 설계 없음. linkage quick 결과는 비단조 (미수렴).
