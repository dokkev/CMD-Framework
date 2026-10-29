# Task: CoDe Hand 설계 비교 실험 (T-RO) — 전체 작업 지시

이 문서는 설계 비교 실험 전체를 다룬다. Figure 상세 사양(Visual / Scientific figure, palmar length budget 정의)은 `paper-figures-task.md`를 따르며, 아래 Task 6에서 그 문서의 Task B–E를 실행한다. 두 문서가 충돌하면 이 문서가 우선한다.

## 0. 배경

논문의 주장은 두 가지 원인으로 정리된다.

1. **Hardware 점유**: servo와 transmission이 phalanx volume을 차지한다. actuator 재배치로 완화 가능하며, 이미 알려진 방법이다.
2. **고정 pivot의 기하학**: kinematic joint와 physical joint가 같은 곳에 있어, hardware가 없어도 flexion할수록 joint 부근 contact 면이 사라진다. 단순 모델로 잃는 연속 contact 길이는 `ΔL ≈ 2·h_b·tan(φ/2)`이며, `h_b`는 contact 면이 pivot 축보다 palmar 쪽으로 떨어진 깊이(≈ pin housing 반경 + interface 두께)다. actuator 위치와 무관하다.

CoDe(crossed four-bar polycentric joint)는 2를 해결하고, servo를 contact 면 반대쪽 A′에 두어 1을 부분적으로 해결한다. 실험은 **"재배치만으로는 부족하고, 고정 pivot의 손실이 남는다"**를 검증하는 것이 목표다. 가장 중요한 비교는 **CoDe vs idealized 2R**(hardware 없는 고정 pivot의 상한)과 **CoDe vs 2R parallel**(servo 배치가 같고 joint 종류만 다른 쌍)이다.

**결과를 스토리에 맞추지 말 것.** 예를 들어 idealized 2R이나 2R parallel이 CoDe와 비슷한 coverage를 내면, 그것이 논문의 무게중심을 바꾸는 가장 중요한 결과이므로 그대로 보고한다.

## 1. 작업 원칙

- 먼저 `docs/optimization/decision-points-2026-10-22.md`, `docs/optimization/retention.md`, `scripts/optimize/interface_coverage/make_configs.py`, `scripts/optimize/pad_cost/make_configs.py`, `scripts/optimize/paper/fingers.py`, 각 메커니즘의 `core/optimization/<mechanism>/{config,evaluate,mechanism}.py`를 읽는다.
- 현재 돌고 있는 study의 formulation은 바꾸지 않는다. 새 설정은 새 formulation 버전이나 새 study 디렉토리로 만든다.
- 장시간 최적화는 직접 실행하지 않는다. config와 실행 명령을 준비하고, smoke run(짧은 generation)으로만 동작을 확인한다. full-budget run은 내가 실행한다.
- 모든 새 계산 로직에는 테스트를 붙인다.
- **Checkpoint**에서는 구현 전에 정의·계획·발견 사항을 보고하고 확인을 기다린다.

## 2. Task 1 — 비교 공정성 점검

### 1a. Servo 배치 규칙 일치 확인

`linkage`(CoDe four-bar)와 `revolute_parallel`(2R parallel)은 둘 다 PIP servo를 A′에 둔다. 두 메커니즘의 차이가 joint 종류 하나가 되려면 아래가 같아야 한다.

- A′ 위치의 탐색 범위와 제약 (예: `A′_z` 하한)
- servo envelope(`ServoFootprint`, tilt 규칙), MCP servo와의 clearance 규칙
- pad와 servo의 충돌 검사 범위와 방식

차이가 있으면 표로 정리하고, 맞추는 방안을 제안한다.

### 1b. `revolute_parallel` 복구

`fingers.MECHANISMS`에서 빠져 있다(2R pads-off-the-joint 규칙 `revolute-goal-v3` 이전 run). evaluate가 v3 규칙을 따르는지 확인하고, 필요하면 수정·버전 업·테스트 갱신 후 `interface_coverage` config를 재생성한다. `fingers.MECHANISMS`에 다시 넣는다.

### 1c. 탐색 예산 표

메커니즘별 generations × population × seeds(총 evaluation 수)를 표로 만든다. baseline(`revolute`, `revolute_parallel`, `revolute_idealized`, `rcj`)의 예산이 `linkage` 이상이 되도록 제안한다.

**Checkpoint 1**: 1a 차이표, 1b 판단 근거, 1c 예산 표, full run 명령.

## 3. Task 2 — Bound 체계 정리 (비율 기반)

### 원칙

모든 bound를 세 종류로 분류해 문서화한다.

- **Requirement**: 비교 조건 정의. 논문에 근거와 함께 명시. binding되어도 된다.
- **Hardware**: CAD·부품에서 나온 물리 한계. mm 단위 유지. binding되어도 된다.
- **Search box**: 탐색 범위. **binding되면 안 된다.** front 설계의 bound hit이 0%에 가까워야 한다.

### 새 config 키 (모든 비율은 extension 자세 |OC| 대비)

| 키 | 의미 | 종류 | 초기값 |
|---|---|---|---|
| `goal.link.proximal_length_ratio` | OA / \|OC\| | Requirement | [0.35, 0.55] |
| `goal.link.distal_length_ratio` | BC / \|OC\| | Requirement | [0.25, 0.55] |
| `goal.link.distal_length_mm` (기존) | BC 하한 | Hardware | ≥ 25 mm 유지 |
| `joint_span` (\|AB\|) | 별도 제약 없음 | Search box | 넓게, bound hit 모니터 |

- 모든 메커니즘에 같은 키를 적용한다. 2R 계열은 AB = 0이므로 OA + BC = |OC|, RCJ는 AB = 2r, linkage는 AB가 four-bar span이다.
- A_z = B_z = 0이므로 extension에서 OA + AB + BC = |OC|임을 테스트로 확인한다.
- 위 값은 시작점이다. 기준 run 후 bound hit을 보고 조정한다.

### 굽힘점(ICR₀) report

- extension 자세의 ICR₀를 메커니즘별로 계산하는 함수를 evaluator 쪽에 둔다: 2R 계열은 A, RCJ는 roller 접점, linkage는 직선 AB′와 A′B의 교점. (`fingers.py`의 `Finger.icr`과 같은 정의를 쓴다.)
- 모든 front 설계의 `|O→ICR₀| / |OC|`를 저장·보고한다. 위 링크 비율 범위에서 2R은 굽힘점이 0.45–0.55로 묶이지만 linkage는 더 넓게 움직일 수 있으므로, 이 값이 공정성 판단의 근거가 된다.

### 검증 run

- **기준 run**: 위 bound로 `interface_coverage` 전체 격자.
- **범위 확장 run**: 두 비율을 [0.25, 0.65]로 넓혀 메커니즘 간 순위가 유지되는지 확인.
- **굽힘점 통제 run**: linkage와 rcj에 `|O→ICR₀| / |OC| ∈ [0.45, 0.55]` 제약을 추가한 run. 2R과 같은 굽힘점 조건에서도 결론이 유지되는지 확인.

**Checkpoint 2**: 분류표(기존 모든 bound를 세 종류로 분류), config 변경 diff, 각 run의 실행 명령.

## 4. Task 3 — 메커니즘 추가

### 3a. 2R tendon (`revolute_tendon`)

idealized 2R을 기반으로 다음만 추가한 가벼운 모델로 만든다.

- 각 joint에 반경 `r_pulley`의 pulley envelope (joint에서 contact 면을 점유)
- flexion tendon이 palmar 쪽을 지나는 두께 `t_route`의 띠 (contact 면과 같은 공간을 점유)

`r_pulley`, `t_route`는 config 값으로 두고, **초기값은 내가 정한다(placeholder로 두고 Checkpoint에서 질문)**. 구조상 coverage가 idealized 2R 이하가 되어야 하므로 테스트로 확인한다.

### 3b. 3R serial — 보류

dexterity 축이 필요해지면 진행한다. 지금은 구현하지 말고, 착수 전 정해야 할 사항만 정리해 보고한다: required flexion 정의(φ 합 기준 vs fingertip 궤적 기준), 두 번째 굽힘점 비율 범위(사람 손 middle : distal ≈ 0.56 : 0.44 근거), dexterity 지표 후보(fingertip workspace 면적, orientation 범위).

**Checkpoint 3**: 3a 모델 설계안과 placeholder 목록, 3b 결정 사항 목록.

## 5. Task 4 — Antiparallelogram variant (v0 설계 선택용)

### 4a. 대칭 모드

linkage에 대칭 모드 플래그를 추가한다. extension 자세(A_z = B_z = 0)에서

- `B′_x = A_x + B_x − A′_x`
- `B′_z = A′_z`

로 B′를 유도한다. 이때 |AA′| = |B′B|, |AB′| = |A′B|가 성립함을 테스트로 확인한다. 독립 변수는 A_x, B_x, A′_x, A′_z, BC(+ servo tilt, pad offset 변수).

### 4b. 대칭성 통계

기존(자유) linkage front 설계마다 `|AA′| / |B′B|`, `|AB′| / |A′B|`를 계산해 분포를 보고한다.

### 4c. 자유 vs 대칭 비교

같은 조건(h 10 mm, φ_req 150°, |OC| 100 mm 기준; 가능하면 `interface_coverage` 전체 격자)에서 자유와 대칭 best의 coverage, k_max를 비교한다.

### 4d. v0 후보 선택

대칭 front에서 **최적값이 아니라 여유가 큰 설계**를 고른다. 기준:

- transmission angle 최소값의 여유
- pad clearance 최소값의 여유
- closure와 transmission 검사가 φ_req보다 충분히 넘어선 각도까지 통과 (antiparallelogram change point 회피)
- A′의 servo envelope 여유

후보 3개를 위 지표와 coverage, k_max와 함께 표로 보고하고, 각 후보의 CAD용 좌표(mm, 0.1 mm grid)를 출력한다.

**Checkpoint 4**: 4b 분포, 4c 비교표, 4d 후보표.

## 6. Task 5 — 보조 분석

### 5a. 해석식 overlay

idealized 2R에 대해 `ΔL ≈ 2·h_b·tan(φ/2)`로 예측한 coverage를 두께 × flexion 격자에서 계산해 수치 결과와 비교한다. `h_b`는 모델의 실제 기하(pad 바깥면과 pivot 축 사이 깊이)에서 계산한다. 예측과 수치가 크게 다른 조건이 있으면 원인을 보고한다.

### 5b. 유효 h_b(φ)

메커니즘별 best 설계에서, 자세 φ마다 contact 면(pad 바깥면)이 ICR(φ)로부터 palmar 쪽으로 떨어진 깊이를 계산해 csv로 저장한다. 2R은 상수, RCJ·linkage는 φ에 따라 변한다. **CoDe의 ICR이 flexion 중 contact 면 쪽으로 내려오는지**를 확인하는 것이 목적이다.

### 5c. 공차 민감도

각 메커니즘 best 설계에서 pivot 좌표에 ±0.1 mm 균일 오차를 주는 Monte Carlo(예: 500회)로 k_max, coverage, 최소 pad clearance의 분포를 구한다. linkage가 더 민감하게 나와도 그대로 보고한다.

### 5d. Interface sensitivity (최소 변경)

`paper-figures-task.md`의 interface envelope(고정 morphology)에 더해, envelope 밖의 interface 요구조건에 대해 기준 설계에서 출발하는 최소 변경 재최적화를 interface sweep을 따라 이어간다. 각 단계에서 morphology 변수(Checkpoint C에서 확정한 분류)가 받는 물리적 변화량과, 불연속(전략 전환, branch 점프)이 생기는 지점을 보고한다.

## 7. Task 6 — Figure

`paper-figures-task.md`의 Task B(palmar length budget), C(Visual figure), D(Scientific figure), E(보고서)를 실행한다. 다음을 반영한다.

- 메커니즘 순서: `revolute` → `revolute_parallel` → `revolute_idealized` → `revolute_tendon` → `rcj` → `linkage`.
- Scientific (a) waterfall에서 relocation 단계는 `revolute` → `revolute_parallel`, polycentric 단계는 `revolute_parallel` → `linkage`. `revolute_idealized`는 점선 기준선.
- Scientific (b)에 5a의 해석 곡선 또는 해석 예측값을 함께 표시.
- 모든 figure의 행·범례에 best 설계의 k_max와 ICR₀ 비율을 작게 표기.

## 8. 진행 순서

1. 문서·코드 읽기 → 전체 계획 보고
2. Task 1 → **Checkpoint 1** (내가 `revolute_parallel` full run 시작)
3. Task 2 → **Checkpoint 2** (기준·확장·통제 run 시작)
4. Task 4 → **Checkpoint 4** (v0 CAD가 급하므로 Task 3보다 먼저)
5. Task 3 → **Checkpoint 3**
6. Task 5
7. Task 6 (run 결과가 나오는 대로)
8. 최종 보고서: `docs/figures/paper-figures.md`에 각 Task의 결과, 예상과 다른 결과, 수렴이 의심되는 조건, 정의상 한계(RCJ cable 미모델, tendon placeholder 값)를 별도 섹션으로 정리
