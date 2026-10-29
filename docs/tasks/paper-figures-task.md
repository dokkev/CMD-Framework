# Task: CoDe Hand 논문 핵심 figure 2개 (Visual / Scientific)

## 0. 배경 — 이 figure들이 증명해야 하는 것

논문(T-RO)의 핵심 주장은 다음과 같다.

> 고정 pivot finger에서는 kinematic joint와 physical joint가 같은 곳에 있어, flexion이 커질수록 joint 부근의 contact 면이 잠식된다. CoDe(crossed four-bar polycentric joint)는 회전 중심을 움직여 이 둘을 분리하므로, 같은 finger 길이에서 더 많은 **연속적으로 쓸 수 있는 contact interface**를 확보하고, interface를 바꿔도 morphology를 바꿀 필요가 적다.

이를 위해 두 figure를 만든다.

- **Visual figure (`fig_visual`)**: 같은 요구조건에서 메커니즘별로 contact 면을 *어디서, 어떻게* 잃는지 보여준다.
- **Scientific figure (`fig_scientific`)**: 이득이 *무엇 덕분이고(a)*, *어떤 조건에서 커지며(b)*, *그래서 interface 교체가 가능한지(c)*를 보여준다.

메커니즘은 아래 ablation 순서로 항상 같은 순서·색으로 표시한다.

| 순서 | key | 의미 |
|---|---|---|
| 1 | `revolute` | 2R, servo at joint (LEAP형) |
| 2 | `revolute_parallel` | 2R, servo off joint (parallelogram) — actuator 재배치 효과만 분리 |
| 3 | `revolute_idealized` | 2R, hardware 없음 — 고정 pivot의 기하학적 한계 (기준선) |
| 4 | `rcj` | rolling contact joint (provisional: cable routing 미모델) |
| 5 | `linkage` | CoDe (crossed four-bar) |

**중요:** 결과를 스토리에 맞추지 말 것. 예상과 다른 결과(예: CoDe가 idealized 2R을 넘지 못함, polycentric 이득이 조건에 따라 커지지 않음)가 나오면 그대로 그리고 보고서에 명시한다.

## 1. 작업 원칙

- 먼저 `docs/optimization/decision-points-2026-10-22.md`, `docs/optimization/retention.md`, `scripts/optimize/interface_coverage/make_configs.py`, `scripts/optimize/paper/fingers.py`, `scripts/optimize/paper/fig_mechanisms.py`, `scripts/optimize/paper/style.py`를 읽고 기존 구조를 따른다.
- 새 figure 스크립트는 `scripts/optimize/paper/`에, 출력은 `output/paper/`에 `.png`, `.pdf`, 수치 `.csv`로 저장한다(`style.save` 사용).
- `fingers.py`의 `Finger`(`margins`, `icr`, `k`, `draw`, `pads_mm`)와 `style.py`의 색·마커를 재사용한다. 새 계산 로직은 figure 스크립트가 아니라 `fingers.py` 또는 별도 모듈에 두고 테스트를 붙인다.
- **현재 돌고 있는 study의 formulation, objective, constraint는 바꾸지 않는다.** 바꿔야 할 이유가 보이면 멈추고 보고한다.
- 장시간 최적화는 직접 실행하지 않는다. config와 실행 명령을 준비하고, 동작 확인은 적은 generation의 smoke run으로만 한다. full-budget run은 내가 실행한다.
- 아래 **Checkpoint**에서는 구현 전에 정의·계획을 보고하고 확인을 기다린다.

## 2. Task A — 전제 조건: `revolute_parallel` 복구

`fingers.MECHANISMS`에서 `revolute_parallel`이 빠져 있다(기존 run이 2R의 pads-off-the-joint 규칙, `revolute-goal-v3` 이전이라서). 이 메커니즘 없이는 Scientific (a)(b)의 분해가 불가능하다.

1. `core/optimization/revolute_parallel/evaluate.py`가 `revolute` v3 규칙(pad가 joint housing에서 떨어져 각 segment에 붙음, proximal pad drop 탐색)을 따르는지 확인한다. 상속으로 이미 따른다면 그 근거를, 아니라면 수정 방안을 보고한다.
2. 필요하면 수정하고 formulation 버전을 올리고 테스트(`tests/test_revolute_parallel.py`)를 갱신한다.
3. `interface_coverage/make_configs.py`로 `revolute_parallel` config를 재생성하고 smoke run으로 동작을 확인한다.
4. **탐색 예산 공정성**: baseline이 덜 최적화되면 polycentric 효과가 과대평가된다. 메커니즘별 generations × population × seeds(총 evaluation 수)를 표로 정리하고, `revolute_parallel`과 `revolute`의 예산이 `linkage` 이상이 되도록 제안한다.
5. `fingers.MECHANISMS`에 다시 넣고 위 표의 순서로 정렬한다(`LABELS`, `SHORT_LABELS` 포함).

**Checkpoint A**: 1번 결과와 4번 예산 표를 보고하고, full run 명령을 알려준다.

## 3. Task B — Palmar length budget (두 figure의 공통 기반)

Visual figure의 strip과 Scientific (a)는 **finger palmar 면을 1D로 펼쳐서 길이를 범주별로 나눈 것**에 기반한다. 메커니즘마다 effective length의 의미가 조금 다르므로(linkage/parallel: link가 가로지르기 전까지의 면, 2R/RCJ: 충돌 제약을 만족하도록 줄어든 pad 면 자체), 모든 메커니즘에 동일하게 적용되는 정의를 새로 만든다.

### 제안 정의 (구현 전에 검토·수정 의견을 줄 것)

1. **좌표**: proximal은 proximal mounting line을 따라 `O`부터, distal은 distal mounting line을 따라 tip `C`까지. 두 segment를 이어 붙인 palmar 좌표 `s`를 만든다.
2. **Candidate face**: 각 segment의 mounting line 중 자기 body hardware(servo, pin housing, roller 등)가 static하게 차지하지 않는 구간. 그 위에 두께 `h`의 pad가 놓인다고 가정한다.
3. **Conflict interval(φ)**: 자세 φ에서 candidate face 위의 pad가 다른 body와 충돌하거나 link가 가로지르는 구간. 기존 pad margin/충돌 함수(`packaging.pad_margins` 등)를 재사용해 구한다.
4. **누적 손실**: `lost(φ) = ∪_{ψ∈[0,φ]} conflict(ψ)`. 검증 grid(0.05°)에서 계산.
5. **Usable**: candidate face − `lost(φ_req)` 중 segment별로 가장 긴 연속 구간 하나(pad는 segment당 한 블록이므로). 나머지 짧은 조각은 `fragment`로 분류.
6. **범주**: `usable`, `lost_flexion`, `fragment`, `hardware`, `other`(tip offset, MCP clearance 등). 모든 범주의 합은 펼친 palmar 길이와 같아야 한다.

### 구현

- `fingers.py`에 `Finger.palmar_budget(phi_req_deg, height_mm) -> dict` 와 `Finger.conflict_intervals(phi_deg) -> list[(s0, s1, category)]`를 추가한다.
- 테스트:
  - 범주 합 = 펼친 palmar 길이 (모든 메커니즘, 허용오차 0.05 mm)
  - `lost(φ)`는 φ에 대해 단조 증가
  - **일관성 검사**: `usable`의 segment별 길이 ≥ 해당 설계의 `proximal_effective_mm`, `distal_effective_mm`(optimizer가 고른 pad는 usable 구간 안에 있어야 함). 차이가 크면 원인(정의 차이 vs 최적화 미수렴)을 보고.
  - `revolute_idealized`는 `hardware`가 0에 가까워야 함.

**Checkpoint B**: 정의에 대한 의견(특히 2R/RCJ에서 "pad를 줄여서 충돌을 피한 것"이 `lost_flexion`으로 제대로 잡히는지)과 기준 조건(h 5 mm, φ 150°)의 메커니즘별 budget 표를 보고한다.

## 4. Task C — Visual figure (`scripts/optimize/paper/fig_visual.py`)

```
python scripts/optimize/paper/fig_visual.py [height_mm] [flexion_deg]   # 기본 5, 150
```

### 레이아웃

- **행**: 메커니즘 5개, ablation 순서. 행 라벨 옆에 `k_max`를 작게 표기.
- **열 1–3**: 같은 설계를 extension(0°), 중간(φ_req/2), φ_req 자세로 그림. 모든 셀 같은 스케일, `O`를 같은 위치에 정렬. 링크는 회색, candidate face 위에 `usable`(초록), 그 자세까지의 누적 `lost`(빨강 + hatching), `hardware`(회색)를 칠한다. proximal과 distal pad 모두 표시.
- **ICR 궤적**: 회전 중심이 움직이는 메커니즘(`rcj`, `linkage`)은 `Finger.icr`로 0..현재 자세까지의 궤적을 점선/점으로 그림. 2R은 pin 위치 고정이므로 점 하나.
- **열 4 (strip)**: `palmar_budget`을 가로 stacked bar 하나로. 모든 행이 같은 x축(펼친 palmar 길이, mm)을 공유해 행끼리 바로 비교되게 한다. bar 끝에 `usable` 비율(%) 숫자.
- 범례: usable / lost to flexion / hardware / (fragment) / ICR trail.

### 요구사항

- 각 행의 설계는 해당 조건에서 `pad_coverage`가 가장 높은 verified front 설계(`fingers.best`). 선택 규칙을 caption용 텍스트로 csv에 함께 기록.
- 색맹 대응: 색과 함께 hatching/선 스타일을 병행.
- 수치(`usable`, `lost_flexion`, `hardware`, `k_max`, coverage)를 `output/paper/fig_visual_h<h>_flex<φ>.csv`로 저장.
- `fig_mechanisms.py`의 그리기 코드를 재사용하되, 기존 figure 출력은 깨지지 않게 한다.

## 5. Task D — Scientific figure (`scripts/optimize/paper/fig_scientific.py`)

```
python scripts/optimize/paper/fig_scientific.py [height_mm] [flexion_deg]   # (a)(c)의 기준 조건
```

### Panel (a) — Effect decomposition (waterfall)

- 기준 조건에서 coverage를 waterfall로: `revolute` 막대 → `+relocation` (= `revolute_parallel` − `revolute`) → `+polycentric` (= `linkage` − `revolute_parallel`) → `linkage` 총합.
- `revolute_idealized`는 단계가 아니라 **수평 점선 기준선**으로 표시.
- `rcj`는 waterfall 밖에 별도 막대(또는 점)로 참고 표시.
- seed 간 spread(`fingers.seed_bests`)를 whisker로. 차이(gain)의 불확실성은 seed best의 min/max 조합으로 보수적으로 계산.

### Panel (b) — Where it matters (heatmap)

- `interface_coverage`의 전체 격자(두께 3/5/10 × flexion 90/120/150)에서 polycentric gain(`linkage` − `revolute_parallel`, coverage %p)을 heatmap으로. 같은 스케일로 relocation gain(`revolute_parallel` − `revolute`) heatmap을 옆에 둔다.
- 셀 안에 숫자 표기. 한쪽이라도 feasible 설계가 없는 셀은 회색 + "×".
- 색 스케일은 0을 중심으로 diverging(음수 gain도 정직하게 보이도록).

### Panel (c) — Interface envelope (고정 morphology)

새 모듈/스크립트가 필요하다: `scripts/optimize/interface_envelope.py` (계산), 결과를 (c)가 읽는다.

1. **Morphology vs interface 변수 분리**: 메커니즘별 설계 변수를 "morphology"(pivot 위치, 링크 길이, servo 배치 등)와 "interface"(pad 길이, A″/B″ offset, pad drop 등 pad에 속한 변수)로 나눈 표를 만든다. **Checkpoint C**: 이 표를 보고하고 확인을 기다린다.
2. 기준 조건의 best 설계에서 morphology를 고정하고, interface 변수만 바꿔가며 (L_p, L_d) 격자(예: 1 mm 간격)에서 해당 길이의 pad를 둘 다 동시에 장착할 수 있는지(φ_req까지 모든 제약 만족) 평가한다. 최적화가 아니라 평가 sweep이다. interface 변수가 여러 개라 한 (L_p, L_d)에 여러 배치가 가능하면 feasible한 배치가 하나라도 있으면 feasible.
3. 두께별(3/5/10 mm) feasible 영역을 구해 csv로 저장.
4. (c)에서는 기준 두께의 feasible 영역을 메커니즘별 반투명 영역으로 겹쳐 그리고, 다른 두께는 윤곽선으로. config의 후보 contact module 목록(`(name, L_p, L_d, h)`, 처음엔 placeholder)을 점으로 찍는다.

### 공통

- 모든 panel 수치를 `output/paper/fig_scientific_*.csv`로 저장.
- (b)의 셀 중 seed spread가 gain보다 큰 곳은 표시(예: 점 marker)해 해석 주의를 알린다.

## 6. Task E — 결과 보고서

`docs/figures/paper-figures.md`에 다음을 정리한다.

- 각 figure의 생성 명령, 입력 run 경로, git commit
- Palmar budget 정의(최종본)와 일관성 검사 결과
- 메커니즘별 탐색 예산 표
- 각 panel의 핵심 수치와, caption 첫 문장 후보 1개씩
- **예상과 다른 결과, 수렴이 의심되는 조건, 정의상 한계**(RCJ provisional 포함)를 별도 섹션으로

## 7. 진행 순서 요약

1. 문서·코드 읽기 → 전체 계획 보고
2. Task A → **Checkpoint A**
3. Task B → **Checkpoint B**
4. Task C (Visual figure) — 기존 run으로 먼저 그려보고, `revolute_parallel` full run 후 갱신
5. Task D (a)(b) → **Checkpoint C** → (c)
6. Task E 보고서
