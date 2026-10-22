# CoDe Hand — Linkage Optimization

> **상태:** 설계 Master Document (2026-10-20, formulation `palmar-pad-v3`, 목적 `transmission`·`curl` 확정)  
> **역할:** 주어진 pad requirement 아래 linkage 메커니즘의 Pareto front와 대표 seed 생성 (open chain·RCJ와 비교)  
> **원칙:** 여기서 최종 hand design을 결정하지 않는다.

## 1. 목적과 범위

Linkage optimization은 **기구학적으로 유효하고 서로 다른 장점을 가진 finger linkage 후보들을 생성하는 seed generator**다. 특정 contact interface나 hand-level opposition task에 최적화된 최종 손가락을 선정하는 단계가 아니다.

**포함**
- Linkage kinematics, transmission, flexion capability
- Contact pad(`O–A''`, `B''–C` 아래 블록)의 장착 길이와 그 pad의 간섭
- 기본 servo/housing 및 mechanism packaging
- Pareto frontier와 대표 seed 추출

**제외**
- Proximal/distal contact interface(pad, tactile module)의 실제 형상, 재질, mounting 세부 (여기서는 잠정 두께의 사각 블록만 쓴다)
- Pad–pad collision, contact mechanics 및 contact coverage
- Hand layout, thumb/finger opposition motion
- Contact morphology optimization 또는 co-design

결과는 같은 pad requirement 아래 open chain·RCJ와 비교된다(6절).

## 2. 설계변수와 기준 형상

기존 extension 기준의 XZ 평면 좌표와 linkage 정의를 유지한다. 원점은 `O`, `+X`는 proximal 축, `+Z`는 dorsal이다. Extension에서 `BC`는 `+X`와 평행하다. `A`는 proximal 축 위에 둔다(`A_z = 0` 고정).

**Body topology.** 강체는 proximal(`O–A–A'`, PIP servo가 `A'`에 고정), middle link(`A–B'`), distal(`B'–B–C`), coupler(`A'–B`, PIP servo가 구동), palm(MCP servo)이다. Four-bar `A–B'–B–A'`와 그 solver는 바뀌지 않았다.

**Contact pad (§2.1).** Contact interface(pad)는 kinematic pivot 사이가 아니라 `A''`, `B''`까지 연장된 장착선 아래에 붙는다.

\[
\boldsymbol\xi=[A_x,A_z,BC,B_x,B_z,A'_x,A'_z,B'_x,B'_z,\alpha_{\mathrm{servo}},\delta_{A''}]^{\mathsf T}.
\]

| 설계변수 | 현재 탐색 범위 | 비고 |
|---|---|---|
| `A_x` | 5–120 mm | Proximal 길이. 사실상 unbound; servo 간섭 제약이 결정 |
| `A_z` | 0 (고정) | `A = (A_x, 0)`. 변수는 남아 있으나 bounds `[0, 0]`으로 고정 |
| `BC` | 30–50 mm | Distal 길이 (middle + distal phalanx) |
| `B_x` | 40–100 mm | Extension 기준 |
| `B_z` | 0 (고정) | `B = (B_x, 0)`; bounds `[0, 0]` |
| `A'_x` | 20–100 mm | |
| `A'_z` | 0–20 mm | OA 기준면 제약의 closed form으로 탐색 시 `>= 9.5 mm`로 축소 |
| `B'_x` | −50–200 mm | 사실상 unbound |
| `B'_z` | −60–80 mm | 사실상 unbound |
| Servo tilt | 0–45° | |

`B''`는 이 벡터에서 유도되며, `A''`는 `A'_x`에서 `δ_{A''}`(`app_offset_mm`)만큼 떨어진다.

**탐색 범위 (`search_bounds`).** config의 bounds는 그대로 두고, 닫힌 식이 있는 제약으로 실제 탐색 범위만 좁힌다. feasible 설계는 잃지 않는다.

- `A'_z >= servo 반경` (OA 기준면)
- 선형 제약: `B_x + BC`가 손가락 길이 범위 안, `A_x < B_x`, `A'_x >= A_x`, `B'_x <= B_x`, pad 장착선의 최소/고정 길이
- `A_z = B_z = 0`이고 pad 길이가 자유일 때, extension 자세에서 두 pad와 `A`·`B` 핀이 모두 `z = 0` 위에 있으므로 정적 pad 검사도 선형이 된다: `B'_x >= A'_x + clearance`, `B'_x >= A_x + 핀 외곽 반경 + clearance`, `A'_x <= B_x − 핀 외곽 반경 − clearance`, (MCP servo 검사가 켜져 있으면) `B'_x >= MCP servo 반경 + clearance`

기본 config에서 `A_x` [5, 65], `B_x` [40, 70], `A'_x` [20, 65], `B'_x` [21, 70]이 된다. 변수끼리 얽힌 제약(예: `B_x + BC`)은 상자로 다 표현되지 않으므로 그대로 제약으로도 남는다.

현재 0.1 격자: 거리·좌표 **0.1 mm**, 각도 **0.1°**. 기존 구현의 변수 정의와 이산화 방식을 재사용한다.

**가공 기준 (production 전용, config의 `link_length_resolution_mm = 0.1`; seed 생성에는 쓰지 않음).** 부품마다 자기 기준의 치수가 0.1 mm 격자에 놓인다: proximal은 `O` 기준 `A`, `A'`; distal은 `B` 기준 `B'`와 `BC`; 별도 bar는 `|A'B|`, `|AB'|` 길이. 설계 벡터에서 잰 두 bar 길이를 반올림한 뒤 extension 자세의 `B`를 두 원의 교점으로 다시 조립하므로, 조립 치수인 `B`(따라서 `C`, `|OC|`)는 격자 밖에 있을 수 있다.

구현: `core/optimization/linkage` (config, design, evaluate, objectives, `mechanism.py`). Linkage는 공통 최적화 엔진(`core/optimization/mechanism.py`의 `Mechanism` 인터페이스, `core/optimization/study.py`의 NSGA-II·저장·로드)에 꽂히는 메커니즘 하나다. Open chain·RCJ도 같은 인터페이스를 구현해 `MECHANISMS`에 등록하면 같은 스크립트로 돈다. 최적화 알고리즘도 같은 방식으로 바꿔 낀다: `core/optimization/optimizer.py`의 `Optimizer` 인터페이스와 `OPTIMIZERS`(현재 `nsga2`, `smsemoa`; 모두 multi-objective). YAML `optimizer.algorithm`(또는 `run.py --algorithm`)으로 고르고, 알고리즘 전용 설정은 `optimizer.options`에 둔다. `run.json`에는 `optimizer`(algorithm, population, generations, seed, options)가 메커니즘 config와 따로 기록된다. 실행 `scripts/optimize/run.py --mechanism linkage`, 검증 `scripts/optimize/select_seeds.py`, 시각화 `scripts/optimize/plot_pareto.py`(여러 메커니즘의 run을 함께 주면 메커니즘별 색으로 front 비교), `scripts/animate_finger.py`. 범위, 요구 조건, 하드웨어는 `scripts/optimize/linkage/config.yaml`(`optimization_parameter_bounds`, `packaging`, `pad`)이 기준이다.

Extension에서 `O=(0,0)`, `C=(B_x+BC,B_z)`이면:

\[
80\le |OC|_{\mathrm{extension}}=\sqrt{(B_x+BC)^2+B_z^2}\le100\quad[\mathrm{mm}].
\]

`BC >= 30`인 현재 범위에서는 이 제약으로 `B_x <= 70 mm`를 해석적으로 얻을 수 있다. 기존 bounds를 즉시 변경할 필요는 없으며, 필요하면 단순 탐색 pruning에 활용한다.

### 2.1 Contact pad

**원칙: kinematic pivot 배치와 contact pad의 범위는 더 이상 같은 경계를 공유하지 않는다.** 이전 `OA + BC` objective는 `A`와 `B`를 가깝게 붙여 장착 길이를 늘리는 방향으로 linkage를 왜곡할 수 있었다. 이제 pivot은 closure·transmission·굽힘·packaging으로 정하고, pad 장착선은 pivot을 넘어 `A''`, `B''`까지 연장된다.

기준 자세의 끝점:

\[
A''=(A'_x+\delta_{A''},\,0),\qquad B''=(B'_x,\,B_z).
\]

`A''`는 `A'`를 proximal mounting line(`x`축)에 정사영한 점에서 `δ_{A''} ≥ 0`만큼 distal로 간 점이고(`δ = 0`이면 `A'` 바로 아래), `B''`는 `B'`를 distal mounting line(`BC`)에 정사영한 점이다. 둘 다 회전 관절이 아니라 body에 고정된 끝점이다.

**Pad 형상.** Pad는 기존 링크(proximal `O–A–A'`, distal `B'–B–C`)에 붙는 블록 `a–b–d–c`이며, 링크 밖으로 돌출할 수 있다. `a–b`는 장착선 위, `c–d`는 그로부터 palmar(`−Z`) 쪽으로 height만큼 내려간다. 각 pad는 config `packaging.proximal_pad`, `packaging.distal_pad`의 세 값으로 정한다(`ContactPad`):

| 값 | 의미 |
|---|---|
| `height_mm` | 장착선에서 palmar 쪽 높이 (z) |
| `offset_mm` | 바깥 끝에서 안쪽으로 시작점 offset (x) |
| `chamfer_mm` | PIP 쪽 palmar 모서리의 chamfer 길이; 각도는 45° 고정 |

길이(x)는 정하지 않는다: 시작점에서 `A''`(proximal) 또는 `B''`(distal)까지이며, 이 길이가 F2다.

- **Proximal pad**: 시작점 `a`에서 오른쪽(`A''` 방향)으로 extrusion한다. 시작점 `x` = (`pad.allow_collision_with_mcp_servo: false`면 MCP servo 반경 + `pad.mcp_servo_clearance_mm`, `true`면 0) + proximal pad의 `offset_mm`. 기본(on, offset 0)은 9.5 + 1.0 = 10.5 mm이며, 이때 두 pad를 MCP servo와 간섭 검사한다. 첫 변 `a–c`는 장착선에 수직인 직선이다. MCP flexion으로만 회전한다.
- **Distal pad**: `C`(EE)에서 distal pad의 `offset_mm`만큼 안쪽을 시작점으로 왼쪽(`B''` 방향)으로 extrusion한다. Four-bar가 준 distal 자세를 따라 강체로 움직인다:

\[
B''(q)=B(q)+\frac{B'_x-B_x}{BC}\big(C(q)-B(q)\big).
\]

  움직이는 `B'`을 매 자세에서 고정된 수평선에 다시 투영하지 않는다(그러면 `B''`가 body 위에서 미끄러진다). Pad의 모든 점이 같은 방식(`B + u·e + v·n`, `e`는 `B→C` 단위벡터, `v <= 0`)으로 움직인다.
- **Chamfer**: PIP를 향한 끝(proximal의 `d–b`, distal의 `a–c`)은 직선일 필요가 없다. palmar 모서리(`d`, `c`)를 `chamfer_mm`의 45° chamfer로 깎는다(기본 2.5 mm, 0이면 직각, height 이하). `a–b` 장착 길이는 바뀌지 않는다.
- `A'–b`, `B'–a` 점선은 reference일 뿐 형상이 아니다. Coupler(`A'–B`)와 middle link(`A–B'`)는 다른 층이라 pad와 충돌하지 않는다.
- **장착 길이**: `L_P = A'_x − (proximal pad 시작 x)`, `L_D = B_x + BC − (distal offset) − B'_x`. offset은 설계와 무관한 상수라 F2 순위에는 영향이 없다.
- PIP servo는 기존대로 `OA` 선 위(`oa_interface`, `z >= 0`)에 있다. 같은 body인 proximal pad와는 이 제약으로만 분리된다.
- 구현: 끝점은 `FingerLinkage.app/bpp`, `FingerPose.app/bpp` (`core/morphology/linkage/model.py`). Pad는 장착선 좌표계의 한 윤곽 `pad_outline`을 `proximal_pad_polygon`(proximal 링크)과 `distal_pad_polygon`(distal 링크)이 각자 자세에 붙인다 (`core/morphology/linkage/packaging.py`). 시각화(`scripts/animate_finger.py`)도 같은 함수를 쓴다.
- **잠정 치수** (config `pad`): 각 pad height 5 mm, offset 0, chamfer 2.5 mm, `allow_collision_with_mcp_servo: false`. 실제 pad/tactile module이 정해지면 바꾼다.

**Y-layer (2.5D).** 각 solid는 XZ quad와 `|y|` 구간을 가진다(`COMPONENTS`).

| Layer | `\|y\|` 구간 | 부품 |
|---|---|---|
| central | 0 – servo depth/2 (11 mm) | proximal·distal pad, PIP·MCP servo |
| outer | 11 mm 바깥 | middle link 판, coupler 판 |
| through | 전체 | 핀 `A`, `A'`, `B'`, `B` (bearing은 양쪽 판에) |

**Collision group.** 각 부품은 rigid body 단위의 collision group에 속한다(`COLLISION_GROUPS`, `COMPONENTS`): palm(MCP servo), proximal(proximal pad, PIP servo, `A`·`A'` 핀), distal(distal pad, `B'`·`B` 핀), 그리고 outer layer의 middle link, coupler. 같은 group은 한 강체라 서로 충돌하지 않고, **다른 group이면서 Y 구간이 겹치는** 부품은 충돌하면 안 된다. 핀은 central layer에서 자신을 지지하는 body의 group에 속한다. 시각화는 각 부품을 group 색 테두리로 그린다. 그중 linkage optimization이 검사하는 것은 pad가 낀 쌍(`checked_pairs`)과 기존 servo 검사다: proximal pad–distal pad, pad–다른 body의 핀, distal pad–PIP servo, 그리고 `allow_collision_with_mcp_servo: false`일 때 pad–MCP servo. 같은 body(proximal pad–PIP servo, `A`/`A'` 핀; distal pad–`B`/`B'` 핀)와 관절 자신의 핀은 의도된 접합부다. Pad–middle link/coupler는 XZ에서 교차해도 층이 달라 충돌이 아니다(`layer_separated_pairs`로 보고).

## 3. Pareto Objectives — 2개 (확정)

\[
\boxed{
\min_{\boldsymbol\xi}
\begin{bmatrix}
 k_{\max}(\boldsymbol\xi)\\
 180^\circ-\phi_{\mathrm{safe,max}}(\boldsymbol\xi)
\end{bmatrix}
}
\]

| Objective | 최적화 방향 | 의미 |
|---|---|---|
| **1. Transmission** | `k_max` 최소 | 최악의 angular transmission ratio 개선 |
| **2. Full curl** | `φ_safe,max` 최대 | Extension에서 180°에 가능한 한 가깝게 굽힘 |

**Pad 길이는 목적이 아니라 요구조건이다** (§3.2). Pad 길이를 목적으로 두면(이전 3목적) 최적화가 손가락 길이·BC를 경계로 밀거나 링크 배치를 희생해 pad를 늘렸다. 같은 φ에서 k_max는 2목적이 같거나 더 낮았고(run_044 vs run_045), pad는 최소 요구보다 길게 자연히 확보됐다. 목적 목록은 config `optimizer.objectives`이며, `support_length`(pad 길이)와 `flatness`(`k_max/k_min`)는 필요할 때 추가하는 선택 목적이다 (`core/optimization/linkage/objectives.py`).

### 3.1 Transmission

\[
k=\frac{d\phi_{\mathrm{distal}}}{d\theta_{\mathrm{servo}}},\qquad
k_{\max}=\max_{q\in\mathcal Q_{\mathrm{safe}}(\boldsymbol\xi)}|k(q)|.
\]

- `φ_distal`은 proximal에 대한 distal의 연속적인 상대 방향각이다. Extension에서 0°, 반대 방향으로 접히면 180°가 된다.
- 이상적인 **순수 distal moment**에 대해 `|τ_servo| = |k| |M_distal|`이다. Fingertip force 성능과 동일시하지 않는다.
- **별도의 최소 flexion 하한을 도입하지 않으므로**, 각 후보의 실제 safe motion interval에서 `k_max`를 계산하고 그 평가 범위를 반드시 함께 기록한다. 서로 다른 구간의 `k_max`를 직접 비교할 때 생기는 편향은 결과 해석에서 명시한다.

### 3.2 Pad length (요구조건)

\[
L_P = A'_x - (\text{proximal pad 시작 } x),\qquad L_D = B_x + BC - (\text{distal offset}) - B'_x.
\]

§2.1 pad의 **장착 길이**(`a–b`)는 설계변수에서 따라 나오는 값이고, 요구조건으로 다룬다:

- 최소 길이 `minimum_proximal_support_mm`, `minimum_distal_support_mm` (현재 30, 40 mm; 실제 pad/모듈 크기가 정해지면 바꾼다), 또는 pad별 고정 `length_mm`.
- `pad.allow_longer_pad_than_link`: `false`면 pad가 자기 링크(`|OA|`, `BC`)보다 길 수 없다. 자유 길이 pad는 링크 길이로 잘리고 고정 길이 pad는 넘으면 위반이다. 현재 `true`(제한 없음).
- Pad는 간섭 검사에 참여하므로 충돌하면 안전 굽힘 범위가 끝난다(F2에 반영). 목적이 아니라서 pad를 늘릴 압력은 없다.
- `|AB|`(`ab_mm`)와 링크가 pad 블록을 지나가지 않는 길이(`*_usable_length_mm`)는 diagnostic이다.

여기서의 pad 길이는 **최종 usable pad 길이가 아니다**: 장착 나사, 관절 housing, pad 모서리, interface별 형상이 실제 길이를 줄인다. 실제 contact module 형상과 장착 가능성은 이 최적화의 범위 밖이다.

### 3.3 Full curl

\[
\phi_{\mathrm{safe,max}}=
\max\{\phi\in[0^\circ,180^\circ]\mid
\text{extension부터 해당 자세까지 연속적으로 안전하게 도달 가능}\},
\]

\[
f_3=180^\circ-\phi_{\mathrm{safe,max}}.
\]

- `180°`는 **선호하는 목표**이며 hard constraint가 아니다.
- 최소 굽힘으로 **`φ_safe,max >= 150°`를 hard constraint**로 둔다. 거의 굽지 못하는 후보가 짧은 평가 구간 덕분에 작은 `k_max`를 얻어 transmission best가 되는 것을 막는다. 그 이상 필요한 굽힘 범위는 메커니즘 비교에서 판단한다.
- `φ_safe,max`는 동일한 kinematic branch에서 특이점, 역행 금지, 기본 mechanism collision(pad 간섭 포함) 조건을 만족하는 최대 굽힘각이다.
- **Pad 간섭은 안전 범위의 경계다** (`pad_collision`, unsafe 조건 (f)). 첫 간섭 샘플에서 `flexion_stopper_margin_deg` 앞까지가 사용 범위이며, 그 결과 `φ_safe,max >= 150°`이면 feasible로 남고 줄어든 범위는 curl 목적(F2)에 반영된다. 150° 전에 간섭하면 infeasible, 기준 자세의 간섭은 정적 packaging 실패(`pad_collision_mm`)다. **과신전(PIP·MCP)은 linkage RoM 여유일 뿐이므로 그 범위에서는 pad 간섭을 검사하지 않는다.** 180°까지 전 구간 무간섭을 hard constraint로 두지 않는다.
- 기존처럼 servo total travel 상한은 두지 않는다. 실제 필요한 회전량은 diagnostic으로 기록한다.
- 사용 범위는 PIP `−hyperextension_deg`(과신전 stopper, 기본 10°)부터 `min(최초 unsafe 샘플, sweep 끝) − flexion_stopper_margin_deg`(굽힘 stopper 뒤 여유, 기본 5°)와 curl 중 먼저 오는 곳까지다. 따라서 full curl(180°)은 curl 이후 여유만큼 sweep에서 안전이 확인된 경우에만 인정한다. 두 값은 config의 `motion`에서 따로 정한다.
- **구동은 PIP 서보 기준**이다: 굽힘 쪽은 서보 각을 `servo_step_deg`(1°) 간격으로 돌려 four-bar를 닫는다(크랭크 `A'–B`를 돌리고 `B'`를 `A`, `B` 기준 두 원의 교점으로). 과신전은 PIP 요구 조건이므로 PIP를 stopper(`−hyperextension_deg`)까지 정확히 풀어 확인한다; `B`에서의 토글을 넘는 각은 닫히지 않는다. (d)는 서보와 PIP가 함께 도는지(`pip_backoff`)로 판정한다.
- 경계는 탐색 격자(서보 1°) 샘플 기준이며 샘플 사이를 refine하지 않는다. 오차가 Pareto 순위에 영향을 줄 수 있으므로, seed export 전에 0.05° 격자로 재검증한다.

## 4. Constraints

### 4.1 유지할 기구학·형상 제약

- Extension 기준 자세에서 closed-loop가 성립하고 특이점이 없을 것
- 평가 범위에서 continuous branch closure 유지
- **B′ transmission-angle margin ≥ 30°** (일직선 정렬의 양방향 모두 회피)
- DIP 역행 금지, 서보와 PIP가 함께 돌 것 (`pip_backoff`)
- **`φ_safe,max >= 150°`** (OA–BC 최소 굽힘)
- Extension `|OC| = 80–100 mm` (최대 손가락 길이 10 cm)
- 같은 강체의 pivot 간격 `|A–B'|`, `|B'–B| >= 8.0 mm` (두 핀 외곽, 즉 bearing housing이 맞닿는 거리)
- 기존 변수 bounds 및 이미 유도된 해석적 feasibility 조건

### 4.2 유지할 기본 packaging

| 검사 | 현재 기준 |
|---|---|
| 두 servo 몸체 | clearance ≥ 1 mm, 기존 MCP −10°~0° sweep |
| A 핀 외곽 (housing Ø8.0) – `A'`의 virtual clearance (Ø`ap_clearance_diameter_mm` = 7.0) | ≥ 1 mm, 즉 `|AA'| ≥ 4.0 + 3.5 + 1.0` (`allow_collision_pip_servo_to_joint_A: true`면 검사 안 함). Servo 몸체와 A 핀은 검사하지 않는다 |
| B′ 핀 외곽 – PIP servo 몸체 | 검사 안 함: middle link가 다른 Y layer (`allow_collision_pip_servo_to_joint_Bp: false`면 ≥ 1 mm, 사용 범위를 끊는 조건) |
| PIP servo 하단의 OA 기준면 침범 | 기존 `z >= 0` 유지 |
| Pad topology | `A''_x = A'_x + δ_{A''} >= A_x` (`app_offset_mm`, 기본 범위 0–20 mm), `B'_x <= B_x`, `L_P`, `L_D` ≥ `minimum_*_support_mm`(기본 0) |
| Pad – 다른 body (§2.1 쌍) | ≥ 1 mm, 기준 자세(정적) + extension~사용 범위 끝(운동); 과신전은 검사 안 함 |
| Pad – MCP servo (`allow_collision_with_mcp_servo: false`) | ≥ `mcp_servo_clearance_mm`(1 mm), MCP 0° ~ `mcp_flexion_max_deg`(90°), `mcp_step_deg`(1°) 간격 |

- Pin bearing은 McMaster **57155K424** (flanged ball bearing 63: 3 mm shaft, OD 6 mm, 폭 2 mm, flange Ø7.2 × 0.6), 관절마다 양쪽 판에 하나씩. housing 벽은 **알루미늄 가공 1.0 mm**(압입): housing Ø8.0. 간섭 검사에는 이 결과값만 쓴다: 핀 외곽 지름 `pivot_clearance_diameter_mm = 8.0`(housing과 flange 중 큰 쪽), 여유 `body_clearance_mm = 1.0` (config의 `packaging`).
- 실제 Y-layer가 분리된 부품은 XZ 투영 교차만으로 collision 판정하지 않는다 (§2.1 표).
- `A'_x < B''_x` 같은 1D 순서 조건은 두지 않는다. 두 pad의 분리는 collision geometry가 정한다.
- 최소 지지 길이는 장착 요구가 알려졌을 때만 설정한다. Contact module 치수는 주어진 pad requirement다.
- 비용: 각 쌍은 bounding-box gap(하한)으로 먼저 거르고, clearance + `EXACT_BAND_MM`(5 mm) 안의 샘플만 정확히 계산한다. MCP servo는 `O`로부터의 거리 하한으로 거른 뒤 5° coarse 격자(회전 이동량 `reach·Δ/2`로 보정한 하한)로, 그래도 가까운 edge만 1° 전체로 검사한다. 사용 범위가 이미 정해진 샘플(다른 unsafe 이후, curl + stopper 여유 이후)은 계산하지 않는다. 따라서 보고되는 여유는 5 mm까지 정확하고 그 이상은 하한일 수 있다.
- 기존 **middle link–servo** 간섭은 별도 층이라는 전제하에 report-only 상태를 유지한다.
- **Servo 회전량 hard upper bound는 없음.**
- 선택한 fingertip/contact pad의 외곽 형상이 필요한 **fingertip–MCP servo 간섭** 검사는 보류한다(held). 장착선 아래 잠정 두께의 pad 블록의 간섭 검사는 여기서 수행한다.
- `BC <= OA` proxy(`bc_to_oa_ratio_max`)는 이 formulation 이전에 이미 제거되었다. 지지 길이 objective와 무관하므로 되살리지 않는다; curl tip 위치는 held `curl_tip_x_mm`이 다룬다.

### 4.3 Exact curl에 종속된 기존 제약 — 재검토 대상

기존의 `φ = 180°` hard constraint를 완화하므로, 아래 검사를 **그대로 강제해서는 안 된다.**

| 기존 조건 | 현재 방침 |
|---|---|
| `q_PIP + q_DIP = 180°` | Hard constraint 제거 → Objective 3 |
| `−10° ~ curl + 10°` 범위 닫힘 | 과신전 범위(`hyperextension_deg`)와 굽힘 여유(`flexion_stopper_margin_deg`)로 분리해 적용 |
| Curl gap: OA선–BC선 ≥ 20 mm | 비평행 상태에서 기존 infinite-line distance는 부적절. **삭제**, 필요하면 contact interface와 함께 정의 |
| Curl에서 `MCP=0`, `C_x >= O_x` | Candidate-specific endpoint로 재정의하기 전에는 **미적용** |

미적용 항목을 코드에서 조용히 삭제하지 말고, 결과/설정에 **보류 이유**를 남긴다. Contact interface와 연결된 구체적인 clearance 요구는 pad requirement로 둔다.

## 5. 평가와 Seed Selection

### Candidate Evaluation

1. 설계변수로 linkage를 구성하고 extension feasibility를 검사한다.
2. 의도한 굽힘 방향으로 **동일한 closed-loop branch**를 추적한다.
3. 기구학·기본 packaging 제약을 만족하는 `φ_safe,max`를 180°까지 탐색한다.
4. 해당 safe interval에서 `k_max`를 평가하고 `L_P`, `L_D`, `e_curl`을 계산한다.
5. Feasibility, constraint margin, servo travel을 기록한다.

### Exported Seeds

**각 Pareto objective의 best candidate를 반드시 포함한다.**

- **Best transmission:** 최소 `k_max`
- **Best length** (`best_length`): `support_length`를 목적으로 켤 때만, 최대 pad 길이
- **Best curl:** 최대 `φ_safe,max`
- **Balanced:** 검증된 frontier에서 각 objective를 `[0, 1]`로 정규화했을 때 ideal point에 가장 가까운 후보 (objective 간 절충)
- **Intermediate seeds:** 서로 다른 trade-off를 대표하는 non-dominated 후보

best candidate가 겹치면 한 후보가 여러 역할을 갖는다. 가능하면 전체 feasible non-dominated frontier도 보존한다.

기존 `k_max <= 1.05 × k_best` 대표 설계 규칙은 사용하지 않는다 (Balanced와 역할이 겹침).

### 저장할 정보

- Design vector `ξ` 및 geometry
- `k_max`와 **그 평가 motion interval**
- `proximal_support_length_mm`, `distal_support_length_mm`, `support_length_mm`; 비교용 `ab_mm`
- `stop_reason`(첫 unsafe 조건; `pad_collision`이면 `collision_pair`), `pad_clearance_mm`(검사 쌍의 최소 여유)
- `φ_safe,max`, `e_curl`
- Servo travel
- Transmission-angle / packaging 최소 여유
- Feasibility 및 제약 판정 결과

**Formulation 식별.** `run.json`에 `mechanism: linkage`와 `formulation: palmar-pad-v3`을 기록한다(`mechanism`이 없는 run은 linkage). Formulation 기록이 없는 run은 `OA + BC` formulation(`oa-bc`)이고, `support-structure-v1`은 등쪽 deck과 web을 쓰던 이전 모델, `palmar-pad-v2`는 링크가 가리지 않는 pad 길이를 F2로 썼던 실험이다(run_034, 035). `palmar-pad-v1`(run_036–094)은 A 핀과 B′ 핀을 PIP servo 몸체와 검사하던 이전 규칙이다. `run.json`에는 최적화한 목적 목록(`objectives`)도 기록되며, `select_seeds.py`와 `plot_pareto.py`는 formulation이나 목적 목록이 다른 run을 섞지 않는다. 이전 run의 설계 벡터는 `run.py --warm-start`로 재사용할 수 있으며, 새 geometry와 제약으로 전부 다시 평가된다.

### Optimizer 운영 원칙

2026-10-21, 30개 run(nsga2·smsemoa·agemoea × seed 0–9, 200×200)의 결과(미세 grid 재검증 front의 hypervolume, 기준점 `k_max = 2.3`, `φ_safe = 150°`):

- 알고리즘 간 차이(중앙값 HV 11.11–11.43)는 seed 편차 안에 있다. 결과는 seed가 정하는 초기 population에 좌우되고, 같은 seed의 세 알고리즘은 함께 움직인다(seed 7은 세 알고리즘 모두 실패).
- 단일 run은 front 전체에 이르지 못한다(최고 11.98, 30개 병합 12.42). 알고리즘과 seed가 모두 다른 3–4개 run의 병합이 병합 HV의 99% 이상을 낸다.

그래서:

| 용도 | 실행 |
|---|---|
| 개발·디버깅 | 단일 NSGA-II (YAML 기본값 `nsga2`) |
| 최종 결과 | `run.py --portfolio`: 3 algorithms × 1 independent seed, 미세 grid 재검증과 non-dominated 병합(`core/optimization/seeds.py`) |

Portfolio 효과를 검증할 때는 같은 총 평가 예산으로 **NSGA-II × 3 independent seeds**와 **NSGA-II + SMS-EMOA + AGE-MOEA × 1 seed씩**만 비교한다. 그래야 향상이 알고리즘 다양성에서 오는지, 단순한 multi-start에서 오는지 구분된다. 400세대 실험과 새 optimizer 탐색은 보류한다.

이 HV는 **linkage seed 품질**을 평가하는 지표다. 메커니즘 비교의 지표(주어진 contact module 두께와 required ROM에서 유지되는 contact interface 비율)와는 구분한다(6절).

## 6. 메커니즘 비교에서의 위치

```text
주어진 pad requirement (최소 길이, 높이, offset, chamfer, MCP servo clearance)
   ├─ Linkage (four-bar, PIP servo at A')   ← 이 문서
   ├─ Open chain (serial 2R, core/morphology/open_chain)
   └─ RCJ (equal-radius rolling contact, core/morphology/rolling_contact_joint)
                     ↓
같은 pad requirement 아래 transmission · curl · packaging 비교
```

Stage-0 → Stage-1 파이프라인은 쓰지 않는다. Linkage optimization의 Pareto front와 대표 seed는 다음 단계의 초기값이 아니라 **linkage 메커니즘의 대표 성능**으로, open chain·RCJ와 같은 pad requirement 아래 비교된다. 실제 contact module 형상과 contact 성능은 이 비교의 범위 밖이다.

비교 프로토콜(계획): required ROM(90°, 120°, 150°, 180°)마다 메커니즘(idealized 2R, packaged 2R, packaged RCJ, packaged linkage)별로 **독립된 최적화**를 하고(required ROM은 objective 선택 기준이 아니라 constraint), 각각을 portfolio로 병합·재검증한 뒤 module 두께에 따른 contact interface 유지율을 비교한다. Transmission은 각 설계의 최대 safe ROM이 아니라 **공통 required ROM 구간**에서 평가한다. Pad 길이(예: 82.9 mm)만으로 두꺼운 module의 수용을 주장할 수 없으며, module 형상과 전체 motion의 collision 검사가 필요하다.

## 7. 남은 정의 (지금은 범위를 확장하지 않음)

1. `φ_safe,max`의 탐색 종료 조건과 수치 tolerance (특이점·역행·충돌 경계).
2. 후보마다 다른 motion interval에서 측정되는 `k_max`의 비교 및 보고 방식.
3. 기존 `C_x` 조건 중 pad와 무관하게 필요한 **순수 mechanism safety** 조건이 있는지.
4. 현재 servo collision sweep과 각도 샘플링 설정의 유효성.

**성공 기준:** 최종 hand-level 성능을 결정하는 것이 아니라, 메커니즘 비교에 쓸 수 있는 **유효하고 다양하며 재현 가능한 linkage seeds**를 생성하는 것.
