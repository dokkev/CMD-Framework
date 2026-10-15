# CoDe Hand — Stage-0: Linkage Seed Generation

> **상태:** 설계 Master Document (2026-10-15)  
> **역할:** Stage-1의 초기값으로 사용할 다양한 linkage seed 생성  
> **원칙:** Stage-0에서 최종 hand design을 결정하지 않는다.

## 1. 목적과 범위

Stage-0은 **기구학적으로 유효하고 서로 다른 장점을 가진 finger linkage 후보들을 생성하는 seed generator**다. 특정 contact interface나 hand-level opposition task에 최적화된 최종 손가락을 선정하는 단계가 아니다.

**포함**
- Linkage kinematics, transmission, flexion capability
- OA/BC structural length
- 기본 servo/housing 및 mechanism packaging
- Pareto frontier와 대표 seed 추출

**제외 (Stage-1 이후)**
- Proximal/distal contact interface의 형상, 두께, 길이, mounting
- Pad–pad collision, contact mechanics 및 contact coverage
- Hand layout, thumb/finger opposition motion
- Contact morphology optimization 또는 co-design

Stage-1에서는 Stage-0 seed를 **초기값으로 재사용하며 linkage를 다시 최적화할 수도 있다.**

## 2. 설계변수와 기준 형상

기존 extension 기준의 XZ 평면 좌표와 linkage 정의를 유지한다. `OA`는 proximal structural segment, `BC`는 distal structural segment이며, extension에서 `BC`와 `OA`는 같은 방향으로 평행하다.

\[
\boldsymbol\xi=[A_x,BC,B_x,B_z,A'_x,A'_z,B'_x,B'_z,\alpha_{\mathrm{servo}}]^{\mathsf T}.
\]

| 설계변수 | 현재 탐색 범위 | 비고 |
|---|---|---|
| `A_x = OA` | 5–120 mm | Proximal 길이. 사실상 unbound; servo 간섭 제약이 결정 |
| `BC` | 30–50 mm | Distal 길이 (middle + distal phalanx) |
| `B_x` | 40–100 mm | Extension 기준 |
| `B_z` | −10–10 mm | Extension 기준 |
| `A'_x` | 20–100 mm | |
| `A'_z` | 0–20 mm | OA 기준면 제약의 closed form으로 탐색 시 `>= 9.5 mm`로 축소 |
| `B'_x` | −50–200 mm | 사실상 unbound |
| `B'_z` | −60–80 mm | 사실상 unbound |
| Servo tilt | 0–45° | |

현재 0.1 격자: 거리·좌표 **0.1 mm**, 각도 **0.1°**. 기존 구현의 변수 정의와 이산화 방식을 재사용한다.

구현: `core/optimization/stage0` (config, design, evaluate, run), 실행 `scripts/optimize/stage0/run.py`, 검증 `scripts/optimize/stage0/select_seeds.py`, 시각화 `scripts/optimize/stage0/plot_pareto.py`, `scripts/animate_finger.py`. 범위와 하드웨어는 `scripts/optimize/stage0/run.py`의 `CONFIG`(bounds, `Packaging`)가 기준이다.

Extension에서 `O=(0,0)`, `C=(B_x+BC,B_z)`이면:

\[
80\le |OC|_{\mathrm{extension}}=\sqrt{(B_x+BC)^2+B_z^2}\le100\quad[\mathrm{mm}].
\]

`BC >= 30`인 현재 범위에서는 이 제약으로 `B_x <= 70 mm`를 해석적으로 얻을 수 있다. 기존 bounds를 즉시 변경할 필요는 없으며, 필요하면 단순 탐색 pruning에 활용한다.

## 3. Pareto Objectives — 3개

\[
\boxed{
\min_{\boldsymbol\xi}
\begin{bmatrix}
 k_{\max}(\boldsymbol\xi)\\
 -(OA+BC)\\
 180^\circ-\phi_{\mathrm{safe,max}}(\boldsymbol\xi)
\end{bmatrix}
}
\]

| Objective | 최적화 방향 | 의미 |
|---|---|---|
| **1. Transmission** | `k_max` 최소 | 최악의 angular transmission ratio 개선 |
| **2. Structural length** | `OA + BC` 최대 | Proximal/distal contact-support **capacity** 확보 |
| **3. Full curl** | `φ_safe,max` 최대 | Extension에서 180°에 가능한 한 가깝게 굽힘 |

### 3.1 Transmission

\[
k=\frac{d\phi_{\mathrm{distal}}}{d\theta_{\mathrm{servo}}},\qquad
k_{\max}=\max_{q\in\mathcal Q_{\mathrm{safe}}(\boldsymbol\xi)}|k(q)|.
\]

- `φ_distal`은 proximal에 대한 distal의 연속적인 상대 방향각이다. Extension에서 0°, 반대 방향으로 접히면 180°가 된다.
- 이상적인 **순수 distal moment**에 대해 `|τ_servo| = |k| |M_distal|`이다. Fingertip force 성능과 동일시하지 않는다.
- **별도의 최소 flexion 하한을 도입하지 않으므로**, 각 후보의 실제 safe motion interval에서 `k_max`를 계산하고 그 평가 범위를 반드시 함께 기록한다. 서로 다른 구간의 `k_max`를 직접 비교할 때 생기는 편향은 결과 해석에서 명시한다.

### 3.2 Structural length

\[
f_2=-(OA+BC).
\]

RCM joint 주변보다 contact interface를 설치하기 유리한 **두 structural segment의 총 길이**를 최대화한다. 실제 pad usable length, mounting margin, pad geometry를 Stage-0에 추가하지 않는다.

논문에서 objective를 직관적으로 표현하기 위한 형태다. `|OC|`가 상한에 붙는 경우 기존의 `|AB|` 최소화와 거의 같은 방향이다.

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
- 최소 굽힘으로 **`φ_safe,max >= 150°`를 hard constraint**로 둔다. 거의 굽지 못하는 후보가 짧은 평가 구간 덕분에 작은 `k_max`를 얻어 transmission best가 되는 것을 막는다. 그 이상 필요한 굽힘 범위는 Stage-1에서 판단한다.
- `φ_safe,max`는 동일한 kinematic branch에서 특이점, 역행 금지, 기본 mechanism collision 조건을 만족하는 최대 굽힘각이다.
- 기존처럼 servo total travel 상한은 두지 않는다. 실제 필요한 회전량은 diagnostic으로 기록한다.

## 4. Constraints

### 4.1 유지할 기구학·형상 제약

- Extension 기준 자세에서 closed-loop가 성립하고 특이점이 없을 것
- 평가 범위에서 continuous branch closure 유지
- **B′ transmission-angle margin ≥ 30°** (일직선 정렬의 양방향 모두 회피)
- DIP 및 구동 링크(`A'–B`)의 역행 금지 — 기존 판정 방식 재사용
- **`φ_safe,max >= 150°`** (OA–BC 최소 굽힘)
- Extension `|OC| = 80–100 mm` (최대 손가락 길이 10 cm)
- **`BC <= OA`** (`bc_to_oa_ratio_max = 1`): 접었을 때 손끝이 O 뒤로 크게 넘어가지 않도록 하는 단순 대리 제약. 정확한 `C_x >= O_x`는 4.3에서 보류
- 같은 강체의 pivot 간격 `|A–B'|`, `|B'–B| >= 6.5 mm` (두 bushing housing이 맞닿는 거리)
- 기존 변수 bounds 및 이미 유도된 해석적 feasibility 조건

### 4.2 유지할 기본 packaging

| 검사 | 현재 기준 |
|---|---|
| 두 servo 몸체 | clearance ≥ 1 mm, 기존 MCP −10°~0° sweep |
| A housing (Ø6.5) – PIP servo | ≥ 1 mm |
| B′ housing – PIP servo | ≥ 1 mm, 평가 motion 전 범위 |
| PIP servo 하단의 OA 기준면 침범 | 기존 `z >= 0` 유지 |

- Pin bushing은 McMaster **2706T72** (3 mm shaft, OD 4.5 mm, 길이 5 mm, flange Ø7.5 × 0.75), housing 벽은 **알루미늄 가공 1.0 mm**: housing Ø6.5. 치수는 모두 `Packaging`에서 바꾼다.
- 실제 Y-layer가 분리된 부품은 XZ 투영 교차만으로 collision 판정하지 않는다.
- 기존 **middle link–servo** 간섭은 별도 층이라는 전제하에 report-only 상태를 유지한다.
- **Servo 회전량 hard upper bound는 없음.**
- 선택한 fingertip/contact pad의 외곽 형상이 필요한 **fingertip–MCP servo 간섭** 검사는 Stage-1에서 수행한다. Mechanism 자체의 간섭 검사는 Stage-0에 남긴다.

### 4.3 Exact curl에 종속된 기존 제약 — 재검토 대상

기존의 `φ = 180°` hard constraint를 완화하므로, 아래 검사를 **그대로 강제해서는 안 된다.**

| 기존 조건 | 현재 방침 |
|---|---|
| `q_PIP + q_DIP = 180°` | Hard constraint 제거 → Objective 3 |
| `−10° ~ curl + 10°` 범위 닫힘 | Extension 쪽 여유는 유지 가능한지 확인; `curl+10°`는 endpoint 정의 후 재검토 |
| Curl gap: OA선–BC선 ≥ 20 mm | 비평행 상태에서 기존 infinite-line distance는 부적절. **Stage-0에서 삭제**, contact interface와 함께 Stage-1에서 정의 |
| Curl에서 `MCP=0`, `C_x >= O_x` | Candidate-specific endpoint로 재정의하기 전에는 **미적용** |

미적용 항목을 코드에서 조용히 삭제하지 말고, Stage-0 결과/설정에 **보류 이유**를 남긴다. Contact interface와 연결된 구체적인 clearance 요구는 Stage-1로 넘긴다.

## 5. 평가와 Seed Selection

### Candidate Evaluation

1. 설계변수로 linkage를 구성하고 extension feasibility를 검사한다.
2. 의도한 굽힘 방향으로 **동일한 closed-loop branch**를 추적한다.
3. 기구학·기본 packaging 제약을 만족하는 `φ_safe,max`를 180°까지 탐색한다.
4. 해당 safe interval에서 `k_max`를 평가하고 `OA + BC`, `e_curl`을 계산한다.
5. Feasibility, constraint margin, servo travel을 기록한다.

### Exported Seeds

**각 Pareto objective의 best candidate를 반드시 포함한다.**

- **Best transmission:** 최소 `k_max`
- **Best length:** 최대 `OA + BC`
- **Best curl:** 최대 `φ_safe,max`
- **Balanced:** 검증된 frontier에서 각 objective를 `[0, 1]`로 정규화했을 때 ideal point에 가장 가까운 후보 (세 objective의 절충)
- **Intermediate seeds:** 서로 다른 trade-off를 대표하는 non-dominated 후보

세 best candidate가 같으면 중복을 제거한다. 가능하면 전체 feasible non-dominated frontier도 보존한다.

기존 `k_max <= 1.05 × k_best` 규칙은 **대표 설계 하나를 표시하는 옵션**으로만 남기며, 다른 seed를 버리는 필터로 사용하지 않는다.

### 저장할 정보

- Design vector `ξ` 및 geometry
- `k_max`와 **그 평가 motion interval**
- `OA`, `BC`, `OA + BC`
- `φ_safe,max`, `e_curl`
- Servo travel
- Transmission-angle / packaging 최소 여유
- Feasibility 및 제약 판정 결과

## 6. Stage-1과의 연결

```text
Stage-0: Linkage Seed Generation
   └─ Pareto best (transmission / length / curl)
   └─ Diverse intermediate feasible seeds
                     ↓
Stage-1: Contact-Aware Hand Optimization
   └─ Contact interface geometry 입력
   └─ Hand layout + opposition motion 평가
   └─ 필요한 경우 linkage parameters 재최적화
                     ↓
Future: Contact Morphology–Mechanism Co-Design
```

Stage-0 결과가 Stage-1의 linkage geometry를 **고정하지는 않는다**. 기존 linkage model, kinematics, 결과를 재사용하되 Stage-0에 미리 pad 관련 데이터 모델이나 optimizer를 만들지 않는다.

## 7. 남은 정의 (지금은 범위를 확장하지 않음)

1. `φ_safe,max`의 탐색 종료 조건과 수치 tolerance (특이점·역행·충돌 경계).
2. 후보마다 다른 motion interval에서 측정되는 `k_max`의 비교 및 보고 방식.
3. 기존 `C_x` 조건 중 pad와 무관하게 필요한 **순수 mechanism safety** 조건이 있는지.
4. 현재 servo collision sweep과 각도 샘플링 설정의 유효성.

**Stage-0 성공 기준:** 최종 hand-level 성능을 결정하는 것이 아니라, Stage-1이 활용할 수 있는 **유효하고 다양하며 재현 가능한 linkage seeds**를 생성하는 것.
