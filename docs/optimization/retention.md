# Contact-Interface Retention — 메커니즘 비교

> **상태:** 구현 (2026-10-21, formulation `contact-retention-v1`)
> **대상:** idealized 2R, packaged 2R, packaged RCJ (linkage는 다음 단계)
> **구현:** `core/optimization/retention` (공통), `core/optimization/open_chain`, `core/optimization/rolling_contact_joint`

## 1. 지표

각 segment에 고정된 nominal envelope(두께 `h`, nominal 길이)의 contact module이 붙는다. Nominal 길이는 구조 링크 길이와 독립이고 링크보다 길 수 있다. 설계가 extension부터 required ROM까지 **연속으로** 충돌 없이 유지하는 module의 XZ 단면적을 nominal 면적으로 나눈 것이 retention이다.

- Module은 linkage pad의 4-vertex block `a-b-d-c`(chamfer 0)다. 바깥 edge `a-c`는 고정하고, 관절 쪽 vertex `b`(장착선)와 `d`(palmar 면)의 local X만 설계변수로 움직인다(`proximal_line_mm`, `proximal_palmar_mm`, `distal_line_mm`, `distal_palmar_mm`, 각각 `[0, nominal]`). 그래서 남은 module은 사다리꼴이고, `r = (l_b + l_d) / (2 L_nominal)`이다.
- Proximal module의 바깥 edge는 `Packaging.proximal_pad_start_mm`(MCP servo shaft disk + clearance)에, distal module의 바깥 edge는 distal segment 끝(2R: `EE`, RCJ: `C`)에서 offset만큼 안쪽에 둔다. 메커니즘이 MCP servo를 모델링하든 안 하든 같다.
- 목적함수는 `r_p`, `r_d`의 최대화 두 개다. Front의 통합 지표는 `max min(r_p, r_d)`이고, 그 후보가 `maximin` seed다(`front_retention`).
- Required ROM(distal의 proximal 대비 굴곡)은 90°, 120°, 150°, 180° 각각 **독립된 study의 hard constraint**다. 0°부터 φ_req까지 `rom_step`마다, MCP는 0–90°를 sweep하며 검사한 모든 쌍의 여유가 제약이다. `select_seeds.py`는 0.05° grid로 다시 검증한다.
- Transmission(servo torque / distal moment)은 required ROM 구간의 diagnostic일 뿐이다. 2R은 1, RCJ는 2(`dφ/dθ`)이고, 공통 외부 하중이 정해지기 전에는 torque 제약을 두지 않는다.

## 2. 메커니즘

| | 설계변수 | 고체 |
|---|---|---|
| Idealized 2R | `|OA|`, `|A EE|` | module, segment centreline, `O`·`A` pin envelope |
| Packaged 2R | 같음 | 위 + XM335 at `O` (palm, MCP sweep), at `A` (proximal body, rear `joint_servo_rear`) |
| RCJ | `A_x`, `A_z`, `r`, carrier angle, `BC` | module, centreline, roller 2개(`r`), carrier, XM335 at `A`, MCP servo, MCP pin |

- 2R의 servo output axis는 관절축과 일치하고, housing은 parent body에 고정된다. Idealized와 packaged는 housing 유무만 다르다.
- 서로 다른 body의 고체는 body clearance를 유지한다. Module과 MCP servo 사이는 MCP clearance를 유지한다. Module은 자기 body의 hardware(servo, roller)와 맞닿을 수는 있어도 겹칠 수는 없다(`OWN_BODY_GAP_MM`). 관절 자체는 검사에서 뺀다: roller끼리의 contact, roller 안의 carrier, 자기 관절 pin에 붙은 centreline.
- RCJ는 이상적인 no-slip kinematics(`φ = 2θ`)를 따른다. Cable routing, bearing 폭, Y-layer 치수는 아직 정의되지 않아 모델링하지 않고, 모든 고체를 한 평면에서 검사한다. 모든 RCJ 후보에는 `provisional: cable routing unverified`가 붙는다.

## 3. 실행

공통 조건(`scripts/optimize/contact_retention.yaml`)을 메커니즘별 `config.yaml`이 `base:`로 상속하고, 각 파일에는 탐색 범위만 있다.

```text
python scripts/optimize/run.py scripts/optimize/open_chain_packaged/config.yaml
python scripts/optimize/run.py scripts/optimize/rolling_contact_joint/config.yaml
python scripts/optimize/select_seeds.py output/rolling_contact_joint/run_001
```

## 4. 보류·잠정

- Nominal module 크기(`h = 5`, 45 / 45 mm)와 `joint_servo_rear = +Z`(servo 몸통을 등쪽으로 세움)는 잠정값이다.
- 2R의 module 장착선은 segment centreline이다(관절축이 장착선 위). 그래서 2R은 180°에서 module끼리 겹쳐 도달하지 못한다.
- Linkage(CoDe)의 retention evaluator는 아직 없다.
