# Drake pad 형상 flat → 2 mm crowned, 형상마다 맞춘 배치 (34차, 실험용, Study 3 본 실험 아님)

Isaac 8차는 flat pad에서 O가 닿는 배치에서 crowned pad를 쟀다. 그래서 crowned에서 Gate 0이 무너졌다. 마찰 1.3, 132개 배치 중 flat/crowned 통과가 O 56/17, R 48/16, U 52/8이었고, 최종 쌍은 R 13, U 5였다. 논문 Table I에서 형상 줄을 뺀 이유다(`report/study3/study3.md`의 형상 항목).

사용자 지시: "각 형상에 맞춘 배치와 조건으로 따로 다시 잽니다." 34차는 Drake pull-out(21차)에서 각 형상을 O 자신의 그 형상 접촉으로 배치하고 다시 잰다. 결과는 report/에 넣지 않는다. 데이터는 `sim/drake/logs`에 둔다.

## 34차 사전 기록 (2026-11-25, O만으로 보정, R·U는 gate만 확인, 재설계 손의 D와 E를 보기 전)

### 구현 (`sim/drake/pad_shape.py`)

- **pad:** 모든 손의 두 pad를 상자와 같은 외곽(길이, 폭, 두께, 붙는 면)의 볼록 prism으로 바꾼다.
  - flat은 상자 그대로를 볼록 mesh로 만든다.
  - crowned는 접촉 면이 pad 길이 방향의 원호다. 가운데는 두께 그대로이고 양 끝은 2 mm 높다(sagitta 2 mm, chord 24개). `sim/isaaclab` `contact_interface.CROWNED`와 같은 단면이다.
  - 두 형상이 같은 볼록 mesh 충돌 경로를 지난다(Isaac과 같은 이유). `two_pad_drake._build` 안의 `pydrake.geometry.Box`를 대신해 만든다. `two_pad_drake.py`는 고치지 않았다.
  - Drake point contact가 볼록 mesh와 원기둥의 관통 쌍을 찾는지 먼저 확인했다.
- **과제:** 21차 조건 A.
  - point contact, K_Q 5, setpoint q0 + 5°. 원기둥을 잡은 채 40 + 30 step 닫고, 놓고 120 step 안정화한다.
  - Gate 0 뒤 네 방향 하중 램프. 방향마다 안정화된 상태에서 새 simulator로 가른다.
  - **μ 1.3 하나.** interface 변화가 형상이므로 마찰 쌍 구조는 쓰지 않는다.
- **Gate 0:** 두 pad ≥ 0.2 N, 정지(10 step 속도 < 2 mm/s, 회전 < 0.1 rad/s), 그리고 두 접촉점이 각 pad 면 위(pad 길이 안, ±0.5 mm). 21차의 법선 각 검사(두 접촉 법선의 각이 굽힘과 4° 안)는 crowned pad에서 접촉 법선이 면 법선과 다르므로 이 "면 위" 검사로 바꿨다. 두 형상에 같은 규칙을 쓴다.

### 배치와 짝 (형상마다 O의 접촉으로)

- **배치:** ρ 10, 12.5, 15, 17.5, 20 mm × q0 40–115°(5° 간격). O에서 만든다.
  - flat: two_pad_grasps.json의 규칙(중심 깊이 face + ρ, q0에서 distal 면 선에서 ρ).
  - crowned: 원기둥이 두 원호에 바깥에서 닿는 중심. 두 원호 중심에서 각각 R + ρ(R = (L/2)²/(2c) + c/2)인 두 원의 교점 중 손바닥 쪽이다.
  - 두 형상 모두 두 접촉점이 O의 pad 길이 안에 있는 (ρ, q0)만 남긴다. 80개 모두 남았다.
  - crowned 중심은 flat 중심에서 x로 0.1–0.6 mm, 링크 쪽으로 0.7–1.7 mm 옮겨진다. 8차가 flat 배치를 그대로 쓰면서 놓친 것이 이 차이다.
- **짝:** (ρ, q0, 방향). 같은 반지름, 같은 명령 q0 + 5°이고, 원기둥은 그 형상의 O의 pad 둘이 q0에서 닿는 곳에 둔다. 위치는 형상마다 다르지만 "같은 물체를 같은 명령으로 쥘 때 O가 실제로 닿는 곳"이라는 정의는 같다. 위치를 같게 하면 한 형상은 닿지 않는다(8차). 명령을 바꾸면 조건 A가 아니다.
- **손마다:** O에서 만든 배치를 각 손의 proximal-face 좌표로 준다(21차와 같다: 중심 깊이 face_H + ρ). 처음 R·U 확인에서 O의 world 깊이를 그대로 줘서 7 mm pad 손이 원기둥 안으로 4 mm 닫혔다(관통 최대 6.6 mm). 이 규칙으로 고친 뒤 다시 확인했다. O의 결과는 비트 단위로 같다.
- **D(H) = F_limit(crowned) − F_limit(flat)**, 같은 (ρ, q0, 방향). E(H) = 쌍들의 |D(H) − D(O)| / |D(O)| 중앙값.

### O만의 보정 (이 기록 전)

- **Gate 0 (μ 1.3, 80개 배치):** flat 34/80(q0 ≥ 85° 34/35, 그 아래 0/45; 21차와 같은 양상), crowned 41/80(q0 ≥ 85° 29/35, 그 아래 12/45). solver 0, 면 밖 접촉 0, 관통 최대 0.70 / 0.55 mm.
- **제외 (O만으로, 미리 정함, 첫 이유로 센다):** solver 실패, flat Gate 0 실패, crowned Gate 0 실패, 관통(안정화 끝 pad 법선력 최대 / 2.5 kN/m) > 1.5 mm, 한계 없음, **|D(O)| < 0.6 N**.
- **|D(O)| 바닥의 선택:** O 명령 +0.05°(tag e34n)를 잡음으로 보았다. 중앙값은 바닥에 거의 무관하게 약 4%다. 이 과제의 쥐기 자체가 0.05° 명령 변화에 그만큼 민감하다(31차 pull-out의 1.1%보다 크다).

  | |D(O)| 바닥 | O 쌍 | 잡음 중앙 / 75분위 |
  |---|---|---|
  | 0.2 N(3 step) | 98 | 4.35 / 14.3% |
  | 0.4 N | 71 | 4.35 / 10.6% |
  | **0.6 N(9 step)** | **59** | **4.55 / 9.3%** |
  | 1.0 N | 40 | 4.10 / 5.6% |

  40쌍 이상을 남기면서 75분위를 10% 아래로 두는 **0.6 N**을 주 결과로 정한다. 0.2와 1.0 N을 민감도로 보고한다.
- **0.6 N에서 O:** 59쌍(방향별 +x 23, +x−z 12, −z 13, −x 11). D(O)는 −11.6 ~ +33.4 N으로 부호가 섞인다(0.2 N 바닥에서 98쌍 중 40개 음수). 형상의 효과는 배치와 방향에 따라 F_limit을 올리기도 하고 내리기도 한다.
- **R·U gate 확인 (D, E는 보지 않음):**
  - R: Gate 0 flat 30/80, crowned 36/80, solver 0, 관통 최대 0.48 / 0.39 mm.
  - U: flat 33/80, crowned 31/80, solver 0, 관통 최대 1.52 / 1.53 mm(각 1개 > 1.5).

### 지표와 판정 (사전, 30b·33차와 같은 형식)

- **E(H):** O의 59쌍 중 H가 두 형상 모두 Gate 0을 지나고 관통 ≤ 1.5 mm이고 한계가 있는 쌍에서 |D(H) − D(O)| / |D(O)|의 중앙값.
  - H의 Gate 0 실패, over-pressed, solver 실패는 따로 세고 E에서 뺀다. 민감도로 무한대로 둔 E와 ρ를 보고한다.
  - 한계 없음은 뺀다(21차와 같음).
- **판정 1:** E_R < E_U. 5, 9 mm 쌍도 보고한다.
- **판정 2:** ρ(E, Δ_contact) > 0이고 순열 p < 0.05(손 18개, 2만 번, seed 0, 단측; "순열 p").
- **판정 3:** ρ(E, Δ_skel)과 p를 보고하고 비교한다.
- **궤적:** fig4_sync의 pull-out deviation과 같은 구조. 각 형상에서 R − O와 U − O의 held load(하중 방향 변위까지 버틴 최대 하중)를 0–5 mm 격자에서 O의 쌍에 대해 구하고, 중앙값과 사분위를 낸다(키 `R_flat`, `U_flat`, `R_crowned`, `U_crowned`). 최대(절댓값)와 끝을 보고한다.
- **해석:** 판정 1·2가 성립하면 형상 interface에서도 재설계 오차가 Δ_contact를 따른다. 8차의 "순서는 반복하지만 둘을 가르지 못함"을 손 18개로 다시 본다. 지표가 안정하고 gate가 유효한데 성립하지 않으면 그대로 보고한다.
- **미리 알 수 있는 것:** 잡음 바닥이 다른 과제보다 크다(중앙 4.6%, 75분위 9.3%). D(O)의 부호가 쌍마다 달라 E의 분모가 작은 쌍이 있다(0.6 N 바닥이 그것을 막는다).

### 순서와 계산

이 기록을 커밋한 뒤 나머지 16개 손을 worker 6으로 돌린다(손 둘에 약 2분, 합계 약 17분 예상). 보정 wall time: O 55 s, 잡음 57 s, R·U 124 s(잘못된 깊이로 한 1차 260 s).

### 명령

```
cd sim/drake
uv run python pad_shape.py place
uv run python pad_shape.py run --hands O --tag e34
uv run python pad_shape.py run --hands O --tag e34n --delta_deg 0.05
uv run python pad_shape.py run --hands R U ... --tag e34
uv run python pad_shape.py analyze --tag e34 --noise_tag e34n --out logs/e34_shape.json [--d_min 0.2 | 1.0]
```
