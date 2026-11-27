# PR과 MR: 재설계는 무엇을 기준으로 접촉면을 지켜야 하는가 (2026-11-27)

**상태:** 탐색 실험 (playground). 논문과 SSoT에는 아직 넣지 않았다.
- 사전 등록, 원자료, 커밋: `docs/experimental/pr_mr.md`
- 시도 9 (7b′ 손 격자): `docs/experimental/rl_pip_only.md`
- 이 보고서는 결론과 핵심 숫자만 모은다.

## 1. 질문

논문 C2는 재설계가 접촉 Jacobian, 곧 proximal pad 면에서 본 distal 접촉점의 경로(Δ_contact)를 지켜야 한다고 말한다. 그런데 이 경로에는 상수 하나가 들어 있다. 초기 접촉 위치를 무엇에 맞추느냐다.

- **PR (proximal-face referenced):** proximal pad 면에 맞춘다. 기존 R, 논문의 CoDe 재설계다. 지표는 Δ_contact.
- **MR (MCP referenced):** MCP 축과 손바닥(F₁)에 맞춘다. 지표는 Δ_MCP다. MCP 회전은 F₁을 강체로 돌리므로, distal pad 접촉에 한해 MCP를 써도 이 값은 변하지 않는다.

## 2. 이론: 두 개의 하한

**하한 1.** 두 기준은 proximal 면의 깊이 변화 Δt만큼 어긋나 있다. 그래서 모든 q에서 Δ_contact + Δ_MCP ≥ |Δt|다.

**하한 2.** distal 접촉점은 손끝 C에 distal 면 깊이를 더한 점이다. 그래서 Δ_MCP + Δ_skel ≥ |Δd|이고, Δd는 distal 면의 깊이 변화다. 손바닥에서 볼 때 접촉점과 손끝 골격을 함께 지킬 수 없다. 논문의 "접촉 대 골격" 상충이 손바닥 기준에서 다시 나타난 것이다.

**두 하한에서 나오는 설계 규칙.** Δt를 줄이면 두 pad 기준을 함께 지킬 수 있다.
- 논문은 pivot A를 링크선 위에 두었고, 그 결과 proximal mount가 3 mm에 고정되어 Δt = h − 3이 되었다.
- A를 올리면 pin housing이 위로 가고, proximal pad를 더 얕게(mount 0까지) 붙일 수 있다.
- 다만 mount ≥ 0이므로 Δt ≥ max(0, h − 6)이다.

## 3. 설계 (CFL, 120°, 3 → t mm, C̄_pad ≥ 0.70)

| 손 | t | Δt | Δ_contact | Δ_MCP | Δ_skel | C̄_pad |
|---|---|---|---|---|---|---|
| PR | 7 | 4 | 0.86 | 4.65 | 7.67 | 0.725 |
| U (coverage만) | 7 | 4 | 3.91 | 4.26 | 3.38 | 0.711 |
| MR | 7 | 3 | 3.26 | **0.54** | 4.29 | 0.700 |
| **MRM** (둘 다) | 7 | **1** | **1.07** | **1.06** | 4.78 | **0.735** |
| MRS (셋 다) | 7 | 1 | 2.29 | 2.02 | 2.31 | 0.701 |
| MRM5 | 5 | 0 | 0.28 | 0.28 | 2.02 | 0.763 |
| MRM9 | 9 | 3 | 2.43 | 2.39 | 7.00 | 0.707 |

(mm. 스크립트는 `design/scripts/study/exploratory/e20_*`, `e21_*`, `e22_*`.)
- MRM은 두 기준을 약 1 mm로 지키면서 coverage는 PR보다 높다.
- 5 mm에서는 두 기준을 거의 완벽하게 지킨다.
- 이런 설계는 A와 B를 링크선 밖으로 옮겨야 나온다. "pad와 골격이 펴진 자세에서 정렬"이라는 제약은 사용자가 풀어도 된다고 했다.

## 4. 결과

### 4.1 Drake: 물체를 무엇에 고정했는가

논문의 손끝 과제는 물체를 각 손의 pad 면 좌표에 놓는다. 즉 처음부터 PR 기준이다. 같은 물체를 O의 손바닥 좌표(F₁)에 놓고 다시 쟀다.

| 과제 (E) | 기준 | PR | U | MR | MRM |
|---|---|---|---|---|---|
| dial y | face | 9.3% | 26.5% | 18.5% | 8.0% |
| dial y | F₁ | 23.4% | 7.7% | 6.8% | 9.2% |
| pivot | face | 5.4% | 22.2% | 18.4% | 7.4% |
| pivot | F₁ | 11.7% | 7.8% | 2.8% | 3.0% |
| pull-out (point) | face | 19.4% | 71.7% | 17.3% | **8.5%** |
| pull-out (hydro) | face | 16.7% | 254% | 41.6% | **12.8%** |

- **pad 면 기준이면 오차는 Δ_contact를 따른다.** 논문 결과 그대로이고, 손 29개에서 ρ 0.72와 0.61이다.
- **F₁ 기준이면 Δ_MCP를 따른다.**
  - 26–29개 손에서 ρ는 dial y 0.84, pivot 0.77이다.
  - Δ_MCP와 Δ_skel을 떼어 놓은 손(e21)을 넣으면, pivot은 Δ_MCP만 따른다: partial 0.68, Δ_skel은 −0.05.
  - 손끝 가장자리 접촉인 dial y는 Δ_skel도 따른다.
- **MRM은 두 기준 모두에서 오차가 9% 이하다.** pull-out에서는 PR보다 작다: 7 mm에서 8.5 대 19.4%, 9 mm에서 8.0 대 14.8%.

### 4.2 RL: O 정책의 zero-shot 이전 (O 대비 비율)

| 과제 | PR | U | MR | MRM | MRS |
|---|---|---|---|---|---|
| 7b′ (pad 위 굴리기) | 0.67 | 0.07 | 0.06 | 0.65 | 0.66 |
| cube 회전 (rotation) | 0.75 | 0.70 | 0.77 | **0.94** | 0.93 |
| cube full turn | 0.70 | 0.80 | 0.73 | **0.98** | 1.01 |
| relay dial, face | 0.57 | **0.70** | 0.52 | 0.57 | 0.72 |
| relay dial, F₁ | 0.69 | **0.87** | 0.63 | 0.66 | 0.77 |

- O는 기록된 값을 ±4.5% 안에서 재현했다. 평가 잡음은 0.01–0.08이다.
- **cube 이전을 정하는 것은 proximal 면 이동량 Δt다.**
  - 손 14개(5/7/9 mm)에서 ρ(비율, Δt)는 −0.98이다. Δ_contact −0.60, Δ_MCP −0.76, Δ_skel −0.66보다 강하다.
  - Δt 0–1인 손(MRM5 0.99, MR5 0.96, MRM 0.94)은 정책을 거의 그대로 물려받는다.
  - 기존 7 mm 재설계는 모두 Δt 4이고 0.70–0.77이다.
- **7b′에서도 Δt가 보호 효과를 낸다.**
  - MRS는 Δ_contact 2.29로 시도 9의 Dc50(1.7)을 넘지만 0.66을 낸다.
  - 반면 Δt 4인 MRMbz(Δ_contact 2.08)는 0.15로 무너진다.
  - 시도 9의 격자는 모든 손의 Δt가 같아서 이 효과를 볼 수 없었다.
- **relay dial(손끝 끝 쓰다듬기)은 예외다.** 어느 기준으로 놓아도 Δ_skel이 가장 작은 U가 가장 잘 넘겨받는다. 손끝 끝 접촉은 골격 손끝 경로를 따른다. 범위 밖으로 둬야 한다.

### 4.3 사전 등록 대비

**맞은 것**
- MR 설계 예측 3개 모두
- Drake F₁ 기준에서 Δ_MCP를 따른다 (dial y, pivot)
- pad 면 기준에서는 논문 결과가 유지된다
- 7b′에서 MRM ≈ PR
- cube에서 Δt 용량-반응 3개 모두
- MRS의 pull-out 악화

**틀린 것**
- z disk의 F₁ 기준 (과압 탈락)
- cube "MR > PR". cube가 손바닥 기준이라는 전제가 틀렸고, 결과를 보기 전에 정정했다.
- MRM이 두 과제 모두에서 O의 0.1 안
- relay F₁에서 MR/MRM 우위
- MRS의 relay ≥ U
- MRS의 7b′ < 0.5

## 5. 논문에 주는 의미 (셀프리뷰)

1. **지금 논문 결과는 물체를 pad 면에 맞춰 놓은 설정에 의존한다.** 리뷰어가 "환경을 손마다 Δt만큼 옮긴 결과"라고 지적할 수 있다. 정직하게 쓰면 다음과 같다. 물체가 proximal 면 위에 놓이는 grasp에서는 그 설정이 자연스럽다. 손바닥에 고정된 물체라면 같은 접촉점을 손바닥 기준(Δ_MCP)으로 봐야 한다.
2. **더 강한 설계 원칙이 보인다.** proximal 접촉면을 손바닥에 대해 제자리에 두고(Δt → 0), 그다음 접촉 Jacobian을 지킨다. 이렇게 하면 두 기준을 함께 지킨다(하한 1). 그 결과:
   - Drake 다섯 조건에서 가장 좋거나 거의 가장 좋다.
   - RL cube에서 0.94–0.99이고, 7b′에서는 PR과 같다.
   - coverage 손해도 없다.
   지금 C2("접촉 Jacobian을 지킨다")를 "접촉면의 자리와 운동을 지킨다"로 넓히는 후보다.
3. **범위의 한계도 분명해졌다.** 손끝 끝 접촉(relay)은 골격 손끝 경로를 따르고, 하한 2 때문에 pad 기준과 동시에 지킬 수 없다.
4. **넣기 전에 확인할 것:**
   - MRM의 proximal mount 0 (A pin housing을 링크선 위로 올림)이 실제로 패키징되는지 CAD로 본다.
   - Study 2의 설계 공간에 a_z와 b_z를 넣어 PR 쪽 연구와 같은 설정으로 다시 탐색한다.
   - RL cube는 학습 seed가 하나뿐이다(model_1497). 두 번째 seed로 재현해야 한다.
5. **넣지 말 것:** relay에서의 MR 우위, MRS의 일반성.

## 6. 색인

- 설계: `design/scripts/study/exploratory/e20_mcp_referenced.py`, `e20_make_hands.py`, `e21_mcp_vs_skeleton.py`, `e22_all_three.py`. 출력은 `design/output/study/exploratory/e2{0,1,2}_*`.
- Drake: `sim/drake/two_pad_drake.py --anchor`, `box_pivot.py --anchor`, `anchor_f1.py`. 결과는 `sim/drake/logs/anchor_f1_*.json`.
- RL: `sim/isaaclab/scripts/rotate_cube/run_mr_transfer.sh`, `mr_transfer_summary.py`, `CODE_RELAY_ANCHOR=f1`. 결과는 `sim/isaaclab/logs/pr_mr_transfer/`.
- 주요 커밋: 7e1a6bc, 5b48abf, c8b7042, b9e5b6b (Drake anchor), e96c534 (RL 등록), 8652d3b, e72db8f, 9cce48a, 9023a15, e9d9926, 2e7fc15, cae0769
