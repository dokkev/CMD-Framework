# CFL의 가능한 설계 공간: 제약의 분류와 좌표

작성: 2026-11-21. 상태: 2절의 식과 4절의 좌표는 평가기와 대조했다(5절). GA에서의 효과는 측정 중이다. Study 0(`derivation.md`)의 연장이다.

## 1. 문제

Study 1은 CFL의 설계를 pivot 네 개의 위치와 pad의 끝을 좌표로 뽑고, 평가기로 걸러낸다. 그 좌표의 범위에서 무작위로 뽑은 설계 80,000개 가운데 study goal을 만족하는 것은 25개(0.03%)이다(q_req 120°, pad 7 mm). GA의 첫 집단에는 가능한 설계가 없고, run마다 처음 발견한 가능한 설계의 계열로 모인다. 같은 조건의 run 6개가 coverage 2 %p 차이의 서로 다른 linkage로 끝나는 것과 맞는 그림이다.

목표: 가능한 설계 공간 안에서 뽑는다. 제약을 식으로 옮겨, 좌표의 범위가 가능한 공간이 되게 한다.

## 2. 운동 조건은 치수의 닫힌 식이다

**기호.** $A=(a,0)$, $B=(b,0)$은 link 선 위, $A'$은 proximal phalanx, $B'$은 distal phalanx의 pivot이다(기준 자세의 위치). Link는 $A'B$(길이 $r_1$)와 $AB'$(길이 $r_2$)이다.

$$
\mathbf u=A-A'\ (\text{길이 }e,\ \text{방향각 }\alpha),\qquad
\mathbf w=B'-B\ (\text{길이 }c,\ \text{방향각 }\beta),\qquad \delta=\alpha-\beta .
$$

**닫힘.** Flexion $q$에서 distal phalanx는 $R_q$만큼 돌므로 $B'(q)-B(q)=R_q\mathbf w$이다. 두 link의 길이 조건은

$$
\|B(q)-A'\|=r_1,\qquad \|B(q)-\hat A(q)\|=r_2,\qquad \hat A(q):=A-R_q\mathbf w .
$$

곧 $B(q)$는 중심 $A'$, 반지름 $r_1$의 원과 중심 $\hat A(q)$, 반지름 $r_2$의 원의 교점이다. 두 중심 사이의 거리는

$$
D(q)^2=\|\mathbf u-R_q\mathbf w\|^2=e^2+c^2-2ec\cos(\delta+q) \tag{F1}
$$

이다($R_q\mathbf w$의 방향각이 $\beta-q$이므로).

**명제 F1 (굽힘 한계).** Four-bar가 자세 $q$에서 닫힐 필요충분조건은 $|r_1-r_2|\le D(q)\le r_1+r_2$이다. 두 교점(두 조립 형태)은 등호에서만 만나므로, 기준 자세의 조립 형태로 $q_{\mathrm{req}}$까지 이어질 필요충분조건은

$$
\boxed{\;|r_1-r_2|<D(q)<r_1+r_2\quad\forall q\in[0,q_{\mathrm{req}}]\;} \tag{F2}
$$

이다. (F1)을 넣으면 $\cos(\delta+q)$가 두 상수

$$
k_{\mathrm{lo}}=\frac{e^2+c^2-(r_1+r_2)^2}{2ec},\qquad k_{\mathrm{hi}}=\frac{e^2+c^2-(r_1-r_2)^2}{2ec}
$$

사이에 머무는 것이다. 따라서 운동으로 닿을 수 있는 최대 flexion은 $\delta+q$가 $\pm\arccos k_{\mathrm{lo}}$, $\pm\arccos k_{\mathrm{hi}}$ 가운데 처음 만나는 값까지이다.

$$
q_{\max}^{\mathrm{kin}}=\min\big\{\,q>0:\ \cos(\delta+q)\in\{k_{\mathrm{lo}},k_{\mathrm{hi}}\}\big\} \tag{F3}
$$

탐색도, 굽힘 구간의 계산도 필요 없다.

*확인.* Study 1의 범위에서 무작위로 뽑은 four-bar 552개에서 (F3)과, 0.25° 간격으로 실제로 굽혀 본 한계를 비교했다. 552개 모두 0.25° 이내로 일치했다.

**각 자세의 형상도 닫힌 식이다.** $B(q)$는 두 원의 교점이므로 명시적으로 쓰인다. 따라서 다음도 반복 계산 없이 나온다.
- Transmission angle $\gamma(q)$: $B'$에서 $B'A$와 $B'B$ 사이의 각. $\cos\gamma=(r_2^2+c^2-\|A-B(q)\|^2)/(2r_2c)$.
- Pole $\mathbf P(q)$: Study 0의 보조정리 1. 곧 관절 간격, pad 사이의 clearance, coverage(Study 0의 명제 1–3).

## 3. 제약의 분류

Study의 평가기가 CFL에 거는 제약 전체를, 뽑는 단계에서 어떻게 다룰 수 있는지로 나눈다.

| 제약 (평가기의 이름) | 내용 | 분류 | 방법 |
|---|---|---|---|
| `bounds_finger_length_mm` | $\lvert OC\rvert=L_f$ | 좌표에서 제거 | $\lvert BC\rvert=L_f-b$로 정한다(link 길이를 제작 해상도로 반올림한 뒤의 $b$로) |
| `static_closure` | 기준 자세에서 닫힘 | 구성으로 성립 | $r_1,r_2$를 pivot 위치에서 읽는다 |
| `f_motion_deg` 가운데 운동의 한계 | $q_{\mathrm{req}}$까지 같은 조립 형태 | 닫힌 식 | (F2), (F3) |
| `gamma_range_deg` | $\gamma\in[30^\circ,150^\circ]$ | 닫힌 식(자세별) | 2절의 $\gamma(q)$ |
| `static_pin_spacing_mm` | 한 link 위의 pin housing이 겹치지 않음 | 좌표의 범위 | $r_2\ge\rho_A+\rho_{B'}$, $c\ge\rho_{B'}+\rho_B$ |
| `static_pivot_pin_mm` | $A$의 pin과 servo 축 $A'$ | 좌표의 범위 | $e\ge$ 정해진 거리 |
| `bounds_proximal_pad_mm`, `bounds_distal_pad_mm` | Pad 길이 하한, 받침 범위, 자기 pin을 피함 | 계산 | Pad의 끝을 뽑지 않고 Study 0으로 정한다(`placement.py`). 하한을 채울 수 있는지는 pole 경로에서 나온다 |
| `static_pad_collision_mm`, `f_motion_deg` 가운데 pad의 충돌 | Pad끼리 겹치지 않음 | 계산 | Study 0의 명제 1 |
| `bounds_mount_depth_mm` | 장착 깊이가 규칙의 값 | 계산 | 장착 규칙이 정한다(탐색하지 않음) |
| `bounds_proportions_mm` | 비율의 범위 | 해당 없음 | Study에서는 범위를 두지 않는다 |
| `static_mcp_servo_mm` | PIP servo와 MCP servo, $A$의 pin과 MCP servo | 일부 좌표의 범위 | $A'$의 $x$ 하한을 표로 준다(지금도 그렇게 함). 나머지는 평가기 |
| `static_pip_servo_pad_mm` | PIP servo와 proximal pad | 평가기 | Servo 외형과 pad의 간격. Pad를 정한 뒤 확인 |
| `bounds_dorsal_space_mm` | 손등 쪽 공간 | 평가기 | |
| `f_motion_deg` 가운데 그 밖의 충돌 | Link, pin, servo가 굽히는 중에 부딪힘 | 평가기 | |
| `ma_min` | MA 하한 | 해당 없음 | Study에서는 두지 않는다 |

정리하면 운동과 pad에 관한 제약은 닫힌 식 또는 Study 0의 계산으로 옮겨지고, 평가기에 남는 것은 servo와 그 밖의 몸체 충돌이다.

## 4. Closure가 범위가 되는 좌표

처음 제안한 여섯 좌표 $(a,s,e,\alpha,c,\beta)$에서는 (F2)가 좌표의 부등식일 뿐 범위가 아니다. 좌표를 바꾸면 범위가 된다. $D(q)$는 $e,c,\delta$만의 함수이므로, 구간 $[0,q_{\mathrm{req}}]$에서의 최소와 최대 $D_{\min},D_{\max}$는 두 link의 길이와 무관하다. 따라서

$$
\Sigma=r_1+r_2=D_{\max}+\sigma,\quad \sigma>0;\qquad r_1-r_2=\rho\,D_{\min},\quad |\rho|<1 \tag{F4}
$$

로 두면 (F2)는 $(\sigma,\rho)$의 상자이다. 좌표는 $(a,\ e,\ c,\ \delta,\ \sigma,\ \rho)$와 조립 형태(둘 중 하나)이다.

**재구성.** $A$를 원점에 두고 $\alpha=0$으로 놓으면 $A'=(-e,0)$, $\hat A(0)=A-\mathbf w=-c(\cos\delta,-\sin\delta)$이다. $B$는 중심 $A'$(반지름 $r_1$)과 $\hat A(0)$(반지름 $r_2$)의 두 원의 교점 가운데 조립 형태가 정하는 쪽이다. 전체를 $A$를 중심으로 돌려 $B$를 link 선 위에 놓으면 $\alpha$가 정해진다. 자유도는 6이고 변환은 닫힌 식이다(`core/geometry/fourbar.py`).

**고른 표본의 뜻.** 이 좌표의 상자에서 고르게 뽑은 표본은 이 좌표에서 고른 것이다. Pivot 위치의 분포는 변환의 Jacobian만큼 다르다. 기존 좌표의 표본과 가능한 비율을 비교할 때 두 수치는 서로 다른 분포에 대한 것이다.

## 5. 구현 전 검증 (2026-11-21)

조건: Study 1의 CFL, 평가기와 목적함수는 그대로. 표본 실험은 $q_{\mathrm{req}}=120^\circ$, pad 7 mm (`scripts/study/exploratory/e12_closure_sampling.py`).

| 검증 | 방법 | 결과 |
|---|---|---|
| 1. Closure가 평가기와 같은가 | Closure 한계에서 0.5 mm 이내의 four-bar 3,000개. 모델의 sweep(0.05° 간격)과 비교 | $q_{\mathrm{req}}$에서의 판정이 3,000개 모두 같다. 굽힘 한계의 차이는 간격 이내 |
| | Transmission angle의 닫힌 식과 평가기의 값 | 가능한 설계 187개에서 차이 0.000° |
| 2. 가능한 설계를 버리지 않는가 | 평가기가 받아들인 CFL, CFL ideal 설계 623개($q_{\mathrm{req}}$ 90/120/150°) | Closure 조건이 623개 모두 받아들인다. 한계까지의 여유는 최소 2.99 mm, 중앙값 11.7 mm |
| | 그 가운데 CFL 187개에 transmission angle 필터 | 187개 모두 받아들인다 |
| 4. 좋은 설계를 놓치지 않는가 | 같은 187개를 새 좌표로 옮기고 되돌림 | 모두 새 좌표의 범위 안. 되돌린 설계의 coverage가 모두 같다. 조립 형태는 양쪽 모두 나온다(CFL ideal을 더한 254개 가운데 7개가 반대쪽) |

좌표와 pivot 사이의 변환은 정확하다. 평가기는 link 길이를 0.1 mm로 반올림한 뒤 $B$를 다시 풀기 때문에 $B$가 link 선에서 최대 0.27 mm 벗어나는데, 그 회전을 빼면 재구성의 오차는 $3\times10^{-14}$ mm이다.

**3. 표본의 효율.** 종류마다 200,000개. 평가기는 단계로 나뉘어 있어, 표는 표본이 처음 떨어진 단계이다.

| | 기존 좌표 | Closure 좌표 | Closure 좌표 + transmission angle 필터 |
|---|---|---|---|
| 필터에서 버림 | – | – | 44.5% |
| 설계가 되지 않음(범위 밖) | 8.0% | 97.9% | 54.9% |
| 조립되지 않음 | 1.5% | 0 | 0 |
| 범위와 pin (손가락 길이, pin 간격) | 55.7% | 1.4% | 0.43% |
| Servo | 6.9% | 0.04% | 0.01% |
| 편 자세의 pad | 12.5% | 0.13% | 0.01% |
| 운동 | 15.4% | 0.54% | 0.08% |
| **가능** | **33개 (0.017%)** | **19개 (0.009%)** | **14개 (0.007%)** |
| 평가기에 간 것 가운데 가능 | 0.018% | 0.46% | 1.3% |
| 가능한 설계 하나당 CPU 시간 | 18.6 s | 4.4 s | 5.5 s |
| 가능한 설계의 coverage: 최고 / 중앙값 | 0.647 / 0.343 | 0.610 / 0.400 | 0.532 / 0.325 |

읽는 법.

- 가능한 비율은 오르지 않았다. Closure 좌표의 표본은 98%가 Study의 탐색 범위 밖(대부분 $B$가 손가락 끝을 넘음)이어서 설계가 되지 않는다. 이 탈락은 평가기를 부르지 않아 싸고, 그래서 가능한 설계 하나당 시간은 1/4로 줄었다.
- Transmission angle 필터는 평가기에 가는 수를 줄이지만(4,171 → 1,066) 필터 자체의 비용과 상쇄되어 시간은 줄지 않는다.
- 기존 좌표에서 운동 단계에 닿은 표본의 99.9%가 거기서 떨어진다. 그런데 closure와 transmission angle을 모두 보장한 뒤에도 운동 단계에 닿은 174개 가운데 160개(92%)가 떨어진다. 따로 뽑은 109개에서 원인은 전부 굽히는 중의 충돌이고, 처음 부딪히는 쌍은 pad와 pad 68개(62%), $A$의 pin과 distal pad 29개(27%), distal pad와 PIP servo 6개였다. 모자란 flexion의 중앙값은 108°로, 대부분 굽히기 시작하자마자 부딪힌다.

**결론.** CFL의 가능한 공간을 좁히는 것은 closure도 transmission angle도 아니고, 굽히는 동안 pad가 부딪히지 않는다는 조건이다. 이것은 Study 0의 명제 1이 다루는 조건이고, 가설 H1(coverage를 최대화한 설계에서 pad 접촉이 활성 제약인 경우가 많다)과 같은 방향의 관찰이다. Closure 좌표는 정확하고 좋은 설계를 놓치지 않지만, 표본의 낭비를 줄이는 데에는 pad의 배치를 linkage의 운동에서 계산하는 것(3절의 "계산" 분류, `placement.py`)이 필요하다.

기존 좌표의 55.7%를 떨어뜨리는 손가락 길이 조건은 link 길이의 반올림 뒤에 $B$가 옮겨져 생기는 것으로, 좌표에서 없앨 수 있다(3절).

## 6. Conditional sampling: linkage 먼저, pad는 그 뒤에 (2026-11-21)

5절의 결론에 따라 pad를 linkage와 함께 뽑지 않고, linkage가 정해진 뒤에 뽑는다: $M\sim p(M)$, $C\sim p(C\mid M)$. Linkage의 운동을 한 번 계산하고, pad 배치의 격자(1 mm)에서 명제 1의 정확한 clearance를 구간 전체(1° 간격)에 대해 구한 뒤, 겹치지 않고 최소 pad 길이를 지키는 배치 가운데 하나를 무작위로 뽑는다. 배치를 최적화하지 않는다. 뽑은 설계는 평가기가 판정한다(linkage당 최대 3번). 스크립트: `scripts/study/exploratory/e14_conditional_pads.py`.

같은 linkage 1,066개(closure 좌표 표본 200,000개 가운데 transmission angle 필터와 범위를 통과한 것)에 대한 비교.

| | Pad를 함께 뽑음 | Pad를 조건부로 뽑음 |
|---|---|---|
| 가능한 linkage | 14개 (1.3%) | 78개 (7.3%) |
| 맞는 배치 없음 | – | 28.0% |
| 범위와 pin에서 탈락 | 80.8% | 56.8% |
| 운동에서 탈락 | 15.0% | 5.1% |
| 그 가운데 pad끼리의 충돌 | 115개 | 2개 |
| 평가기 호출 | 1,066 | 2,168 |
| 가능한 linkage 하나당 CPU 시간 | 6.5 s | 2.2 s |
| Coverage: 최고 / 중앙값 | 0.532 / 0.325 | 0.603 / 0.386 |
| 차지한 칸 ($A$의 위치 5 mm × $\delta$ 15° × 조립 형태) | 14 | 51 |
| CPU 1분당 칸 | 9.2 | 17.6 |

가능한 linkage는 5.6배, 서로 다른 칸은 3.6배로 늘었다. Pad끼리의 충돌은 거의 없어졌다. 남는 탈락은 linkage 쪽의 범위와 pin 조건(57%), $A$의 pin과 distal pad의 충돌(34개)이다. 칸 수는 linkage family 수의 대용이며, family 자체는 아직 재지 못했다.

## 7. 남은 것

- GA가 같은 계산 시간에 더 나은 linkage를 찾는가(최종 기준). 기존 GA, closure 좌표의 GA, 기존 좌표에 closure 표본의 시작점만 준 GA의 비교가 진행 중이다(`e11_placement_ablation.py`의 A, E, F).
- Pad의 배치를 운동에서 정하는 방법의 비용. 명제 1의 정확한 계산은 linkage마다 pad 배치의 2차원 탐색이 필요하다(`placement.py`). 대칭 간격 법칙은 일반 CFL에서 근사일 뿐이라 필터로 쓰지 않는다.
- 가능한 공간이 여러 조각인지는 아직 모른다.
- 2R과 RCJ에는 적용하지 않는다. 변수가 적고 지금도 run마다 같은 값에 도달한다.

평가기, 목적함수, Study 1의 기존 좌표와 결과는 바꾸지 않았다. 새 좌표는 별도의 경로(`core/optimization/analysis/closure_search.py`)이다.
