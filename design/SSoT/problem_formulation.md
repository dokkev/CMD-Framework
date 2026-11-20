# Appendix A. 수학적 정식화

이 부록은 손가락 설계 문제의 전체 구조를 정의한다. 흐름은 다음과 같다.

1. **Morphology (A.1).** 손가락을 mechanism, contact interface, actuation, structure의 네 요소로 기술한다.
2. **요구사항 (A.2–A.3).** Contact interface의 치수와 손가락의 크기·비율을 하한과 상한으로 지정한다.
3. **기하와 운동 (A.4).** Morphology는 joint 자세에 따라 형상을 만들고, 그 형상에 self-collision이 없는 자세의 집합이 유효한 운동 범위를 정한다.
4. **최적화 (A.5).** 요구사항, 요구 운동, 전달 성능 조건을 만족하는 morphology 가운데 contact pad coverage가 가장 큰 것을 찾는다.
5. **비교 연구 (A.6).** 여러 mechanism 구조를 같은 외형에 맞춘 뒤, contact interface만 바꿔 가며 각 구조가 얼마나 수용하는지(Study 1), interface에 맞춰 손가락을 다시 설계하면 morphology가 어떻게, 얼마나 달라지는지(Study 2a), 한 interface에 맞춘 손가락으로 다른 interface를 쓰면 coverage를 얼마나 잃는지(Study 2b) 평가한다.

Contact interface는 이 문제의 출발점이다. 그 치수는 요구사항으로 먼저 주어지고, mechanism, actuation, structure는 그것을 수용하도록 설계된다.

## A.1. Finger Morphology

Finger morphology $M$은 네 요소로 이루어진 결과 설계로 정의한다.

$$
M=(K,C,A,S)
$$

$K$, $C$, $A$, $S$는 각각 mechanism, contact interface, actuation, structure를 나타낸다.

Mechanism $K$는 link, joint, 기구학적 구속을 포함한 기구학적 구조를 정의한다. 이 틀 안에서 여러 mechanism 구조를 표현할 수 있으며, serial revolute joint, crossed four-bar linkage(CFL), rolling contact joint(RCJ)가 여기에 포함된다. Serial revolute joint는 PIP actuator가 joint에 직접 붙은 2R-DD와, 떨어진 PIP actuator가 transmission으로 구동하는 2R-remote로 나뉜다.

Contact interface $C$는 손가락에 장착되는 물리적 contact module을 기술한다. 두 contact interface에 대해 다음과 같이 정의한다.

$$
C=(C_1,C_2),
\qquad
C_i=(\ell_i,h_i,\mathbf p_i)
$$

$i\in\{1,2\}$에 대해 $\ell_i$, $h_i$, $\mathbf p_i$는 각각 interface의 길이, 두께, 장착 위치이다. 장착 위치는 독립된 형태 요구사항으로 미리 정하는 값이 아니라, 결과 설계의 일부로 결정된다.

Actuation $A$는 2-DoF 손가락의 두 actuator, 각 actuator의 packaging 형상과 배치, 그리고 actuator와 그것이 구동하는 joint 사이의 transmission을 기술한다.

$$
A=(A_1,A_2,\Gamma),
\qquad
A_j=(\mathcal G_j,T_j),
\qquad
T_j\in SE(2)
$$

$A_1$은 palm에 놓여 MCP joint를 구동하는 MCP actuator이고, $A_2$는 proximal phalanx에 놓여 PIP joint를 구동하는 PIP actuator이다. 여기서 PIP joint는 proximal phalanx와 distal phalanx 사이의 관절부 전체를 뜻한다. CFL이나 RCJ처럼 하나의 hinge가 없는 구조에서는 그 mechanism 전체가 PIP joint이다. $\mathcal G_j\subseteq\mathbb R^2$는 actuator housing 형상이고, $T_j=(\mathbf r_j,\alpha_j)$는 그 actuator를 싣는 body의 좌표계에서 본 출력축 위치 $\mathbf r_j$와 housing 방향 $\alpha_j$이다. $\Gamma$는 PIP actuator와 PIP joint 사이의 transmission 형상이다(예: belt drive의 pulley). Actuator가 joint를 직접 구동하면 $\Gamma$는 비어 있다.

Structure $S$는 phalanx의 물리적 형상, contact interface를 받치는 구조적 지지, 그리고 mechanism의 joint와 pivot 주변의 물리적 여유 공간을 기술한다. 특히 pivot의 기구학적 위치는 $K$가 정의하고, pivot 주변의 물리적 여유 형상은 $S$가 정의한다. 또한 각 contact interface는 해당 phalanx 구조가 전체 길이에 걸쳐 받친다고 가정한다. 즉, pad는 자기 phalanx 구조 바깥으로 걸쳐 나가지 않는다.

## A.2. Desired Morphology

원하는 finger morphology는 형태 요구사항의 집합으로 지정한다.

$$
\mathbf m(M)=
\begin{bmatrix}
L_f & \rho_1 & \rho_2 &
\ell_1 & \ell_2 & h_1 & h_2
\end{bmatrix}^{\mathsf T}
$$

$L_f$는 손가락 전체 길이, $\rho_i$는 phalanx 길이 비율, $\ell_i$와 $h_i$는 contact interface의 치수이다.

요구사항은 하한과 상한으로 주어진다.

$$
\boxed{
\mathbf m_{\min}
\leq \mathbf m(M)
\leq \mathbf m_{\max}
}
$$

모든 부등식은 성분별로 적용한다. 고정된 설계 요구사항은 하한과 상한을 같게 두어 표현하고, 설계 범위는 서로 다른 하한과 상한으로 표현한다.

따라서 desired morphology는 하나의 손가락 형상이 아니라, 형태 descriptor 공간 안의 유계 영역을 정의한다.

## A.3. 기본 기하 관계

모든 형태 길이와 비율은 완전히 편 기준 자세에서 정의한다.

$$
\mathbf q=\mathbf 0,
\qquad
\mathbf q=
\begin{bmatrix}
q_1\\q_2
\end{bmatrix}
$$

$q_1$은 MCP flexion이고, $q_2$는 두 phalanx의 길이 방향 축 사이의 각도, 즉 proximal phalanx에 대한 distal phalanx의 상대 회전각이며, 완전히 편 자세에서 $0$이다. 이 정의는 joint 하나의 회전각에 의존하지 않는다. 따라서 하나의 hinge가 없고 회전 중심이 움직이는 CFL과 RCJ에도 revolute joint와 같은 의미로 적용된다.

> **표기 노트.** 편의상 자세를 일반화 좌표 $\mathbf q=(q_1,q_2)$로 쓰지만, 이 부록의 평가는 모두 MCP를 $q_1=0$에 고정하고, 완전히 편 자세부터 $q_2$에 대해서만 수행한다. 따라서 $q_{\max}$처럼 첨자 없이 쓰는 flexion 값은 $q_2$의 값을 뜻한다.

기준 자세에서 proximal phalanx와 distal phalanx의 길이 방향 축은 한 직선 위에 놓인다고 가정한다. CFL은 A.5의 pivot 배치로 이 가정을 만족한다. 이 직선의 방향을 단위벡터 $\mathbf e_x$로 쓴다. MCP 축을 $\mathbf r_O$라 하고, proximal phalanx가 $\mathbf r_O$에서 $\mathbf r_{1,\mathrm{end}}$까지, distal phalanx가 $\mathbf r_{2,\mathrm{start}}$에서 손끝 $\mathbf r_C$까지 이어진다고 하자. 모든 형태 길이는 점 사이의 유클리드 거리로 정의한다. Proximal phalanx와 distal phalanx의 구조 길이는 다음과 같다.

$$
L_1=\left\|\mathbf r_{1,\mathrm{end}}-\mathbf r_O\right\|,
\qquad
L_2=\left\|\mathbf r_C-\mathbf r_{2,\mathrm{start}}\right\|
$$

$L_{vp}$는 virtual pivot length로, proximal phalanx의 distal 쪽 끝과 distal phalanx의 proximal 쪽 시작 사이의 간격이다.

$$
L_{vp}=\left\|\mathbf r_{2,\mathrm{start}}-\mathbf r_{1,\mathrm{end}}\right\|
$$

손가락 전체 길이는 다음과 같이 정의한다.

$$
\boxed{
L_f=L_1+L_{vp}+L_2
}
$$

기준 자세에서 네 점 $\mathbf r_O$, $\mathbf r_{1,\mathrm{end}}$, $\mathbf r_{2,\mathrm{start}}$, $\mathbf r_C$가 이 순서로 한 직선 위에 놓이므로, $L_f=\|\mathbf r_C-\mathbf r_O\|$이다.

Phalanx 비율은 손가락 전체 길이에 대해 정의한다.

$$
\boxed{
\rho_i=\frac{L_i}{L_f},
\qquad i\in\{1,2\}
}
$$

따라서 다음이 성립한다.

$$
\rho_1+\rho_2+
\frac{L_{vp}}{L_f}=1
$$

**자세에 따른 길이.** 위의 길이는 모두 기준 자세에서의 값이며, 손가락 크기 요구사항(A.2)은 이 값으로 지정한다. 한편 두 phalanx 사이의 간격은 mechanism에 따라 자세에 따라 변한다. CFL과 RCJ에서는 회전 중심이 움직이므로, 펼친 자세에서 크던 간격이 굽히면 일반적으로 줄어든다. $L_1$, $L_2$는 강체의 길이이므로 자세와 무관하다. 따라서 자세 $\mathbf q$에서의 간격과 손가락 길이를 다음과 같이 정의한다.

$$
L_{vp}(\mathbf q)=
\left\|
\mathbf r_{2,\mathrm{start}}(\mathbf q)-\mathbf r_{1,\mathrm{end}}
\right\|,
\qquad
L_f(\mathbf q)=L_1+L_{vp}(\mathbf q)+L_2
$$

여기서 두 점은 proximal phalanx 좌표계에서 나타낸다. 기준 자세의 길이와 같은 유클리드 거리이므로 $L_{vp}(\mathbf 0)=L_{vp}$이고 $L_f(\mathbf 0)=L_f$이다. 일반적인 revolute joint에서는 이 간격이 모든 자세에서 0으로 유지된다.

$L_f(\mathbf q)$는 손가락 양 끝 사이의 거리 $\|\mathbf r_C(\mathbf q)-\mathbf r_O\|$가 아니다. 그 거리는 손가락이 말릴수록 줄어드는, 말림 정도를 나타내는 양이다. $L_f(\mathbf q)$는 자세 $\mathbf q$에서 손가락의 palmar 쪽 윤곽을 따라 잰 길이이다.

각 contact interface는 해당 phalanx 구조가 전체 길이에 걸쳐 받친다는 structure의 가정(A.1)에서 다음 길이 조건이 따라 나온다.

$$
0<\ell_i\leq L_i
$$

## A.4. 기하 표현과 운동

주어진 joint 자세에서 finger morphology의 2차원 형상을 다음과 같이 나타낸다.

$$
\boxed{\mathcal G(M,\mathbf q)}
$$

이 형상은 mechanism, contact interface, actuator, structure 요소를 모두 포함하며, 자세에 따라 바뀐다.

유효한 configuration space를 손가락 형상이 존재하고 그 형상에 self-collision이 없는 자세의 집합으로 정의한다.

$$
\mathcal Q_{\mathrm{valid}}(M)
=
\left\{
\mathbf q\in\mathcal Q
\;\middle|\;
\mathcal G(M,\mathbf q)\text{가 존재하고 self-collision이 없음}
\right\}
$$

Self-collision은 형상 $\mathcal G(M,\mathbf q)$ 안의 두 요소(pad, phalanx 구조물, actuator housing, 링크 등)의 내부가 겹치는 것이다. 맞닿는 것은 허용한다. 다음 두 경우는 겹쳐도 self-collision으로 보지 않는다.
- Actuator의 출력축이 지나는 자기 pivot과 그 actuator housing. 예를 들어 MCP actuator와 $O$, CFL의 PIP actuator와 $A'$가 그렇다.
- 서로 다른 lateral layer에 놓여 측면에서만 겹쳐 보이는 요소. 예를 들어 CFL의 링크와 2R-remote의 belt는 pad와 다른 layer에 놓인다.

Mechanism이 어떤 자세에서 닫히지 않으면 그 자세의 형상 $\mathcal G(M,\mathbf q)$는 존재하지 않는다. 따라서 mechanism의 조립 가능성은 형상의 존재에 포함된다. Actuator가 joint를 구동할 수 없는 특이 자세는 transmission 품질 제약(A.5.2)이 배제한다.

**Static feasibility.** 운동을 평가하기 전에, 기준 자세에서 손가락이 self-collision 없이 조립되는지를 먼저 확인한다.

$$
\boxed{
\mathcal F_{\mathrm{static}}(M)\;:\Longleftrightarrow\;\mathbf 0\in\mathcal Q_{\mathrm{valid}}(M)
}
$$

즉 기준 자세의 형상 $\mathcal G(M,\mathbf 0)$가 존재하고, 그 안에서 pad, 구조물, actuator housing이 서로 겹치지 않아야 한다. 같은 body에 붙은 요소끼리의 쌍(pad와 자기 phalanx 구조물, PIP actuator housing과 proximal phalanx 구조물 등)은 함께 움직이므로, 기준 자세에서 한 번만 확인하면 모든 자세에서 성립한다. MCP actuator와 proximal 쪽 요소의 쌍도 $q_1=0$ 고정 아래에서는 마찬가지다.

Static feasibility는 기준 자세가 요구 운동 구간에 속하므로 수학적으로는 아래의 motion feasibility에 이미 포함된다. 그래도 따로 두는 이유는 판정 방식이 다르기 때문이다. Static feasibility는 기준 자세 하나의 형상만으로 정해지므로, 요소들 사이의 거리로 해석적으로 판정할 수 있다. 반면 motion feasibility는 자세를 요구 운동 구간을 따라 움직여 가며 평가해야 한다. 따라서 static feasibility를 만족하지 않는 설계는 운동을 평가할 필요가 없다. 또한 "기준 자세에서 조립되지 않는다"는 실패와 "굽히다가 부딪힌다"는 실패를 구분해 보고할 수 있다.

**Motion feasibility.** 요구되는 distal flexion을 $q_{\mathrm{req}}$라 하고, 요구 운동 구간을 다음과 같이 쓴다.

$$
\mathcal Q_{\mathrm{req}}=
\left\{
\begin{bmatrix}0\\q_2\end{bmatrix}
\;\middle|\;
0\leq q_2\leq q_{\mathrm{req}}
\right\}
$$

요구 운동 구간의 모든 자세에서 형상이 존재하고 self-collision이 없으면 motion feasible이라 한다.

$$
\boxed{
\mathcal F_{\mathrm{motion}}(M)\;:\Longleftrightarrow\;\mathcal Q_{\mathrm{req}}\subseteq\mathcal Q_{\mathrm{valid}}(M)
}
$$

즉 손가락은 완전히 편 자세에서 요구 flexion까지 굽히는 동안 어느 자세에서도 self-collision이 없어야 한다.

## A.5. Prototype 최적화 정식화

이 절의 정식화를 prototype formulation이라 하며, production formulation 또는 CoDe formulation이라고도 부른다. Prototype 설계는 crossed four-bar linkage 구조($K=K_{\mathrm{CFL}}$)를 대상으로 한다. Contact interface 치수의 허용 범위는 형태 요구사항(A.2)으로 주어지고, 최적화는 contact pad coverage와 최악의 경우 mechanical advantage를 함께 최대화하는 finger morphology들, 곧 두 값의 Pareto front를 찾는다. Mechanical advantage에는 하한 제약도 두고, 운동 범위는 제약으로만 다룬다.

**CFL의 pivot.** CFL은 네 개의 pivot으로 이루어진다. $A$와 $A'$는 proximal phalanx에, $B$와 $B'$는 distal phalanx에 있다. PIP actuator $A_2$의 출력축은 $A'$에 놓이며, 링크 $A'B$를 돌려 distal phalanx를 굽힌다. 링크 $AB'$는 $A$를 중심으로 돌면서 distal phalanx를 받친다. 설계를 단순하게 하기 위해, 기준 자세에서 $A$는 proximal phalanx의 길이 방향 축 위에, $B$는 distal phalanx의 길이 방향 축 위에 둔다. 각 phalanx 구조는 자기 pad를 끝까지 받치고 자기 pivot의 housing도 품는다. 따라서 $\mathbf r_{1,\mathrm{end}}$는 proximal pad의 안쪽 끝과 $A$ housing의 distal 쪽 끝 가운데 관절에 더 가까운 점이고, $\mathbf r_{2,\mathrm{start}}$는 distal pad의 안쪽 끝과 $B$ housing의 proximal 쪽 끝 가운데 관절에 더 가까운 점이다. 두 구조 사이 간격 $L_{vp}$는 $|AB|$보다 길 수 없다.

**정렬.** 위 배치를 관절 축 정렬(joint axes)이라 하며, 기준 자세에서 두 phalanx 축이 한 직선 위에 놓인다. Prototype은 여기에 접촉면 정렬(contact faces)을 더한다. 두 pad의 접촉면을 각자의 링크 축에서 같은 깊이에 두어(장착 깊이와 pad 두께의 합이 같음), 기준 자세에서 두 접촉면도 한 직선 위에 놓이게 한다. 두 정렬을 함께 두면 기준 자세에서 두 pad는 한 직선 위에서 겹칠 수 없다. 따라서 두 pad 길이의 하한의 합이, 가장 긴 손가락에서 그 직선에 남는 길이를 넘으면 해가 없다. 이 조건은 필요조건이며, 굽히는 동안의 충돌은 $\mathcal F_{\mathrm{motion}}$이 판정한다.

**설계 변수와 고정 요소.** Morphology $M=(K,C,A,S)$의 각 요소 가운데 탐색하는 것과 주어지는 것은 다음과 같다.

| 요소 | 구성 | 취급 |
|---|---|---|
| $K$ | CFL pivot $A, A', B, B'$의 위치 | 설계 변수. 단, $A$와 $B$는 길이 방향 축 위 |
| $C$ | Pad 치수 $(\ell_i, h_i)$ | 요구 범위 안의 설계 변수. 길이는 하한 $\ell_{i,\min}$만 두고 상한은 두지 않는다 |
| $C$ | 장착 위치 $\mathbf p_i$ | 설계 변수 (길이 방향 위치와 깊이) |
| $A$ | MCP actuator $A_1$ | 고정. 출력축은 MCP 축 $\mathbf r_O$ 위 |
| $A$ | PIP actuator $A_2$ | Housing 형상 $\mathcal G_2$는 고정. $T_2=(\mathbf r_2,\alpha_2)$ 중 출력축 위치 $\mathbf r_2$는 $A'$로 정해지고, 방향 $\alpha_2$는 범위 $[\alpha_{\min},\alpha_{\max}]$ 안의 설계 변수 |
| $S$ | Phalanx 구조의 끝점 $\mathbf r_{1,\mathrm{end}}$, $\mathbf r_{2,\mathrm{start}}$, 손끝 $\mathbf r_C$ | 설계 변수 (즉 $L_1$, $L_2$, $L_f$) |

### A.5.1. Contact Pad Coverage

Contact pad coverage는 두 pad를 **함께** 써서 물체를 잡을 수 있는 정도를 잰다. 한 pad가 아무리 길어도, 그 부분이 다른 pad와 함께 물체를 받칠 수 없으면 쓸모가 없다고 본다. 이를 원기둥 물체를 기준으로 정의한다.

**원기둥 기준 유효 영역.** 자세 $\mathbf q$에서 두 pad의 contact face를 직선으로 연장하면, 두 직선은 점 $X(M,\mathbf q)$에서 만나 V자를 이룬다. $\mathbf u_i$를 $X$에서 pad $i$의 바깥 끝(proximal pad는 MCP 쪽, distal pad는 손끝 쪽)을 향하는 단위벡터라 하자. 이 V 안에 내접하는 원(원기둥 물체의 단면)은 두 face에 $X$로부터 **같은 거리** $t$에서 닿는다. 한 점에서 원에 그은 두 접선의 길이는 같기 때문이다. 따라서 $t$를 $0$부터 키우면 작은 원기둥부터 큰 원기둥까지를 차례로 나타낼 수 있다.

Pad $i$가 덮는 거리의 집합을

$$
T_i(M,\mathbf q)=
\left\{
t\ge 0
\;\middle|\;
X+t\,\mathbf u_i\text{가 pad }i\text{의 contact face 위에 있고 가려지지 않음}
\right\}
$$

라 하면, 두 pad로 함께 잡을 수 있는 거리의 집합은 $T=T_1\cap T_2$이다. 가용 pad 영역의 크기는 두 pad에서 같다.

$$
c_1(M,\mathbf q)=c_2(M,\mathbf q)=\big|T_1(M,\mathbf q)\cap T_2(M,\mathbf q)\big|
$$

$t\in T$이면 그 크기의 원기둥을 두 pad로 동시에 받칠 수 있다. 한 pad만 덮는 거리, 즉 상대 pad에 짝이 없는 부분은 $T$에서 빠진다. $X$를 넘어 V의 닫힌 쪽에 놓인 pad 부분은 $t<0$이므로 역시 빠진다. 원기둥이 V 안에 놓이므로, pad 뒤쪽(dorsal)에 있는 손가락 요소와의 충돌은 따로 고려하지 않는다. Pad 치수 $\ell_i$는 장착된 pad의 물리적 길이인 반면, $c_i$는 설계와 자세에 따라 정해지는 결과값이며 $0\le c_i(M,\mathbf q)\le\ell_i$이다. 이 정의는 두 face가 교차하는 자세, 즉 $q_2>0$에서 쓴다.

**자세별 coverage.** 정규화된 contact pad coverage는 다음과 같이 정의한다.

$$
\boxed{
C_{\mathrm{pad}}(M,\mathbf q)
=
\frac{
c_1(M,\mathbf q)+c_2(M,\mathbf q)
}{
L_f(\mathbf q)
}
=
\frac{2\,\big|T(M,\mathbf q)\big|}{L_f(\mathbf q)}
}
$$

분자와 분모를 같은 자세에서 잰다. 물체가 접촉하는 것은 펼친 손가락이 아니라 그 자세에서의 손가락 윤곽이기 때문이다. 이 정규화에서는 굽히면서 줄어드는 phalanx 사이 간격, 즉 접촉할 수 없는 구간이 coverage에 그대로 반영된다. 따라서 간격이 변하지 않는 revolute joint와 간격이 변하는 CFL·RCJ를 같은 기준으로 비교할 수 있다.

**평균 coverage.** 자세마다 잡는 물체의 크기가 다르다. 덜 굽힌 자세는 큰 물체를, 더 굽힌 자세는 작은 물체를 잡는다. 따라서 coverage는 여러 평가 자세에서의 평균으로 정의한다.

$$
\mathcal Q_{\mathrm{eval}}=\left\{90^\circ,\;\tfrac{1}{2}\left(90^\circ+q_{\mathrm{req}}\right),\;q_{\mathrm{req}}\right\},
\qquad
\bar C_{\mathrm{pad}}(M)=
\frac{1}{|\mathcal Q_{\mathrm{eval}}|}
\sum_{q\in\mathcal Q_{\mathrm{eval}}}
C_{\mathrm{pad}}\big(M,(0,q)\big)
$$

평가 자세는 $90^\circ$, 요구 flexion $q_{\mathrm{req}}$, 그리고 그 중간점이다. 예를 들어 $q_{\mathrm{req}}=150^\circ$이면 $\{90^\circ,120^\circ,150^\circ\}$, $q_{\mathrm{req}}=120^\circ$이면 $\{90^\circ,105^\circ,120^\circ\}$이다. 이 정의는 $q_{\mathrm{req}}\ge 90^\circ$를 전제로 한다. 평가 자세가 모두 요구 운동 구간 안에 있으므로, 모든 평가 자세에서 형상이 유효하다는 것은 $\mathcal F_{\mathrm{motion}}$(A.4)이 보장한다.

$\bar C_{\mathrm{pad}}$는 prototype의 첫째 목적함수이다(A.5.4). Pad 길이에 상한이 없어도, 이 목적함수는 두 pad의 길이와 배치를 서로 맞추는 방향으로 작용한다.

### A.5.2. Mechanical Advantage

$\theta_{\mathrm{pip}}$를 PIP actuator의 입력 각도, $\tau_{\mathrm{pip}}$를 그 토크, $q_2$를 proximal phalanx에 대한 distal phalanx의 출력 각도라 하자.

이상적인 준정적 transmission에서 mechanical advantage는 가상일로 정의한다.

$$
\tau_{\mathrm{pip}}\,d\theta_{\mathrm{pip}}
=
\tau_{\mathrm{out}}\,dq_2
$$

따라서

$$
MA(M,\mathbf q)
=
\left|
\frac{\tau_{\mathrm{out}}}{\tau_{\mathrm{pip}}}
\right|
=
\left|
\frac{d\theta_{\mathrm{pip}}}{dq_2}
\right|
$$

이다.

요구 운동 전체에서의 전달 성능을 평가하기 위해 최악의 경우 mechanical advantage를 다음과 같이 정의한다.

$$
MA_{\mathrm{wc}}(M)
=
\min_{\mathbf q\in\mathcal Q_{\mathrm{req}}}
MA(M,\mathbf q)
$$

$MA_{\mathrm{wc}}$는 요구 운동 구간 어디에서든 actuator가 distal phalanx에 전달하는 토크 비율의 최솟값이다. Prototype에서는 둘째 목적함수이며, 그 하한은 제약으로도 둔다(A.5.4).

Mechanical advantage가 커도 linkage가 특이 자세에 가까우면 힘 전달이 불안정해진다. 따라서 transmission 품질을 따로 본다. CFL에서 transmission angle $\gamma(M,\mathbf q)$는 pivot $B'$에서 링크 $B'A$와 distal phalanx 위의 선분 $B'B$가 이루는 각도이다.

$$
\gamma(M,\mathbf q)=\angle\big(\mathbf r_A-\mathbf r_{B'}(\mathbf q),\;\mathbf r_B(\mathbf q)-\mathbf r_{B'}(\mathbf q)\big)\in[0,\pi]
$$

PIP actuator는 링크 $A'B$를 통해 $B$를 움직이고, 그 힘은 $B'B$를 거쳐 링크 $AB'$에 전달된다. $\gamma$가 $0$이나 $\pi$에 가까우면 $B'A$와 $B'B$가 한 직선에 가까워진다. 이때 $B'B$를 따라 전달되는 힘은 링크 $AB'$를 돌리는 방향 성분을 거의 갖지 못한다. 그래서 distal phalanx에 걸리는 하중을 버티려면 링크에 걸리는 힘이 급격히 커진다. 또 작은 입력 변화에도 distal phalanx의 자세가 크게 변해 mechanical advantage가 $0$에 가까워지는 특이 자세가 된다. 따라서 요구 운동 구간에서 이 각도가 정해진 범위 $[\gamma_{\min},\gamma_{\max}]$ 안에 있도록 A.5.4에서 제약으로 둔다.

### A.5.3. 운동 범위

$q_{\max}(M)$는 완전히 편 자세에서 출발해 안전하게 도달할 수 있는 최대 distal flexion이다. 여기서 안전하다는 것은 그 구간의 모든 자세가 유효하고(형상이 존재하고 self-collision이 없음, A.4), transmission angle이 허용 범위 안에 있다는 뜻이다(A.5.2). Distal phalanx가 proximal phalanx와 반대 방향을 향하는 $\pi$ 이상의 flexion은 더 이상 쓸모가 없으므로, $q_{\max}$의 상한을 $\pi$로 둔다.

$$
q_{\max}(M)=
\sup\left\{
0\le\bar q\le\pi
\;\middle|\;
(0,q_2)\in\mathcal Q_{\mathrm{valid}}(M)
\;\text{이고}\;
\gamma_{\min}\le\gamma\big(M,(0,q_2)\big)\le\gamma_{\max},
\;\;\forall\,q_2\in[0,\bar q]
\right\}
$$

$q_{\max}$는 제약에 직접 쓰이지 않는다. $\mathcal F_{\mathrm{motion}}$과 transmission angle 조건을 함께 만족하는 것이 $q_{\max}(M)\ge q_{\mathrm{req}}$와 같기 때문이다. $q_{\max}$는 요구 flexion 이상으로 확보한 운동 범위를 나타내며, 결과 설계를 특성화하는 값으로 보고한다.

### A.5.4. 통합 최적화 문제

Prototype 설계는 2목적 최적화 문제로 정식화된다.

$$
\boxed{
\begin{aligned}
\underset{M}{\operatorname{maximize}}
\quad&
\big(\bar C_{\mathrm{pad}}(M),\;MA_{\mathrm{wc}}(M)\big)\\[8pt]
\text{subject to}\quad&
K=K_{\mathrm{CFL}},\\
&
\mathbf m_{\min}\leq\mathbf m(M)\leq\mathbf m_{\max},\\
&
\mathcal F_{\mathrm{static}}(M),\quad
\mathcal F_{\mathrm{motion}}(M),\\
&
\gamma_{\min}\leq\gamma(M,\mathbf q)\leq\gamma_{\max}\quad\forall\,\mathbf q\in\mathcal Q_{\mathrm{req}},\\
&
MA_{\mathrm{wc}}(M)\geq MA_{\min}
\end{aligned}
}
$$

$\mathcal F_{\mathrm{static}}$과 $\mathcal F_{\mathrm{motion}}$은 형상 $\mathcal G(M,\mathbf q)$로 판정하는 기하학적 조건이다. 앞의 것은 기준 자세에서 손가락이 조립된다는 것을, 뒤의 것은 요구 운동 구간 전체에서 self-collision이 없다는 것을 뜻한다(A.4). Transmission angle과 mechanical advantage 조건은 같은 구간에서 힘 전달이 안정적이고 충분하다는 것을 뜻한다(A.5.2). $MA_{\mathrm{wc}}\ge MA_{\min}$은 최악의 경우 전달비 $\left|dq_2/d\theta_{\mathrm{pip}}\right|$가 $1/MA_{\min}$ 이하라는 것과 같다. Motion feasibility와 transmission angle 조건을 함께 만족하는 것은 $q_{\max}(M)\ge q_{\mathrm{req}}$와 같다.

두 목적은 서로 맞선다. Pad를 길게 두어 coverage를 늘리는 링크 배치와, 요구 운동 구간 전체에서 PIP actuator의 토크를 손끝까지 잘 전하는 링크 배치는 일반적으로 같지 않다. 따라서 해는 하나의 설계가 아니라 Pareto front이며, 제작할 설계는 그 위에서 고른다. $MA_{\min}$은 front가 쓸모없는 전달까지 내려가지 않게 하는 하한으로 남는다.

결과 설계의 $q_{\max}$는 제약을 얼마나 여유 있게 만족하는지를 보여 주는 값으로 함께 보고한다.

## A.6. Study 정식화

이 절의 실험은 `research-objective.md` 6절이 보이라고 한 네 가지에 하나씩 대응한다.

| 보여야 할 것 (research-objective 6절) | 실험 | 절 |
|---|---|---|
| 1. 구조가 contact interface를 제한한다 | 구조별 native 설계의 비교 | A.6.1 |
| 2. 고정된 손가락이 여러 interface를 받는다 | Study 1 | A.6.2 |
| 3. Interface를 바꿀 때 morphology가 얼마나, 어디가 바뀌는가 | Study 2a | A.6.3 |
| 4. 손가락 하나로 interface family를 쓴다 | Study 2b | A.6.4 |

### A.6.1. 공통 설정

Study는 mechanism 구조를 서로 비교한다. 비교 대상 구조의 집합은 다음과 같다.

$$
\mathcal K=\{K_{\mathrm{2R\text{-}DD}},\;K_{\mathrm{2R\text{-}remote}},\;K_{\mathrm{CFL}},\;K_{\mathrm{RCJ}}\}
$$

$K_{\mathrm{2R\text{-}DD}}$는 PIP actuator가 PIP joint에 직접 붙은 serial revolute 구조이다. $K_{\mathrm{2R\text{-}remote}}$는 같은 revolute joint를 떨어진 PIP actuator가 belt로 구동하는 구조로, 이때 $\Gamma$가 비어 있지 않다. 2R-remote의 PIP actuator는 proximal phalanx 위에 고정된 자세로 둔다. 출력축 $A'$는 PIP joint보다 10 mm proximal, 15 mm dorsal에 두고 방향은 15°이다. Belt 때문에 $MA\equiv 1$이고 가림은 보지 않으므로 목적함수는 actuator 자세에 영향을 받지 않는다. 따라서 탐색하지 않는다. $K_{\mathrm{RCJ}}$에서 joint actuator를 없앤 이상화 구조($\mathcal G_2=\varnothing$)는 물리적으로 만들 수 있는 구조가 아니므로, 상한으로만 따로 보고한다. $K_{\mathrm{CFL}}$에서 PIP actuator의 housing을 없앤 이상화 구조($\mathcal G_2=\varnothing$, 링크와 $A'$의 pivot은 남김)도 같은 이유로 상한으로만 보고한다. Transmission angle 조건($\gamma$)은 linkage의 성질이므로 그대로 둔다.

**외형 descriptor.** 손가락의 외형을 다음 네 양으로 나타낸다.

$$
\mathbf d(M)=\big(L_f,\;L_1,\;L_2,\;x_{\mathrm{art}}\big)
$$

여기서 $x_{\mathrm{art}}=\mathbf e_x^{\mathsf T}\big(\mathbf r_{\mathrm{ICR}}(\mathbf 0)-\mathbf r_O\big)$는 articulation 위치로, 기준 자세에서 distal phalanx의 순간 회전 중심 $\mathbf r_{\mathrm{ICR}}$를 MCP 축에서 $\mathbf e_x$ 방향으로 잰 위치이다. Revolute joint에서는 joint 위치이고, CFL과 RCJ에서는 회전 중심이 움직이므로 기준 자세에서의 값을 쓴다.

네 양은 두 종류이다.
- **손가락이 굽는 기하.** $L_f$와 $x_{\mathrm{art}}$. 펼친 손가락에서 MCP 축부터 굽는 곳까지가 $x_{\mathrm{art}}$, 굽는 곳부터 손끝까지가 $L_f-x_{\mathrm{art}}$이다. 손가락을 보았을 때의 phalanx 길이는 이 두 값이다.
- **Pad를 받치는 구조의 범위.** $L_1$과 $L_2$(A.3). 각 phalanx 구조가 MCP 쪽 끝과 손끝 쪽 끝에서 어디까지 뻗어 있는지, 곧 pad가 놓일 수 있는 범위이다. 굽는 곳까지의 길이가 아니다. Pad 치수가 정해지면 대체로 그 길이를 따른다.

두 종류는 서로 독립이다. 예를 들어 revolute joint에서 proximal pad를 joint 아래로 깊게 내려 달면 proximal 구조가 굽는 곳을 넘어 뻗을 수 있고, 이때 $L_1>x_{\mathrm{art}}$이다. 이런 배치는 금지하지 않는다. 굽는 곳을 넘은 pad 부분은 굽힌 자세에서 상대 pad와 함께 원기둥을 받치지 못하므로, coverage(A.5.1)가 그 손실을 그대로 반영한다. 형태 변화를 보고할 때는 phalanx 길이를 $x_{\mathrm{art}}$와 $L_f-x_{\mathrm{art}}$로 읽고, $L_1$, $L_2$는 pad가 놓인 범위로 읽는다.

**기준 morphology.** 각 구조 $K\in\mathcal K$와 요구 flexion $q_{\mathrm{req}}$마다 기준 morphology $M_0^K(q_{\mathrm{req}})=(K,C_0,A_0^K,S_0^K)$를 하나씩 둔다. 기준은 그 구조 자신의 native 설계이다. 즉 모든 구조에 같은 study goal(그 $q_{\mathrm{req}}$, $L_f=90$ mm, pad 요구, actuator, 충돌 규칙)을 주고, 구조마다 따로 $\bar C_{\mathrm{pad}}$를 최대화해 얻은 설계이다.

$$
M_0^K(q_{\mathrm{req}})=\arg\max_{M\in\mathcal M_K}\;\bar C_{\mathrm{pad}}(M)
\quad\text{s.t. study goal}(q_{\mathrm{req}}),
\qquad
\mathbf d_0^K=\mathbf d\big(M_0^K(q_{\mathrm{req}})\big)
$$

각 기준은 자기 $q_{\mathrm{req}}$에서만 평가한다. 한 $q_{\mathrm{req}}$에 맞춘 설계를 다른 $q_{\mathrm{req}}$에서 평가하면, module을 받는 능력과 운동 범위의 부족이 섞이기 때문이다. 손가락 길이는 모든 구조에서 $L_f=90$ mm로 같다(Study 1은 이 크기에서 비교한다). 기준의 외형 $\mathbf d_0^K$에서 구조마다 다를 수 있는 것은 phalanx 길이와 articulation 위치이다. 한 구조(예: CFL)의 외형에 다른 구조를 맞추면 그 구조에 유리한 형상에서 비교하게 되므로, 외형을 서로 맞추지 않는다. 대신 goal을 같게 두고, 각 구조가 자기에게 가장 좋은 형상에서 출발하게 한다. PIP actuator의 housing 방향 $\alpha_2$도 이 탐색에서 구조마다 정해진다.

**평가 기준.** Study에서도 A.4–A.5의 정의를 그대로 쓴다. 유효 자세 $\mathcal Q_{\mathrm{valid}}$, 최대 안전 flexion $q_{\max}$, coverage $\bar C_{\mathrm{pad}}$가 그 예이다. 다만 다음이 다르다.
- Transmission angle 조건은 linkage가 있는 CFL에만 적용한다. 다른 구조에서는 이 조건이 항상 만족된다고 본다.
- Mechanical advantage에는 제약을 두지 않는다($MA_{\min}$ 없음). $MA_{\mathrm{wc}}$는 결과로 보고해 비교한다. CFL 외의 구조에서는 구조로 정해지는 상수이다. 2R-DD와 2R-remote는 $MA\equiv 1$, 대칭 RCJ는 $MA\equiv 1/2$이다.
- Coverage를 계산할 때 transmission 요소(링크, belt)가 pad를 가리는 효과는 기본적으로 고려하지 않는다. 이 요소들은 pad와 다른 lateral layer에 놓이기 때문이다(A.4). 가림의 영향은 민감도 분석으로 따로 본다.

### A.6.2. Study 1: 고정 morphology의 contact 수용 능력

Study 1은 손가락을 전혀 바꾸지 않은 상태에서 각 구조가 어떤 contact interface를 수용할 수 있는지 묻는다.

**Contact module 목록.** 평가할 contact module의 치수 쌍을 목록 $\mathcal C_{\mathrm{cat}}$로 준다.

$$
\mathcal C_{\mathrm{cat}}=\left\{(\ell_1,h_1,\ell_2,h_2)\right\}
$$

**장착 규칙.** 기준 morphology에서 mechanism, actuation, structure $(K,A_0^K,S_0^K)$는 그대로 두고 contact interface의 치수만 바꾼다. 장착 위치 $\mathbf p_i$는 탐색하지 않고 정해진 장착 규칙으로 결정한다. 각 pad는 바깥 끝(proximal pad는 MCP 쪽, distal pad는 손끝 쪽)에 맞춰 놓는다. 깊이는 자기 phalanx의 joint 하드웨어와 겹치지 않는 가장 얕은 위치로 하되, 그 phalanx 구조가 받칠 수 있는 범위 안이어야 한다. 이런 위치가 없으면 module을 장착할 수 없다. Spacer는 쓰지 않는다. 이렇게 얻은 morphology를 다음과 같이 쓴다.

$$
M^K(c)=\big(K,\;C(c),\;A_0^K,\;S_0^K\big),
\qquad c\in\mathcal C_{\mathrm{cat}}
$$

**수용 판정.** 요구 flexion $q_{\mathrm{req}}$에서 module $c$를 수용한다는 것은 다음 세 조건을 모두 만족한다는 뜻이다.
1. 장착 규칙이 장착 위치를 준다.
2. $M^K(c)$가 structure의 지지 가정(A.1)을 만족한다. 즉 각 pad가 고정된 phalanx 구조 위에 전체 길이로 놓인다.
3. $M^K(c)$가 $\mathcal F_{\mathrm{static}}$과 $\mathcal F_{\mathrm{motion}}$을 만족하고, CFL이면 transmission angle 조건도 만족한다. 즉 $q_{\max}\big(M^K(c)\big)\ge q_{\mathrm{req}}$이다.

구조 $K$가 수용하는 module의 집합을 다음과 같이 정의한다.

$$
\boxed{
\mathcal C_{\mathrm{feas}}(K;q_{\mathrm{req}})
=
\left\{
c\in\mathcal C_{\mathrm{cat}}
\;\middle|\;
M^K(c)\text{가 }q_{\mathrm{req}}\text{에서 module }c\text{를 수용함}
\right\}
}
$$

요구 flexion은 $q_{\mathrm{req}}\in\{90^\circ,120^\circ,150^\circ\}$이고, 각 $q_{\mathrm{req}}$에서 그 $q_{\mathrm{req}}$의 기준 $M_0^K(q_{\mathrm{req}})$를 평가한다. 수용된 module에 대해서는 coverage $\bar C_{\mathrm{pad}}\big(M^K(c)\big)$와 $q_{\max}\big(M^K(c)\big)$를 함께 보고한다. 수용되지 않은 module은 손가락을 다시 설계하지 않고, 어느 조건에서 실패했는지만 기록한다.

**해석.** $\mathcal C_{\mathrm{feas}}(K;q_{\mathrm{req}})$가 클수록, 그 구조는 손가락을 바꾸지 않고도 더 다양한 contact interface를 수용한다. 각 구조는 같은 goal 아래의 자기 native 설계에서 평가하므로, 이 차이는 한 구조에 맞춘 외형에서 오는 것이 아니다. 다만 기준의 외형이 구조마다 다르므로, 기준의 외형 $\mathbf d_0^K$를 함께 보고한다. 이 결과는 2차원 측면 모델에서 기하학적으로 공간이 있다는 뜻이며, 실제로 제작하거나 교체할 수 있다는 뜻은 아니다.

### A.6.3. Study 2a: contact interface에 맞춘 morphology 재최적화

Study 2는 두 부분으로 나뉜다. Study 2a(이 절)는 더 큰, 또는 다른 contact interface를 쓰려고 손가락을 다시 설계하면 morphology가 얼마나, 어디가 바뀌는지 묻는다(지지 질문 2). Study 2b(A.6.4)는 한 interface에 맞춘 손가락으로 다른 interface를 쓰면 coverage를 얼마나 잃는지 묻는다(지지 질문 1과 2). 두 부분은 같은 재최적화를 쓰고, 손가락 길이의 취급만 다르다.

Study 2a는 contact interface가 주어졌을 때 그에 맞춰 손가락을 다시 설계하면 어떤 morphology가 되는지 묻는다. 또 interface의 치수가 조금씩 달라질 때 그 최적 morphology가 얼마나, 그리고 어느 부분이 움직이는지 묻는다. Study 1이 손가락을 전혀 바꾸지 않는 극단이라면, Study 2a는 손가락 전체를 interface에 다시 맞추는 반대쪽 극단이다. 몇 개의 module에서만 재지 않고, interface 치수를 연속적으로 바꾸어 가며 최적 morphology를 그 함수로 그린다.

**재최적화.** Contact interface $c=(\ell_1,h_1,\ell_2,h_2)$에 대해, 구조 $K$의 morphology를 다시 최적화한다.

$$
\boxed{
M_K^\star(c;q_{\mathrm{req}})
=
\arg\max_{M\in\mathcal M_K}\;\bar C_{\mathrm{pad}}(M)
\quad\text{s.t.}\quad
\text{study goal}^{-}(q_{\mathrm{req}}),\;\;
C(M)=c,\;\;
\big|L_f(M)-L_{f,0}^K\big|\le\Delta_f
}
$$

- $\text{study goal}^{-}$는 A.6.1의 study goal에서 손가락 길이 조건($L_f=90$ mm)과 pad 길이 하한을 뺀 것이다. 손가락 길이는 아래 범위로, pad 치수는 $C(M)=c$로 대신 정한다.
- 탐색 공간은 native 설계(A.6.1)와 같다. Mechanism 좌표, structure, PIP actuator의 방향 $\alpha_2$, pad의 길이 방향 장착 위치를 탐색한다. 장착 깊이는 native와 같이 장착 규칙(A.6.2)으로 정한다. 2R-remote의 actuator 자세는 A.6.1대로 고정한다.
- Native 탐색과 다른 점은 두 가지이다. Pad 치수를 $c$로 고정하고, 손가락 길이를 기준 길이에서 $\pm\Delta_f$ 안에서 풀어 준다($\Delta_f>0$). 큰 interface를 쓰려면 손가락이 길어져야 할 수 있고, 그 변화도 이 질문의 답이기 때문이다. 손가락 길이를 고정하면 기준 크기에 들어가지 않는 interface는 "해 없음"이 되어, 얼마나 바뀌어야 하는지를 잴 수 없다.
- $\bar C_{\mathrm{pad}}$는 같은 자세의 손가락 길이 $L_f(\mathbf q)$로 나눈 값이므로, 최적화는 pad를 받칠 수 있는 범위에서 짧은 손가락을 선호한다. 따라서 결과를 읽을 때 손가락 크기의 변화($\Delta L_f$)와 나머지 외형의 변화를 나누어 본다.
- 수용 조건은 A.6.2와 같다. 각 pad는 자기 phalanx 구조 위에 전체 길이로 놓이고, $\mathcal F_{\mathrm{static}}$, $\mathcal F_{\mathrm{motion}}$을 만족하며, CFL이면 transmission angle 조건도 만족한다.

**Morphology의 변화는 운동으로 잰다.** 같은 interface 변화를 받는 두 설계를 비교할 때 중요한 것은 손가락이 어떻게 움직이는가이다(`research-objective.md` 5.6: 고정해야 할 것은 articulation kinematics와 link geometry이다). 펼친 모습이 비슷해도 운동은 다를 수 있다. 예를 들어 CFL에서 pivot이 옮겨지면 펼친 외형은 거의 그대로여도 굽히는 동안 회전 중심의 경로가 달라진다. 또 mechanism의 좌표는 구조마다 개수와 의미가 달라 그대로 비교할 수 없다. 따라서 morphology의 변화를 손끝의 운동으로 잰다. 한 flexion에서 distal phalanx의 방향은 mechanism과 무관하게 같으므로, 손끝의 위치가 그 자세 전체를 정한다.

$$
\varepsilon_{\mathrm{kin}}(M)=\frac{1}{L_{f,0}^K}\max_{q\in[0,\,q_{\mathrm{req}}]}\big\|\mathbf r_C(M,(0,q))-\mathbf r_C(M_0^K,(0,q))\big\|
$$

**거의 같은 최적해 가운데 고르기.** 최적값 근처가 평평하면 최적 morphology가 하나로 정해지지 않는다. 격자점마다 다른 해를 고르면 interface가 조금 바뀌어도 morphology가 튄다. 따라서 coverage가 최댓값에서 작은 허용 폭 $\delta_C$ 안에 드는 설계 가운데, 운동이 기준에 가장 가까운 것을 그 interface의 최적 morphology로 정한다.

$$
\hat M_K(c;q_{\mathrm{req}})
=
\arg\min_{M}\;\varepsilon_{\mathrm{kin}}(M)
\quad\text{s.t.}\quad
\bar C_{\mathrm{pad}}(M)\ge\bar C^\star_K(c)-\delta_C,\;\;
\text{위 재최적화의 제약}
$$

여기서 $\bar C^\star_K(c)=\bar C_{\mathrm{pad}}\big(M_K^\star(c)\big)$이다. 선택 기준이 기준 설계 하나에만 의존하므로, 격자를 훑는 순서와 무관하게 정해진다. 이 선택은 "그 interface에서 최선에 가까운 coverage를 내면서, 손가락의 운동을 가장 적게 바꾸는 설계"를 뜻한다.

**보고하는 값.** 각 interface $c$에서 다음을 기록한다.
- 최적 coverage $\bar C^\star_K(c)$. 같은 interface와 같은 goal에서 각 구조가 낼 수 있는 최선이므로, 구조 사이에 직접 비교할 수 있다.
- 최적 외형 $\mathbf d_K^\star(c)=\mathbf d(\hat M_K)$, mechanism 내부 형상 $\boldsymbol\kappa_K^\star(c)=\boldsymbol\kappa(\hat M_K)$, PIP actuator의 방향 $\alpha_2^\star(c)$. $\boldsymbol\kappa$는 mechanism $K$의 좌표 가운데 외형으로 정해지지 않는 나머지이다. CFL에서는 pivot $A$, $B$, $B'$의 위치이고, RCJ에서는 roller 반지름이다. 2R-DD와 2R-remote에서는 외형이 정해지면 mechanism도 정해지므로 $\boldsymbol\kappa$가 없다.
- 운동의 변화 $\varepsilon_{\mathrm{kin}}$(위). Morphology 변화의 주 지표이다.
- 기준 대비 외형과 내부 형상의 변화. 무엇이 바뀌어 운동이 바뀌었는지를 설명하는 보조 지표이다. 성분마다 부호를 남기고, 그 구조의 기준 손가락 길이로 나눈다. Phalanx 길이의 변화는 $x_{\mathrm{art}}$와 $L_f-x_{\mathrm{art}}$의 변화로 읽고, $L_1$, $L_2$의 변화는 pad가 놓인 범위의 변화로 읽는다(A.6.1).

$$
\Delta\mathbf d_K(c)=\frac{\mathbf d_K^\star(c)-\mathbf d_0^K}{L_{f,0}^K},
\qquad
\varepsilon_d=\big\|\Delta\mathbf d_K\big\|_\infty,
\qquad
\varepsilon_\kappa=\frac{\big\|\boldsymbol\kappa_K^\star(c)-\boldsymbol\kappa_0^K\big\|_\infty}{L_{f,0}^K}
$$

- Interface에 대한 민감도. Interface 치수가 조금 바뀔 때 최적 morphology가 움직이는 비율이다. 이웃 격자점 사이의 차분으로 구하며, 양쪽 이웃에서 모두 해를 찾은 점에서만 계산한다.

$$
\frac{\partial\mathbf d_K^\star}{\partial\ell},\quad
\frac{\partial\mathbf d_K^\star}{\partial h},\quad
\frac{\partial\boldsymbol\kappa_K^\star}{\partial\ell},\quad
\frac{\partial\boldsymbol\kappa_K^\star}{\partial h}
$$

**단면.** 4차원 전체를 훑지 않고 대칭 단면 $\ell_1=\ell_2=\ell$, $h_1=h_2=h$ 위의 1차원 단면에서 평가한다.
- 길이 단면: 두께 $h$를 고정하고 길이 $\ell$을 바꾼다.
- 두께 단면: 길이 $\ell$을 고정하고 두께 $h$를 바꾼다.

한 pad만 바꾸는 비대칭 단면은 이후에 같은 방식으로 더한다. 요구 flexion은 $q_{\mathrm{req}}\in\{90^\circ,120^\circ,150^\circ\}$이고, 각각 그 $q_{\mathrm{req}}$의 기준에서 출발한다. 단면의 고정값, 격자 간격, $\Delta_f$, $\delta_C$는 config에서 정한다.

**탐색 규율.**
- 격자점마다 양쪽 이웃 점의 해와 기준 설계를 초기해로 쓰고, 여러 seed로 탐색해 가장 좋은 결과를 남긴다.
- 해를 찾지 못한 점에는 값을 두지 않는다. 이는 주어진 탐색 범위와 예산 안에서 찾지 못했다는 뜻이며, 그런 설계가 존재하지 않는다는 증명은 아니다.
- 최적 morphology가 급격히 바뀌는 곳은 격자를 더 촘촘히 하고, 더 큰 예산과 다른 seed로 다시 확인한다.
- 그림에서는 최적화한 점을 표시하고, 점 사이를 보간한 부분은 그 점들과 구분한다. 최적 morphology가 불연속적으로 바뀌는 곳은 매끄럽게 만들지 않는다.

**해석.** 같은 interface 변화에 대해 $\bar C^\star$가 덜 떨어지는 구조는 그 interface 변화에 더 잘 맞춰질 수 있다. 같은 변화를 받으려고 외형 $\mathbf d^\star$가 덜 움직이는 구조는 interface가 바뀌어도 손가락의 겉모습을 더 많이 유지한다. 외형 대신 mechanism 내부 $\boldsymbol\kappa^\star$가 움직이는 구조는 interface 변화를 mechanism 안에서 흡수한다. 손가락을 다시 설계하지 않았을 때와의 비교는 Study 2b(A.6.4)에서 한다.

### A.6.4. Study 2b: 교차 평가, 한 손가락으로 여러 interface 쓰기

Study 2b는 Study 1과 Study 2a를 하나의 질문으로 묶는다. 어떤 interface에 맞춰 설계한 손가락에 다른 interface를 달면, 그 interface에 맞춰 다시 설계한 손가락에 비해 coverage를 얼마나 잃는가. 이것이 연구의 중심 주장을 재는 값이다. 손가락 하나로 여러 interface를 쓸 수 있는 구조에서는 이 손실이 작다.

**손가락 길이.** Study 2b는 Study 2a의 재최적화를 손가락 길이를 기준에 고정하고($\Delta_f=0$, $L_f=90$ mm) 다시 한다. 손가락 길이를 풀면, 긴 pad에 맞춘 긴 손가락에 짧은 pad를 달 때와 짧은 pad에 맞춘 짧은 손가락에 긴 pad를 달 때 모두 크기 때문에 coverage를 잃는다. 이 손실은 모든 구조에 같이 생겨 구조 사이의 차이를 가린다. 또 실제로 interface를 바꿔 끼우는 손은 크기가 정해져 있다.

**교차 평가.** 단면의 격자점 $c_a$마다 손가락 길이를 고정한 재최적화가 최적 morphology $\hat M_K(c_a)$를 준다. 이 손가락의 mechanism, actuation, structure를 그대로 두고, 같은 단면의 다른 interface $c_b$를 Study 1의 장착 규칙(A.6.2)으로 단다. 탐색은 하지 않는다.

$$
M_K(a\to b)=\big(K,\;C(c_b),\;A\big(\hat M_K(c_a)\big),\;S\big(\hat M_K(c_a)\big)\big)
$$

**유지율.** $c_a$에 맞춘 손가락이 $c_b$를 달고 내는 coverage를, $c_b$에 맞춰 다시 설계했을 때의 최적 coverage로 나눈다.

$$
\boxed{
\rho_K(a\to b)=
\begin{cases}
\dfrac{\bar C_{\mathrm{pad}}\big(M_K(a\to b)\big)}{\bar C^\star_K(c_b)} & M_K(a\to b)\text{가 }q_{\mathrm{req}}\text{에서 }c_b\text{를 수용할 때 (A.6.2)}\\[10pt]
0 & \text{수용하지 못할 때}
\end{cases}
}
$$

여기서 $\bar C^\star_K(c_b)$도 손가락 길이를 고정한 재최적화의 값이다. $\rho=1$이면 다시 설계할 필요가 없고, $\rho=0$이면 그 손가락으로는 그 interface를 쓸 수 없다. 손실(regret)은 $\bar C^\star_K(c_b)\,(1-\rho_K(a\to b))$이다. 수용하지 못한 경우는 그 이유(A.6.2의 어느 조건)를 함께 기록한다.

**일관성.** $M_K(a\to b)$는 $c_b$의 재최적화가 탐색하는 설계 가운데 하나이므로 $\rho\le 1$이어야 한다. 교차 평가에서 $\bar C_{\mathrm{pad}}\big(M_K(a\to b)\big)>\bar C^\star_K(c_b)$가 나오면 재최적화가 최적에 못 미친 것이다. 이때는 그 설계를 초기해로 넣어 $c_b$의 재최적화를 다시 하고, $\bar C^\star_K(c_b)$를 고친 뒤 보고한다. 이 검사가 고친 횟수와 크기는 탐색 품질의 지표로 함께 보고한다. $a=b$에서는 재최적화와 장착 규칙의 pad 위치가 다를 수 있으므로 $\rho(a\to a)\le 1$이며, 이 값은 장착 규칙으로 잃는 양을 보여 준다.

**요약 값.** Interface의 범위(family) $\mathcal C_{\mathrm{fam}}$을 미리 선언하고, 구조마다 다음을 보고한다.
- 유지율 행렬 $\rho_K(a\to b)$ 전체.
- 손가락 하나의 최악 유지율과 그 손가락. 범위 전체에서 가장 나쁜 경우가 가장 좋은 설계점을 고른다.

$$
\rho_K^{\mathrm{one}}=\max_{a}\;\min_{c_b\in\mathcal C_{\mathrm{fam}}}\rho_K(a\to b)
$$

- 기준 interface $c_{\mathrm{ref}}$에 맞춘 손가락 하나의 유지율 곡선 $\rho_K(c_{\mathrm{ref}}\to b)$. 모든 구조에 같은 $c_{\mathrm{ref}}$를 쓴다.

**단면.** A.6.3과 같은 길이 단면과 두께 단면에서 평가한다. 두께 단면이 주 시나리오이다. 두께가 서로 다른 센서와 pad를 한 손가락에 바꿔 다는 것이 실제 교체 상황에 가장 가깝기 때문이다.

Study 1(A.6.2)은 이 행렬에서 설계점을 native 기준으로 둔 한 행에 해당한다. Native 기준은 pad 길이를 탐색해 얻었으므로 구조마다 설계점이 다르다. 교차 평가는 모든 구조에 같은 설계점들을 쓴다.

**해석.** $\rho_K^{\mathrm{one}}$이 높은 구조는 손가락 하나로 선언한 범위의 interface를 모두, 다시 설계한 것에 가까운 coverage로 쓴다. 행렬이 대각선 근처에서만 높은 구조는 interface마다 손가락을 다시 설계해야 한다. Study 2a의 $\Delta\mathbf d$와 민감도는 다시 설계할 때 손가락의 어느 부분이 얼마나 바뀌어야 하는지를 설명한다. 이 결과는 2차원 측면 모델과 이 문서의 coverage 정의 안에서의 비교이며, 실제 교체 가능성은 하드웨어에서 따로 보여야 한다.

## A.7. 기호 정리

**Morphology.** 설계를 이루는 요소.

| 기호 | 의미 |
|---|---|
| $M=(K,C,A,S)$ | Finger morphology: mechanism, contact interface, actuation, structure |
| $\mathcal K$ | 비교 구조 집합: $K_{\mathrm{2R\text{-}DD}}$, $K_{\mathrm{2R\text{-}remote}}$, $K_{\mathrm{CFL}}$, $K_{\mathrm{RCJ}}$ |
| $A, A', B, B'$ | CFL의 pivot. $A, A'$는 proximal phalanx, $B, B'$는 distal phalanx. PIP actuator의 출력축은 $A'$ |
| $C_i=(\ell_i,h_i,\mathbf p_i)$ | Contact interface $i$의 길이, 두께, 장착 위치 ($i=1$: proximal, $i=2$: distal) |
| $A_j=(\mathcal G_j,T_j)$, $T_j=(\mathbf r_j,\alpha_j)$ | Actuator $j$의 housing 형상과 자세(출력축 위치, housing 방향) ($j=1$: MCP actuator, $j=2$: PIP actuator) |
| $\Gamma$ | PIP actuator와 PIP joint 사이의 transmission |
| $\mathbf e_x$ | 기준 자세에서 두 phalanx 축이 놓이는 직선의 방향 단위벡터 |
| $L_1,L_2$ | Proximal phalanx, distal phalanx의 구조 길이: 각 구조가 뻗은 범위, 곧 pad가 놓일 수 있는 범위. 굽는 곳까지의 phalanx 길이($x_{\mathrm{art}}$, $L_f-x_{\mathrm{art}}$)와 다르다 |
| $L_{vp}$ | Virtual pivot length: 기준 자세에서 두 phalanx 구조 사이의 간격 |
| $L_f$ | 손가락 전체 길이 (기준 자세) |
| $\rho_i=L_i/L_f$ | Phalanx 길이 비율 |
| $M_0^K$, $\mathbf d_0^K$ | Study에서 구조 $K$의 기준 morphology(native 설계)와 그 외형 |

**Evaluation.** 설계가 주어지면 계산되는 양.

| 기호 | 의미 |
|---|---|
| $\mathbf q=(q_1,q_2)$ | MCP flexion, 두 phalanx 사이 각도(상대 회전각) |
| $\mathcal G(M,\mathbf q)$ | 자세 $\mathbf q$에서의 2차원 형상 |
| $\mathcal Q_{\mathrm{valid}}$ | 유효한 configuration space: 형상이 존재하고 self-collision이 없는 자세 |
| $\mathcal F_{\mathrm{static}}(M)$ | Static feasibility: $\mathbf 0\in\mathcal Q_{\mathrm{valid}}(M)$, 기준 자세에서 self-collision 없이 조립됨 |
| $\mathcal F_{\mathrm{motion}}(M)$ | Motion feasibility: $\mathcal Q_{\mathrm{req}}\subseteq\mathcal Q_{\mathrm{valid}}(M)$, 요구 운동 구간 전체에서 self-collision 없음 |
| $L_{vp}(\mathbf q)$, $L_f(\mathbf q)$ | 자세 $\mathbf q$에서의 phalanx 간격(유클리드)과 palmar 윤곽 길이 |
| $c_i(M,\mathbf q)$ | 가용 pad 영역의 크기: 두 pad로 함께 원기둥을 잡을 수 있는 부분 ($\le\ell_i$) |
| $C_{\mathrm{pad}}(M,\mathbf q)$ | 자세별 contact pad coverage: $(c_1+c_2)/L_f(\mathbf q)$ |
| $\theta_{\mathrm{pip}}$, $\tau_{\mathrm{pip}}$ | PIP actuator의 입력 각도와 토크 |
| $MA$, $MA_{\mathrm{wc}}$ | Mechanical advantage, 요구 운동 구간에서의 최악값(prototype의 둘째 목적) |
| $\gamma(M,\mathbf q)$ | CFL의 transmission angle: $B'$에서 $B'A$와 $B'B$ 사이 각도 |
| $q_{\max}$ | 편 자세에서 출발해 안전하게(유효하고 transmission angle이 허용 범위 안) 도달할 수 있는 최대 distal flexion |
| $x_{\mathrm{art}}$ | Articulation 위치: 기준 자세에서 순간 회전 중심 $\mathbf r_{\mathrm{ICR}}$의 $\mathbf e_x$ 방향 위치. 펼친 손가락에서 proximal phalanx의 길이(MCP 축부터 굽는 곳까지); distal phalanx는 $L_f-x_{\mathrm{art}}$ |
| $\mathbf d(M)$ | 외형 descriptor $(L_f,L_1,L_2,x_{\mathrm{art}})$ |
| $\boldsymbol\kappa(M)$ | Mechanism 내부 형상: 외형으로 정해지지 않는 mechanism 좌표 |
| $M_K^\star(c)$, $\hat M_K(c)$ | Interface $c$에 맞춘 재최적화의 최적 morphology, 그 가운데 기준 외형에 가장 가까운 것 (Study 2a) |
| $\bar C^\star_K(c)$ | Interface $c$에서의 최적 coverage (Study 2a) |
| $\mathbf d_K^\star(c)$, $\boldsymbol\kappa_K^\star(c)$ | 최적 외형, 최적 mechanism 내부 형상 (Study 2a) |
| $\varepsilon_{\mathrm{kin}}(M)$ | 운동의 변화: $[0,q_{\mathrm{req}}]$에서 손끝 위치가 기준과 가장 멀어지는 거리, $L_{f,0}^K$로 나눔. Study 2a에서 morphology 변화의 주 지표 |
| $\Delta\mathbf d_K(c)$, $\varepsilon_d$, $\varepsilon_\kappa$ | 기준 대비 외형 변화(성분별, $L_{f,0}^K$로 정규화), 그 최댓값, mechanism 내부 변화량 (Study 2a) |
| $\Delta_f$, $\delta_C$ | 재최적화에서 손가락 길이의 허용 범위(Study 2a: $>0$, Study 2b: $0$), 최적 coverage의 허용 폭 |
| $M_K(a\to b)$, $\rho_K(a\to b)$ | $c_a$에 맞춘 손가락에 $c_b$를 단 morphology, 그 유지율: 다시 설계한 최적 coverage 대비 비율 (Study 2b, A.6.4) |
| $\mathcal C_{\mathrm{fam}}$, $\rho_K^{\mathrm{one}}$ | 선언한 interface 범위, 손가락 하나로 그 범위에서 얻는 최악 유지율의 최댓값 (Study 2b, A.6.4) |

**Optimization.** 요구사항, 허용 범위, 목적함수와 제약.

| 기호 | 의미 |
|---|---|
| $\mathbf m(M)$, $\mathbf m_{\min}$, $\mathbf m_{\max}$ | 형태 descriptor와 그 요구 범위 |
| $\ell_{i,\min}$ | Pad $i$ 길이의 하한 |
| $[\alpha_{\min},\alpha_{\max}]$ | PIP actuator housing 방향 $\alpha_2$의 허용 범위 |
| $q_{\mathrm{req}}$, $\mathcal Q_{\mathrm{req}}$ | 요구 distal flexion, 요구 운동 구간 |
| $\mathcal Q_{\mathrm{eval}}$, $\bar C_{\mathrm{pad}}$ | Coverage 평가 자세 $\{90^\circ,\ (90^\circ+q_{\mathrm{req}})/2,\ q_{\mathrm{req}}\}$, 그 평균(prototype의 첫째 목적, study의 목적) |
| $MA_{\min}$ | Mechanical advantage의 하한 (prototype의 CFL 제약; study에는 없음) |
| $[\gamma_{\min},\gamma_{\max}]$ | Transmission angle의 허용 범위 (CFL 제약) |
