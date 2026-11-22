# Appendix A. 수학적 정식화

이 부록은 손가락 설계 문제의 전체 구조를 정의한다. 흐름은 다음과 같다.

1. **Morphology (A.1).** 손가락을 mechanism, contact interface, actuation, structure의 네 요소로 기술한다.
2. **요구사항 (A.2–A.3).** Contact interface의 치수와 손가락의 크기·비율을 하한과 상한으로 지정한다.
3. **기하와 운동 (A.4).** Morphology는 joint 자세에 따라 형상을 만들고, 그 형상에 self-collision이 없는 자세의 집합이 유효한 운동 범위를 정한다.
4. **최적화 (A.5).** 요구사항, 요구 운동, 전달 성능 조건을 만족하는 morphology 가운데 contact pad coverage가 가장 큰 것을 찾는다.
5. **비교 연구 (A.6).** 해석(Study 0)이 관절이 contact interface에서 빼앗는 것과 접촉면 상대 운동(proximal phalanx 좌표계에서 본 distal 접촉점의 경로) 변화의 하한을 유도하고, 구조마다 자기 최적 설계로 두 가지를 확인한다. 두께와 요구 flexion에 따라 각 구조가 확보하는 coverage(Study 1), 더 두꺼운 interface에 맞춰 다시 설계할 때 접촉점 경로를 얼마나 지키는가(Study 2). 구조끼리 외형을 맞추지 않는다.

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

이 절은 해석 하나와 그것을 확인하는 최적화 실험 둘로 이루어진다. 해석(Study 0)이 관계식과 예측을 주고, Study 1과 Study 2가 최적화 결과로 그 예측을 확인한다. `research-objective.md` 6절이 보이라고 한 것과의 대응은 다음과 같다.

| 보여야 할 것 (research-objective 6절) | 근거 | 절 |
|---|---|---|
| 관절이 contact interface에서 빼앗는 것을 정하는 관계 | Study 0 (해석) | A.6.0 |
| 1. 구조가 contact interface를 제한한다 | 구조별 native 설계의 비교, Study 1 | A.6.1, A.6.2 |
| 2. Interface의 두께와 요구 flexion에 따라 구조가 확보하는 접촉면이 다르다 | Study 1 | A.6.2 |
| 3. Interface가 두꺼워져도 접촉점 경로(proximal phalanx 좌표계)를 지키도록 다시 설계할 수 있는지가 구조마다 다르다 | Study 2 | A.6.3 |

### A.6.0. Study 0: 관절이 pad에서 빼앗는 길이 (해석)

Study 0은 최적화 없이, 요구(pad 두께 $h$, 요구 flexion $q_{\mathrm{req}}$)와 morphology 사이의 관계를 기하로 유도한다. 명제의 정확한 가정과 증명은 `report/study0/derivation.md`가 기준이며, 이 절은 그 요약이다. 계산은 `core/geometry/pole.py`, 최적화 결과와의 대조는 `scripts/study/study2_morphology_adaptation/pole_law.py`가 한다.

**Pole.** 평면에서 방향이 바뀌는 강체 변위는 한 점을 중심으로 한 회전과 같다. Flexion $q$에서 distal phalanx의 자세는 기준 자세를 점 $\mathbf P(M,q)$를 중심으로 $q$만큼 돌린 것이며, 이 점을 (finite) pole이라 부른다. Pole은 기준 자세에서 그 자세까지의 유한한 변위의 성질이고, 순간 회전 중심과는 다르다(pole이 고정된 관절에서만 같다). Pole이 link 선(기준 자세에서 관절 축들을 잇는 직선)에서 palmar 쪽으로 내려온 깊이를 $z(q)$라 쓴다(dorsal 쪽이면 음수).

**접촉면의 깊이.** Pad $i$의 contact face가 link 선에서 palmar 쪽으로 떨어진 거리를 $t_i=d_i+h_i$라 쓴다. $d_i$는 장착 깊이(link 선에서 pad의 윗면까지)이다. Pole에서 본 contact face의 깊이는 $\tau_i(q)=t_i-z(q)$이다.

**관절 간격.** 기준 자세에서 두 pad의 관절 쪽 끝 사이의 길이를 $G$라 한다. Proximal pad가 $x_0$에서 시작하고 distal pad가 손끝에서 끝나면 $(\ell_1+\ell_2)/L_f=1-(x_0+G)/L_f$이다.

**명제 1 (일반 clearance).** 임의의 pole과 유한한 두 직사각형 pad에 대해, 자세 $q$에서 두 pad가 양의 넓이로 겹치지 않을 필요충분조건은 네 축 $\{\mathbf e_x,\mathbf e_z,R_q\mathbf e_x,R_q\mathbf e_z\}$ 가운데 두 pad의 사영이 분리되는 축이 있는 것이다(separating axis). 그 틈의 최댓값을 $m(q)$라 하면 설계가 만족해야 하는 조건은

$$
\min_{0<q\le q_{\mathrm{req}}}m(q)\ \ge 0
$$

이다. Pole의 위치, 두 pad의 깊이, 길이, 두께에 조건이 없다. 설계의 판정에는 이 관계를 쓴다. 한 자세에서의 판정은 정확하고, 구간에 대한 조건은 $q$의 격자에서 확인한다(연속 구간에 대한 인증은 아니다).

**명제 2 (대칭 clearance 법칙).** $q_{\mathrm{req}}<\pi$에서, 두 contact face의 깊이가 같고(S1), 모든 자세에서 두 pad의 관절 쪽 아래 모서리가 pole에서 $x$ 방향으로 같은 거리에 있으면(S2, 곧 pole의 $x$ 좌표가 굽히는 동안 두 모서리의 가운데에 머물면), $(0,q_{\mathrm{req}}]$ 전체에서 겹치지 않는 가장 작은 관절 간격은

$$
\boxed{
G_{\mathrm{sym}}(h,q_{\mathrm{req}})=\max_{0<q\le q_{\mathrm{req}}}\;2\,\big[d+h-z(q)\big]_+\tan\frac{q}{2}
}
$$

이다. 최댓값을 주는 flexion을 임계 flexion $q^\ast$라 한다. 이 식은 구간 전체에 대한 조건이며 한 자세에서의 필요충분조건이 아니다. (S2)가 깨지면 추정값이고, pole에 대해 고르지 않게 놓은 배치는 더 작은 간격으로도 겹치지 않을 수 있다. $z(q)$와 $d$가 고정이면 $G_{\mathrm{sym}}$은 $h$에 대해 볼록하고, $q^\ast$가 유일한 곳에서 $dG_{\mathrm{sym}}/dh=2\tan(q^\ast/2)$이다.

**명제 3 (coverage).** 두 contact face의 깊이가 같으면 그 연장선은 각 face 위에서 pole의 수선의 발로부터 $\tau\tan(q/2)$ 떨어진 점 $X$에서 만난다. A.5.1의 coverage는 $X$에서 같은 거리만큼만 세므로, $X$보다 관절 쪽에 놓인 pad의 부분은 세지 않는다. (S1), (S2)이고 두 pad의 길이가 같으면

$$
C_{\mathrm{pad}}(q)=\frac{L_f-x_0-G}{L_f-G\,(1-\cos\frac{q}{2})+2\,z(q)\sin\frac{q}{2}}
$$

이다. 분모의 둘째 항은 관절 간격이 굽히면 짧아지는 것이고, 셋째 항은 pole이 palmar 쪽에 있으면 두 phalanx 끝이 벌어지는 것이다. 필요한 것보다 깊은 pole($z>t$)은 $G$를 줄이지 못하고 분모만 키운다. 이 조건 밖의 설계에서는 $X$, 덮는 거리, 굽힌 윤곽을 pole과 pad의 치수로 직접 계산한다.

**명제 4 (접촉점 경로).** Contact face를 기준으로 본 pole의 위치(두 성분 모두)를 $\mathbf p(q)$라 하면, contact face 위의 점은 $\mathbf c(q)=\mathbf p+R_q(\mathbf c_0-\mathbf p)$로 움직인다. 두 손가락의 pole이 contact face에 대해 $\Delta\mathbf p(q)$만큼 다르면

$$
\boxed{\;\|\Delta\mathbf c(q)\|=2\sin\frac{q}{2}\,\|\Delta\mathbf p(q)\|\;}
$$

이다. 기준 자세에서 두 contact face의 상대 위치가 같을 때, 접촉점 경로(A.6.3의 $\mathbf c(q)$)가 같을 필요충분조건은 $\mathbf p(q)$가 모든 $q$에서 같은 것이다(깊이 $\tau$만이 아니라 pole의 $x$ 좌표도). 그때 같은 pad를 같은 자리에 그대로 실을 수 있다(명제 1의 $m(q)$가 같다).

**구조는 $z(q)$, $P_x(q)$, $d$로 들어간다.**

| 구조 | $z(q)$ | $P_x(q)$ | $d$를 정하는 것 |
|---|---|---|---|
| 2R-DD, 2R-remote | $0$ (관절 축) | 상수 | 2R-remote: 없음. 2R-DD: proximal pad가 관절 actuator 밑으로 들어가는 깊이 |
| RCJ | $r\tan(q/4)$ ($r$: roller 반지름) | 상수 | pad가 roller 밑으로 들어가는 깊이 |
| CFL | Linkage 치수가 정함 | Linkage 치수가 정함 | Pivot housing |

- 고정 축: 두 pad의 깊이가 같고 축에 대해 고르게 놓으면 $q^\ast=q_{\mathrm{req}}$, $G_{\mathrm{sym}}=2(d+h)\tan(q_{\mathrm{req}}/2)$이다.
- 구름 접촉: $t\ge r\sin(q_{\mathrm{req}}/2)$이면 $q^\ast=q_{\mathrm{req}}$이고, 아니면 최대가 구간 안쪽 $q^\ast=2\arcsin(t/r)$에서 생긴다.
- Crossed four-bar 일반에 대한 주장은 없다. Study의 최적화가 찾은 CFL 설계에서는 $z(q)$가 굽힐수록 커지고 $q^\ast<q_{\mathrm{req}}$이며, $P_x(q)$가 움직이는 설계가 있다(관찰).

**가설과 예측.** 명제 1–4는 기하의 정리이다. 아래는 최적화 결과에 대한 것이며 Study 1과 Study 2가 확인한다.
- H1 (경험적 가설). Coverage를 최대화한 설계에서는 pad끼리의 접촉이 자주 활성 제약이다: 설계의 pad로 계산한 $\min_q m(q)$가 0에 가깝다. 정리가 아니다. Actuator의 자리, pad 길이의 하한, 다른 제약이 먼저 걸리면 틈이 남는다. 성립한 설계의 수와 벗어난 설계를 보고한다. 대칭 법칙은 그 설계의 $G$에 대한 추정으로 함께 보고한다.
- P2 (계산의 일치). 최적 설계의 $\bar C_{\mathrm{pad}}$는 pole의 경로, pad의 치수, phalanx 구조의 끝, 기준 자세의 distal link 선으로 직접 계산한 값과 같다(명제 3).
- P3. 고정 축에서 두 pad가 고르게 놓인 설계는 $G=2(d+h)\tan(q_{\mathrm{req}}/2)$를 따른다. 요구 flexion과 두께가 커지면 pad 길이가 이 식대로 줄어든다. $q^\ast<q_{\mathrm{req}}$인 설계에서는 $G_{\mathrm{sym}}$이 $q_{\mathrm{req}}$에 따라 커지지 않는다.
- P4. 두 pad의 깊이가 같은 고정 축 손가락을 contact face가 축에서 $\delta=\Delta h-\Delta d$만큼 멀어지도록 다시 설계하면($\Delta d$: 장착 깊이를 줄인 양), 접촉점 경로는 $2\sin(q_{\mathrm{req}}/2)\,|\delta|$ 이상 바뀐다. 등호는 축이 contact face에 대해 $x$ 방향으로 움직이지 않을 때이다. 이것은 어떤 다시 설계로도 넘을 수 없는 하한이고, $r$이 고정된 구름 접촉에서도 같은 형태이다. Pole을 옮겨 $\mathbf p(q)$를 유지하는 구조는 접촉점 경로와 pad를 함께 유지한다(명제 4).
- 이 모델에서 가장 좋은 관절은 contact face 위의 점을 중심으로 도는 관절이다($\tau=0$, $G=0$).

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
- Coverage를 계산할 때 transmission 요소(링크, belt)가 pad를 가리는 효과는 기본적으로 고려하지 않는다. 이 요소들은 pad와 다른 lateral layer에 놓이기 때문이다(A.4). Study에서는 가림을 고려한 분석을 따로 하지 않는다.

### A.6.2. Study 1: pad 두께와 요구 flexion에 따른 최대 coverage

Study 1의 질문은 하나이다. Contact interface의 두께와 요구 flexion이 주어졌을 때, 각 구조에서 최적화한 손가락은 contact coverage를 얼마나 확보하는가.

**조건의 격자.** Pad 두께 $h$(두 pad 같은 두께)와 요구 flexion $q_{\mathrm{req}}$의 격자에서 평가한다. 값은 config에서 정하고 결과와 함께 보고한다(본 실험: $h\in\{3,5,7,9\}$ mm, $q_{\mathrm{req}}\in\{90^\circ,120^\circ,150^\circ\}$).

**최적화.** 격자의 각 조건에서 구조 $K$의 morphology를 최적화한다. 목적함수는 원기둥 기준 pad coverage $\bar C_{\mathrm{pad}}$(A.5.1) 그대로이며 바꾸지 않는다.

$$
\boxed{
\bar C^\star_K(h,q_{\mathrm{req}})=\max_{M\in\mathcal M_K}\;\bar C_{\mathrm{pad}}(M)
\quad\text{s.t.}\quad
\text{study goal}(q_{\mathrm{req}}),\;\;h_1=h_2=h
}
$$

- 이것은 A.6.1의 native 설계 탐색을 두께마다 반복하는 것이다. 탐색 공간, 제약, 탐색 예산은 A.6.1과 같다. Pad의 길이와 위치는 설계 변수이고, 두께만 조건으로 준다.
- $h=5$ mm의 결과는 A.6.1의 native 설계 $M_0^K(q_{\mathrm{req}})$ 자체이다.
- 설계를 찾지 못한 조건은 값을 두지 않고 그 사실을 보고한다.

**보고.** 구조마다 $q_{\mathrm{req}}$별 곡선 $h\mapsto\bar C^\star_K(h,q_{\mathrm{req}})$를 보고한다. 그림에서는 pad coverage $(\ell_1+\ell_2)/L_f$를 함께 보여, 손가락 길이 가운데 pad가 차지하는 비율로 읽을 수 있게 한다(목적함수는 $\bar C_{\mathrm{pad}}$). 각 최적 설계의 pad 길이, $q_{\max}$, $MA_{\mathrm{wc}}$, 외형 $\mathbf d$를 표로 남긴다.

**Study 0의 확인.** 각 최적 설계에 대해 설계의 pad로 계산한 정확한 clearance의 최솟값이 0에 가까운지(H1), $\bar C_{\mathrm{pad}}$가 직접 계산한 값과 같은지(P2), 두께와 요구 flexion에 따른 $G$의 변화가 P3을 따르는지 본다. Clearance가 남는 설계는 pad가 한계가 아니거나 탐색이 한계에 닿지 못한 것이므로 그 수와 이유를 보고한다.

**해석.** 곡선이 높은 구조는 같은 크기의 손가락에서 더 많은 접촉면을 확보한다. 두께가 늘 때 곡선이 덜 떨어지는 구조는 두꺼운 interface를 쓰는 데 드는 비용이 작다. 요구 flexion이 클 때 곡선이 덜 떨어지는 구조는 깊이 굽혀도 접촉면을 유지한다. 각 점은 그 조건만을 위해 설계한 손가락이므로, 이 곡선은 구조가 그 조건에서 낼 수 있는 상한이다. 손가락 하나로 여러 두께를 쓰는 경우는 Study 2에서 본다. 이 결과는 2차원 측면 모델에서의 비교이다.

**고정된 손가락에 module을 다는 규칙.** 손가락을 그대로 두고 contact interface만 바꿔 다는 평가(2026-11-20까지의 Study 1·Study 2 정의, 지금은 보조 자료)는 다음 규칙을 쓴다. 기준 morphology에서 mechanism, actuation, structure $(K,A_0^K,S_0^K)$는 그대로 두고 contact interface의 치수만 바꾼다. 장착 위치 $\mathbf p_i$는 탐색하지 않고 정해진 장착 규칙으로 결정한다. 각 pad는 바깥 끝(proximal pad는 MCP 쪽, distal pad는 손끝 쪽)에 맞춰 놓는다. 깊이는 자기 phalanx의 joint 하드웨어와 겹치지 않는 가장 얕은 위치로 하되, 그 phalanx 구조가 받칠 수 있는 범위 안이어야 한다. 이런 위치가 없으면 module을 장착할 수 없다. Spacer는 쓰지 않는다. Study 1과 Study 2의 탐색에서 pad의 깊이도 같은 규칙(가장 얕은 깊이, spacer 없음)을 따른다. 이렇게 얻은 morphology를 다음과 같이 쓴다.

$$
M^K(c)=\big(K,\;C(c),\;A_0^K,\;S_0^K\big),

$$

2026-11-20까지 Study 1은 native 설계에 module 목록 $\mathcal C_{\mathrm{cat}}$을 이 규칙으로 달아 수용 여부를 세는 실험이었다(`report/study1/`). 그 결과는 보조 자료로 남긴다.

### A.6.3. Study 2: interface가 바뀌어도 proximal phalanx에서 본 distal 접촉점의 경로를 지킬 수 있는가

Study 1은 접촉면을 얼마나 확보하는가를 묻고, Study 2는 그 접촉면이 서로에 대해 어떻게 움직이는지를 묻는다. 더 두꺼운 contact interface를 쓰려고 손가락을 다시 설계할 때, proximal phalanx에서 본 distal 접촉점의 경로(아래의 $\mathbf c(M,q)$, 줄여서 접촉점 경로)를 기존 손가락과 같게 유지할 수 있는가, 그리고 그때 coverage는 얼마나 남는가. Contact interface를 독립 변수로 다루려면 접촉이 일어나는 면의 운동이 유지되어야 하고, 뼈대의 운동은 그 수단이다. 이 실험에서는 접촉점 경로가 처음으로 설계의 목표가 된다. Study 0의 P4가 결과를 예측한다.

**기준 손가락과 새 interface.** 구조 $K$와 요구 flexion $q_{\mathrm{req}}$마다, Study 1에서 가장 얇은 두께 $h_0$에 최적화한 설계 $M_0=M^\star_K(h_0)$를 기준 손가락으로 둔다(본 실험 $h_0=3$ mm). 새 interface는 Study 1의 격자에서 더 두꺼운 두께 $h>h_0$이다. 얇은 쪽으로 바꾸는 경우는 다루지 않는다. 기존 손가락에 그대로 달 수 있어 다시 설계할 이유가 없기 때문이다.

**기준 좌표계.** Proximal phalanx에 붙어 MCP에서 함께 도는 좌표계 $\mathcal F_p$를 쓴다. 원점은 MCP 축에서 proximal contact face에 내린 수선의 발, $x$축은 face를 따라 손끝 쪽, $z$축은 face의 법선이다. Proximal contact face는 이 좌표계에서 고정된 직선 $z=0$이다.

**접촉점 경로와 그 자코비안.** Distal contact face 위의 손끝 아래 점을 $\mathcal F_p$에서 본 위치를 $\mathbf c(M,q)$라 한다. 이 경로는 flexion $q$로 매개화되며 시간의 함수가 아니다. 그 자코비안은 다음과 같다.

$$
\mathbf J(M,q)=\frac{d}{dq}\begin{bmatrix}\mathbf c(M,q)\\ \varphi(M,q)\end{bmatrix}=\begin{bmatrix}\mathbf c'(M,q)\\ 1\end{bmatrix}
$$

두 face는 어느 설계에서나 $q$만큼 돌므로($\varphi=q$, 명제 4) 한 점의 경로가 distal face의 자세를 정하고, 설계는 $\mathbf c'(M,q)$로만 다르다. $\mathbf c'$는 distal face가 proximal face에 대해 도는 순간 회전 중심(pole)의 위치와 같은 정보다. $\mathbf J$는 손바닥 좌표계에서 본 손가락 contact Jacobian의 PIP 열이다. MCP 열은 지렛대에 pad 깊이가 들어가서 interface 두께에 따라 바뀌고, 다시 설계로 지킬 수 없다(research-objective §8.2). Servo 각에서 $q$로 가는 전달(속도와 MA)은 포함하지 않는다. 그것은 mechanism마다 다르며 재설계로 바뀔 수 있고, 따로 기록하고 제어로 보상한다.

**그 변화.**

$$
\Delta_{\mathrm{contact}}(M,M_0)=\max_{q\in[0,\,q_{\mathrm{req}}]}\big\|\mathbf c(M,q)-\mathbf c(M_0,q)\big\|
$$

$\mathbf c(M,q)-\mathbf c(M_0,q)=[\mathbf c(M,0)-\mathbf c(M_0,0)]+\int_0^q\big(\mathbf c'(M,s)-\mathbf c'(M_0,s)\big)\,ds$이다. 그래서 $q=0$의 자세가 같으면, $\Delta_{\mathrm{contact}}=0$과 $[0,q_{\mathrm{req}}]$에서 $\mathbf J$가 같다는 것은 동치이다. 일반적으로 $\Delta_{\mathrm{contact}}\le\|\Delta\mathbf c(0)\|+\int_0^{q_{\mathrm{req}}}\|\Delta\mathbf J\|\,dq$이다. 원점을 proximal contact face 위에 두므로 pad 깊이의 차이 자체는 $\Delta_{\mathrm{contact}}$에 들어가지 않는다. 단위는 mm이며, 코드는 $L_f$로 나눈 값을 `contact_change`로 기록한다.

뼈대의 운동 변화 $\Delta_{\mathrm{skel}}$은 같은 식을 $\mathcal F_p$에서 본 손끝 $\mathbf r_C$의 경로로 잰 것이며(코드의 `skeleton_change`는 $L_f$로 나눈 값) 함께 보고한다.

**두 변화는 같은 pole의 변화를 서로 다른 기준에서 본 것이다.** 접촉점과 손끝은 모두 distal phalanx 위의 점이므로 같은 상대 운동($\mathbf J$의 pole)을 따른다. $q=0$에서의 변위를 회전 $R_q$와 유한 pole $\mathbf p(q)$로 쓰면(Study 0, 명제 4), $q=0$에서 같은 자리에 있던 점은 두 손가락에서 $\lVert\Delta\mathbf x(q)\rVert=2\sin\tfrac q2\,\lVert\Delta\mathbf p(q)\rVert$만큼 벌어지며, 이 값은 점에 따르지 않는다. 그래서

$$
\lVert\Delta\mathbf c(q)\rVert=2\sin\tfrac q2\,\big\lVert\Delta\mathbf p_{\mathrm{face}}(q)\big\rVert,\qquad
\lVert\Delta\mathbf r_C(q)\rVert=2\sin\tfrac q2\,\big\lVert\Delta\mathbf p_{\mathrm{link}}(q)\big\rVert,\qquad
\Delta\mathbf p_{\mathrm{face}}=\Delta\mathbf p_{\mathrm{link}}-\Delta d\,\mathbf e_z
$$

이다. $\Delta\mathbf p_{\mathrm{face}}$는 proximal contact face에서, $\Delta\mathbf p_{\mathrm{link}}$는 proximal 링크 선(뼈대)에서 본 pole의 변화이고, $\Delta d$는 face 깊이의 변화(가장 얕은 장착에서 $\Delta h$)이다. 등식은 $q=0$의 위치가 같을 때 성립하고, 그 차이($\lVert\Delta\mathbf c(0)\rVert$, $\lVert\Delta\mathbf r_C(0)\rVert$)만큼 어긋난다. 따라서 $\Delta_{\mathrm{contact}}$를 지킨다는 것은 pole을 contact face에 대해 지키는 것이고, $\Delta_{\mathrm{skel}}$을 지킨다는 것은 pole을 뼈대에 대해 지키는 것이다. pad가 $\Delta d$만큼 깊어지면 둘을 함께 지킬 수 없다. 고정 축은 pole이 뼈대에 붙어 있어 $\Delta\mathbf p_{\mathrm{link}}$를 0 가까이 둘 수밖에 없으므로 $\lVert\Delta\mathbf p_{\mathrm{face}}\rVert\ge\Delta d$이고, 이것이 따름정리(P4)의 하한이다. 어느 구조든 삼각부등식으로 $\lVert\Delta\mathbf p_{\mathrm{face}}\rVert+\lVert\Delta\mathbf p_{\mathrm{link}}\rVert\ge|\Delta d|$이므로, $q=0$의 위치가 같을 때 $\Delta_{\mathrm{contact}}+\Delta_{\mathrm{skel}}\ge2\sin(q_{\mathrm{req}}/2)\,|\Delta d|$이다(합 하한). 같은 두께에서 두 변화는 서로 맞바뀌므로, 둘 중 무엇이 측정 오차를 예측하는지는 한쪽을 고정한 partial 상관으로만 가를 수 있다.

**다시 설계.** 두께 $h$에서 두 목적을 함께 최적화한다.

$$
\boxed{
\min_{M\in\mathcal M_K}\ \Big(\Delta_{\mathrm{contact}}(M,M_0),\ -\bar C_{\mathrm{pad}}(M)\Big)
\quad\text{s.t.}\quad
\text{study goal}(q_{\mathrm{req}}),\;\;h_1=h_2=h
}
$$

- 탐색 공간, 제약, 장착 규칙은 Study 1과 같다(가장 얕은 깊이, spacer 없음). 목적함수의 coverage도 Study 1의 원기둥 기준 $\bar C_{\mathrm{pad}}$ 그대로이다.
- Coverage의 하한은 두지 않는다. 결과는 front이며, 접촉점 경로를 얼마나 지키면 coverage가 얼마나 남는지를 그대로 보인다.
- Study 1의 설계 $M^\star_K(h)$는 이 탐색의 초기해이므로 front의 coverage 쪽 끝은 $\bar C^\star_K(h)$보다 낮지 않다. 더 높게 나오면 Study 1의 탐색이 최적에 못 미친 것이므로 그 값을 함께 보고한다.

**보고.** 구조, $q_{\mathrm{req}}$, 두께마다
- front $\{(\Delta_{\mathrm{contact}},\bar C_{\mathrm{pad}})\}$;
- 변화가 가장 작은 설계의 $\Delta_{\mathrm{contact}}^{\min}$과 그 coverage, coverage가 가장 높은 설계의 coverage와 그 $\Delta_{\mathrm{contact}}$;
- 고정 축의 하한 $2\sin(q_{\mathrm{req}}/2)\,(h-h_0)$ (A.6.0 P4);
- Study 1의 설계 $M^\star_K(h)$의 $\Delta_{\mathrm{contact}}$(접촉점 경로를 목표로 주지 않았을 때의 값);
- 뼈대의 변화 $\Delta_{\mathrm{skel}}$, pad 길이, $q_{\max}$, $MA_{\mathrm{wc}}$.

결과는 seed를 바꿔 반복해 front를 합친다.

**확인하는 것 (P4).**
- 고정 축(2R-remote)은 어떤 다시 설계로도 $\Delta_{\mathrm{contact}}$가 하한보다 작아지지 않는다. Front는 그 하한의 한 점으로 모인다.
- 2R-DD와 RCJ는 장착 깊이를 줄인 만큼만 하한보다 작다.
- CFL은 하한보다 훨씬 작은 $\Delta_{\mathrm{contact}}$로 다시 설계되며, 그때 coverage는 $\bar C^\star_K(h)$에 가깝다(접촉점 경로를 지키는 것과 pad 길이를 지키는 것은 같은 조건이므로). 두께가 늘면서 이것이 어디까지 성립하는지가 결과이다.

**해석.** $\Delta_{\mathrm{contact}}^{\min}$이 작고 그때의 coverage가 높은 구조는, interface가 두꺼워져도 접촉점 경로와 접촉면의 양을 함께 지키도록 다시 설계할 수 있다. 이때 바뀌는 것은 mechanism 내부와 뼈대의 운동 $\Delta_{\mathrm{skel}}$이다. Front가 하한의 한 점에 모이는 구조는 interface가 두꺼워지면 접촉점 경로가 반드시 바뀐다. 해를 찾지 못했다는 것은 주어진 탐색 범위와 예산 안에서 찾지 못했다는 뜻이며, $\Delta_{\mathrm{contact}}^{\min}$은 찾은 값이지 증명된 최솟값이 아니다. 이 결과는 2차원 측면 모델, 두 pad의 두께가 같은 경우, 장착 규칙(spacer 없음) 안에서의 비교이다. 얇은 pad를 spacer로 가장 두꺼운 pad의 깊이까지 올려 달면 어느 구조에서나 접촉점 경로가 유지된다. 그러나 이것은 interface가 쓸 공간을 기능 없는 받침으로 채우고, 얇은 interface마다 깊은 설계의 coverage만 남기며(120°에서 3 mm를 9 mm 설계에 달면 CFL 8.6 %p, 2R-remote 18.2 %p 손실), 쓸 수 있는 가장 두꺼운 interface를 미리 정해야 한다. 그래서 spacer는 비교 대상이 아니라 요구조건으로 배제한다. 모든 interface는 자기 두께의 가장 얕은 깊이에 달고, 그 조건 아래에서 접촉점 경로의 변화를 최소화한다.

### A.6.4. Study에서 다루지 않는 것

다음은 다루지 않으며, 필요해지면 따로 정의한다.
- 손가락 하나에 여러 두께를 바꿔 다는 경우, 길이와 두께의 격자에서 pad 활용률로 수용 여부를 세는 정의, 선언한 범위(family)의 최악 활용률, tolerance 곡선: 2026-11-20까지의 Study 2 정의였으나 뺐다. Study 0의 관계에서, 손가락을 고정하면 어느 구조도 더 두꺼운 pad를 요구 flexion까지 쓰지 못한다.
- CFL이 pole을 내릴 수 있는 한계의 해석. Study 2의 탐색이 찾은 값으로 대신한다.
- 손가락 길이를 푸는 실험, 두 pad의 두께가 다른 interface.
- Transmission 요소의 가림과 mechanical advantage 하한. 가림은 study 전체에서 고려하지 않고(A.6.1), $MA_{\mathrm{wc}}$는 제약 없이 결과로만 보고한다.

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
| $\bar C^\star_K(h,q_{\mathrm{req}})$, $M^\star_K(h)$ | 두께 $h$와 요구 flexion마다 최적화한 손가락의 coverage와 그 설계 (Study 1) |
| $\mathbf P(M,q)$, $z(q)$ | Pole: flexion $q$의 자세가 기준 자세를 그 점을 중심으로 돌린 것이 되는 점, 그 점이 link 선에서 palmar 쪽으로 내려온 깊이 (Study 0) |
| $d_i$, $t_i$, $\tau_i(q)$ | 장착 깊이, contact face의 깊이 $d_i+h_i$, pole에서 본 contact face의 깊이 $t_i-z(q)$ |
| $G$, $G_{\mathrm{sym}}$, $q^\ast$ | 관절 간격(기준 자세에서 두 pad의 관절 쪽 끝 사이 길이), 대칭 clearance 법칙이 주는 값, 그것을 정하는 임계 flexion |
| $m(q)$ | 자세 $q$에서 두 pad의 정확한 clearance: 분리축에서의 틈의 최댓값(0: 맞닿음, 음수: 겹침) (Study 0, 명제 1) |
| $\mathbf p(q)$, $\mathbf c(M,q)$ | Contact face를 기준으로 본 pole의 위치, proximal contact face를 기준으로 본 distal contact face 위의 점의 경로 |
| $\Delta_{\mathrm{contact}}(M,M')$, $\Delta_{\mathrm{skel}}(M,M')$ | 두 손가락의, proximal phalanx 좌표계 $\mathcal F_p$에서 본 distal 접촉점 경로의 차이와 손끝 경로(뼈대 운동)의 차이 [mm]. $[0,q_{\mathrm{req}}]$에서의 최대 거리를 $L_f$로 나눔 (Study 2) |
| $\mathcal F_p$, $\mathbf c(M,q)$, $\mathbf J(M,q)$ | Proximal phalanx 좌표계(MCP에서 함께 돎, 원점은 proximal contact face 위), 그 좌표계에서 본 distal 접촉점의 경로, 그 경로와 distal face 자세의 $q$에 대한 자코비안 $[\mathbf c';1]$ (손가락 contact Jacobian의 PIP 열; Study 2) |
| $M_0$, $h_0$, $\Delta_{\mathrm{contact}}^{\min}$ | Study 2의 기준 손가락(가장 얇은 두께 $h_0$의 Study 1 설계)과, 더 두꺼운 두께에서 다시 설계해 찾은 가장 작은 $\Delta_{\mathrm{contact}}$ |

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
