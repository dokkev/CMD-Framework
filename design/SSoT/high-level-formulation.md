# Appendix A. 수학적 정식화

이 부록은 손가락 설계 문제의 전체 구조를 정의한다. 흐름은 다음과 같다.

1. **Morphology (A.1).** 손가락을 mechanism, contact interface, actuation, structure의 네 요소로 기술한다.
2. **요구사항 (A.2–A.3).** Contact interface의 치수와 손가락의 크기·비율을 하한과 상한으로 지정한다. 이 요구사항은 하나의 형상이 아니라 허용 가능한 morphology의 영역을 정한다.
3. **기하와 운동 (A.4).** Morphology는 joint 자세에 따라 형상을 만들고, 그 형상에 self-collision이 없는 자세의 집합이 유효한 운동 범위를 정한다.
4. **최적화 (A.5).** 요구사항과 요구 운동을 만족하는 morphology 가운데, contact pad coverage, mechanical advantage, 운동 범위 사이의 trade-off를 찾는다.

Contact interface는 이 문제의 출발점이다. 그 치수는 요구사항으로 먼저 주어지고, mechanism, actuation, structure는 그것을 수용하도록 설계된다.

## A.1. Finger Morphology

Finger morphology $M$은 네 요소로 이루어진 결과 설계로 정의한다.

$$
M=(K,C,A,S)
$$

$K$, $C$, $A$, $S$는 각각 mechanism, contact interface, actuation, structure를 나타낸다.

Mechanism $K$는 link, joint, 기구학적 구속을 포함한 기구학적 구조를 정의한다. 이 틀 안에서 여러 mechanism 구조를 표현할 수 있으며, serial revolute joint(2R), crossed four-bar linkage(CFL), rolling contact joint(RCJ)가 여기에 포함된다.

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

$A_1$은 palm에 놓이는 MCP actuator이고, $A_2$는 proximal phalanx에 놓이는 distal joint actuator이다. $\mathcal G_j\subseteq\mathbb R^2$는 actuator housing 형상이고, $T_j$는 그 actuator를 싣는 body의 좌표계에서 본 위치와 방향이다. $\Gamma$는 $A_2$와 distal joint 사이의 transmission 형상이다(예: belt drive의 pulley). Actuator가 joint를 직접 구동하면 $\Gamma$는 비어 있다.

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

$q_1$은 MCP flexion이고, $q_2$는 proximal phalanx에 대한 distal phalanx의 flexion이다. $q_2$는 두 phalanx의 길이 방향 축 사이의 각도, 즉 proximal phalanx에 대한 distal phalanx의 상대 회전각으로 정의하며, 완전히 편 자세에서 $0$이다. 이 정의는 joint 하나의 회전각에 의존하지 않는다. 따라서 하나의 hinge가 없고 회전 중심이 움직이는 CFL과 RCJ에도 revolute joint와 같은 의미로 적용된다.

> **표기 노트.** 편의상 자세를 일반화 좌표 $\mathbf q=(q_1,q_2)$로 쓰지만, 대부분의 평가는 MCP를 $q_1=0$에 고정하고 $q_2$에 대해서만 수행한다. 따라서 $q_{\max}$처럼 첨자 없이 쓰는 flexion 값은 $q_2$의 값을 뜻한다.

모든 형태 길이는 **longitudinal** 길이이다. 즉 점 사이의 유클리드 거리가 아니라, 기준 자세에서 손가락의 길이 방향 단위벡터 $\mathbf e_x$로 투영한 길이이다. MCP 축을 $\mathbf r_O$라 하고, proximal phalanx가 $\mathbf r_O$에서 $\mathbf r_{1,\mathrm{end}}$까지, distal phalanx가 $\mathbf r_{2,\mathrm{start}}$에서 손끝 $\mathbf r_C$까지 이어진다고 하자. 그러면 proximal phalanx와 distal phalanx의 구조 길이는 다음과 같다.

$$
L_1=\mathbf e_x^{\mathsf T}\left(\mathbf r_{1,\mathrm{end}}-\mathbf r_O\right),
\qquad
L_2=\mathbf e_x^{\mathsf T}\left(\mathbf r_C-\mathbf r_{2,\mathrm{start}}\right)
$$

손가락 전체 길이는 다음과 같이 정의한다.

$$
\boxed{
L_f=L_1+L_{vp}+L_2
}
$$

이는 $L_f=\mathbf e_x^{\mathsf T}(\mathbf r_C-\mathbf r_O)$와 같다. $L_{vp}$는 virtual pivot length이다.

일반적인 revolute joint에서는 $L_{vp}=0$이다. RCJ와 CFL에서 $L_{vp}$는 기준 자세에서 proximal phalanx의 distal 쪽 끝과 distal phalanx의 proximal 쪽 시작 사이의 longitudinal 간격이다.

즉,

$$
L_{vp}=
\mathbf e_x^{\mathsf T}
\left(
\mathbf r_{2,\mathrm{start}}
-
\mathbf r_{1,\mathrm{end}}
\right)
$$

이며, 같은 longitudinal 투영을 사용한다.

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

여기서 두 점은 proximal phalanx 좌표계에서 나타낸다. 굽힌 자세에서 간격에는 정해진 길이 방향이 없으므로, 간격은 유클리드 거리로 잰다. 두 끝점의 높이가 같으면 $L_{vp}(\mathbf 0)=L_{vp}$이고 $L_f(\mathbf 0)=L_f$이다. 일반적인 revolute joint에서는 이 간격이 모든 자세에서 0으로 유지된다.

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

이 표현은 mechanism, contact interface, actuator, structure 요소의 기하학적 실체를 모두 포함한다.

형상은 joint 자세에 따라 바뀌며, 손가락 요소들 사이의 물리적 상호작용을 결정한다.

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

Mechanism이 어떤 자세에서 닫히지 않으면 그 자세의 형상 $\mathcal G(M,\mathbf q)$는 존재하지 않는다. 따라서 mechanism의 조립 가능성은 형상의 존재에 포함된다. Actuator가 joint를 구동할 수 없는 특이 자세는 transmission 품질 제약(A.5.2)이 배제한다.

Prototype 설계에서는 MCP를 $q_1=0$에 고정하고, distal flexion 범위를 완전히 편 자세부터 평가한다.

요구되는 distal flexion을 $q_{\mathrm{req}}$, 그 자세를 $\mathbf q_{\mathrm{req}}=(0,q_{\mathrm{req}})$라 하고, 요구 운동 구간을 $\mathcal Q_{\mathrm{req}}$라 한다. 설계는 요구 운동 구간 전체를 안전하게 지나야 한다.

$$
\mathcal Q_{\mathrm{req}}=
\left\{
\begin{bmatrix}0\\q_2\end{bmatrix}
\;\middle|\;
0\leq q_2\leq q_{\mathrm{req}}
\right\}
\subseteq
\mathcal Q_{\mathrm{valid}}(M)
$$

## A.5. Prototype 최적화 정식화

이 절의 정식화를 prototype formulation이라 하며, production formulation 또는 CoDe formulation이라고도 부른다. Prototype 설계는 crossed four-bar linkage 구조를 대상으로 한다.

$$
K=K_{\mathrm{CFL}}
$$

Contact interface의 치수 $(\ell_i,h_i)$는 요구사항이다. 치수는 형태 요구사항(A.2)으로 정해지고, 손가락의 나머지 부분은 contact interface에 맞춰 설계된다. 이 요구사항 아래에서 최적화는 contact pad coverage, mechanical advantage, 도달 가능한 운동 범위 사이의 균형을 이루는 finger morphology를 찾는다.

### A.5.1. Contact Pad Coverage

$c_i(M,\mathbf q)$를 자세 $\mathbf q$에서 contact interface $i$의 가용 pad 영역의 크기라 하자. 이는 장착된 contact face 중 기하학적 가림과 접힘 때문에 접근할 수 없는 부분을 제외한 크기이며, 2차원 모델에서는 길이이다. Pad 치수 $\ell_i$는 요구사항으로 주어진 고정값인 반면, $c_i$는 설계와 자세에 따라 정해지는 결과값이며 $0\le c_i(M,\mathbf q)\le\ell_i$이다.

정규화된 contact pad coverage는 다음과 같이 정의한다.

$$
\boxed{
C_{\mathrm{pad}}(M,\mathbf q)
=
\frac{
\sum_{i=1}^{2}c_i(M,\mathbf q)
}{
L_f(\mathbf q)
}
}
$$

분자와 분모를 같은 자세에서 잰다. 물체가 접촉하는 것은 펼친 손가락이 아니라 그 자세에서의 손가락 윤곽이기 때문이다. 이 정규화에서는 굽히면서 줄어드는 phalanx 사이 간격, 즉 접촉할 수 없는 구간이 coverage에 그대로 반영된다. 따라서 간격이 변하지 않는 revolute joint와 간격이 변하는 CFL·RCJ를 같은 기준으로 비교할 수 있다.

Prototype 목적함수는 요구 flexion 자세에서 contact pad coverage를 최대화한다.

$$
\boxed{
\max_M
\quad
f_1(M)=C_{\mathrm{pad}}(M,\mathbf q_{\mathrm{req}})
}
$$

Pad 치수는 요구사항으로 고정되어 있으므로, 이 목적함수는 pad를 늘리는 것이 아니라 주어진 pad 중 실제로 사용할 수 있는 부분을 늘리는 방향으로 작용한다.

### A.5.2. Mechanical Advantage

$\theta_A$를 actuator 입력 각도, $q_2$를 proximal phalanx에 대한 distal phalanx의 출력 각도라 하자.

이상적인 준정적 transmission에서 mechanical advantage는 가상일로 정의한다.

$$
\tau_A\,d\theta_A
=
\tau_{\mathrm{out}}\,dq_2
$$

따라서

$$
MA(M,\mathbf q)
=
\left|
\frac{\tau_{\mathrm{out}}}{\tau_A}
\right|
=
\left|
\frac{d\theta_A}{dq_2}
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

대응하는 목적함수는 다음과 같다.

$$
\boxed{
\max_M
\quad
f_2(M)=MA_{\mathrm{wc}}(M)
}
$$

이는 최악의 경우 전달비 $\left|dq_2/d\theta_A\right|$를 최소화하는 것과 같다.

Mechanical advantage가 커도 linkage가 특이 자세에 가까우면 힘 전달이 불안정해진다. 따라서 transmission 품질은 별도의 제약으로 둔다. CFL에서 transmission angle $\gamma(M,\mathbf q)$는 distal body를 이루는 coupler와 output link 사이의 각도이며, $0$이나 $\pi$에 가까울수록 특이 자세에 가깝다. 요구 운동 구간 전체에서 이 각도가 정해진 범위 안에 있어야 한다.

$$
\gamma_{\min}\le\gamma(M,\mathbf q)\le\gamma_{\max},
\qquad
\forall\,\mathbf q\in\mathcal Q_{\mathrm{req}}
$$

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

운동 범위 목적함수는 다음과 같다.

$$
\boxed{
\max_M
\quad
f_3(M)=q_{\max}(M)
}
$$

요구 flexion이 최소 운동 능력을 지정하는 것과 달리, 이 목적함수는 그 이상으로 추가로 확보할 수 있는 flexion을 최대화한다.

### A.5.4. 통합 최적화 문제

Prototype 설계는 3목적 최적화 문제로 정식화된다.

$$
\boxed{
\begin{aligned}
\underset{M}{\operatorname{maximize}}
\quad&
\begin{bmatrix}
C_{\mathrm{pad}}(M,\mathbf q_{\mathrm{req}})\\
MA_{\mathrm{wc}}(M)\\
q_{\max}(M)
\end{bmatrix}\\[8pt]
\text{subject to}\quad&
K=K_{\mathrm{CFL}},\\
&
\mathbf m_{\min}\leq\mathbf m(M)\leq\mathbf m_{\max},\\
&
q_{\max}(M)\geq q_{\mathrm{req}},\\
&
\text{물리적, 기구학적 실현 가능성}
\end{aligned}
}
$$

제약 $q_{\max}(M)\ge q_{\mathrm{req}}$는 요구 운동 구간 전체가 유효하고($\mathcal Q_{\mathrm{req}}\subseteq\mathcal Q_{\mathrm{valid}}$, A.4) 그 구간에서 transmission angle이 허용 범위 안에 있다는 것(A.5.2)을 함께 뜻한다. 따라서 같은 양 $q_{\max}$가 요구 운동 범위의 실현 가능성 제약인 동시에, 그 이상으로 확보한 운동 범위를 평가하는 목적함수가 된다.

세 목적함수를 동시에 최적화하므로, 결과 설계는 하나의 가중 성능 지표가 아니라 목적함수 사이의 trade-off로 특성화된다.

## A.6. 기호 정리

| 기호 | 의미 |
|---|---|
| $M=(K,C,A,S)$ | Finger morphology: mechanism, contact interface, actuation, structure |
| $K_{\mathrm{CFL}}$ | Crossed four-bar linkage mechanism |
| $C_i=(\ell_i,h_i,\mathbf p_i)$ | Contact interface $i$의 길이, 두께, 장착 위치 ($i=1$: proximal, $i=2$: distal) |
| $A_j=(\mathcal G_j,T_j)$ | Actuator $j$의 housing 형상과 자세 ($j=1$: MCP, $j=2$: distal joint) |
| $\Gamma$ | $A_2$와 distal joint 사이의 transmission |
| $\mathbf q=(q_1,q_2)$ | MCP flexion, proximal phalanx와 distal phalanx 사이 각도(상대 회전각) |
| $\mathbf e_x$ | 기준 자세에서 손가락 길이 방향 단위벡터 |
| $L_1,L_2$ | Proximal phalanx, distal phalanx의 구조 길이 (longitudinal) |
| $L_{vp}$ | Virtual pivot length: 두 phalanx 구조 사이의 longitudinal 간격 |
| $L_f$ | 손가락 전체 길이 (기준 자세, longitudinal). 크기 요구사항에 사용 |
| $L_{vp}(\mathbf q)$, $L_f(\mathbf q)$ | 자세 $\mathbf q$에서의 phalanx 간격(유클리드)과 palmar 윤곽 길이 |
| $\rho_i=L_i/L_f$ | Phalanx 길이 비율 |
| $\mathbf m(M)$, $\mathbf m_{\min}$, $\mathbf m_{\max}$ | 형태 descriptor와 그 요구 범위 |
| $\mathcal G(M,\mathbf q)$ | 자세 $\mathbf q$에서의 2차원 형상 |
| $\mathcal Q_{\mathrm{valid}}$ | 유효한 configuration space: 형상이 존재하고 self-collision이 없는 자세 |
| $q_{\mathrm{req}}$, $\mathbf q_{\mathrm{req}}$, $\mathcal Q_{\mathrm{req}}$ | 요구 distal flexion, 그 자세 $(0,q_{\mathrm{req}})$, 요구 운동 구간 |
| $c_i(M,\mathbf q)$ | 가용 pad 영역의 크기: contact interface $i$ 중 가림과 접힘을 제외하고 사용할 수 있는 크기 ($\le\ell_i$) |
| $C_{\mathrm{pad}}$ | 정규화된 contact pad coverage: $\sum_i c_i/L_f(\mathbf q)$ |
| $\theta_A$ | Actuator 입력 각도 |
| $MA$, $MA_{\mathrm{wc}}$ | Mechanical advantage, 요구 운동 구간에서의 최악값 |
| $\gamma(M,\mathbf q)$, $[\gamma_{\min},\gamma_{\max}]$ | CFL의 transmission angle과 그 허용 범위 |
| $q_{\max}$ | 편 자세에서 출발해 안전하게(유효하고 transmission angle이 허용 범위 안) 도달할 수 있는 최대 distal flexion |
