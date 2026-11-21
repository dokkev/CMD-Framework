# Study 0: 수학 정리

작성: 2026-11-20, 고침: 2026-11-21. 이 문서가 Study 0의 수학과 증명의 기준이다. `design/SSoT/problem_formulation.md` A.6.0은 이 문서의 명제를 요약한다. 계산은 `design/core/geometry/pole.py`, 최적화 결과와의 대조는 `design/scripts/study/study2_morphology_adaptation/pole_law.py`가 한다.

2026-11-21에 고친 것: (1) finite pole과 순간 운동을 구분함, (2) 두 pad가 만나지 않을 조건을 임의의 pole과 유한한 직사각형에 대한 정확한 관계(명제 1)와, 대칭 조건에서만 성립하는 닫힌 식(명제 2)으로 나눔, (3) 두께에 대한 볼록성에서 "구간별 선형"을 뺌, (4) 고정 축, 구름 접촉, crossed four-bar에 대한 주장의 범위를 좁힘.

![pole과 물러나는 거리](figures/study0_pole_sketch.png)

## 1. 설정

**좌표.** Proximal phalanx에 고정한 평면 좌표를 쓴다. $x$는 link 선을 따라 손끝 쪽, $z$는 dorsal 쪽이 양수이다. Link 선은 기준 자세($q=0$)에서 관절 축들을 잇는 직선 $z=0$이다.

**굽힘.** Flexion $q\in(0,\pi]$에서 distal phalanx는 palmar 쪽으로 $q$만큼 돈다.

$$
R_q=\begin{pmatrix}\cos q&\sin q\\-\sin q&\cos q\end{pmatrix},\qquad \mathbf y(q)=R_q\,\mathbf y_0+\mathbf b(q).
$$

$\mathbf y_0$는 distal phalanx 위의 점의 기준 자세 위치이고 $\mathbf b(q)$는 구조가 정한다.

**Pad.** Pad $i$($i=1$ proximal, $i=2$ distal)는 자기 phalanx에 고정된 유한한 직사각형이다. 장착 깊이 $d_i$는 link 선에서 pad 윗면까지, 두께는 $h_i$, contact face(아랫면)의 깊이는 $t_i=d_i+h_i$이다. 기준 자세에서 proximal pad는 $x\in[x_0,\,x_{1e}]$, distal pad는 $x\in[x_{2s},\,L_f]$를 차지한다. 관절 간격은 $G=x_{2s}-x_{1e}$이고

$$
\frac{\ell_1+\ell_2}{L_f}=1-\frac{x_0+G}{L_f}. \tag{1}
$$

**겹침.** 두 pad가 양의 넓이로 겹치는 것을 충돌이라 한다. 경계가 맞닿는 것은 허용한다. 두 가지를 구분한다.
- 한 자세 $q$에서 겹치지 않는다.
- $(0,q_{\mathrm{req}}]$의 모든 자세에서 겹치지 않는다. 손가락은 기준 자세에서 출발해 연속으로 굽으므로 설계가 만족해야 하는 것은 이것이다.

**가정.** (A1) 평면 운동. (A2) Pad는 강체 직사각형이고 contact face는 자기 phalanx의 link 선에 평행하다. (A3) 이 문서는 두 pad 사이의 충돌만 다룬다. 다른 부품과의 충돌은 $d_i$와 pad의 범위로 들어온다. (A4) Distal phalanx의 방향은 어느 구조에서나 $q$이다.

## 2. Pole

**보조정리 1 (finite pole).** $q\in(0,2\pi)$이면 변위 $\mathbf y_0\mapsto R_q\mathbf y_0+\mathbf b$는 한 점 $\mathbf P$를 중심으로 한 회전 $\mathbf y_0\mapsto\mathbf P+R_q(\mathbf y_0-\mathbf P)$와 같고,

$$
\mathbf P(q)=(I-R_q)^{-1}\mathbf b(q)=\tfrac12\Big(\mathbf b-\cot\tfrac q2\,J\,\mathbf b\Big),\qquad J=\begin{pmatrix}0&-1\\1&0\end{pmatrix}. \tag{2}
$$

*증명.* $\det(I-R_q)=4\sin^2\frac q2>0$이므로 고정점 방정식 $\mathbf P=R_q\mathbf P+\mathbf b$의 해가 유일하다. $(I-R_q)^{-1}=\frac12(I-\cot\frac q2\,J)$는 직접 곱해 확인한다. $\square$

손끝의 경로 $\mathbf r_C(q)$만 알면 $\mathbf b(q)=\mathbf r_C(q)-R_q\,\mathbf r_C(0)$이므로, pole은 구조의 종류와 무관하게 같은 식으로 계산된다. Pole이 link 선 아래로 내려온 깊이를 $z(q)=-P_z(q)$, pole에서 본 contact face의 깊이를 $\tau_i(q)=t_i-z(q)$라 쓴다.

**Finite pole은 순간 회전 중심이 아니다.** Pole은 기준 자세에서 자세 $q$까지의 유한한 변위의 성질이다. $\mathbf y(q)=\mathbf P(q)+R_q(\mathbf y_0-\mathbf P(q))$를 미분하면

$$
\mathbf y'(q)=R'_q\big(\mathbf y_0-\mathbf P(q)\big)+(I-R_q)\,\mathbf P'(q) \tag{3}
$$

이고, 둘째 항 때문에 순간 속도는 $\mathbf P(q)$를 중심으로 한 회전의 속도가 아니다. Pole이 고정된 관절(고정 축)에서만 둘이 같다. $q\to0$에서 $\mathbf P(q)$는 기준 자세의 순간 회전 중심으로 수렴한다. 아래의 명제는 모두 유한한 변위만 쓰며 순간 속도에 기대지 않는다. (2)는 $q\to0$에서 $\cot\frac q2$가 발산하므로, 계산에서는 $|q|<10^{-6}$ rad에서 pole을 정의하지 않는다.

**보조정리 2 (반사).** 점 $\mathbf P$를 지나는 수직선에 대한 반사를 $F$, $\mathbf P$를 지나고 법선이 $\mathbf n_q=(\cos\frac q2,\,-\sin\frac q2)$인 직선 $m_q$에 대한 반사를 $M_q$라 하면, $\mathbf P$를 중심으로 한 $q$ 회전에 대해 $R_q\circ F=M_q$이다.

*증명.* $\mathbf P$를 원점으로 두면 $F=\mathrm{diag}(-1,1)$이고 $M_q=I-2\,\mathbf n_q\mathbf n_q^{\!\top}$이다. 두 행렬을 곱해 비교한다. $\square$

## 3. 명제

### 명제 1. 일반 clearance: 임의의 pole과 유한한 두 pad

Proximal pad의 중심과 반길이를 $\mathbf c_A$, $(a_x,a_z)$, distal pad의 기준 자세 중심과 반길이를 $\mathbf c_B(0)$, $(b_x,b_z)$라 하자. 자세 $q$에서 $\mathbf c_B(q)=\mathbf P(q)+R_q\big(\mathbf c_B(0)-\mathbf P(q)\big)$이다. 분리축의 후보를 $\mathcal U(q)=\{\mathbf e_x,\mathbf e_z,R_q\mathbf e_x,R_q\mathbf e_z\}$로 두고

$$
m_{\mathbf u}(q)=\big|\mathbf u^{\!\top}(\mathbf c_B(q)-\mathbf c_A)\big|
-\big(a_x|\mathbf u^{\!\top}\mathbf e_x|+a_z|\mathbf u^{\!\top}\mathbf e_z|\big)
-\big(b_x|\mathbf u^{\!\top}R_q\mathbf e_x|+b_z|\mathbf u^{\!\top}R_q\mathbf e_z|\big)
$$

라 하면

$$
\boxed{\;\text{자세 }q\text{에서 두 pad가 겹치지 않는다}\iff m(q):=\max_{\mathbf u\in\mathcal U(q)}m_{\mathbf u}(q)\ \ge 0\;} \tag{4}
$$

이고, 설계가 만족해야 하는 조건은

$$
\min_{0<q\le q_{\mathrm{req}}}m(q)\ \ge 0 \tag{5}
$$

이다.

*증명.* 두 볼록 다각형은 어느 한 변의 법선 방향으로 사영이 분리될 때에만 내부가 겹치지 않는다(separating axis theorem). 직사각형의 변의 법선은 자기 두 축뿐이므로 후보는 네 개이다. $m_{\mathbf u}$는 그 축에서 두 사영 사이의 틈이다. $\square$

**정확한 것과 표본으로 확인하는 것.** (4)는 한 자세에서 정확하다. (5)는 연속 구간에 대한 조건이지만, 계산에서는 $q$의 격자에서 확인한다(대조 스크립트 0.05°, study의 평가기는 1° 격자로 탐색하고 0.05° 격자로 다시 확인). 격자 사이에서 생기는 겹침까지 수학적으로 배제하는 것은 아니다. 이 문서는 연속 구간에 대한 인증(certification)을 주지 않는다. 대칭 조건에서는 명제 2가 닫힌 식으로 연속 구간의 조건을 준다.

(4)는 pole의 위치, 두 pad의 깊이, 길이, 두께에 아무 조건도 두지 않는다. $m(q)>0$의 값은 가장 좋은 축에서의 틈이며 두 pad 사이의 유클리드 거리와는 다르다. 부호와 0은 정확하다. 이 관계는 기존 기하이고, 이 문서에서는 (6)의 닫힌 식이 성립하지 않는 경우까지 같은 틀로 다루는 데 쓴다.

### 명제 2. 대칭 clearance 법칙

$q_{\mathrm{req}}<\pi$로 제한한다($\tan\frac q2$가 $q=\pi$에서 정의되지 않는다. Study의 요구 flexion은 150° 이하이다). 다음 두 조건을 둔다.
- (S1) 두 contact face의 깊이가 같다: $\tau_1(q)=\tau_2(q)=\tau(q)$.
- (S2) 모든 자세에서 두 pad의 관절 쪽 아래 모서리가 pole에서 $x$ 방향으로 같은 거리 $s$에 있다. 곧 pole의 $x$ 좌표가 굽히는 동안 두 모서리의 가운데에 머문다.

그러면

$$
\boxed{\;(0,q_{\mathrm{req}}]\text{의 모든 자세에서 겹치지 않는다}\iff s\ \ge\ \max_{0<q\le q_{\mathrm{req}}}\ \big[\tau(q)\big]_+\tan\frac q2\;} \tag{6}
$$

이고, 이 조건을 만족하는 가장 작은 관절 간격은

$$
G_{\mathrm{sym}}(h,q_{\mathrm{req}})=\max_{0<q\le q_{\mathrm{req}}}\ 2\,\big[d+h-z(q)\big]_+\tan\frac q2 \tag{7}
$$

이다. 최댓값을 주는 $q$를 임계 flexion $q^\ast$라 한다.

*증명.* 자세 $q$에서 pole을 원점으로 둔다. (S1), (S2)에 의해 기준 자세의 distal pad는 proximal pad $K$를 pole을 지나는 수직선에 대해 반사한 것의 부분 또는 연장이고, 관절 쪽 아래 모서리는 서로의 거울상이다. 먼저 두 pad의 길이가 같다고 하자. 보조정리 2에 의해 돌린 distal pad는 $M_q K$이다. 볼록 집합 $K$와 그 거울상의 내부가 겹치지 않을 필요충분조건은 $K$의 내부가 $m_q$와 만나지 않는 것, 곧 $K$가 $m_q$의 한쪽 닫힌 반평면에 들어 있는 것이다($K$의 내부가 $m_q$를 지나면 그 점은 반사에 고정되어 두 내부에 모두 속한다). $K$의 점은 $(-s-\lambda,\,-\tau+\mu)$, $0\le\lambda\le\ell$, $0\le\mu\le h$이고

$$
\mathbf n_q\cdot\mathbf u=-(s+\lambda)\cos\tfrac q2+(\tau-\mu)\sin\tfrac q2
$$

는 $q\in(0,\pi)$에서 $\lambda=\mu=0$(관절 쪽 아래 모서리)에서 최대이다. $q\to0^+$에서 $K$는 $\mathbf n_q\cdot\mathbf u<0$ 쪽에 있다. $K$가 반대쪽 반평면으로 넘어가려면, $\max_K\mathbf n_q\cdot\mathbf u$가 $q$에 대해 연속이므로 그 사이의 어느 자세에서 $K$의 내부가 $m_q$를 지나야 한다. 따라서 구간 전체에서 겹치지 않는 것은 모든 $q$에서 $-s\cos\frac q2+\tau(q)\sin\frac q2\le0$인 것과 같다. 두 pad의 길이가 다르면 짧은 쪽을 늘려도 조건의 좌변(모서리에서의 값)이 바뀌지 않고, 긴 쪽에서 겹치지 않으면 짧은 쪽에서도 겹치지 않으며, 역으로 모서리가 $m_q$를 넘으면 모서리 근방이 두 pad에 모두 속한다. $\square$

**이 법칙이 말하지 않는 것.**
- *한 자세에서의 필요충분조건이 아니다.* 유한한 두 pad는 굽히는 도중에 겹쳤다가 서로를 지나쳐, 나중 자세에서는 떨어져 있을 수 있다. (6)은 구간 전체에 대한 조건이다.
- *(S2)가 깨지면 정확하지 않다.* Pole의 $x$ 좌표가 움직이거나 두 pad가 pole에 대해 고르게 놓이지 않으면 (7)은 추정값이다.
- *모든 배치 가운데 최소가 아니다.* (7)은 pole에 대해 고르게 놓은 배치 가운데 가장 작은 간격이다. 고르지 않게 놓으면 더 작은 간격으로도 겹치지 않을 수 있다. 예: 고정 축에서 $q_{\mathrm{req}}=90^\circ$일 때 한 pad의 모서리를 축 바로 아래에, 다른 pad의 모서리를 $t$만큼 떨어뜨려 놓으면 간격 $t$로 겹치지 않는다. (7)의 값은 $2t$이다.
- 그럼에도 대칭 배치가 기준이 되는 이유는 명제 3에 있다. 원기둥 기준 coverage는 두 contact face가 만나는 점에서 같은 거리만큼만 센다. Pole에 더 가까이 붙인 pad의 부분은 세지 않는다.

**따름정리 2.1 (두께에 대한 볼록성).** $z(q)$와 $d$를 고정하면 $G_{\mathrm{sym}}$은 $h$에 대해 볼록하다. $q^\ast(h)$가 유일하고 $\tau(q^\ast)>0$인 곳에서

$$
\frac{dG_{\mathrm{sym}}}{dh}=2\tan\frac{q^\ast(h)}2\ \le\ 2\tan\frac{q_{\mathrm{req}}}2. \tag{8}
$$

*증명.* (7)은 $h$에 대한 아핀 함수들과 0의 상한이므로 볼록하다. 미분은 최댓값을 주는 원소가 유일할 때의 envelope 정리이다. $\square$

$q$가 연속인 구간에서는 $q^\ast(h)$가 $h$에 따라 움직일 수 있으므로 $G_{\mathrm{sym}}(h)$는 일반적으로 구간별 선형이 아니다. 고정 축처럼 $q^\ast$가 $h$와 무관한 경우에만 선형이다.

### 명제 3. 두 contact face가 만나는 점과 원기둥 기준 coverage

(S1)만 가정한다. 자세 $q$에서 pole을 원점으로 두면 두 contact face의 직선은 서로의 거울상($m_q$에 대해)이므로 $m_q$ 위의 점

$$
X(q)=\Big(-\tau\tan\frac q2,\ -\tau\Big) \tag{9}
$$

에서 만난다. 각 face 위에서 $X$는 pole의 수선의 발에서 $\tau\tan\frac q2$만큼 떨어져 있다. 이것은 pad의 길이와 배치에 무관하다.

A.5.1의 정의에서 pad $i$가 $X$로부터 덮는 거리의 집합은 $T_i=[\max(a_i,0),\ a_i+\ell_i]$, $a_i=s_i-\tau\tan\frac q2$이고($s_i$: pad $i$의 관절 쪽 끝이 pole에서 떨어진 거리), $C_{\mathrm{pad}}(q)=2\,|T_1\cap T_2|/L_f(q)$이다. $a_i<0$인 부분, 곧 $X$보다 관절 쪽에 놓인 pad의 부분은 세지 않는다. 따라서 coverage에 대해서는 pole에 대해 고르게 놓은 배치보다 더 붙여도 얻는 것이 없다.

(S1), (S2)이고 두 pad의 길이가 같으며($\ell_1=\ell_2=\ell$, $G=2s$) phalanx 구조의 끝이 link 선 위에서 pad의 관절 쪽 끝과 같은 $x$에 있으면

$$
\boxed{\;C_{\mathrm{pad}}(q)=\frac{L_f-x_0-G}{\,L_f-G\,(1-\cos\frac q2)+2\,z(q)\sin\frac q2\,}\;} \tag{10}
$$

*증명.* $a_1=a_2=s-\tau\tan\frac q2\ge0$(명제 2)이므로 $T_1=T_2$이고 $|T_1\cap T_2|=\ell$이다. 두 구조의 끝은 pole에서 본 $\mathbf u_1=(-s,\,z)$와 그 거울상이므로 $L_{vp}=2\,|\mathbf n_q\cdot\mathbf u_1|=2\,(s\cos\frac q2+z\sin\frac q2)$이다. $L_1+L_2=x_0+2\ell=L_f-G$를 대입한다. $\square$

이 조건들이 성립하지 않는 설계에서는 $X$, $T_i$, $L_{vp}$를 pole과 pad의 치수로 직접 계산한다(`pole_law.coverage_law`). 그 계산의 입력은 pole의 경로와 pad만이 아니다. $L_{vp}$는 phalanx 구조의 끝 $\mathbf r_{1,\mathrm{end}}$, $\mathbf r_{2,\mathrm{start}}$ 사이의 거리이고(A.3), CFL에서는 구조의 끝이 pad의 끝이 아니라 자기 pivot housing일 수 있다. 또 기준 자세에서 distal phalanx의 link 선이 proximal의 것과 정확히 한 직선이 아니면(5절) 그 어긋남도 입력이다. $\bar C_{\mathrm{pad}}$는 $q\in\{90^\circ,\ (90^\circ+q_{\mathrm{req}})/2,\ q_{\mathrm{req}}\}$에서의 평균이다.

**따름정리 3.1 (깊이가 고정된 pole을 내리는 것).** Pole의 깊이가 상수 $z<t$이고 $G=2(t-z)\tan\frac{q_{\mathrm{req}}}2$인 대칭 모델에서, $z$를 늘리면 (10)은 $q\le q_{\mathrm{req}}$의 모든 평가 자세에서 커진다.

*증명.* $T=\tan\frac{q_{\mathrm{req}}}2$라 하자. (10)의 분자 $N$과 분모 $D$에 대해 $\partial N/\partial z=2T$, $\partial D/\partial z=2T(1-\cos\frac q2)+2\sin\frac q2$이다. $q=q_{\mathrm{req}}$에서 둘이 같으므로 $\partial_z(N/D)=2T\,(D-N)/D^2>0$이다. $q<q_{\mathrm{req}}$에서는 $\partial D/\partial z<2T$이다. $\square$

이득은 $z=t$까지이고 그때 $G=0$이다. $z>t$로 더 내리면 $G$는 줄지 않고 (10)의 분모만 커진다. 곧 필요한 것보다 깊은 pole은 coverage를 깎는다. 이 모델에서 가장 좋은 관절은 contact face 위의 점을 중심으로 도는 관절이다.

### 명제 4. 접촉면에서 본 운동

Proximal contact face의 직선을 기준으로 본 pole의 위치를 $\mathbf p(q)$(두 성분 모두), distal contact face 위의 한 점의 기준 자세 위치를 $\mathbf c_0$라 하면 $\mathbf c(q)=\mathbf p(q)+R_q(\mathbf c_0-\mathbf p(q))$이다. 같은 $\mathbf c_0$를 가진 두 손가락에서 pole이 $\Delta\mathbf p(q)$만큼 다르면

$$
\boxed{\;\|\Delta\mathbf c(q)\|=2\sin\frac q2\,\|\Delta\mathbf p(q)\|\;} \tag{11}
$$

*증명.* $\Delta\mathbf c=(I-R_q)\Delta\mathbf p$이고 $(I-R_q)^{\!\top}(I-R_q)=4\sin^2\frac q2\,I$이다. $\square$

**따름정리 4.1.** 기준 자세에서 두 contact face의 상대 위치가 같은 두 손가락에서, 접촉면에서 본 운동이 같을 필요충분조건은 $\mathbf p(q)$가 모든 $q$에서 같은 것이다. 깊이 $\tau(q)$만 같아서는 부족하고 pole의 $x$ 좌표도 contact face에 대해 같아야 한다. 기준 자세에서 두 face의 단차 $t_1-t_2$가 다르면 $\mathbf c_0$가 달라지고, 그 차이가 (11)에 더해진다.

**따름정리 4.2.** $\mathbf p(q)$가 같고 pad의 치수와 contact face 위의 위치가 같으면 (4)의 $m(q)$도 같다. 접촉면에서 본 운동을 유지하는 다시 설계는 같은 pad를 같은 자리에 그대로 실을 수 있다.

## 4. 구조별 결과

### 4.1 고정 축 (2R-DD, 2R-remote)

Pole은 관절 축이고 $q$와 무관하다: $z\equiv0$, $P_x$는 상수. 두 pad의 깊이가 같고 축에 대해 고르게 놓으면 (S1), (S2)가 성립하고 $q^\ast=q_{\mathrm{req}}$이다.

$$
G_{\mathrm{sym}}=2\,(d+h)\tan\frac{q_{\mathrm{req}}}2,\qquad
\frac{\partial G_{\mathrm{sym}}}{\partial h}=2\tan\frac{q_{\mathrm{req}}}2,\qquad
\frac{\partial G_{\mathrm{sym}}}{\partial q_{\mathrm{req}}}=(d+h)\sec^2\frac{q_{\mathrm{req}}}2. \tag{12}
$$

| $q_{\mathrm{req}}$ | $90^\circ$ | $120^\circ$ | $150^\circ$ |
|---|---|---|---|
| 두께 1 mm당 $G_{\mathrm{sym}}$의 증가 | 2.00 mm | 3.46 mm | 7.46 mm |

두 pad의 깊이가 다르거나(2R-DD) 길이 하한 때문에 고르게 놓지 못하면(요구 flexion이 클 때) (12)는 추정값이고, 명제 1로 판정한다.

**접촉면에서 본 운동.** 두 pad의 깊이가 같은 고정 축 손가락에서 contact face가 축에서 $\delta=\Delta h-\Delta d$만큼 멀어지도록 다시 설계하면($\Delta d$: 장착 깊이를 줄인 양), 다시 설계한 축이 어디에 있든 $\Delta\mathbf p$는 $q$에 무관한 상수이고 $\|\Delta\mathbf p\|\ge|\delta|$이다. (11)에서

$$
\max_{q\le q_{\mathrm{req}}}\|\Delta\mathbf c(q)\|\ \ge\ 2\sin\frac{q_{\mathrm{req}}}2\,|\Delta h-\Delta d|, \tag{13}
$$

등호는 축이 contact face에 대해 $x$ 방향으로 움직이지 않을 때이다. 이 하한은 다시 설계의 내용에 무관하다. $d=0$인 2R-remote는 $\Delta d=0$이다.

### 4.2 구름 접촉 (RCJ)

반지름 $r$의 같은 roller 둘이 미끄럼 없이 구른다. Proximal roller의 중심을 $(a,0)$이라 하자.

$$
\mathbf P(q)=\big(a+r,\ -r\tan\tfrac q4\big),\qquad z(q)=r\tan\frac q4. \tag{14}
$$

*증명.* 같은 반지름의 구름에서 distal phalanx는 proximal phalanx를 두 roller의 공통 접선에 대해 반사한 것이다. 기준 자세의 접선은 $x=a+r$이고, flexion $q$에서 접선은 $\mathbf n_q\cdot(\mathbf y-(a,0))=r$이다. 변위는 두 반사의 합성이므로 두 직선의 교점을 중심으로 한 회전이다. $x=a+r$을 넣으면 $z'=-r\,(1-\cos\frac q2)/\sin\frac q2=-r\tan\frac q4$. $\square$

Pole의 $x$ 좌표가 상수이므로, 두 pad의 깊이가 같고 $x=a+r$에 대해 고르게 놓으면 (S1), (S2)가 성립한다. $f(q)=(t-r\tan\frac q4)\tan\frac q2=t\tan\frac q2-r(\sec\frac q2-1)$에 대해

$$
f'(q)=\tfrac12\sec^2\tfrac q2\,\Big(t-r\sin\tfrac q2\Big)
$$

이므로 $f$는 $t>r\sin\frac q2$인 동안에만 증가한다.

- $t\ge r\sin\frac{q_{\mathrm{req}}}2$이면 $q^\ast=q_{\mathrm{req}}$이고 $G_{\mathrm{sym}}=2\big[t\tan\frac{q_{\mathrm{req}}}2-r(\sec\frac{q_{\mathrm{req}}}2-1)\big]$.
- $t<r\sin\frac{q_{\mathrm{req}}}2$이면 최대는 구간 안쪽 $q^\ast=2\arcsin(t/r)$에서 생긴다.

$q^\ast=q_{\mathrm{req}}$인 경우 두께에 대한 기울기는 고정 축과 같은 $2\tan(q_{\mathrm{req}}/2)$이다. $r$이 관절 actuator로 정해져 있으면 pole을 옮길 수 없으므로 (13)이 같은 형태로 성립한다.

### 4.3 Crossed four-bar (CFL)

Proximal phalanx의 pivot $A$, $A'$과 distal phalanx의 pivot $B$, $B'$을 link $AB'$과 $A'B$가 잇는다. 순간 회전 중심은 두 link의 연장선의 교점이다. Finite pole $\mathbf P(q)$는 (2)로 계산한다. $z(q)$와 $P_x(q)$는 linkage 치수가 정하며 이 문서는 닫힌 식을 주지 않는다.

Crossed four-bar 일반에 대해 이 문서가 주장하는 것은 없다. 다음은 study의 최적화가 찾은 CFL 설계에서 관찰한 것이다.
- $z(q)$가 기준 자세 근처에서는 음수(pole이 dorsal 쪽)이고 굽힐수록 커진다.
- 임계 flexion이 구간 안쪽에 있다($q^\ast<q_{\mathrm{req}}$). 그런 설계에서는 $q_{\mathrm{req}}$를 올려도 $G_{\mathrm{sym}}$이 커지지 않는다. 고정 축에서는 (12)에 의해 항상 커진다.
- $P_x(q)$가 굽히는 동안 움직이는 설계가 있다. 그런 설계에서 (7)은 추정값이다.

다시 설계할 때 linkage 치수를 바꿔 $\mathbf p(q)$를 유지할 수 있는 만큼 (11)의 변화가 작아진다. 얼마나 유지할 수 있는지는 식으로 쓰지 않았고, Study 2가 탐색으로 잰다.

### 4.4 요약

| | $z(q)$ | $P_x(q)$ | (7)의 적용 | 접촉면에서 본 운동 (contact face가 $\delta$ 멀어질 때) |
|---|---|---|---|---|
| 고정 축 | $0$ | 상수 | 깊이가 같고 고르게 놓으면 정확, $q^\ast=q_{\mathrm{req}}$ | 하한 (13) |
| 구름 접촉, $r$ 고정 | $r\tan(q/4)$ | 상수 | 같은 조건에서 정확, $q^\ast=q_{\mathrm{req}}$ 또는 $2\arcsin(t/r)$ | 하한 (13) |
| Crossed four-bar | Linkage가 정함 | Linkage가 정함 | $P_x$가 움직이면 추정값 | $\mathbf p(q)$를 유지하는 만큼 작아짐 |

## 5. 가설과 대조

명제 1–4는 기하의 정리이다. 아래의 H1은 정리가 아니라 최적화 결과에 대한 가설이며, 대조로 얼마나 성립하는지를 보고한다. Study 1의 72개 조건 가운데 2026-11-21 시점에 끝난 51개 설계로 본 결과이다. 대조는 `pole_law.py`가 한다.

**H1 (경험적 가설). Coverage를 최대화한 설계에서는 pad끼리의 접촉이 자주 활성 제약이다.** 곧 설계의 pad로 계산한 $\min_q m(q)$가 0에 가깝다. 이것은 $M^\star\Rightarrow\min_q m(q)=0$이라는 정리가 아니다. Actuator의 자리, pad 길이의 하한, 다른 제약이 먼저 걸리면 최적 설계에서도 pad 사이에 틈이 남는다.

| 대조 | 결과 |
|---|---|
| H1: 설계의 pad로 계산한 $\min_q m(q)$ | 51개 중 47개에서 0.1 mm 이내. 나머지 넷: RCJ 120°/9 mm 1.68 mm, RCJ ideal 120°/5 mm 0.29 mm, RCJ ideal 90°/7 mm 0.24 mm, CFL 150°/5 mm 0.25 mm |
| 대칭 법칙 (7)이 설계의 $G$를 얼마나 맞히는가: $G-G_{\mathrm{sym}}$ | 고정 축 $-3.4$~$+0.2$ mm(크게 벗어나는 것은 두 pad가 고르게 놓이지 않은 150° 설계), CFL $-1.6$~$+0.2$ mm, RCJ ideal $0$~$+0.7$ mm. 추정으로서의 정확도이다 |
| 명제 1의 구현: 네 축의 clearance 대 study의 공용 다각형 계산 | 무작위 400개 배치에서 $10^{-9}$ 이내. 움직이는 pole, 두께와 깊이가 다른 pad의 첫 접촉 각도도 일치(단위 테스트) |
| 명제 3의 계산: pole과 pad로 계산한 $\bar C_{\mathrm{pad}}$ 대 평가기 | 51개 모두 $10^{-13}$ 이내 (아래) |
| 고정한 손가락: 다른 두께의 pad를 단 경우의 첫 접촉 각도 대 평가기의 $q_{\max}$ | 평가기 격자(1°) 수준 |
| (13): Study 2의 다시 설계, $q_{\mathrm{req}}=120^\circ$ | 2R-remote 3.46, 6.93, 10.39 mm로 하한과 같음. CFL 0.5, 0.65, 1.73 mm |
| (14): roller의 닫힌 식 대 수치; CFL의 pole을 독립 구현으로 재계산 | 단위 테스트; 0.13 mm 이하 |

**$\bar C_{\mathrm{pad}}$의 일치에 대하여.** 처음의 계산은 평가기와 CFL에서 0.0045, CFL ideal에서 0.018까지 달랐다. 원인은 기하 관계가 아니라 계산에 넣은 입력 두 가지였다.
1. *구조의 끝.* 평가기는 CFL의 phalanx 구조가 pad의 관절 쪽 끝과 자기 pivot housing(반지름 3 mm) 가운데 관절에 더 가까운 곳에서 끝난다고 본다(A.3). 처음의 계산은 pad의 끝을 썼다. 이것을 맞추면 CFL ideal의 차이가 0.018에서 0.002로 준다.
2. *기준 자세의 distal link 선.* CFL의 link 길이는 제작 해상도(0.1 mm)로 반올림된다. 그 길이로 four-bar를 닫으면 기준 자세에서 distal phalanx의 link 선이 proximal의 것보다 최대 0.24 mm 아래에 평행하게 놓인다. 처음의 계산은 두 선이 한 직선이라고 가정했다(A2). $B$와 손끝을 닫힌 four-bar에서 읽으면 남은 차이가 없어진다.

고정 축과 구름 접촉은 두 가지 모두 해당이 없어 처음부터 일치했다. 둘째는 가정 (A2)가 CFL에서 0.2 mm 수준으로만 성립한다는 뜻이며, 명제 2의 대칭 법칙과 대조 스크립트의 clearance 계산은 이 어긋남을 넣지 않는다.

대칭 법칙은 설명과 추정에 쓰고, 설계를 판정하는 것은 명제 1이다.

## 6. 이 정리가 말하지 않는 것

- Crossed four-bar가 $\mathbf p(q)$를 얼마나 옮기거나 유지할 수 있는지. Transmission angle, pivot의 크기, actuator의 자리가 제한하지만 식으로 쓰지 않았다.
- 장착 깊이 $d$가 어떻게 정해지는지. 2R-DD와 RCJ에서는 pad를 관절 hardware 밑으로 얼마나 넣는지의 절충이며, 지금은 최적화 결과에서 읽는다.
- 대칭 조건 밖에서의 닫힌 식. 그 경우는 명제 1을 계산한다.
- 주어진 pole 경로에서 pad를 어떻게 놓는 것이 coverage에 가장 좋은지. 명제 3은 고르게 놓은 배치보다 더 붙여도 얻는 것이 없음을 말할 뿐이다.
- 가림, mechanical advantage, 3차원 형상, pad의 변형.
