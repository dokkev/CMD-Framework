1. Finger Morphology
\[
\boxed{
M=
\left(
\mathbf p_{\mathrm{mech}},
\mathbf p_{\mathrm{act}},
\mathbf p_{\mathrm{struct}},
\mathbf p_{\mathrm{contact}}
\right)
}
\]

Parameter	Definition
\(\mathbf p_{\mathrm{mech}}\)	Kinematic Mechanism
\(\mathbf p_{\mathrm{act}}\)	Actuator Geometry & Placement
\(\mathbf p_{\mathrm{struct}}\)	Structural Geometry & Placement
\(\mathbf p_{\mathrm{contact}}\)	Contact Interface Geometry & Mounting
각 그룹은 독립적일 필요가 없으며, 전체 morphology는 architecture-specific constraints를 만족해야 해.
2. Mechanism Architecture
\[
\boxed{
m\in
\{
\mathrm{CoDe},
\mathrm{2R},
\mathrm{2R\text{-}remote},
\mathrm{RCJ}
\}
}
\]

\[
\boxed{
M\in\mathcal M(m),\qquad
\mathbf p_{\mathrm{mech}}\in\mathcal P_{\mathrm{mech}}(m)
}
\]

- \(m\): Discrete architecture selection
- \(\mathcal M(m)\): 해당 architecture에서 가능한 Finger Morphology의 집합
- \(\mathcal P_{\mathrm{mech}}(m)\): 해당 architecture의 mechanism parameter space
\(\mathbf p_{\mathrm{mech}}\)의 구성과 독립 자유도는 \(m\)에 따라 달라질 수 있어.
CoDe mechanism
\[
\boxed{
\mathbf p_{\mathrm{mech}}^{\mathrm{CoDe}}
=
(T_A,T_{A'},T_B,T_{B'})
}
\]

각 \(T_i\in SE(2)\)는 pivot frame을 나타내. Pivot orientation은 독립변수로 둘 필요 없이 적절한 frame convention으로 고정할 수 있어.
2R과 2R-remote는 동일한 articulation kinematics를 가질 수 있어. RCJ의 구체적인 parameterization은 아직 정하지 않았어.
3. Actuation Parameters
각 actuator는 2D footprint와 pose로 정의한다.
\[
\boxed{
\mathbf p_{\mathrm{act}}
=
\left((G_i,T_i)\right)_{i=1}^{N_{\mathrm{act}}}
}
\]

\[
G_i\subseteq\mathbb R^2,\qquad T_i\in SE(2)
\]

여기서 \(G_i\)는 actuator housing geometry, \(T_i\)는 finger coordinate frame에 대한 actuator pose야.
현재 2-DoF finger에서는
\[
N_{\mathrm{act}}=2
\]

로 설정해.
\(\mathbf p_{\mathrm{act}}\)는 architecture-independent한 표현을 사용하지만, 실제로 가능한 placement는 mechanism과의 compatibility constraints를 따라야 해.
4. Structural Parameters
Structure는 proximal/distal rigid body의 시작과 끝에 정의한 reference frames로 표현한다.
\[
\boxed{
\mathbf p_{\mathrm{struct}}
=
\left(
T_{p,\mathrm{start}},
T_{p,\mathrm{end}},
T_{d,\mathrm{start}},
T_{d,\mathrm{end}}
\right)
}
\]

\[
T_i\in SE(2)
\]

단, proximal frame을 MCP origin \(O\)에 고정하면
\[
T_{p,\mathrm{start}}=I
\]

이므로 독립적인 parameter로 둘 필요는 없어.
따라서 최소 표현은
\[
\boxed{
\mathbf p_{\mathrm{struct}}
=
\left(
T_{p,\mathrm{end}},
T_{d,\mathrm{start}},
T_{d,\mathrm{end}}
\right)
}
\]

CoDe에서는 각각 \(A''\), \(B''\), \(C\)에 대응해.
Pivot support와 clearance는 \(\mathbf p_{\mathrm{mech}}\)와 \(\mathbf p_{\mathrm{struct}}\) 사이의 constraints로 처리한다.
5. Contact Parameters
Proximal/distal contact interface를 각각 length, height, mounting offset으로 표현한다.
\[
\boxed{
\mathbf p_{\mathrm{contact}}
=
\left(
(\ell_i,h_i,o_i)
\right)_{i\in\{p,d\}}
}
\]

Parameter	Definition
\(\ell_i\)	Contact interface length
\(h_i\)	Contact interface height
\(o_i\)	Axial mounting offset
현재 모델의 contact placement convention은 proximal contact가 MCP 쪽에서 PIP 방향으로, distal contact가 fingertip 쪽에서 PIP 방향으로 확장되는 형태야.
이 offset은 길이 방향 offset이고, palmar 방향 mounting height offset과는 별개야.
6. Common Finger Geometry
Common Finger Geometry는 \(M\)으로부터 유도한다.
\[
\boxed{
\mathbf L(M)=
\begin{bmatrix}
L_f(M)\\
L_p(M)\\
L_d(M)
\end{bmatrix}
}
\]

여기서
- \(L_f\): Extended pose에서 MCP부터 fingertip까지의 거리
- \(L_p\): Proximal contact-bearing structural length
- \(L_d\): Distal contact-bearing structural length
이 길이들은 contact module 자체의 길이가 아니야.
원하는 geometry range는
\[
\boxed{
\mathbf L_{\min}
\leq \mathbf L(M)
\leq \mathbf L_{\max}
}
\]

로 정의한다.

7. 전체 수식 요약
\[
\boxed{
\begin{gathered}
M=
\left(
\mathbf p_{\mathrm{mech}},
\mathbf p_{\mathrm{act}},
\mathbf p_{\mathrm{struct}},
\mathbf p_{\mathrm{contact}}
\right)
\in\mathcal M(m),\\[6pt]
m\in
\{\mathrm{CoDe},\mathrm{2R},
\mathrm{2R\text{-}remote},\mathrm{RCJ}\},\\[6pt]
\mathbf p_{\mathrm{mech}}
\in\mathcal P_{\mathrm{mech}}(m),\\[6pt]
\mathbf p_{\mathrm{act}}
=\left((G_i,T_i)\right)_{i=1}^{N_{\mathrm{act}}},\\[6pt]
\mathbf p_{\mathrm{struct}}
=\left(
T_{p,\mathrm{end}},
T_{d,\mathrm{start}},
T_{d,\mathrm{end}}
\right),\\[6pt]
\mathbf p_{\mathrm{contact}}
=\left((\ell_i,h_i,o_i)\right)_{i\in\{p,d\}},\\[6pt]
\mathbf L_{\min}\leq\mathbf L(M)\leq\mathbf L_{\max}.
\end{gathered}
}
\]

