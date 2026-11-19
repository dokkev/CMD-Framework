# Code Formulation: 구현 규칙

이 문서는 [problem_formulation.md](problem_formulation.md)(이하 SSoT)를 코드로 옮기는 규칙이다. SSoT는 "무엇"을 정의하고, 이 문서는 "어떻게"를 정한다. 둘이 어긋나면 SSoT가 기준이고, 코드나 이 문서를 고친다. 이 문서는 SSoT와 코드를 오가며 함께 고친다.

## 1. 원칙

1. **개념 하나에 구현 하나.** SSoT의 개념(형상, self-collision, feasibility, coverage, MA, γ, $q_{\max}$)은 공용 모듈에 한 번만 구현한다. 구조별 코드는 이 개념을 다시 구현하지 않는다.
2. **구조는 adapter로만 다르다.** 구조(CFL, 2R-DD, 2R-remote, RCJ)별 코드는 정해진 작은 인터페이스(§3)만 구현한다. 그 인터페이스는 설계 벡터에서 morphology를 만들고, 자세별 형상과 기구학을 돌려준다.
3. **문제 정의는 데이터다.** 목적함수와 제약은 SSoT의 이름 그대로 문제 객체(§5)에 나열한다. Evaluator 안에 흩어 두지 않는다.
4. **이름은 SSoT를 따른다.** 코드 식별자와 출력 열 이름은 SSoT 기호에서 정한다(§7). 같은 이름에 다른 뜻을 담지 않는다.
5. **값의 모양으로 의미를 바꾸지 않는다.** 예를 들어 "숫자면 고정, 리스트면 탐색" 같은 설정은 쓰지 않는다. 고정은 하한과 상한이 같은 범위로 표현한다(SSoT A.2).

## 2. 공통 규약

| 항목 | 규칙 |
|---|---|
| 좌표 | 2D 측면, $+x$ distal, $+z$ dorsal. 기준 자세에서 두 phalanx 축은 $x$축 위(SSoT A.3) |
| 각도 | 내부 계산은 rad, 설정과 출력은 deg. Flexion은 $+x$를 $-z$ 쪽으로 돌리는 방향이 양수 |
| 길이 | mm |
| 자세 | $\mathbf q=(q_1,q_2)$. 모든 평가에서 $q_1=0$. $q_2$는 두 phalanx 축 사이의 상대 회전각(SSoT A.3) |
| 맞닿음 | 두 solid 사이의 거리 $g\ge -10^{-6}$ mm이면 겹치지 않은 것으로 본다 |
| Body | 모든 solid는 `palm`, `proximal`, `distal` 중 하나에 속한다 |
| Layer | 모든 solid는 lateral layer 이름을 가진다. 서로 다른 layer의 solid는 충돌 검사에서 제외한다(SSoT A.4) |

## 3. 구조 adapter 인터페이스

각 구조는 다음을 구현한다. 이 인터페이스 밖의 계산은 공용 모듈이 맡는다.

| 메서드 | 돌려주는 것 | SSoT |
|---|---|---|
| `morphology(x, problem)` | 설계 벡터에서 만든 morphology. 길이 $L_1, L_2$, pad, actuator 자세 | $M$ (A.1) |
| `scene(M, q2)` | 자세 $q_2$의 solid 목록. 각 solid는 shape, body, layer, 그리고 drive pivot 예외 표시를 가진다. 기구가 닫히지 않으면 `None` | $\mathcal G(M,\mathbf q)$ (A.4) |
| `faces(M, q2)` | 두 pad의 contact face 선분. 바깥 끝이 먼저 오도록 정렬 | A.5.1 |
| `phalanx_ends(M, q2)` | $\mathbf r_{1,\mathrm{end}}$, $\mathbf r_{2,\mathrm{start}}(\mathbf q)$ | $L_{vp}(\mathbf q)$ (A.3) |
| `theta_pip(M, q2)` | PIP actuator 입력각 | $MA$ (A.5.2) |
| `transmission_angle(M, q2)` | CFL만 구현. 다른 구조는 `None` | $\gamma$ (A.5.2) |
| `descriptor(M)` | $(L_f, L_1, L_2, x_{\mathrm{art}})$ | $\mathbf d$ (A.6.1) |
| `internal(M)` | 외형으로 정해지지 않는 mechanism 좌표 | $\boldsymbol\kappa$ (A.6.3) |

## 4. 공용 모듈과 SSoT 대응

목표 배치는 다음과 같다. 경로는 `design/core/` 아래이며, 이행 중에는 바뀔 수 있다.

| 모듈 | 내용 | SSoT |
|---|---|---|
| `geometry/primitives.py` | 닫힌 식 거리(점–사각형, 사각형–원, 원–원). 다각형과 D형 servo 거리는 `morphology/packaging/_kernels.py`, `shapes.py` | — |
| `geometry/intervals.py` | 1D 구간 집합 | A.5.1의 $T_i$ |
| `morphology/packaging/assembly.py` | Self-collision 엔진: body, layer, joint(drive pivot) 예외, keep-out을 가진 solid 배치와 쌍별 간격. 구조 adapter가 이 배치를 만든다 | self-collision (A.4) |
| `evaluation/feasibility.py` | `f_static(M)`, `f_motion(M, q_req)` | $\mathcal F_{\mathrm{static}}$, $\mathcal F_{\mathrm{motion}}$ (A.4) |
| `evaluation/coverage.py` | `cylinder_region(faces)`, `c_pad(M, q2)`, `c_pad_mean(M, q_req)` | $T$, $c_i$, $C_{\mathrm{pad}}$, $\bar C_{\mathrm{pad}}$ (A.5.1) |
| `evaluation/transmission.py` | `ma(M, q2)`, `ma_wc(M, q_req)`, `gamma_ok(M, q_req)` | $MA$, $MA_{\mathrm{wc}}$, $\gamma$ (A.5.2) |
| `evaluation/motion.py` | 운동 sweep, `q_max(M)` | $q_{\max}$ (A.5.3) |
| `problems/prototype.py` | 목적함수와 제약 목록 | A.5.4 |
| `problems/study1.py`, `problems/study2.py` | 수용 판정, 변화량, 단계적 탐색 | A.6.2, A.6.3 |
| `mechanisms/{cfl,two_r_dd,two_r_remote,rcj}.py` | §3 adapter | A.1, A.6.1 |
| `config/` | 문제별 설정 스키마와 YAML 로더(구조 공용) | A.2, A.5, A.6 |

## 5. 문제 객체

문제 객체는 목적함수와 제약을 SSoT 이름으로 나열한다. Optimizer는 이 목록만 읽는다.

**Prototype (SSoT A.5.4).**

| 종류 | 이름 | 정의 | 위반량(≤ 0이면 만족) |
|---|---|---|---|
| 목적 | `c_pad_mean` | $\bar C_{\mathrm{pad}}$, 최대화 | — |
| 제약 | `morphology_bounds` | $\mathbf m_{\min}\le\mathbf m\le\mathbf m_{\max}$ | 범위 밖으로 벗어난 양(mm 또는 비율) |
| 제약 | `f_static` | $\mathbf 0\in\mathcal Q_{\mathrm{valid}}$ | 기준 자세의 최대 겹침 깊이(mm) |
| 제약 | `f_motion` | $\mathcal Q_{\mathrm{req}}\subseteq\mathcal Q_{\mathrm{valid}}$ | $q_{\mathrm{req}}$ 중 도달하지 못한 각도(deg) |
| 제약 | `gamma_range` | $\gamma\in[\gamma_{\min},\gamma_{\max}]$ on $\mathcal Q_{\mathrm{req}}$ (CFL) | 범위 밖으로 벗어난 최대 각도(deg) |
| 제약 | `ma_min` | $MA_{\mathrm{wc}}\ge MA_{\min}$ (CFL) | $MA_{\min}-MA_{\mathrm{wc}}$ |

정적 제약(`morphology_bounds`, `f_static`)을 만족하지 않으면 운동 평가를 하지 않는다. 그 설계의 운동 제약은 "평가 안 함"으로 표시한다(SSoT A.4의 판정 방식 차이).

## 6. 수치 규칙

| 항목 | 규칙 |
|---|---|
| $\mathcal F_{\mathrm{static}}$ | 기준 자세 한 번. 닫힌 식 거리로 판정한다 |
| $\mathcal F_{\mathrm{motion}}$ 표본 | $q_2$를 균일 간격 $\Delta q$(기본 1°)로 $[0,q_{\mathrm{req}}]$에서 표본한다. 표본 사이에서 간격이 0 밑으로 내려갈 수 있는 구간은 더 촘촘한 간격(기본 0.05°)으로 다시 본다 |
| 같은 body의 쌍 | 기준 자세에서 한 번만 검사하고 운동 표본에서는 뺀다 |
| $MA$ | $\lvert d\theta_{\mathrm{pip}}/dq_2\rvert$. CFL은 기구 구속을 미분한 해석식으로 구한다. 다른 구조는 상수(2R 계열 1, RCJ 1/2) |
| $\gamma$ | 운동 표본마다 계산한다 |
| $q_{\max}$ | 같은 표본을 $\pi$까지 이어 간다. 처음으로 무효이거나 $\gamma$가 범위 밖인 표본의 바로 앞 각도이다 |
| Coverage | 각 평가 자세 $q\in\mathcal Q_{\mathrm{eval}}$에서 face 직선의 교점 $X$를 구하고, $T_1\cap T_2$를 구간 집합으로 계산한다. 두 face가 평행하면 $c_i=0$으로 둔다 |
| $L_f(\mathbf q)$ | $L_1+\lVert\mathbf r_{2,\mathrm{start}}(\mathbf q)-\mathbf r_{1,\mathrm{end}}\rVert+L_2$ |

## 7. 이름 규칙

| SSoT | 코드·출력 이름 |
|---|---|
| $\bar C_{\mathrm{pad}}$ | `c_pad_mean` |
| $C_{\mathrm{pad}}(M,q)$ | `c_pad_q090` 처럼 자세(deg)를 붙인다 |
| $c_i$ | `c_common_mm` ($c_1=c_2$) |
| $MA_{\mathrm{wc}}$ | `ma_wc` |
| $q_{\max}$ | `q_max_deg` |
| $\mathbf d$ | `d_lf_mm`, `d_l1_mm`, `d_l2_mm`, `d_xart_mm` |
| $\varepsilon_d$, $\varepsilon_\kappa$ | `eps_d`, `eps_kappa` |

옛 출력 이름(`pad_coverage`, `torque_ratio_max`, `curl_deg`, `phi_max_deg`)은 새 run에서 쓰지 않는다. 옛 run은 legacy로 보고, 그림이 필요하면 새 정의로 다시 실행한다.

## 8. 현재 코드와의 차이 (이행 목록)

| # | 현재 | SSoT | 단계 |
|---|---|---|---|
| D1 | 형상, 충돌, coverage, k가 세 벌(linkage 규칙 기반, 2R 계열 수작업 쌍 목록, 옛 SSoT 모듈) | 공용 모듈 한 벌(§4) | P1–P3 |
| D2 | Coverage는 pad별 교점 규칙, $L_f(\mathbf 0)$로 정규화, $q_{\mathrm{req}}$ 한 자세 | 원기둥 규칙, $L_f(\mathbf q)$, $\mathcal Q_{\mathrm{eval}}$ 평균 | P3 |
| D3 | 목적함수 셋(transmission, pad_coverage, curl) | 단일 목적 `c_pad_mean` | P4 |
| D4 | Hyperextension 요구(stopper까지 sweep) | 없음 | P4 |
| D5 | Linkage는 servo 각 균일 표본, 2R 계열은 $q_2$ 균일 표본 | 모든 구조 $q_2$ 균일 표본(§6) | P2 |
| D6 | k를 표본 기울기로 계산(3벌) | CFL 해석식 한 벌 | P3 |
| D7 | Layer 예외가 linkage에만 명시 | 모든 solid가 layer를 가짐 | P2 |
| D8 | `LinkageConfig`가 네 구조의 공용 설정. 862줄. 값의 모양으로 의미가 바뀜 | 문제별 설정 스키마, 구조 공용 로더 | P4 |
| D9 | 길이를 longitudinal(e_x 투영)이나 $\lvert OC\rvert$로 잼 | 유클리드 길이(SSoT A.3) | P3 |
| D10 | `canonical.py`, `collision.py`, `coverage.py`, `canonical_mapping.py`가 테스트에서만 쓰임 | 삭제. 쓸 부품만 §4로 옮김 | P1 (완료) |

## 9. 이행 단계

각 단계는 테스트가 통과한 상태로 끝낸다. 정의가 바뀌지 않는 양(충돌 판정, 기구학)은 옛 경로와 새 경로의 값을 비교해 같은지 확인한다.

| 단계 | 내용 |
|---|---|
| P1 (완료) | `geometry/primitives.py`, `geometry/intervals.py`로 기하 부품을 모으고, 옛 SSoT 모듈(`canonical`, `collision`, `coverage`, `canonical_mapping`), 쓰이지 않던 `hypervolume`, `geometry/planar`, `geometry/transform`을 지웠다 |
| P2 | Scene과 self-collision, $\mathcal F_{\mathrm{static}}$, $\mathcal F_{\mathrm{motion}}$을 만들고 CFL adapter부터 옮긴다 |
| P3 | Coverage(원기둥 규칙), $L_f(\mathbf q)$, MA, $\gamma$, $q_{\max}$를 공용 모듈로 옮긴다 |
| P4 | 문제 객체와 설정 스키마를 만든다. 목적함수를 하나로 줄이고 hyperextension을 지운다. 출력 이름을 바꾸고 소비자 코드를 고친다 |
| P5 | 2R-DD, 2R-remote, RCJ adapter를 옮긴다 |
| P6 | Study 1, 2를 새 문제 객체로 옮기고 옛 모듈과 테스트를 지운다 |

## 10. 열린 항목

- $\mathcal F_{\mathrm{motion}}$의 표본 간격과, 표본 사이를 다시 보는 기준(지금 2R 계열은 distal 끝의 이동 거리로 상한을 둔다)
- 맞닿음 허용 오차($10^{-6}$ mm). 지금 코드에는 $10^{-9}$와 $10^{-6}$이 섞여 있다
- 두 face가 평행할 때 $c_i=0$으로 둘지
- $MA_{\min}$, $[\gamma_{\min},\gamma_{\max}]$, $\mathcal Q_{\mathrm{eval}}$ 같은 값은 설정에 두고, 문서에는 기본값만 적을지
