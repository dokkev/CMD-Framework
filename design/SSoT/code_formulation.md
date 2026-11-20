# Code Formulation: 구현 규칙

이 문서는 [problem_formulation.md](problem_formulation.md)(이하 SSoT)를 코드로 옮기는 규칙이다. SSoT는 "무엇"을 정의하고, 이 문서는 "어떻게"를 정한다. 둘이 어긋나면 SSoT가 기준이고, 코드나 이 문서를 고친다. 이 문서는 SSoT와 코드를 오가며 함께 고친다.

## 1. 원칙

1. **개념 하나에 구현 하나.** SSoT의 평가량(coverage, $L_f(\mathbf q)$, $\mathcal F_{\mathrm{motion}}$, $q_{\max}$, $\gamma$, MA)은 `core/evaluation`에 한 번만 구현한다. 구조별 코드는 이 계산을 다시 하지 않는다.
2. **구조는 형상과 표본만 낸다.** 구조(CFL, 2R-DD, 2R-remote, RCJ)별 코드는 자기 형상(충돌 간격, contact face, phalanx 끝점)과 운동 표본만 만든다. 그 결과를 SSoT의 양으로 바꾸는 것은 공용 함수다(§3).
3. **제약은 SSoT의 범주를 이름에 단다.** 형태 범위는 `bounds_*`, $\mathcal F_{\mathrm{static}}$은 `static_*`, 운동 제약은 `f_motion_deg`, `gamma_range_deg`, `ma_min`이다(§5).
4. **이름은 SSoT를 따른다.** 코드 식별자와 출력 열 이름은 SSoT 기호에서 정한다(§7). 같은 이름에 다른 뜻을 담지 않는다.
5. **값의 모양으로 의미를 바꾸지 않는다.** 설정은 "숫자면 고정, 리스트면 탐색" 같은 형태를 쓰지 않는다. 고정은 하한과 상한이 같은 범위로 표현한다(SSoT A.2). PIP servo 기울기 `tilt_angle_deg`는 범위만 받는다. 남아 있는 예외는 §10에 적는다.

## 2. 공통 규약

| 항목 | 규칙 |
|---|---|
| 좌표 | 2D 측면, $+x$ distal, $+z$ dorsal. 기준 자세에서 두 phalanx 축은 $x$축 위(SSoT A.3) |
| 각도 | 내부 계산은 rad, 설정과 출력은 deg. Flexion은 $+x$를 $-z$ 쪽으로 돌리는 방향이 양수 |
| 길이 | mm |
| 자세 | $\mathbf q=(q_1,q_2)$. 모든 평가에서 $q_1=0$(YAML 로더가 MCP 표본을 0 하나로 둔다). $q_2$는 두 phalanx 축 사이의 상대 회전각 |
| 맞닿음 | 간격 $g\ge -10^{-6}$ mm이면 겹치지 않은 것으로 본다. 상수는 `packaging.hardware.MARGIN_TOLERANCE_MM` 하나다 |
| Body와 layer | CFL의 solid는 `morphology/packaging/assembly.py`의 `Part`로 body와 lateral layer를 가진다. 서로 다른 layer의 쌍과 joint(pin과 그것이 잇는 body)는 검사하지 않는다. 2R 계열은 검사할 쌍을 `solids.margins`에 나열하며, 다른 layer에 있는 belt와 carrier는 목록에서 뺀다 |

## 3. 구조와 공용 모듈의 경계

구조별 코드는 다음 두 가지를 공용 모듈에 넘긴다.

| 넘기는 것 | 형태 | 받는 공용 함수 |
|---|---|---|
| 운동 표본 | `core.evaluation.MotionSamples`: 균일한 $q_2$ 격자, 표본별 유효성(self-collision 없음, 손끝이 MCP 앞), PIP actuator 입력각 $\theta$, CFL은 $\gamma$와 허용 범위 | `f_motion_violation_deg`, `gamma_violation_deg`, `q_max_rad`, `worst_mechanical_advantage` |
| 평가 자세의 face와 끝점 | 두 contact face 선분(바깥 끝 먼저), 가려진 구간, $L_1$, $L_2$, $\mathbf r_{1,\mathrm{end}}$, $\mathbf r_{2,\mathrm{start}}(\mathbf q)$ | `common_length_mm`, `posed_finger_length_mm`, `PoseCoverage`, `mean_coverage` |

구조별 구현 위치는 다음과 같다.

| 구조 | 표본과 coverage | 정적 검사와 제약 |
|---|---|---|
| CFL | `morphology/mechanism/linkage/motion.py`: `sweep_flexion`, `coverage` | `optimization/linkage/evaluate.py` |
| 2R-DD, 2R-remote, RCJ | `optimization/revolute/evaluate.py`: `measure_motion`, `joint_coverage`(세 구조 공용, MA 상수만 다름) | 각 구조의 `optimization/<구조>/evaluate.py` |

## 4. 모듈과 SSoT 대응

경로는 `design/core/` 아래다.

| 모듈 | 내용 | SSoT |
|---|---|---|
| `geometry/primitives.py` | 닫힌 식 거리(점–사각형, 사각형–원, 원–원) | — |
| `geometry/intervals.py` | 1D 구간 집합, 선분과 원이 face를 가리는 구간 | A.5.1의 $T_i$와 가림 |
| `morphology/packaging/assembly.py`, `_kernels.py`, `shapes.py` | self-collision 엔진: solid 배치, 쌍별 간격(다각형, D형 servo, capsule) | self-collision (A.4) |
| `evaluation/coverage.py` | `common_length_mm`(원기둥 규칙), `posed_finger_length_mm`, `pad_coverage`, `evaluation_poses_rad`, `PoseCoverage`, `mean_coverage` | $T$, $c_i$, $L_f(\mathbf q)$, $C_{\mathrm{pad}}$, $\mathcal Q_{\mathrm{eval}}$, $\bar C_{\mathrm{pad}}$ (A.3, A.5.1) |
| `evaluation/motion.py` | `MotionSamples`, `reach_rad`, `uniform_samples_rad`, `gamma_violation_deg` | $\mathcal F_{\mathrm{motion}}$, $\gamma$, $q_{\max}$ (A.4, A.5.2, A.5.3) |
| `evaluation/transmission.py` | `mechanical_advantage`, `worst_mechanical_advantage` | $MA$, $MA_{\mathrm{wc}}$ (A.5.2) |
| `optimization/linkage/config.py` | `LinkageConfig`: 요구사항, 탐색 범위, packaging. 네 구조가 공용으로 읽는다(2R 계열은 `goal`로 감싼다) | A.2, A.5 |
| `optimization/linkage/goal.py` | YAML(`goal` / `solver`)을 `LinkageConfig`로 읽는다. 모르는 항목, 빠진 항목, 지워진 항목은 오류다 | — |
| `optimization/linkage/record.py` | `run.json` 기록과 복원. 옛 formulation의 필드는 버린다 | — |
| `optimization/mechanism.py` | `Mechanism` 플러그인 인터페이스, `evaluation_report`(모든 구조 공용 출력 열) | §7 |
| `optimization/mounting.py` | `shallowest_supported` 장착 규칙, `depth_violation_mm` | A.6.2 장착 규칙 |
| `optimization/analysis/adaptation.py` | Study 2의 단계적 탐색, $\varepsilon_d$, $\varepsilon_\kappa$ | A.6.3 |
| `scripts/study/study1_fixed_morphology/run.py` | Study 1 수용 판정 | A.6.2 |

## 5. 제약과 목적함수

목적함수는 `optimization/linkage/mechanism.py`의 `OBJECTIVES`에 있다(2R 계열이 공유). Prototype은 `c_pad_mean`과 `ma_wc` 둘을, study는 `c_pad_mean` 하나를 `solver.objectives`에 적는다. `ma_wc`가 정의되지 않는 설계(정적 검사 실패)는 0으로 둔다. 제약은 구조마다 다음 범주로 나열한다.

| 범주 | 이름 | 정의 | 위반량(≤ 0이면 만족) |
|---|---|---|---|
| 형태 범위 | `bounds_finger_length_mm`, `bounds_proportions_mm`, `bounds_proximal_pad_mm`, `bounds_distal_pad_mm`, `bounds_mount_depth_mm`, `bounds_dorsal_space_mm`, (RCJ) `bounds_pad_drop_mm` | $\mathbf m_{\min}\le\mathbf m\le\mathbf m_{\max}$, pad 길이 하한과 지지, 장착 규칙 | 범위 밖으로 벗어난 양(mm) |
| $\mathcal F_{\mathrm{static}}$ | `static_closure`(CFL), `static_pin_spacing_mm`, `static_pivot_pin_mm`, `static_mcp_servo_mm`, `static_pip_servo_pad_mm`, `static_pad_collision_mm`, (RCJ) `static_roller_pad_mm`, (2R-remote) `static_a_pin_servo_mm` | 기준 자세의 형상이 존재하고 겹침이 없음 | 최대 겹침 깊이(mm) |
| $\mathcal F_{\mathrm{motion}}$ | `f_motion_deg` | $\mathcal Q_{\mathrm{req}}\subseteq\mathcal Q_{\mathrm{valid}}$ | $q_{\mathrm{req}}$ 중 유효하게 도달하지 못한 각도(deg) |
| 전달 | `gamma_range_deg` (CFL) | $\gamma\in[\gamma_{\min},\gamma_{\max}]$ on $\mathcal Q_{\mathrm{req}}$ | 범위 밖으로 벗어난 최대 각도(deg) |
| 전달 | `ma_min` (CFL) | $MA_{\mathrm{wc}}\ge MA_{\min}$, 기본 $MA_{\min}=0.2$ | $MA_{\min}-MA_{\mathrm{wc}}$ (null이면 0) |

정적 제약(`bounds_*`, `static_*`)을 하나라도 어기면 운동을 평가하지 않고, 운동 제약을 `UNSWEPT`(100)로 둔다. 기준 자세에서 CFL이 닫히지 않으면 모든 제약을 `INVALID`(1000)로 둔다.

## 6. 수치 규칙

| 항목 | 규칙 |
|---|---|
| 운동 표본 | $q_2$를 0부터 $\pi$까지 균일 간격 `flexion_step_rad`(기본 1°)로 표본한다. CFL은 `FourBar.sweep_pip`로 각 $q_2$를 직접 푼다(닫히지 않는 표본부터는 없음) |
| 표본 사이 | 한 칸 동안 distal 쪽이 움직일 수 있는 거리보다 두 이웃 표본의 간격 합이 작으면, 그 칸을 `verify_step_rad`(기본 0.05°)로 다시 본다(`core.evaluation.resample_between`). 이동 거리는 2R 계열에서 관절에서 가장 먼 distal 점까지의 거리 × 각도 간격, CFL에서 표본 사이에 실제로 움직인 거리($B$, $B'$, $C$, distal pad 꼭짓점 중 최대)다. Front의 설계는 `verify_step_rad` 격자에서 다시 평가한다 |
| 유효성 | 표본의 모든 검사 쌍 간격이 $\ge -10^{-6}$ mm이고, 손끝이 MCP 앞($x_C\ge -10^{-6}$)이다. 손끝 조건은 palm과의 충돌을 대신한다 |
| $MA$ | $\lvert d\theta/dq_2\rvert$를 균일 격자 위의 중앙 차분(`np.gradient`)으로 구한다. CFL의 $\theta$는 PIP servo가 돌리는 링크 $A'B$의 회전각이다. 2R 계열은 $\theta=q_2$(MA 1), RCJ는 $\theta=q_2/2$(MA 1/2) |
| $MA_{\mathrm{wc}}$ | $[0,q_{\mathrm{req}}]$ 표본의 최솟값. 유효한 운동이 $q_{\mathrm{req}}$에 닿지 못하면 0 |
| $q_{\max}$ | 처음으로 무효이거나 $\gamma$가 범위를 벗어나는 표본의 바로 앞 각도, 상한 $\pi$ |
| Coverage | 각 평가 자세에서 두 face 직선의 교점 $X$를 구하고, 각 face가 $X$에서 열린 쪽으로 덮는 거리 구간에서 가려진 구간을 뺀 뒤 교집합의 길이를 잰다. 두 face가 평행하면 0 |
| 가림 | Prototype profile(`include_linkage_occlusion` 기본 on)에서 CFL은 링크 $AB'$, $A'B$가, 2R-remote는 belt와 pulley가 face 앞을 가린다. Study profile은 기본 off |
| $L_f(\mathbf q)$ | $L_1+\lVert\mathbf r_{2,\mathrm{start}}(\mathbf q)-\mathbf r_{1,\mathrm{end}}\rVert+L_2$. $\mathbf r_{1,\mathrm{end}}=(L_1,0)$ |

## 7. 이름 규칙

| SSoT | 코드 속성 | 출력 열 |
|---|---|---|
| $\bar C_{\mathrm{pad}}$ | `c_pad_mean` | `c_pad_mean` |
| $C_{\mathrm{pad}}(M,q)$, $c_i$, $L_f(\mathbf q)$ | `poses[k]` (`PoseCoverage`) | `c_pad_eval{k}`, `c_common_eval{k}_mm`, `lf_eval{k}_mm`, `q_eval{k}_deg` (k = 1, 2, 3) |
| $MA_{\mathrm{wc}}$ | `ma_wc` | `ma_wc` |
| $q_{\max}$ | `q_max_rad` | `q_max_deg` |
| 유효 운동의 도달 | `valid_reach_rad` | `valid_reach_deg` |
| $q_{\mathrm{req}}$ | `required_rom_rad` | `q_req_deg` |
| $\gamma$의 범위 | `gamma_min_rad`, `gamma_max_rad` | `gamma_min_deg`, `gamma_max_deg` |
| $\varepsilon_d$, $\varepsilon_\kappa$ | `eps_d`, `eps_kappa` | `eps_d`, `eps_kappa` |

옛 출력 이름(`pad_coverage`, `torque_ratio_max`, `curl_deg`, `phi_max_deg`, `safe_reach_deg`, `eps_E`, `eps_Q`)은 새 run에서 쓰지 않는다. 옛 run은 legacy다. `run.json`은 여전히 읽히며, 옛 목적함수 목록은 `[c_pad_mean]`으로 바뀌고 설계는 새 정의로 다시 평가된다. 그림이 필요하면 새 정의로 다시 실행한다. `scripts/study/exploratory/`는 옛 출력을 읽는 기록용 스크립트라 옛 이름을 그대로 둔다.

Formulation 이름(`run.json`의 `formulation`)은 `cfl-pf1`, `cfl-idealized-pf1`(`linkage_idealized`: PIP servo 몸체 없음, `LinkagePackaging.pip_servo_body`), `two-r-dd-pf1`, `two-r-idealized-pf1`, `two-r-remote-pf1`, `rcj-pf1`, `rcj-idealized-pf1`이다. 서로 다른 formulation의 run은 비교하지 않는다.

## 8. SSoT와의 차이 이행 기록

| # | 이전 | 지금 |
|---|---|---|
| D1 | 형상, 충돌, coverage, k가 세 벌 | 평가량은 `core/evaluation` 한 벌. 충돌 엔진은 CFL의 Assembly와 2R 계열의 `solids.margins`가 남는다(§10) |
| D2 | pad별 교점 규칙, $L_f(\mathbf 0)$ 정규화, 한 자세 | 원기둥 규칙, $L_f(\mathbf q)$, $\mathcal Q_{\mathrm{eval}}$ 평균 |
| D3 | 목적함수 셋 | `c_pad_mean` 하나. 단일 목적 GA를 추가했다. 2026-11-19: prototype은 `ma_wc`를 둘째 목적으로 더한다(NSGA-II, SMS-EMOA) |
| D4 | Hyperextension 요구 | 지웠다. YAML에 있으면 오류 |
| D5 | CFL은 servo 각 균일 표본 | 모든 구조 $q_2$ 균일 표본 |
| D6 | k를 세 곳에서 따로 계산 | MA를 공용 함수 하나로 계산 |
| D7 | Layer 예외가 CFL에만 명시 | 그대로다(§10) |
| D8 | `linkage/config.py` 862줄, 값의 모양으로 의미가 바뀜 | `config.py`(데이터), `goal.py`(YAML), `record.py`(`run.json`)로 나눴다. 의미가 바뀌는 설정은 일부 남았다(§10) |
| D9 | longitudinal 길이나 $\lvert OC\rvert$ | coverage의 $L_f(\mathbf q)$는 유클리드. 크기 범위의 $L_f$는 $\lvert OC\rvert$(기준 자세에서 같다) |
| D10 | 옛 SSoT 모듈이 테스트에서만 쓰임 | 지웠다 |

## 9. 검증

- `design/tests/test_evaluation.py`: 원기둥 규칙(짝 없는 pad, 교점 너머, 가림, 평행), $L_f(\mathbf q)$, $\mathcal Q_{\mathrm{eval}}$, 도달 각도, $\gamma$ 위반, MA
- 구조별 테스트: CFL의 coverage가 face와 가림을 독립적으로 계산한 값과 같은지, 2R-DD의 공통 길이가 해석식 $\min(\text{끝})-\max(\text{시작}, D\tan(q/2))$와 같은지, RCJ의 MA가 1/2인지
- 전체 파이프라인: `scripts/optimize/run.py`로 GA와 NSGA-II를 짧게 돌려 run 저장, 0.05° 재검증, 병합까지 확인

## 10. 결정과 남은 항목

**결정(2026-11-19).**
- $MA_{\min}=0.2$(CFL), 맞닿음 허용 오차 $10^{-6}$ mm 하나, 기울기는 범위로만, CFL도 표본 사이를 다시 본다, Study 2는 $q_{\mathrm{req}}\in\{90^\circ,120^\circ,150^\circ\}$ 모두.
- 정렬은 `goal.alignment`의 `joint_axes`, `contact_faces` 두 값이다(`LinkageConfig.align_joint_axes`, `align_contact_faces`). `contact_faces`은 distal 깊이를 proximal 깊이 + (h_P − h_D)로 둔다(`face_shift_mm`). `run.py`와 `sample_seeds.py`는 시작 전에 `alignment_warning`으로 정적 필요조건을 본다. `contact_faces`이면 두께 차이를 두 장착 깊이 범위로 메울 수 있는지, 둘 다 true면 pad 하한 길이의 합이 한 직선에 들어가는지 확인한다. 깨지면 warning을 내고 시작하지 않는다(설정 오류가 아님). CoDe prototype은 둘 다 true다.
- 2R 계열의 충돌 엔진을 Assembly로 옮기지 않는다. 2R 계열의 쌍 목록은 구조당 40줄 정도로 짧고, Assembly로 옮기면 구조마다 Part·Layout·Poses를 새로 만들어야 해서 코드가 늘어난다. 대신 중복된 MCP servo 쌍 처리를 `revolute.solids.with_mcp_pairs`로 모았다.

**남은 항목.**
- **설정 스키마.** 2R 계열이 `LinkageConfig`를 공용 goal로 감싼다. pad 길이 `length_mm`와 장착 깊이 `mount_offset_mm`는 아직 "숫자면 고정"을 받는다
- **Layer 규칙의 표현.** CFL은 Assembly의 layer로, 2R 계열은 쌍 목록에서 빼는 방식으로 같은 규칙을 표현한다
