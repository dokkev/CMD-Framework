# 재설계가 interface 연구의 결론을 뒤집는가 (기존 데이터 재분석, 실험용)

E = |D(H) − D(O)| / |D(O)|는 비율이라 "interface 효과가 원래 작으니 상관없다"는 반론에 약하다. 이 분석은 같은 Study 3 데이터를 결론 수준에서 다시 읽는다. 새 시뮬레이션은 없다. 결과는 report/에 넣지 않고, 데이터 파일만 `report/study3/data/conclusion_flip.json`에 둔다(study3.md, SSoT, 논문은 고치지 않음).

## 사전 기록 (2026-11-26, 계산 전)

### 질문

1. **부호 뒤집힘.** 재설계 손이 interface 효과를 O와 반대 부호로 재는 일이 얼마나 잦은가(예: Fig. 1 부드러운 pad, O −41%, Conventional +48%).
2. **순위 보존.** 같은 손·같은 배치에서 여러 interface 변화를 쟀을 때, 재설계 손이 효과 크기의 순위를 O와 같게 매기는가(손마다 O에 대한 Kendall τ_b).
3. **편향 대 효과.** 재설계가 얹는 편향 |D(H) − D(O)|가 효과 |D(O)| 자체보다 큰(E > 1) 일이 얼마나 잦은가.
4. **설계 지표와의 관계.** 뒤집힘 비율, τ, E > 1 비율이 Δ_contact를 따르고 Δ_skel은 따르지 않는가.

### 설정 (interface 변화 × 과제 × 접촉 모델)

모든 설정에서 D(H) = M(H, c₁) − M(H, c₀), 같은 배치(과제의 키)에서 짝짓는다. 쌍의 유효 조건(Gate 0, 관통, lost, solver 실패, 한계 없음)은 각 과제의 E 코드와 같다(`fig4_sync.py`의 `*_pairs`, `pad_shape.py`, `analyze.py`, Isaac `dose_analysis.py`/`robust_analysis.comp_effect`/`conclusion_analysis.py`).

| id | 엔진, 접촉 | 과제 | interface 변화 c₀ → c₁ | O의 기준 | O-noise run (+0.05°) | 과제 하한 |
|---|---|---|---|---|---|---|
| pull_pt_fric | Drake point | pull-out (21) | μ 1.3 → 2.0 | O | `e27nf` (O의 e27 = e21, 비트 단위 같음) | 0.2 N |
| pull_hy_fric | Drake hydro | pull-out (21) | μ 1.3 → 2.0 | O | 없음 | 0.2 N |
| pull_hy_soft13 | Drake hydro | pull-out | pad 0.1 MPa/3 mm → 0.1 MPa/h (μ 1.3; e21h → e25) | O_h(`e25-ref<h>`) | `e25n-ref7` (7 mm 손만) | 0.2 N |
| pull_hy_soft20 | Drake hydro | pull-out | 같음, μ 2.0 | O_h | `e25n-ref7` | 0.2 N |
| pull_hy_fricsoft | Drake hydro | pull-out (25) | 자기 두께만큼 부드러운 pad에서 μ 1.3 → 2.0 | O_h | `e25n-ref7` | 0.2 N |
| pull_pt_shape | Drake point | pull-out (34) | flat → 2 mm crowned (μ 1.3) | O | `e34n` | 0.6 N (34차 사전 값) |
| acq_pt / acq_hy | Drake point / hydro | 획득 (23) | μ 1.3 → 2.0 | O | `nf` / `e23hn` | 0.3 mm |
| hook_pt / hook_hy | Drake point / hydro | hook (38) | μ 1.3 → 2.0 | O | `e38n` / `e38hn` | 0.2 N |
| dial_pt / dial_hy | Drake point / hydro | 손끝 원판 (29z 64개 / 29zh 앞 32개) | μ 1.3 → 2.0 | O | `e29zn` / `e29zhn` | 0.3° |
| pivot_pt / pivot_hy | Drake point / hydro | 상자 세우기 (35 64개 / 35h 앞 32개) | μ 1.3 → 2.0 | O | `e35n` / `e35hn` | 0.3° |
| is_fric | Isaac point | pull-out (6) | μ 1.3 → 2.0 | O | `A-nf-ptd` (ρ 15 mm만) | 0.2 N |
| is_stiff | Isaac point | pull-out (10) | ke 2.5e3 → 1e3 N/m (μ 1.3) | O | 없음 | 0.2 N |
| is_soft13 | Isaac point | pull-out | ke 2.5e3 → 2.5e3·3/h (μ 1.3; A → A-padk) | O_h(`A-padk-ref<h>`) | 없음 | 0.2 N |
| is_soft20 | Isaac point | pull-out | 같음, μ 2.0 | O_h | 없음 | 0.2 N |
| is_fricsoft | Isaac point | pull-out (17) | ke ∝ 1/h pad에서 μ 1.3 → 2.0 | O_h | 없음 | 0.2 N |
| grid_fric | Isaac point, 격자 | pull-out (13) | μ 1.3 → 2.0 | O | 없음 | 0.2 N |
| grid_soft13 / grid_soft20 / grid_fricsoft | Isaac point, 격자, 새 7 mm 손 27개 (19) | pull-out | 위와 같음 (ref7) | O_7 | 없음 | 0.2 N |
| is_soft_H4, grid_soft_H4 | Isaac point | pull-out (17, 19) | H4 방식: D(H) = F(H, 부드러운 pad) − F(O, 딱딱한 pad), μ 1.3·2.0을 함께 | O_h | 없음 | 0.2 N |
| dial_pt28z | Drake point | 손끝 원판 (28z 12개, Fig. 4의 point) | μ 1.3 → 2.0 | O | `e28zn` | 0.3° (보조) |

- pull-out 하한 0.2 N은 하중 3 step(0.067 N × 3)이다(38차 hook과 같은 값). 형상은 34차가 미리 정한 0.6 N을 쓴다.
- soft 설정(Drake e25, Isaac padk)은 손마다 자기 두께의 기준 O_h와 비교한다. 5, 7, 9 mm 손의 O 값이 다르다. Drake e25에서 못 쥐는 손은 뺀다(25차와 같음).
- `is_soft_H4`, `grid_soft_H4`는 Fig. 1과 H4(19차)의 정의다. D(H)에 재설계 자체의 효과가 들어 있어 다른 설정과 정의가 다르므로 표에만 싣고 손 단위 합산(아래 ρ)에는 넣지 않는다.
- Isaac 8차 형상(손 7개, 34차로 대체)과 22차 굴리기(경계)는 넣지 않는다.

### 잡음 바닥과 쌍 제외

- 설정마다 바닥 f = max(과제 하한, P75 |D(O′) − D(O)|). O′은 O를 명령 +0.05°로 다시 돌린 run이고, P75는 O와 O′이 모두 유효한 쌍에서 잰다. O′ run이 없으면 f = 과제 하한.
- |D(O)| < f인 쌍은 O만 보고 뺀다(O의 결론 자체가 잡음 안에 있음). 이 제외가 과제의 원래 제외보다 넓을 수 있어 쌍 수가 Study 3 표와 다를 수 있다.

### 지표

- **쌍 단위 (신뢰도).** 유효하고 |D(O)| ≥ f인 쌍에서
  - 뒤집힘: D(H)·D(O) < 0. D(H) = 0은 뒤집힘이 아니고 따로 센다.
  - E > 1: |D(H) − D(O)| > |D(O)|. 부호가 뒤집히면 항상 E > 1이므로 E > 1 ⊇ 뒤집힘이다.
- **손 단위 (결론).** 그 손의 같은 쌍들에서 중앙값 D̃(H), D̃(O).
  - 결론 뒤집힘: D̃(H)·D̃(O) < 0이고 |D̃(O)| ≥ f. 쌍이 5개 미만이면 손 단위 판정을 하지 않는다.
  - E > 1: E(H) = 중앙값 |D(H) − D(O)| / |D(O)| > 1.
- **손 묶음.** R = {`h3to5_hr`, `R`, `h3to9_hr`}, U = {`h3to5_hu`, `U`, `h3to9_hu`}, 전체 = 그 설정에서 점수를 받은 재설계 손 전부. 쌍 단위 비율은 묶음의 쌍을 합쳐 낸다. 손 단위는 "뒤집힌 손 수 / 판정한 손 수"다. O-noise는 O′을 한 손처럼 다룬다.

### 순위 (여러 interface를 같은 손·배치에서 잰 집합)

| 집합 | interface | 손 |
|---|---|---|
| RS-Isaac | is_fric, is_stiff, is_soft13, is_soft20, is_fricsoft | 18 |
| RS-grid | grid_fric, grid_soft13, grid_soft20, grid_fricsoft | 27 (격자, 7 mm) |
| RS-Drake-hydro | pull_hy_fric, pull_hy_soft13, pull_hy_soft20, pull_hy_fricsoft | 18 (못 쥐는 손 제외) |
| RS-Drake-point | pull_pt_fric, pull_pt_shape | 18 |

- **항목 (주):** interface × 하중 방향(+x, +x−z, −z, −x). 값 = O와 H가 모두 유효한 그 칸의 쌍에서 D의 중앙값(부호 있음, N). 순위의 항목에는 |D(O)| 바닥을 쓰지 않는다(작은 효과도 순위의 일부). 칸의 쌍이 3개 미만이면 그 손에서 그 칸을 뺀다. 공통 항목이 4개 이상인 손만 τ_b를 낸다.
- **항목 (보조):** interface 단위 중앙값(2–5개). 그리고 "가장 큰 |효과|의 interface가 O와 같은가".
- τ_b는 O의 항목 벡터와 H의 항목 벡터 사이 Kendall τ_b다. soft 항목의 O 값은 H의 두께의 O_h다.

### 통계와 예측

- **손 단위 합산 (18개 손).** 손마다 위 표의 within-hand 설정들(H4 방식, 격자, 28z 보조 제외)에서 (a) 쌍 단위 뒤집힘 비율의 평균, (b) 쌍 단위 E > 1 비율의 평균, (c) 결론이 뒤집힌 설정의 비율, (d) E(H) > 1인 설정의 비율, (e) RS-Isaac, RS-Drake-hydro, RS-Drake-point의 τ_b 평균. 설정마다 같은 가중이다.
- **검정.** 각 지표와 Δ_contact, Δ_skel의 Spearman ρ(동률은 평균 순위), 단측 순열 p(20,000번, seed 0; 예측 방향: (a)–(d)는 ρ > 0, (e)는 ρ < 0). Δ_skel도 같은 방향의 단측 p를 낸다. partial Spearman(Δ_contact | Δ_skel, Δ_skel | Δ_contact)을 기술로 낸다. 설정마다 ρ(쌍 단위 뒤집힘 비율, ·)와 ρ(쌍 단위 E > 1 비율, ·)도 낸다. RS-grid는 27개 손으로 따로 낸다.
- **예측.**
  - P1: 설정마다 뒤집힘 비율과 E > 1 비율이 U > R(설정 수로 셈, 기술).
  - P2: (a), (b), (c)의 ρ(·, Δ_contact) > 0, p < 0.05.
  - P3: (e)의 ρ(τ, Δ_contact) < 0, p < 0.05.
  - P4: Δ_skel에 대한 ρ는 Δ_contact의 것보다 작고 p ≥ 0.05.
  - (d)는 기술로 보고한다(대부분의 설정에서 E(H) > 1인 손이 적어 0이 많을 것으로 봄).
- **실패의 뜻.** P2가 실패하면 "Δ_contact를 지키면 결론 뒤집힘을 막는다"는 말은 쓸 수 없고, 비율 E에서의 결과만 남는다.

### 보고

- 표: 설정 × 접촉 모델마다 O-noise, R, U, 전체의 쌍 단위 뒤집힘 비율과 E > 1 비율, 손 단위 뒤집힘 수, 바닥 f, ρ(·, Δ_contact), ρ(·, Δ_skel).
- 손 단위 합산의 ρ와 p, τ 표.
- 결론이 뒤집힌 구체 사례 2–3개: U 계열이 손 단위로 뒤집히고 같은 두께의 R은 뒤집히지 않은 설정 중 |D̃(O)| / f가 큰 순서로 고른다.
- 스크립트 `sim/drake/conclusion_flip.py` (`cd sim/drake && uv run python conclusion_flip.py`), 출력 `report/study3/data/conclusion_flip.json`.
