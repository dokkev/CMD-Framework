# Study 3에서 뺀 과제와 시도 (2026-11-26)

Study 3의 과제는 point와 hydroelastic 둘 다에서 성립하고 서로 다른 통찰을 주는 다섯 개로 줄였다: 21 power-grasp pull-out, 23 grasp 획득, 38 hook, 29z 손끝 원판, 35 상자 세우기(`report/study3/study3.md`). 나머지는 여기에 모은다. 논문과 SSoT에는 넣지 않는다. 큰 그림은 `findings.md`, 실험별 사전 기록·커밋은 표의 문서에 있다. 모두 Drake, 조건 A(O의 setpoint 재사용), 마찰 1.3 → 2.0, 손 18개; 판정 1 = E_R < E_U, 판정 2 = ρ(E, Δ_contact) > 0, 단측 순열 p < 0.05. hydro는 21차 규칙(모든 손에 O의 pad 강성, 1 ms), 무작위 과제는 앞 32개 뽑기를 point와 짝지음(`docs/experimental/drake_hydro.md`).

## 1. 과제별 숫자

| 과제 | 차 | point: ρ(E, Δ_contact), p (손) | ρ(E, Δ_skel) | point E_R / E_U | hydro | 판정 | 뺀 이유 | 기록 |
|---|---|---|---|---|---|---|---|---|
| 손끝 동전 끌기, 긴 끌기 | 30b | 0.55, 0.0098 (18; 앞 32개 0.53, 0.014) | −0.48 | 2.2 / 16.3% (앞 32개 2.2 / 14.5%) | ρ 0.27, p 0.14; 3.2 / 15.9% | point 성립; hydro 판정 1만 | hydro에서 판정 2 실패 (`d4p5_s0`, `d5p5_s0`, `h3to9_d3p5_maxDs_s0`가 1.8–2.4%) | `docs/experimental/drake_coin.md`; 계획 1d59c5e, 결과 b73f616; hydro 3153013 |
| 손끝 동전 끌기, 짧은 끌기 | 30 | 0.36, 0.073 | — | 7.2 / 22.5% | — | 판정 2 실패 | 짧은 끌기(O 1.8–4.3 mm) | 같은 문서; 695d360, 5fcf277 |
| 엄지–검지 집기 (떠 있는 스프링 pad) | 33 | 0.51, 0.016 | — | 1.7 / 11.4% | — | 통과 | 맞은편 pad를 실제 엄지가 놓을 수 없음 | `drake_pinch.md`; 3ae7026, fae4d3f |
| 세 손가락 집기 (실제 엄지) | 33b | 0.35, 0.077 | **0.58, p 0.0075** | 9.8 / 9.6% | — | 실패, 오차가 skeleton을 따름 | 단단한 맞은편 접촉 | `drake_pinch.md`; a8ab34b, 1c14e58 |
| 엄지 집기 peg-in-hole | 36 | — | — | — | — | 보정에서 막힘 | 관절 한계 안에 평행 pinch 자세 없음(정면 29–31° 기울기) | `drake_peg.md`; c39b158, b6902cd |
| 쥐고 비틀기 | 39 | 0.58, 0.0099 (16) | −0.36 | 16.1 / 54.4% | 9개 손 ρ 0.38, p 0.16; U 쌍 0(힘 gate 48/56). 18개 손 민감도 0.55, p 0.0034(유한한 손 R, `h3to5_hr`뿐). 사후 hydro 등가 gate 0.67, p 0.0033 | point 성립; hydro 판정 1 불가·판정 2 실패 | hydro에서 판정 2 실패 | `drake_twist.md`; 5d76f7d, 7f35337; hydro b45d14a, 183cb7b |
| 레일 위 슬라이더 | 40 | 0.40, 0.053 | −0.52 | 3.2 / 17.0% | 돌리지 않음(사용자 결정) | 근소 실패 | point에서 판정 2 실패 | `drake_slider.md`; 7fd2a3d, 78dbdc9 |
| 손끝으로 공 굴리기 | 41 | 0.38, 0.062 | −0.32 | 10.5 / 32.8% | 돌리지 않음(사용자 결정) | 근소 실패 | point에서 판정 2 실패 | `drake_ball.md`; 6d74eea, a876d44 |
| 구 grasp 획득 | 42 | 0.62, 0.0027 | 0.21 | 84.0 / 100.1% | 다른 세션이 기록 중(`drake_hydro.md`) | 통과(약함; R이 54쌍 중 18 놓침) | 23의 가까운 변형, 평면 모델에서 원기둥과 0.5 mm 안 | `drake_sphere.md`; 2e9f0d9, 41f3048 |

- **반복되는 패턴.** 손끝 하나로 위에서 누르며 끄는 과제(동전, 슬라이더, 공, 상자)는 약하다. `d4p5_s0`, `d5p5_s0`와 maxDs 손들이 Δ_contact가 큰데도 오차가 작다. 동전이 약한 원인은 탁자 compliance가 아니다(레일을 5배 단단하게 한 40차에서도 ρ가 오르지 않음). 맞은편이 단단하면(33b) 오차가 Δ_skel을 따른다.
- **E_R < E_U가 뒤집힌 과제는 없다**(point, hydro 모두).

## 2. 동전 끌기 (30b) 상세: Study 3에서 옮김

- distal pad 하나가 탁자 위 동전(두께 2 mm, 3 g, 지름 28–32 mm)을 눌러 MCP 쪽으로 끈다. 동전은 시작 위치에 끈(45 N/m)으로 매여 있고(끈이 없으면 pad 마찰이 늘 탁자 마찰을 이겨 μ 효과가 0), 탁자는 30 N/m 스프링 위에 있다. 동전–탁자 μ 0.2, y 이동은 막음.
- 명령 0 → q_t(1 s) → q_t + s(4 s, s 45–55°) → 유지 1 s. 지표는 끝 이동, D = 이동(2.0) − 이동(1.3). 무작위 시작 64개(seed 31). O는 64개 모두 남고 8.5–13.1 mm를 끈다(D(O) 5.3–8.2 mm, 모두 양수). 관통 최대 O 0.89 mm, 재설계 0.86–1.14 mm. time step 0.25 ms(1 ms에서는 미끄럼 접촉이 떨려 잡음 4.6%). 잡음 바닥 0.60%.
- 사후 관찰: 순서를 거스르는 손은 `d4p5_s0`, `d5p5_s0`, `h3to5_d3_s0`다. 끌린 거리는 누름(손끝 경로의 수직 성분)을 따르는데 Δ_contact는 방향 없이 경로 변화를 잰다.
- 궤적(R − O, U − O, 끝): 2026-11-25 Fig. 4의 넷째 열이었다(μ 1.3 R −2%, U −18%; 효과 편향 R −2%, U −16%).

## 3. 데이터, 렌더, 코드

- **요약 데이터.** 동전은 Study 3에서 지웠다: `git show c7f812c:report/study3/data/drake_e30b_coin.json`, raw `git show c7f812c:report/study3/data/raw/runs_drake_coin.csv`, Fig. 4 동전 열 데이터 `git show c7f812c:report/study3/data/fig4_sync.json`(`deviation.coin`). 나머지 과제의 요약은 로그에 있다: `sim/drake/logs/e30b_coin.json`, `e30_coin.json`, `e33_pinch.json`, `e33b_pinch.json`, `e39_twist.json`, `e40_slider.json`, `e41_ball.json`, `e42_sphere.json`(로그는 git 밖).
- **렌더.** 동전: `git show c7f812c:report/study3/figures/fingertip_coin.png`, 프레임 `git show c7f812c:report/study3/data/renders/render_coin{0,1,2}.png`. 나머지는 `sim/drake/logs/render_{twist,slider,ball,sphere,pinch,pinch3}_strip.png`(각 프레임 `..0/1/2.png`; `render_twist.py`, `render_slider.py`, `render_ball.py`, `render_sphere.py`, `render_pinch.py`, `render_pinch3.py`). 23차 옛 손가락 렌더 `logs/render_acquire*.png`는 Drake 손가락 렌더라 그림에 쓰지 않는다.
- **코드.** `sim/drake/coin_drag.py`, `peg_pinch.py`, `pinch_rotate.py`, `pinch3.py`, `twist_grasp.py`, `slider_rail.py`, `ball_roll.py`, `sphere_grasp.py`.

## 4. 그 밖에 Study 3 밖인 것

- **긴 손가락 (120–150 mm):** 길이 offset 때문에 Δ_contact를 약 30 mm 아래로 줄일 수 없다. `findings.md` §7, `design/scripts/study/study3_trajectory/long_fingers.py`.
- **RL 전이 (Study 4 확장):** zero-shot, 접촉 기준 관절 사상, 물체를 보지 않는 정책, fine-tuning 모두 R과 U를 가르지 못했다. `findings.md` §1–2, `docs/experimental/rl_transfer.md`.
- **보상 오차 민감도 (32차):** `findings.md` §6, `docs/experimental/drake_comp_sensitivity.md`.
- **34 형상의 hydro:** O가 flat과 crowned 둘 다 통과하는 배치가 2개뿐이라 분석 불가(형상 결과 자체는 point로 Study 3에 남음).
