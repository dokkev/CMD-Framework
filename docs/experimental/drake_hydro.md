# Point 대 hydroelastic: Drake 과제 전체 재실행 — 사전 기록

Study 3 밖의 실험 기록이다. 결과는 사용자가 정하기 전까지 Study 3, 논문, SSoT에 넣지 않는다.

## 목적

과제마다 E가 Δ_contact를 따르는지(판정 1: E_R < E_U, 판정 2: ρ(E, Δ_contact) > 0, 단측 순열 p < 0.05)를 point contact와 hydroelastic contact에서 짝지어 비교한다. 대상 과제는 21, 23, 29z, 38, 39, 34, 30b, 35, 40, 41, 42다.

## Hydro 설정 (모든 과제 공통)

- **pad.** compliant hydroelastic. 모든 손에 O의 pad 기초 강성을 준다: modulus 0.1 MPa / 3 mm × h / 2, h는 그 손의 pad 두께(21차 규칙). resolution hint 2 mm.
  - 25차의 "자기 두께만큼 부드러운 pad"는 쓰지 않는다. 그 설정에서는 두께마다 O_h 기준이 필요하고, E가 O 기준에서 벗어난다. 25차와 같은 점은 modulus 0.1 MPa와 Drake 기본 감쇠다.
- **물체·탁자·레일.** rigid hydroelastic, hint 0.5 mm. 마찰 계수는 point와 같다.
- **time step.**
  - 1 ms. 23, 29z, 34, 38, 39, 42는 point도 1 ms였다.
  - 30b, 35, 40, 41의 point는 0.25 ms였다. hydro 시험에서는 0.25 ms와 1 ms가 같았다: 41차 O 뽑기 3에서 20.10 / 25.75°와 20.10 / 25.78°.
- **구현.** 과제 모듈은 고치지 않는다.
  - 23, 29z, 42는 `two_pad_drake.py --contact hydro`(21차 규칙)를 쓴다.
  - 나머지는 `hydro_run.py <module> ...`를 쓴다. 이 스크립트는 `hydro.enable()`로 pydrake를 패치한다: 모든 plant를 kHydroelasticWithFallback으로 만들고, pad 강성의 재질은 compliant로, 나머지는 rigid로 바꾼다. hydro 접촉은 point pair 하나로 읽는다(중심점, 합력, 면적 가중 법선).
  - **패치 검증.** 21차 pull-out 한 배치(r 15, q0 100°, −z)에서 패치한 point 빌드와 two_pad_drake의 hydro 빌드가 같은 결과를 냈다. O는 둘 다 Gate 0 실패, U는 둘 다 F_limit 6.6 N(loss).
- **실행.** `sim/drake/run_hydro.sh <task> <tag> <delta|-> <hand...>`. tag는 e<task>h, 잡음 바닥은 e<task>hn이다.

## 부분집합 (미리 정한 규칙)

- **무작위 뽑기 과제 (29z, 30b, 35, 40, 41, 42).** 각 뽑기 파일의 앞 32개(draw 0–31)를 쓴다. point 쪽도 같은 32개로 다시 분석해 짝을 맞춘다. point run은 다시 돌리지 않고 기존 log를 쓴다.
- **배치 격자 과제 (21, 23, 34, 38, 39).** 전체 배치를 쓴다. 시험 run에서 hydro 1회가 4–25 s로, point와 비용이 비슷했다.

## Gate

각 과제의 사전 기록 gate를 그대로 쓴다(Gate 0, lost, solver, |D(O)| 하한, O만으로 정한 제외).

- **관통 gate.** point의 관통 gate(force / 2.5 N/mm > 1.5 mm)는 hydro에서 **힘 gate 3.75 N**(pad 힘)이 된다. 30b, 35, 40, 41, 29z처럼 그 gate가 있는 과제에 적용한다. hydro에서는 관통이 아니라 pad를 누르는 힘의 상한이다.
- **O의 제외.** O만의 hydro run으로 다시 정한다. point와 hydro의 유지 쌍은 각자의 gate로 정하고, 표에 둘 다 적는다.

## 보고 (과제마다)

- 표: point(같은 부분집합)와 hydro 각각의 E_R, E_U, ρ(E, Δ_contact), 순열 p, ρ(E, Δ_skel), 유지 쌍 수, 손 수, hydro 잡음 바닥.
- 판정: hydro에서 판정 1·2가 성립하는지. 순위는 hydro의 18개 손(21차는 16개)으로 구한다.

## 21차 확인

21차는 이미 point와 hydro(21차 규칙, 1 ms, 손 16개)가 모두 있다: point ρ 0.67(p 0.0027), hydro 0.86(p < 10⁻⁴). 새로 돌리지 않는다. 다만 O와 U의 r 15, μ 1.3 hydro를 지금의 코드로 다시 돌려 기존 log와 같은지 확인한다(tag e21chk).

## O만의 hydro 보정

O와 그 잡음 바닥만 먼저 돌린다(`run_hydro_O.sh`). 재설계 손의 hydro run은 이 기록을 커밋한 뒤에 돌린다. O만의 결과는 아래에 덧붙인다.
