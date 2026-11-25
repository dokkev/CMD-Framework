# 42차: 구 grasp 획득 — 사전 기록

Study 3 밖의 실험 기록이다. 결과는 사용자가 정하기 전까지 Study 3, 논문, SSoT에 넣지 않는다.

## 과제

- **구성.** 23차 획득(`two_pad_drake.py --task pullin`, 수정 없이 그대로 호출)에서 원기둥 대신 구를 쓴다.
  - 손가락은 펴진 상태(q = 0)에서 시작한다.
  - 구는 proximal face 위에 있다. O의 두 pad가 setpoint q0에서 구에 닿는 자리보다 손끝 쪽으로 offset만큼 떨어져 놓인다(`dial_placements.x_contact`).
  - 하중은 face 쪽 1.5 N과 손끝 방향 1.5 N이다(세 손가락 몫).
  - 2 s 동안 O의 명령까지 닫고, 1 s 유지한다. 회전 감쇠는 2.0 N·m·s/rad, time step은 1 ms다.
- **측정.** drawn_mm, 곧 구가 MCP 쪽으로 끌려온 거리를 잰다. D = drawn(μ 2.0) − drawn(μ 1.3).
  - E(H)는 끝에 쥔 쌍에서 구한 median |D(H) − D(O)| / |D(O)|다.
  - 끝에 두 pad가 모두 쥐지 못하면 lost로 세고, E_sens에서는 무한대로 둔다.
- **구로 바꾼 방법.** `sphere_grasp.use_sphere()`가 이 프로세스에서 roller 길이의 pydrake Cylinder를 Sphere(rho)로, 관성을 구의 것으로 바꾼다. 질량은 23차 값이다.
- **원기둥과의 차이 (숨기지 않음).**
  - 구는 23차의 평면 슬라이드(x, z, y축 회전)에 그대로 있다. 손가락이 평면이므로 옆 방향 운동은 모델에 없다.
  - point contact에서 구와 원기둥의 차이는 접촉점 하나와 선 접촉의 차이뿐이다. 첫 시험의 O 12개에서 같은 배치의 원기둥과 drawn 차이는 0.5 mm 이하였다.
  - 23차와 달라진 점은 반지름(10–16 mm, 23차는 15·20 mm), 무작위 배치, 구 형상이다. 이 과제는 23차의 가까운 변형으로 본다.
- **무작위 시작.** 64개, seed 42, `numpy.random.default_rng([42, 42])`, 모든 손에 같은 뽑기(`logs/sphere_e42.json`).
  - rho U(10, 16) mm
  - q0 U(85, 115°)
  - offset U(3, 6) mm

## O만으로 정한 설계

- **첫 시험 (q0 U(50, 110°), O 12개).** 8개를 O가 끝에 쥐지 못했다. q0가 85° 미만이면 μ 1.3에서 쥐지 못했다. 그래서 q0 범위를 85–115°로 옮겼다.
- **O, 64개 (tag e42).** solver 0. O가 쥐지 못한 10개를 빼서 **54개를 유지**한다.
  - D(O)는 −1.16 / −0.65 / −0.37 mm로 모두 음수다(마찰이 크면 덜 끌려온다).
  - |D(O)| ≥ 0.3 mm는 모든 쌍이 만족한다.
- **잡음 바닥 (O, 명령 +0.05°, tag e42n).** E 0.23%(75분위 0.30%), 54쌍.
- **R/U gate만.** solver 0. 유지된 54쌍에서 lost는 R 18, U 0이다. E는 보지 않았다.

## 판정 (미리 정함)

1. E_R < E_U
2. ρ(E, Δ_contact) > 0, 단측 순열 p < 0.05(20,000회, seed 0), 18개 손
3. ρ(E, Δ_skel)을 함께 보고한다. E_sens(lost는 무한대)에 대한 ρ도 함께 보고한다.

## 파일

`sim/drake/sphere_grasp.py`, `sim/drake/render_sphere.py`, 렌더 `sim/drake/logs/render_sphere_strip.png`(O, 뽑기 15, rho 16 mm, q0 106°).
