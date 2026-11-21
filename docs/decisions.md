# 큰 방향 기록

연구 방향을 바꾼 결정만 적는다. 형식은 **가설 → 실험 → 결과(→ 결정)**이다. 구현 세부와 매개변수, 중간 시행착오는 적지 않는다. 그것은 커밋 기록, `report/`, `docs/experimental/`(playground)에 남긴다. 2026-11-21까지의 상세 기록은 `docs/legacy/decisions_full_2026-11-21.md`에 있다.

## 2026-10

- **2R 대 CoDe 비교 모델.**
  - 가설: 같은 외형에 맞춘 두 구조를 비교하면 CoDe의 이점이 드러난다.
  - 실험: `scripts/optimize/comparison/` 파이프라인.
  - 결과: 외형을 한 구조에 맞추면 비교가 그 구조에 유리해진다.
  - 결정: 파이프라인을 폐기했다(10-30).

## 2026-11 (1–20)

- **모듈 수용 연구에서 기구 연구로.**
  - 가설: interface 모듈을 얼마나 받는지가 구조를 가른다.
  - 실험: contact accommodation study, 탐색 연구 E01–E10.
  - 결과: 두꺼운 pad의 이점은 관절이 pad를 얼마나 빼앗는가에서 온다. 회전 중심(ICR)이 움직이는 것 자체에서 오는 것이 아니다.
  - 결정: kinematic study를 동결하고(11-07), 코드 구조는 monorepo로 정리했다.
- **SSoT를 기능 기준으로.**
  - 가설: 코드에 맞춘 정식화는 연구 질문을 흐린다.
  - 결정: `design/SSoT/problem_formulation.md`를 의도한 기능 기준으로 다시 썼다(11-18). 코드는 SSoT를 따른다.
- **Prototype과 study의 분리.**
  - 결정: 구조는 각자 자기 최적 설계(native)로 비교하고, 외형을 서로 맞추지 않는다(11-19).
- **Study 2의 질문.**
  - 가설과 실험: "최소 외형 변화", "재최적화", "pad 활용률" 등으로 여러 번 바꿔 시험했다.
  - 결정: "interface가 바뀌어도 접촉면의 운동을 지킬 수 있는가"로 확정했다(11-20).
  - 결정: 연구 구성을 "해석(Study 0) → 확인(Study 1, 2)"로 정했다.

## 2026-11-21

- **Study 0(해석).** 고정축 관절은 pad 깊이가 Δh 바뀌면 접촉면의 상대 궤적이 2 sin(q/2)·Δh 이상 바뀐다(하한). polycentric 관절은 그렇지 않다. 최적화 결과와 맞는다.
- **Study 1.**
  - 가설: 구조마다 두께와 flexion에 따라 확보하는 coverage가 다르다.
  - 실험: 두께 3/5/7/9 mm × flexion 90/120/150°, 새 탐색 프로토콜(이웃 두께에서 출발).
  - 결과: 둘 다 설계가 있는 10개 조건에서 CFL이 2R-DD보다 2.4–14.6 %p 높다. 2R-remote와의 우열은 flexion과 두께가 정한다(120°에서 약 5–6 mm에서 교차).
- **Study 2.**
  - 가설: CFL은 두꺼운 interface에 맞춰 다시 설계해도 접촉면 궤적을 지킨다.
  - 결과: 고정축 하한의 10–51%로 줄이고, coverage는 최선에서 0.2 %p 이내다. 2R-remote는 하한 위에 정확히 놓인다. coverage만 보고 재설계하면 궤적이 하한보다 더 크게 바뀐다.
- **Study 3 pilot (RL).**
  - 가설: 손을 고정하고 접촉면 형상만 바꾸면 학습 정책의 행동과 성능이 달라진다.
  - 결과 1: flat에서 학습한 정책은 crowned 손에서 성능을 잃는다(seed 10개 모두, p = 0.002).
  - 결과 2: solver 간 비교(VBD, hydroelastic)는 검증을 통과하지 못했다.
  - 결과 3: 학습에 시간이 많이 들어서, 설계 평가 도구로는 부적합하다.
  - 결정: 형상을 바꾸는 A/B 시험만으로는 CoDe가 필요하다는 근거가 되지 않는다. 그래서 넓은 Study 3 계획을 보관했다(`docs/experimental/study3_interface_platform.md`).
- **논문의 축.**
  - 결정: 기여는 "contact가 중요하다는 발견"이 아니라 **"contact interface와 manipulation의 관계를 morphology 변화와 분리해서 관찰할 수 있다"**이다. 성능 개선은 주장하지 않는다. SSoT를 네 파일로 나눴다(north-star = 문제와 기여).
- **용어와 범위.**
  - "contact-centric kinematics"를 **relative contact trajectory**(proximal 접촉면에서 본 distal 접촉면의 경로)로 바꿨다.
  - spacer는 요구조건으로 배제했다. interface가 쓸 공간을 채우고 coverage를 잃기 때문이다.
  - MCP 단계의 궤적은 한계로 두었다. 고정축이라 지킬 수 없다.
- **왜 RCT인가.**
  - 가설: embodiment 변화 가운데 RCT만 제어로 되돌릴 수 없다.
  - 실험 1: 전달(servo 각 → 굽힘각)은 지령 변환과 강성 일정으로 보정된다. 시뮬레이션에서 굽힘 추종 오차 0.07° 이하를 확인했다.
  - 실험 2: RCT가 바뀌면, 같은 원기둥을 다시 쥘 때 접촉 위치가 pad 위에서 옮겨진다(계산).
  - 미결: MCP 쪽 이동의 보정 가능성 계산.
- **분리 가능성 실험 1차 (실험 기능).**
  - 가설: RCT를 지킨 재설계(R)에서 잰 interface 효과는 기준 손(O)에서 잰 값과 가깝고, coverage만 본 재설계(U)는 그렇지 않다.
  - 실험: 고정한 원기둥에 대한 준정적 굽힘 sweep. flat 대 crowned, 9개 조건.
  - 결과: 원시 측정은 U에서 크게 어긋났다(첫 접촉각 9.5°, distal 힘 15–19 N). R은 O를 따라갔다(0.5°, 1–3 N). 형상 효과의 추정도 R이 O에 더 가까웠다(O 효과의 10–14% 대 25–230%).
  - 한계: 이 task에서는 형상 효과 자체가 작다.
  - 다음: friction을 interface 변수로 두고 T_slip을 잰다(`docs/experimental/`).
