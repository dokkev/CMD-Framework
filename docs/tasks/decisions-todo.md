# 설계 비교 실험: 결정 사항과 TODO

`design-comparison-task.md`, `paper-figures-task.md` 진행 중 내린 결정과 남은 일. 결정이 바뀌면 이 파일을 고친다.

## 결정됨

### 비교 study (`scripts/optimize/comparison/`, 2026-10-30 삭제)
- 2026-10-30: morphology를 라이브러리로 정리. `Packaging`은 공통 하드웨어만, linkage 배치(핀별 지름, servo 위치, 축 여유, 충돌 허용, pad drop, `links_clear_pads`, `pads_off_pins`, `pad_link_ratio_max`)는 `LinkagePackaging(Packaging)`. `run.json` 필드 이름 그대로. `core.morphology.linkage`/`packaging`/`contact`가 공개 API를 export하고 optimization을 import하지 않음(테스트로 확인). `tests/test_morphology_library.py`: study 없이 강체 규칙, 간격, linkage 검사·sweep, pad 늘리기.
- 2026-10-30: 충돌 검사를 강체 + Y 층 규칙으로 일반화(`core/morphology/packaging/assembly.py`: `Part`, `Layout`, `Assembly`, `Poses`, `margins`). linkage는 `PARTS`/`JOINTS`/`layout(packaging)`/`assembly`/`poses`. 규칙: 다른 몸체 & 층 공유 & 관절 아님 → 검사, 같은 몸체는 keep-out으로만. 규칙이 새로 끌어내는 16쌍(핀↔MCP servo, 관절 건너 핀끼리, B 핀↔PIP servo, 핀↔자기가 잇지 않는 링크, carrier↔MCP servo·proximal pad·proximal 핀)은 `NOT_YET_CHECKED`로 빼 둠 → 켤지 결정 필요(결과 바뀜, formulation 버전 업). 제약·feasibility 비트 단위 동일, 보고값 `pad_clearance_mm`만 5 mm 넘는 경우 하한으로 바뀜(carrier–servo 쌍도 broad phase).
- [ ] 정적 keep-out(pad↔자기 핀 등)은 0 근처 반올림을 0으로 안 맞춤: drop = 핀 반경인 설계에서 ±1e-16 위반으로 feasibility가 반올림에 좌우됨. 맞출지 결정(결과 바뀜).
- 2026-10-30: linkage의 충돌 검사는 전부 `core/morphology/linkage/packaging.py`가 맡음. 핀 겹침, pad와 자기 핀, A/B′ 핀과 servo, 구동 링크와 servo, servo끼리, 링크 capsule, 지지구조. optimization은 그 간격을 요구조건과 비교만 함. 결과 비트 단위 동일.
- 2026-10-30: packaging 공통화. 하드웨어(`Packaging`, `ContactPad`)와 기구와 무관한 solid·간격·접촉면 계산은 `core/morphology/packaging/`(`hardware`, `shapes`, `gaps`, `faces`, numba `_kernels`), linkage에 붙는 배치(핀 A/A′/B/B′, pad span, `pad_margins`, `pip_footprint(packaging, linkage)` 등 linkage를 받던 Packaging 메서드 5개)는 `core/morphology/linkage/packaging.py`. 결과 비트 단위 동일.
- 2026-10-30: core 구조 정리. 기구가 무엇이고 어떻게 움직이는지(설계 벡터, 기구학, solid와 간격, 운동 sweep, pad 늘리기, 접촉면 손실, ICR)는 `core/morphology/`(`linkage/{design,measures,motion,pad_extension}`, `revolute`, `revolute_parallel`, `rcj`, `open_chain/solids`, `rolling_contact_joint/solids`, `contact/{loss,retention}`, `icr`), 요구조건 대비 제약·목적함수·탐색·config는 `core/optimization/`. 옮기기 전후 모든 mechanism의 평가 결과(설계 약 3,300개)와 extend_pads 결과가 비트 단위로 같음을 확인.
- 2026-10-30: 이 브랜치는 production linkage만 남기기로 함. `scripts/optimize/accommodation/`(모듈 수용 study, 기준 레이아웃 탐색)과 이를 읽던 테스트(`test_accommodation`, `test_contact_envelope`, `test_evaluator_consistency`), figure(`comparison/contact_envelope.py`, `comparison/modules.py`, `study/accommodation.py`, `study/length_budget.py`) 삭제. 마지막 상태는 커밋 494427d에 있음. core의 비교용 코드는 남김.
- 2026-10-30: `scripts/optimize/`는 production linkage(`run.py`, `linkage/config.yaml`, `sample_seeds.py`, `select_seeds.py`, `extend_pads.py`)와 `accommodation/`만 남김. `interface_coverage`, `pad_cost`, `pad_tradeoff`, `revolute`(linkage config를 base로 쓰던 한 줄짜리), contact retention(`open_chain_*`, `rolling_contact_joint`, `contact_retention.yaml`), `coverage_by_flexion.py` 삭제. 이들만 읽던 figure(`visualize/studies.py`, `paper/fig_{coverage,2r,cost,mechanisms}.py`, `study/{pad_cost,pad_tradeoff,retention}.py`, `comparison/pad_fit.py`)와 `tests/test_contact_retention.py`도 삭제.
- 2026-10-30: 코드베이스 정리로 study 폴더와 이를 모으던 paper pipeline(`visualize/paper/{collect,build,fig_scientific,fig_visual}.py`), `tests/test_comparison_study.py` 삭제. 이를 config로 쓰던 테스트와 `visualize/comparison/modules.py`는 `scripts/optimize/accommodation/`의 config로 옮김. 아래는 기록용.
- production linkage study와 분리. `base.yaml`이 goal을 고정: nsga2, population 200, seeds [0, 1, 2], 출력 `output/comparison`, proximal pad 하한 20 mm. preset 의존 없음.
- warm start: 모든 메커니즘이 쓰지 않음 (linkage seed 상속 금지).
- linkage는 `on_links: false`. 기준 조건(h10, φ150)에 `on_links: true` 변형 run (`linkage_onlinks`).
- `joint_span_ratio` 제거. 대신 `proximal_length_ratio [0.35, 0.55]`, `distal_length_ratio [0.25, 0.55]`를 모든 메커니즘이 읽음 (baseline formulation 버전 업).
- 조건 grid: 두께 10/5/3 mm × 요구 flexion 150/120/90°.
- 예산: baseline을 linkage 이상으로 맞추고 모든 run의 수렴 곡선(`history.npz`, generation별 best) 저장. 세대 수는 미정 (아래 TODO).
- 4b(대칭성 통계)는 현재 production front로 계산하고, production 조건(목적 3개, `on_links: true`)의 통계임을 명시.

### 메커니즘
- 2R parallel → belt 구동(k = 1), servo는 A′. pulley 반경 `goal.hardware.pulley_radius_mm` (기본 6 mm). tendon(Task 3a)과 같은 pulley 모델을 쓰고 차이는 palmar routing 띠만.
- parallelogram 2R은 transmission [30, 150]에서 φ > 120° 불가 → 보고서에 기록.
- 1a 통일: tilt 탐색, A 핀 검사, 공통 distal carrier, drop [0, r], hyperextension −10°.
- overhang: `pads.*.overhang_max_mm` 기본 10 mm, 모든 메커니즘 공통. h10/φ150에 0 mm 민감도 run (`linkage_overhang0`, `rcj_overhang0`).
- formulation: linkage-v4, revolute-goal-v4, revolute-idealized-goal-v3, revolute-parallel-goal-v3 (belt), rcj-goal-v2.

### Y-layer
- central: pad, servo. outer: 링크, carrier 외 bar, belt pulley·strand. through: 핀.
- belt pulley: outer layer, 충돌 검사 없음, 가림 판정에는 포함.
- tendon palmar routing 띠: pad layer(central).

### 손실 판정
- 임시 규칙: v4 segment (상대 pad 면 segment 뒤·맞은편이면 손실). 모든 메커니즘 적용, 테스트: 깊이가 같을 때 기존 규칙과 일치, φ에 대해 연속.
- 구조(제안 수용): 두 판정으로 분리, 판정 함수가 규칙과 D를 인자로 받음 (`core/morphology/contact/loss.py`, 당시 `core/optimization/analysis/loss.py`).
  - integrity: 모든 자세 [0, φ_req], 작은 D (`loss.integrity_depth_mm`). objective와 coverage에 사용.
  - accessibility: 자세별 프로파일 A(φ), 큰 D (`loss.access_depth_mm`, 초기 20 mm). report와 Visual figure 자세 열에 사용.
  - 대상 solid: 자기 pad를 제외한 모든 solid (상대 pad, carrier, 링크, 핀 housing, servo, pulley, roller), Y-layer 무관.
- 손실 진단은 물리적 의미 확인용이며 규칙 선택에 쓰지 않음.

### 실행 / figure
- `scripts/optimize/run.py`는 run_*/portfolio_*가 있는 출력 폴더를 거부 (`--append`로만 추가).
- 논문 figure: heavy search보다 figure 생성 구조 먼저. 초안은 비교 study quick run(`output/comparison_quick`, seed 0, 300세대) + v4 segment, PROVISIONAL 표시. `python -m visualize.paper.build [source]`.

## 2026-10-29 방향 전환: 2R vs CoDe, comparison model

범위를 2R(servo를 joint에 둠)과 CoDe로 줄이고, sweep 전에 공정한 비교 조건부터 정한다. belt, ideal, RCJ, tendon, 비교 study 격자는 보류.

- **Comparison model vs Prototype model 분리.** 비교에는 정확한 기구학과 단순한 점유 공간만 쓴다. servo 외형, 핀 housing, 링크와 bar는 capsule(반경 3 mm, 측면 투영), clearance 0(접촉은 허용하고 관통은 금지). 조립공차, bearing, 강성은 제외한다. prototype 결과와 섞지 않는다.
- **간섭:** pad–pad, pad–servo, pad–링크/joint envelope, 의도하지 않은 구조–구조. 자기 몸체에 장착되는 pad–carrier/bar 접촉은 제외한다 (`goal.collision.param.link_radius_mm`).
- **모듈:** P = (ℓp, hp), D = (ℓd, hd) 독립. 같은 실험에서는 두 기구가 같은 모듈 쌍을 쓰고, 두 모듈을 모두 단 채 평가한다. pad 정렬(coplanarity) 제약은 없다. proximal은 MCP 쪽 끝, distal은 손끝 쪽 끝을 고정하고 palmar에 둔다. overhang은 10 mm 이하.
- **손실 규칙 없음:** 모듈이 동작 전체에서 들어가면 수용으로 본다 (`goal.loss.fold: false`).
- **공통 구조 공간:** pad 장착면 위 dorsal 34 mm (`goal.link.dorsal_space_mm`). 이전의 총두께 35 mm 결정은 이것으로 대체한다.
- **목적:** L*_m(P, D, φ_req) = min |OC|. φ_max는 보지 않는다. φ_req ∈ {90, 120, 150, 180}°. 2R의 180°는 config를 만들지 않는다: 고정 pivot은 pad를 펴서 접을 수 없고, smoke run(80세대)에서도 해가 없었다. 기준 모듈은 25 × 5 mm.
- **실험 A:** D 고정, P ∈ ℓ {15, 25, 35} × h {3, 5, 10}. **실험 B:** P 고정, D를 같은 격자로. φ마다 기구당 17 config, 합계 119 (`scripts/optimize/accommodation/`, 출력 `output/accommodation`).
- **두 연구 질문:** architecture capability(모듈마다 다시 최적화해 L*를 구함)와 fixed architecture versatility(기구를 고정하고 모듈을 sweep, `core/optimization/analysis/tradeoff.py`).

### 리뷰 반영 (2026-10-29, 기준 run 중단 후)
- **2R setback 제거:** revolute-goal-v6, idealized-v5. pad는 joint housing까지, 그리고 그 너머로 overhang(10 mm)만큼 뻗을 수 있다. 그 안에서는 housing(servo 외형 또는 핀)과의 실제 충돌만 판정한다. CoDe의 자기 body 규칙과 같다.
- **hyperextension 0°:** accommodation 비교는 extension부터의 굽힘만 본다. 같은 검사를 두 기구에 적용한다.
- **2R 180° 포함:** 미리 제외하지 않는다. 해가 없으면 "no feasible design found within the tested bounds"로 보고하고, 기구학적 불가능과 구분한다. config 136개.
- **dorsal datum:** O를 지나는 손가락 축선(z = 0) 위의 높이로 잰다. pad drop과 무관하다. 34 mm 유지 (production CoDe는 31 mm).
- **fixed architecture sweep:** A″·B″ 지지구조와 distal carrier(B″부터 손끝까지)를 고정하고 모듈 길이와 두께만 바꾼다 (`tradeoff.with_modules`). 지지구조보다 긴 모듈은 장착 불가.
- **기존 결과:** 중단 전에 끝난 2R 90°와 CoDe 90°는 이전 규칙의 결과라 `output/accommodation_v0`에 보관한다. 인용하지 않는다.

### 2차 리뷰 반영 (2026-10-29)
- **proximal 지지구조:** pad 바깥 끝부터 A″까지, 장착면 위 4 mm 판으로 둔다. 모듈 길이와 무관하게 남는다 (`goal.collision.param.proximal_carrier_height_mm`, `packaging.proximal_carrier_polygon`). distal pad와 distal carrier를 상대로 충돌 검사한다. distal carrier와 proximal pad의 충돌 검사도 추가했다. 2R도 같다. linkage-v6.
- **dorsal 공간:** 링크와 bar를 capsule 반경(3 mm)까지 포함해 잰다.
- **overhang `null`:** 2R에서도 "제한 없음"으로 본다 (linkage와 같은 의미). 0과 구분한다.

### 기준 레이아웃과 고정 레이아웃 benchmark (2026-10-29)
- **주 실험:** 기구마다 기준 레이아웃 하나. 조건은 |OC| 100 mm, P₀ = D₀ = 25 × 5, 0–150° 연속 굽힘. 선정 기준은 max min(S_p − 25, S_d − 25)이고 동점은 S_p + S_d로 깬다 (목적 `support_balance`; `pad_coverage`와는 다르다). config는 `scripts/optimize/accommodation/reference/`, 선정은 `pick.py`, 결과는 `output/accommodation_reference/reference_layouts.json`.
- **평가:** 선정한 레이아웃을 고정하고 120 / 150 / 170°에서 proximal과 distal을 각각 sweep한다 (`python -m visualize.comparison.contact_envelope`). 반대쪽 모듈은 25 × 5로 둔다. 레이아웃 자체가 그 각도에 못 미치는 경우(`layout_reaches`)와 모듈이 안 들어가는 경우를 구분한다. RoM마다 다시 최적화하지 않는다.
- **지지구조 규칙:** 지지구조는 자기 핀에서 끝날 수 있다 (장착). 자기 segment의 servo 몸체는 관통할 수 없다.
- **morphology redesign cost:** 보류.

## 결정 대기 (사용자)
- [ ] **integrity D** 값 (`loss.integrity_depth_mm`). 근거: `visualize.study.loss_profile` 진단 (D ∈ {0, 2, 3, 5, 10, 20} mm).
- [ ] **예산**: 진단 run 결과 보고 후 결정. 기본안 전 메커니즘 1500세대.
- [ ] **메커니즘 공통 최대 두께 requirement** (Task 2 추가 항목 결과 보고 후).
- [ ] **palmar budget 정의** (Checkpoint B).
- [ ] **interface envelope 변수 표** (Checkpoint C, Scientific figure (c)).

## TODO (구현)
- [ ] integrity D 결정 후: `loss.integrity_depth_mm` / `loss.access_depth_mm` config 키, 모든 evaluator에 integrity 연결 (formulation 버전 업), 테스트.
- [ ] 동점 front 중복 제거 검토 (단일 목적 front의 0.05° 검증이 느림).
- [ ] Task 2: bound 분류 표, evaluator의 ICR₀ 보고, extended / ICR-control run, tilt 90° 상한 = 두께 공짜 사용 여부, best 설계의 dorsal–palmar 두께 → CP2.
- [ ] Task 4: 4a 대칭 모드 closed-form 테스트, 4b 대칭 통계(production 조건 명시), 4c free vs symmetric, 4d v0 후보(margin, CAD 좌표 0.1 mm grid) → CP4.
- [ ] Task 3: 3a revolute_tendon (r_pulley, t_route placeholder는 사용자가 정함), 3b 3R 결정 목록 → CP3.
- [ ] Task 5: 5a analytic overlay, 5b h_b(φ), 5c tolerance Monte Carlo (±0.1 mm × 500), 5d minimal-change 재최적화.
- [ ] Task 6 / paper figures: 비교 study 삭제로 `visualize.paper.build` pipeline도 없음. accommodation 기준으로 다시 정함.
- [ ] 최종 보고서 `docs/figures/paper-figures.md`. 포함할 것: parallelogram φ > 120° 불가, v3 → v4 규칙 변경, production front v4 재검증에서 coverage 상승 (+0.96%p, 전부 endless line → segment 효과), belt pulley outer layer.

## 관찰 (provisional, 인용 금지)
- quick run: 설계가 나온 42칸 모두 CoDe coverage 최대. 2R belt = 2R ideal (모든 칸). 차이는 굽힘·두께가 클수록 큼 (h5: 90° +2.3, 120° +5.4, 150° +15.9 %p). 90°는 seed 편차 안일 수 있음.
- h5/φ150 CoDe는 링크가 φ ≈ 45°부터 proximal 면 앞 0–1 mm를 지남. production linkage는 하드웨어가 면에서 7 mm 이상 떨어짐.
- h10/φ150: 2R 계열 feasible 설계 없음. linkage quick 결과는 비단조 (미수렴).
