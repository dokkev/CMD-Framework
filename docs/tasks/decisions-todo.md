# 설계 비교 실험: 결정 사항과 TODO

`design-comparison-task.md`, `paper-figures-task.md` 진행 중 내린 결정과 남은 일. 결정이 바뀌면 이 파일을 고친다.

## 결정됨

### 비교 study (`scripts/optimize/comparison/`, 2026-10-30 삭제)
- 2026-11-04: contact accommodation study 리뷰 반영. (P0) `evaluate_fit`이 손끝이 MCP 뒤로 넘어가는 조건(C_x < 0, `tip_behind_mcp`)도 실패로 봄(evaluator의 `curl_tip_x_mm`과 같음, 최대 안전 굴곡과 요구 굴곡 판정 모두). (P1) 고정 core 모듈 교체 규칙을 d_req ≤ d_core ≤ d_native로 완화(`mount_incompatible`). study 공통 baseline을 production config의 고정 사본(`scripts/study/baseline.yaml`)으로 분리. core에 most_curl과 2R-DD tilt 0을 추가(중복 설계는 한 번만). 비대칭 모듈 4쌍 추가. CoDe transmission angle 민감도 config(`code_tilt10_ta10`). manifest dirty 검사를 `scripts/study`까지 확대, core의 run과 candidate 기록. 2R-DD distal 지지 가정은 CAD 확인 전까지 보류(distal 0, align 시만 servo reach).
- 2026-11-04: study의 장착 깊이는 탐색하지 않고 정책으로 정함(`goal.pads.mount_policy: shallowest_supported`, `core.optimization.mounting`). 각 pad를 독립적으로 처리. 자기 관절 하드웨어(CoDe: 자기 segment의 핀 housing과 proximal pad의 PIP servo, 2R-DD: servo)에 닿지 않으면 d = 0(Case A), 닿으면 그것을 피하는 최소 격자 깊이(Case B, 판정은 기존 충돌 함수). 그 하드웨어가 자기 body에서 선 아래로 내려오는 깊이(native support)를 넘으면 spacer가 필요하므로 설계 없음(Case C). 2R-DD의 servo는 proximal body라 proximal pad만 지지하고, distal pad(출력축 A에서 truss로 매달림)의 native support는 0. 단 align on이면 두 pad가 더 깊은 쪽의 깊이 하나를 쓰고 distal도 servo reach까지 지지(사용자 결정). search가 깊이를 설계 벡터에 기록하고, evaluator가 `mount_depth_mm`로 일치 여부를 검사(production은 `searched`, 항상 0). study는 align off. Stage B에서 모듈이 다른 깊이를 요구하면 `mount_depth_changed`(재장착 안 함).
- 2026-11-04: 논문용 메커니즘 비교는 `scripts/study/`에 둠. 공통 goal(|OC| 80~90, φ_req 120°, align on, search preset, warm start 없음)은 `base.yaml` 하나, 메커니즘·조건마다 파일 하나(`code_tilt10`, `2r_dd_tilt0/45/90`), 결과는 `output/study/<mechanism>/<파일명>/`. 2R-DD tilt 0만 align off(align on이면 feasible 없음). prototype용 run은 여기 넣지 않음.
- 2026-11-04: 두 pad의 장착 offset을 같게 묶는 옵션 `goal.pads.align_mount_offsets`(기본 false). true면 두 범위가 겹치는 구간에서 offset 하나를 탐색해 두 pad에 같이 쓰고, 겹치지 않으면 config 오류. 2R-DD는 distal drop이 두 drop을 정함. 옵션이라 formulation은 그대로.
- 2026-11-04 (linkage-v12, revolute-goal-v10, idealized-v9, parallel-v9): 장착 offset을 설계 변수로. 기준선은 링크선 하나(단일 datum)로 두고, 각 pad의 장착면 offset d_p, d_d ≥ 0을 탐색(면 깊이 D = d + h). 같은 범위를 모든 메커니즘에(`goal.pads.*.mount_offset_mm`, 운영 [0, 10]: 2R-DD servo 반지름 9.5까지 덮음). 핀·servo·링크는 어떤 d에서도 hard 충돌. 전략 A(d = 0, 핀 앞에서 끝)와 B(d ≥ 핀 반지름, 핀 아래로 연장, 접힘 손실 증가)를 optimizer가 직접 비교. 최종 설계의 d는 설계 벡터에 있으므로 fixed-layout 모듈 교체에서도 그대로(어댑터 재설계 없음). linkage 설계 벡터 14개(`proximal_mount_offset_mm`, `distal_mount_offset_mm`, `Finger.pad_offsets_mm`), 2R은 기존 drop 두 칸을 다시 탐색. 링크선 고정 비교는 `mount_offset_mm: 0`.
- 2026-11-04: linkage 탐색의 pad 끝 범위를 자기 관절 핀 밖으로 제한(`search._clear_of_pins`: A″_x ≤ A_x − r_A, B″_x ≥ B_x + r_B). v10부터 pad가 링크선 위라 핀 아래를 지날 수 없는데, 탐색이 그 영역을 계속 제안했고 위반값(핀 반지름)이 일정해 방향을 못 잡음: v11 quick run이 feasible 0개. 범위 제한 뒤 quick run front 1,697개. 평가 규칙은 그대로(formulation 유지).
- 2026-11-04 (linkage-v11, revolute-goal-v9, idealized-v8, parallel-v8, rcj-goal-v6): 요구조건 계층.
  - Level 1 finger(|OC|, φ_req), Level 2 contact module(pad 최소 길이, 두께), Level 3 segment 비례(length ratio, joint span ratio), Level 4 mechanism(링크, servo, 핀). 충돌·closure·구동 한계 같은 물리적 feasibility는 계층과 무관하게 항상 hard.
  - 상위 요구를 위해 하위 요구를 완화할 수 있고 반대는 안 됨. 요구조건은 그것이 속한 물리적 대상에 귀속(finger 길이는 finger, pad는 module, 비례는 segment, 간섭은 mechanism).
  - 구현: segment에 module 요구(pad 최소 길이 > 0, 또는 고정 길이)가 있으면 그 segment의 하한은 module이 정함(이미 support 제약으로 강제). segment 비례 하한은 그때 hard 제약에서 빠지고, 부족량을 `proximal_ratio_shortfall_mm`/`distal_ratio_shortfall_mm`로 보고(`LinkageConfig.module_sets_segment`, `length_ratio_shortfalls`). module 요구가 없으면 비례 하한은 여전히 hard. 비례 상한과 joint span 비율은 그대로 hard(지금 module과 겹치지 않음). YAML 구조는 그대로(계층형 migration은 나중).
- 2026-11-04 (linkage-v10): contact interface 장착 기준선을 하나로 통일. CoDe(OA, BC)와 2R-DD(OA, AC) 모두 장착면이 링크선 위(offset 항상 0), 두께는 palmar 쪽. 핀·servo·링크는 충돌 제약으로만 다루고 pad를 내려서 피하지 않음(자기 핀 아래를 지나는 pad는 infeasible). 2026-11-03의 조건부 offset(핀 아래면 3 mm)과 fixed-layout의 offset 고정(`pad_offsets_mm`, `mounting_offsets`)은 없앰: 기준선이 하나라 모듈을 바꿔도 변하지 않음. idealized·belt 2R은 그대로(A 핀 housing에 접함).
- 2026-11-03 (linkage-v9, revolute-goal-v8, idealized-v7, parallel-v7, rcj-goal-v5), 외부 리뷰 반영:
  - 길이 비율은 segment(|OA″|, |B″C|) 기준으로 모든 메커니즘에 일관되게 적용. linkage 탐색은 더 이상 A_x를 비율로 좁히지 않음(평가가 허용하는 설계를 탐색이 미리 빼던 문제). 2R·RCJ도 OA·AC 대신 segment로 검사하고 탐색 범위도 비율로 좁히지 않음.
  - linkage hyperextension: driver 각이 아니라 distal 상대 회전각(2R의 φ)으로 재고, 되돌아가는 구간에서도 pad 충돌 검사(예전엔 펴진 자세부터만 검사).
  - linkage의 pad 교차 계산을 공통 `faces.crossing_free_lengths`로 통일(전용 함수의 `rise == 0` 특수 처리가 교차를 놓쳐 pad 전체를 세던 경우 제거).
  - fixed-layout 분석(`tradeoff.with_modules`, contact envelope)은 설계의 장착면을 고정: 모듈이 바뀌어도 pad 시작선(offset)은 원래 설계의 것(`tradeoff.mounting_offsets`, `LinkagePackaging.pad_offsets_mm`). 보고의 `pad_drops_mm`도 그 값.
- 2026-11-03: linkage pad도 자기 링크 안에 들어가면 링크선에 붙임(`packaging.pad_offsets`): 그 자리에서 자기 몸체 핀 housing(proximal: A, A′, distal: B, B′)과 PIP servo(proximal)를 body clearance 이상 피하면 offset 0, 아니면(핀 아래를 지나면) 핀 clearance에 접하는 선(3 mm). 2R-DD와 같은 원칙: 시작선은 링크선, 하드웨어는 충돌. pad를 핀 양쪽으로 나누는 것은 하지 않음(핀 너머 조각은 요구 굴곡에서 접힌 V 안쪽이라 거의 빠짐).
- 2026-11-03: 2R-DD(servo at joint, 코드의 `revolute`)는 pad를 OA·AC 링크선에 바로 붙임(drop 0): servo 축이 관절이라 따로 bearing housing이 없음. servo 앞쪽 원(9.5 mm)은 충돌 검사로만 다룸(pad가 A 양쪽 9.5 mm 안으로 못 들어옴). idealized·belt는 A 핀 housing(3 mm)에 접함. servo 기울기 범위 `goal.hardware.pip_servo.tilt_angle_deg` [0, 90](linkage와 공유).
- 2026-11-03: 교차점 너머로 장착된 pad는 낭비로 보고 제약 `pad_waste_mm`으로 막음(장착 길이 − 유효 길이 ≤ 해상도 0.1 mm). linkage와 2R 계열·RCJ 모두. 교차점은 링크 형상과 pad 면 깊이로만 정해지고 pad 길이와 무관해서, pad 끝(탐색 변수)만 맞추면 됨. 같은 날 formulation(linkage-v8, revolute-goal-v7 등)에 포함. `output/v8/`의 탐색 run들은 이 제약 전 코드로 돌아서 다시 돌려야 함.
- 2026-11-03 (revolute-goal-v7, revolute-idealized-goal-v6, revolute-parallel-goal-v6, rcj-goal-v4): 2R도 linkage와 같게. pad = contact interface space, A의 housing(servo 반지름, idealized·belt는 핀 반지름)에 접하는 선에서 시작(drop 탐색 없음, 설계 벡터의 drop 칸은 읽을 때 무시: `evaluate.interfaced`). carrier 없음. coverage는 요구 굴곡에서 두 pad 면 연장선 교차까지(`faces.crossing_free_lengths`, linkage 규칙과 같은 값). RCJ는 2R의 `usable_lengths`를 같이 써서 coverage 규칙만 같이 바뀜(자기 pad·drop·carrier는 그대로).
- 2026-11-03 (linkage-v8): pad coverage는 요구 굴곡(`goal.motion.dip_flexion_deg`) 자세 하나에서 잼. 각 pad 면을 바깥 끝(MCP 쪽, C 쪽)부터 상대 pad 면의 연장선과 만나는 점까지 유효로 셈(`packaging.face_crossing_lengths`), 교차점 너머(접힌 V 안쪽)는 뺌. 자기 충돌은 충돌 제약에 맡김(깊이 D 같은 물체 크기 가정 없음). v4~v7은 [0, φ_req] 동안 상대 면 segment가 가리는 길이를 뺐는데, 충돌이 먼저 막아 가리는 일이 거의 없어 사실상 펴진 자세의 pad 길이를 셌음.
- 2026-11-03: A″·B″를 링크와 함께 탐색(`goal.pads.on_links` 기본 false). pad를 관절 너머로 늘려 얻는 coverage와 잃는 curl을 optimizer가 맞바꿈. 예전(on_links true)엔 탐색 중 A″=A, B″=B로 고정하고 `extend_pads.py`가 φ_max를 지키며 늘려 B″가 거의 안 늘었음.
- 2026-11-03: `proximal_length_ratio`/`distal_length_ratio` 상한 없앰(`[0.35, null]`, `[0.25, null]`). |OA″|, |B″C| 기준이라 상한 0.55가 pad 끝을 막았음(quick run에서 A″_x 54.9에서 멈춤).
- 2026-11-03: pad = 링크의 contact interface space. 자기 링크 핀(proximal은 A, distal은 B)의 clearance 원에 접하고 링크선(OA, BC)과 평행한 선에서 시작해 `height_mm`만큼 palmar 쪽(모듈과 장착부를 다 포함), `LinkagePackaging.interface_offset_mm(pad)` = 그 핀 반지름. drop 개념 삭제: `Segment.drop_mm`, 설계 벡터의 drop 두 칸(14 → 12개, 예전 14·16개 벡터는 마지막 drop을 버리고 읽음), `goal.pads.*.drop_mm` 탐색, `proximal_pad_below_a`/`distal_pad_below_b`와 check `a_pin_and_proximal_pad`/`b_pin_and_distal_pad`(예전 run·goal에선 무시). carrier 삭제: distal carrier 부품·검사(`distal_carrier_and_pip_servo`, `distal_carrier_height_mm`), proximal carrier 옵션(`proximal_carrier_height_mm`, `support_margins`). linkage config를 goal로 읽는 다른 메커니즘도 carrier 0. 운영 front feasible 11,152개 그대로; 회귀 표본(같은 설계 재평가) linkage는 carrier 쌍이 빠진 제약값만 바뀌고 판정 변화 없음, 실험용 acc config의 2R은 carrier가 빠져 feasible 3 → 19.
- 2026-11-03: 보고 열은 `app_offset_used_mm`/`bpp_offset_used_mm`(A′/B′ 기준 offset) 유지, segment 길이 열은 안 넣음. 2R은 핀 하나(A)라 B″를 A 쪽으로 늘이면 바로 proximal 몸체와 부딪혀 segment 길이가 CoDe처럼 자유로운 설계 치수가 아님 → 같은 이름으로 비교할 의미가 적음.
- 2026-11-03: `proximal_length_ratio`/`distal_length_ratio`는 손가락 segment 길이로 잼: |OA″|(`finger.proximal.length_mm`), |B″C|(`finger.distal.length_mm`), 예전엔 |OA|, |BC|(링크 핀 기준). 비율은 손가락의 치수이므로. |B″C| ≥ |BC|라 distal 하한이 느슨해짐(front는 |BC| 기준 8%가 하한 0.25에 붙어 있었음), A″는 거의 A라 proximal은 거의 같음. front feasible 11,152개 그대로, 회귀 표본 feasibility 변화 없음. mm 경계(`proximal_length_mm` 등)와 보고값 `proximal_length_mm`은 |OA| 그대로.
- 2026-11-03: 핀 원 = bearing housing, 자기 몸체의 central 층에만 있음(링크 층까지 관통하지 않음, 핀 축은 모델에 넣지 않음). housing은 PIP servo 몸체와 pad(contact interface)에만 닿으면 안 됨(`PIN_CHECKED`): MCP servo, 다른 몸체의 핀, distal carrier, 링크는 검사 안 함. `NOT_YET_CHECKED` 삭제: 여기서 B 핀↔PIP servo(운동 중 `dip_pin` 정지 조건에 포함), carrier↔MCP servo, carrier↔proximal pad가 새로 켜짐. 같은 링크의 두 핀(A–B′, B′–B), A↔servo 축, A↔servo 몸체 keep-out은 그대로. B′ 핀↔PIP servo는 검사하지 않음(`allow_collision_pip_servo_to_joint_bp` 켠 채로 유지, 2026-11-03 결정). production front feasible 11,152개 그대로(이 쌍들의 최소 여유 ≥ 2.88 mm); 회귀 표본에서는 비-production config 설계 1개의 정지 이유가 `pad_collision`→`dip_pin`, 보고값 `pad_clearance_mm` 2개만 바뀜. [x] formulation linkage-v7로 올림(2026-11-03, 핀 규칙·violation_mm·segment 비율·contact interface space를 함께 묶음). 기존 run은 merge·pareto에서 거부됨.
- 2026-10-30: `Finger`는 관절과 무관한 이름만 씀: `origin`(MCP, 예전 `o`), `tip`(예전 `c`), `proximal_end`(예전 `app`, A″), `distal_start`(예전 `bpp`, B″); 자세·sweep이 손가락에게서 받는 칸과 `Kinematics`도 같은 이름. A″/B″와 offset(`app_offset_mm` 등, A′/B′ 기준)은 CoDe의 설계 벡터·보고 열·그림 라벨에만 남김. 관절 자신의 점(`a`, `ap`, `b`, `bp`, CoDe 자세의 `o`)과 packaging의 점 이름(`"c"`)은 그대로. 결과 불변.
- 2026-10-30: 손가락의 `q_pip` = distal segment의 proximal에 대한 굽힘(O-A 대 B-C 상대각, 예전 `total_flexion`). `Finger.solve`/`sweep_pip`/`kinematics`는 이 각으로 풂(`FourBar.solve`/`sweep_pip`: distal 회전을 주고 B'·B를 닫음). four-bar 내부 좌표는 `q_driver_rad`(driver link A-B'의 A 회전, 예전 `q_pip_rad`)와 `q_dip_rad`(그대로), `FourBar.solve_driver`/`sweep_driver`. 평가는 예전처럼 driver 각: hyperextension stopper(10°), backoff 검사, `servo_clearance(driver_samples_rad=…)`, 보고값 `usable_pip_deg`/`deepest_pip`/`closes_to`(이름은 그대로, 값은 driver 각) → 결과 불변. [ ] hyperextension 요구조건을 손가락 PIP(φ)로 바꿀지, 보고 열 이름을 `usable_driver_deg`로 바꿀지 결정(결과/파일 형식 바뀜).
- 2026-10-30: 2R도 `Finger`가 소유하는 관절: `core.morphology.mechanism.revolute.Pin`(A의 핀, ICR = A, servo 각 = 굽힘각), `PinPose`/`PinSweep`, `revolute.decode(x)`/`make_finger`(proximal O→A'', distal B''→C, drop은 segment, tilt는 `ServoMount`). 최적화(`RevoluteGeometry`)는 아직 자체 계산 그대로(결과 불변). `Finger.kinematics(q_pip, q_mcp)` → `Kinematics`(굽힘각, servo 각, 프레임 원점, ICR, C, B'', A'', `transmission_ratio` = dφ/dθ): 관절 종류와 무관한 운동학 API.
- 2026-10-30: ICR(원격 회전중심)을 매 자세 추적. 관절이 계산(`FourBar.icr`, `bar_crossings`: 막대 `A-B'`, `A'-B` 직선의 교점, 평행이면 None/NaN), `FingerPose.icr`(MCP와 함께 회전), `PipSweep.icr` `(n, 2)`, `Finger.icr`(펴진 자세), `FingerMotion.icr_mm`(`[0, phi]`의 servo 샘플마다). `extended_icr_x`/`linkage_icr_x`도 같은 함수로 계산(제약값 비트 단위 동일). `visualize/finger.py`의 자체 계산은 sweep 값으로 대체.
- 2026-10-30: 손가락 = segment + 관절 + servo 장착(`core/morphology/finger.py`: `Finger`, `Segment`, `ServoMount`). CoDe는 `FourBar(a, ap, b, bp)` 관절과 두 segment: proximal은 O→A″, distal은 프레임이 핀 B에 있고 B″→C(의미상 distal 길이 = |B″C|, `end_mm` = |BC|). `FingerLinkageParameters`·`FingerLinkage` 삭제: `Finger`가 관절(`FourBar`)을 소유하고 `solve`/`sweep_servo`로 자기 자세를 냄(기구학은 관절, 마디 위 점 C·A″·B″은 손가락). `PointXZ`는 `core.morphology.finger`로. servo 기울기는 관절이 아니라 `ServoMount`. servo를 A에 다는 변형(`pip_servo_at`) 삭제(쓰는 run 없음; 예전 값 "ap"는 무시). 제약값·판정·보고값 동일(`app/bpp_offset_used_mm`은 1e-9 격자로 반올림). 독립 리뷰 반영: linkage 검사·측정(`evaluate`, `sweep_motion`, `assembly`)은 관절이 `FourBar`가 아니면 `TypeError`(`four_bar`), middle segment는 아직 `NotImplementedError`.
- 2026-10-30: pad chamfer 폐기(pad는 단순 블록). `ContactPad.chamfer_mm`/`chamfer_free`, `pad_outline`의 chamfer, 설계 벡터의 `proximal/distal_chamfer_mm`(16 → 14개), face 길이(`*_face_mm`, 이제 support 길이와 같음) 삭제. 저장된 run의 chamfer 정보는 조용히 무시함 (legacy linkage_v1 25개, linkage_v2 162개는 chamfer 2.5 mm였으므로 다시 평가하면 블록 pad로 평가됨). `ContactPad()` 기본값도 chamfer 2.5 → 없음. production(chamfer 0) 결과는 동일. 위치로 읽는 설계 벡터: 9–12개(뒤 변수는 기본값), 14개(현재), 16개(예전 chamfer 포함, 12–13번째를 버림); 13·15개는 거부(14개는 항상 현재 형식으로 읽음). `FingerLinkageParameters`의 A, B는 `PointXZ`(`a`, `b`); 설계 벡터 변수 이름은 그대로.
- 2026-10-30: morphology를 라이브러리로 정리. `Packaging`은 공통 하드웨어만, linkage 배치(핀별 지름, servo 위치, 축 여유, 충돌 허용, pad drop, `links_clear_pads`, `pads_off_pins`, `pad_link_ratio_max`)는 `LinkagePackaging(Packaging)`. `run.json` 필드 이름 그대로. `core.morphology.mechanism.linkage`/`packaging`/`contact`가 공개 API를 export하고 optimization을 import하지 않음(테스트로 확인). `tests/test_morphology_library.py`: study 없이 강체 규칙, 간격, linkage 검사·sweep, pad 늘리기.
- 2026-10-30: 충돌 검사를 강체 + Y 층 규칙으로 일반화(`core/morphology/packaging/assembly.py`: `Part`, `Layout`, `Assembly`, `Poses`, `margins`). linkage는 `PARTS`/`JOINTS`/`layout(packaging)`/`assembly`/`poses`. 규칙: 다른 몸체 & 층 공유 & 관절 아님 → 검사, 같은 몸체는 keep-out으로만. 규칙이 새로 끌어내는 16쌍(핀↔MCP servo, 관절 건너 핀끼리, B 핀↔PIP servo, 핀↔자기가 잇지 않는 링크, carrier↔MCP servo·proximal pad·proximal 핀)은 `NOT_YET_CHECKED`로 빼 둠 → 켤지 결정 필요(결과 바뀜, formulation 버전 업). 제약·feasibility 비트 단위 동일, 보고값 `pad_clearance_mm`만 5 mm 넘는 경우 하한으로 바뀜(carrier–servo 쌍도 broad phase).
- 2026-11-03: 정적 keep-out 제약(핀 간격, pad↔자기 핀, A↔servo 축·몸체, servo↔proximal pad)은 `violation_mm(margin)`(`core/morphology/packaging/hardware.py`)으로 만듦: −margin, 단 |margin| < `MARGIN_TOLERANCE_MM`(1e-9 mm)이면 0. 간격 값·보고값은 그대로. 예전엔 drop = 핀 반경이라 B 핀↔distal pad가 정확히 닿는 설계가 +1e-16 위반(`distal_support_mm`)으로 정적 단계에서 멈춤: 회귀 표본에서 운영 config 설계 약 10%가 이제 sweep까지 감(feasibility가 바뀐 설계는 없음, front 그대로).
- 2026-10-30: linkage의 충돌 검사는 전부 `core/morphology/mechanism/linkage/packaging.py`가 맡음. 핀 겹침, pad와 자기 핀, A/B′ 핀과 servo, 구동 링크와 servo, servo끼리, 링크 capsule, 지지구조. optimization은 그 간격을 요구조건과 비교만 함. 결과 비트 단위 동일.
- 2026-10-30: packaging 공통화. 하드웨어(`Packaging`, `ContactPad`)와 기구와 무관한 solid·간격·접촉면 계산은 `core/morphology/packaging/`(`hardware`, `shapes`, `gaps`, `faces`, numba `_kernels`), linkage에 붙는 배치(핀 A/A′/B/B′, pad span, `pad_margins`, `pip_footprint(packaging, linkage)` 등 linkage를 받던 Packaging 메서드 5개)는 `core/morphology/mechanism/linkage/packaging.py`. 결과 비트 단위 동일.
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
