# OmniReset 레시피와 CoDe 손 큐브 회전으로의 대응

Yin et al., "Emergent Dexterity via Diverse Resets and Large-Scale Reinforcement Learning" (ICLR 2026, arXiv 2603.15789)의 구현을 읽고, `CoDe-Rotate-Cube-O-OmniReset` task에 옮긴 기록이다.

- 출처: UW Lab `UW-Lab/UWLab`, commit `2b40afb9693d` (2026-09-28).
- 라이선스: BSD-3-Clause (`LICENCE`). 코드는 복사하지 않았고 구조와 값만 따랐다.
- 아래 경로는 모두 `source/uwlab_tasks/uwlab_tasks/manager_based/manipulation/omnireset/` 기준이다. 줄 번호는 위 commit의 것이다.

## 1. 레시피 (구현 기준)

### 1.1 Reset 네 종류와 dataset 생성

생성 스크립트:
- `scripts_v2/tools/record_reset_states.py`: type마다 4096 env로 10,000개 상태를 모은다.
- 각 episode는 2.0 s이고 action은 0이다 (`config/ur5e_robotiq_2f85/reset_states_cfg.py:527-531`).
- 저장하는 것은 settle이 **끝난** 상태다 (`mdp/recorders/recorders.py:15` `StableStateRecorder.record_pre_reset`). 성공한 episode만 내보낸다.

| type | 생성 (`reset_states_cfg.py`) | 허용 물체 이동 |
|---|---|---|
| Reaching `ObjectAnywhereEEAnywhere` | 물체: 테이블 위 임의 위치, 전 방향 무작위 회전. EE: IK로 무작위 pose (192-231) | ∞ (561-567) |
| Near-object `ObjectRestingEEGrasped` | 물체: reaching dataset에서 가져옴. EE: grasp dataset pose ± 2 cm, ± π/16 (234-265) | 1 cm (570-576) |
| Stable grasp `ObjectAnywhereEEGrasped` | 물체: 임의 위치·회전. EE: grasp dataset pose 그대로 (268-308) | 5 cm (579-585) |
| Near-goal `ObjectPartiallyAssembledEEGrasped` | 물체: partial-assembly dataset. EE: grasp ± 1 cm, ± π/32 (352-391) | 2.5 cm, 절반은 1.5× 성공 임계 안 / 절반은 밖 (601-611) |

**Grasp sampler**
- antipodal 후보 1e6개를 만들고 standoff 32 × orientation 16으로 펼친다 (`mdp/events.py:69-489`, `config/.../grasp_sampling_cfg.py:86-106`).
- gripper를 닫은 뒤 1 s 후에 중력을 켠다. 4 s 동안 물체가 붙어 있으면 통과다 (`mdp/terminations.py:115-218`).

**Near-goal 생성 (partial assembly)**
- 조립된 pose에서 시작해 물체에 매 step ±0.005 N의 무작위 힘·토크를 준다. 마찰은 0이다.
- 충돌이 없고 서로 다른 pose만 기록한다 (`config/.../partial_assemblies_cfg.py:82-158`, `scripts_v2/tools/record_partial_assemblies.py`).

**거부 조건** (`mdp/terminations.py:219-424` `check_reset_state_success`). 모두 만족해야 저장된다:
- 어떤 관절도 속도 한계의 2배를 넘지 않음
- 5 step 연속 안정 (관절 Σ|q̇| < 5, 물체 Σ|v| < 0.1, Σ|ω| < 1)
- 물체 이동이 허용치 이내
- 바닥 위
- gripper가 아래를 향함 (60° cone)
- collision analyzer: SDF 침투가 0.5–1 mm 이내
- time-out에 도달

생성할 때 마찰은 0.3 / 0.2로 낮게 고정한다 (`reset_states_cfg.py:111-149`).

**Cache와 sampling** (`mdp/events.py:1046-1219` `MultiResetManager`)
- `.pt` 네 개를 GPU에 올린다.
- reset 때마다 env별로 `torch.multinomial(probs)`로 type을 고른다. probs는 0.25씩이다 (`rl_state_cfg.py:241-255`).
- type 안에서는 상태를 균등하게 뽑는다.
- 즉 "합집합에서 균등"이 아니라 **type 균등, 그 안에서 균등**이다.
- 쓰는 값: 관절 q·q̇, 관절 target = q, 물체 pose·속도.
- reset 시점에 type별 성공률을 log한다.

### 1.2 Reward (`rl_state_cfg.py:525-566`, 함수는 `mdp/rewards.py`)

| 항 | 식 | weight |
|---|---|---|
| `action_magnitude` | Σa², clamp 1e4 | −1e-4 |
| `action_rate` | Σ(a − a_prev)² | −1e-3 |
| `joint_vel` | Σq̇² (arm 관절) | −1e-2 |
| `abnormal_robot` | \|q̇\| > 2×한계 (`uwlab/envs/mdp/terminations.py:35-37`) | −100 |
| `progress_context` | 성공·거리 계산, 반환값 0 (86-161) | 0.1 |
| `ee_asset_distance` (reach) | 1 − tanh(\|p_grasp − p_obj\| / 1.0) (27-83) | 0.1 |
| `dense_success_reward` | mean(exp(−θ_xy / 1), exp(−d / 1)) (164-174) | 0.1 |
| `success_reward` | 정렬돼 있는 동안 매 step 1 (177-181) | 1.0 |

- 성공 임계값은 receptive 물체의 USD metadata에서 읽는다 (`mdp/commands.py:100-101`). 이 repo 안에는 없다.
- 성공해도 종료하지 않는다.

### 1.3 Observation (`rl_state_cfg.py:367-522`)

**Actor** (372-418):
- 항목: last action, 관절 위치, EE pose, 물체 pose (wrist frame, axis-angle), receptive pose, 물체-in-receptive pose
- `history_length = 0`
- noise 항 없음

**Critic** (421-518):
- 위 항목에 더해 time left, 관절 속도, EE 속도, 재질, 질량, 관절 마찰·armature·stiffness·damping을 쓴다.
- **그러나 `critic: CriticCfg | None = None`이다 (522).** agent도 `obs_groups = {"actor": ["policy"], "critic": ["policy"]}`이다 (`agents/rsl_rl_cfg.py:22`).
- 현재 구현은 **대칭 actor-critic**이다. 비대칭 critic은 정의만 있고 꺼져 있다.

### 1.4 Action과 controller (`config/ur5e_robotiq_2f85/actions.py:14-53`)

- 상대 Cartesian OSC: Δpose scale (0.02, 0.02, 0.02, 0.02, 0.02, 0.2), Kp (200, 200, 200, 3, 3, 3), damping ratio (3, 3, 3, 1, 1, 1)
- binary gripper
- 제어 10 Hz: dt 1/120, decimation 12 (`rl_state_cfg.py:692-696`)

### 1.5 PPO (`agents/rsl_rl_cfg.py:17-65`)

| 항목 | 값 |
|---|---|
| steps/env | 32 |
| network | actor, critic [512, 256, 128, 64] ELU, obs normalization |
| 분포 | **HeteroscedasticGaussian** (state-dependent log std, init 1.0, [0.001, 2.0]) |
| loss | value coef 1.0 (clipped), clip 0.2, entropy 0.006 |
| update | epochs 5, mini-batches 4 |
| learning rate | 1e-4 adaptive, KL 0.01 |
| γ, λ | 0.99, 0.95 |
| grad norm | 1.0 |
| 기타 | save 100, max 40,000 iter |
| env 수 | 16,384 (4 GPU × 4096; `docs/.../rl_training.rst`) |

- gSDE는 현재 레시피에 없다. 주석이 "latent noise matrix 없음"이라고 명시하고, exporter에만 gSDE 분기가 남아 있다.

### 1.6 종료와 domain randomization

**종료:** time-out (16 s)과 abnormal robot뿐이다 (569-575). 물체 낙하로는 끝나지 않는다. arm은 테이블 위 어디든 다시 닿을 수 있기 때문이다.

**Stage-1 DR** (108-234, startup; gain만 reset마다):

| 대상 | 범위 |
|---|---|
| robot 마찰 | 0.3–1.2 / 0.2–1.0 |
| 물체 마찰 | 1.0–2.0 / 0.9–1.9 |
| 물체 질량 | 20–200 g |
| robot 질량 | ×0.7–1.3 |
| gripper gain | log-uniform ×0.5–2 (reset마다) |

- Stage 2 (sysid와 OSC gain의 ADR)는 sim2real용이라 제외했다.

**Eval** (`TrainEvalEventCfg`, 258-271): reaching reset만 쓴다.

## 2. 우리 task로의 대응 (`sim/isaaclab/src/code_isaaclab/tasks/rotate_cube_omnireset/`)

| OmniReset | `CoDe-Rotate-Cube-O-OmniReset` | 비고 |
|---|---|---|
| goal = 조립 pose | **움직이는 yaw goal**: 쉬는 위치(pad 위)에서 시작 yaw + k·90°. 0.5 s(15 step) 유지하면 다음 goal로 넘어감 | UWLab에 in-hand 예제는 없음 |
| 성공 (metadata 임계) | d < 20 mm 이고 \|yaw 오차\| + tilt < 0.2 rad | 우리가 정한 값. tilt는 대칭(가장 가까운 면 기준) |
| 4 reset type, 각 0.25, type 안 균등 | 같음 (`mdp.OmniResetStates`) | dataset 10,000개씩 |
| Reaching | 손 관절 전체 범위에서 균등(PIP flexion 0–120°), 큐브를 pad 위 5–25 mm에서 무작위 방향으로 떨어뜨림 | |
| Stable grasp (antipodal sampler) | cradle 주변 관절 window에서 균등 샘플링. 한계에서 쌓이지 않음 (thumb_ro −90…−50°). 손가락마다 0.25 확률로 들어 올림(regrasp형). 0–10° 열린 상태에서 닫음. 큐브는 ±8 mm, 임의 yaw, tilt ≤ 10°. thumb와 손가락 1개 이상이 접촉해야 통과 | thumb 한계 문제를 여기서 해결 |
| Near-object | grasp에서 손가락과 thumb를 0–20° 연 상태 | |
| Near-goal (partial assembly) | 쉬는 pose 근처(< 15 mm, tilt < 10°)의 grasp에 ±2° jitter. goal offset은 절반이 ±0.3 rad, 절반이 0.3–0.8 rad 앞 | 0.3 = 1.5× 성공 임계 |
| 거부 조건 | abnormal, 5 step 안정(같은 수치), 이동 허용치(같은 수치), 손 위에 남음, 생성 마찰 0.3/0.2 | SDF 침투 검사 대신 안정성 검사로 거름 |
| reward 8항 | 같은 식과 weight | 아래 차이 참고 |
| reach std 1.0 m | 손가락 끝 4개 평균 거리, std 0.1 m | 작업공간 크기 비율로 조정. 1.0이면 상수가 됨 |
| `joint_vel` (arm) | 구동 관절 13개, PIP는 flexion 속도 | |
| actor obs | last action 13, 관절 13, 큐브 pose 6, goal 대비 오차 6 (Δp, tilt xy, yaw 오차) = 38 | history와 noise 없음 (UWLab과 같음) |
| critic | 기본은 대칭 (UWLab 구현과 같음) | `CODE_OMNIRESET_ASYM_CRITIC=1`이면 privileged critic: time left, 관절·큐브 속도, 큐브 질량·마찰 |
| OSC + gripper | rotate_cube의 상대 관절 target (1/24 rad/step, 30 Hz, flexion 공간 PIP impedance) 유지 | OSC는 손에 대응하지 않음 |
| PPO | `agents.py`에 그대로. HeteroscedasticGaussian은 rsl_rl 5.5.1에 있음 | gSDE 불필요 |
| 종료: time-out, abnormal | 같음. 단 큐브가 쉬는 위치에서 80 mm 넘게 벗어나면 종료 | 손은 떨어진 큐브에 닿지 못함 |
| episode 16 s | 학습 16 s. Eval task는 20 s (600 step) | |
| DR stage 1 | 손 마찰 0.3–1.2, 큐브 마찰 1.0–2.0, 큐브 질량 20–200 g, servo gain log-uniform 0.5–2 (PIP는 action term에서) | robot 질량 DR 제외 (닫힌 4절 링크) |

**알아둘 점:**
- γ = 0.99는 10 Hz에서 약 10 s의 horizon이지만 우리 30 Hz에서는 약 3.3 s다. step 단위 값을 그대로 썼다.
- MuJoCo는 마찰을 큰 값으로 합친다. 그래서 큐브 마찰 1.0–2.0이 pad 마찰을 늘 덮는다.

**Eval** (`CoDe-Rotate-Cube-O-OmniReset-Eval`):
- rotate_cube held-out grasp (`heldout_crown1_final4`)를 쓰고, goal은 +90°다.
- DR 없음, 20 s.
- rotate_cube 종료 (`object_off_grasp`, `object_tilted`)를 쓴다.
- `eval_policy.py`가 같은 양(적분 yaw와 heading의 min)을 측정하고 `goals_per_episode`를 더 기록한다.

## 3. 자산 변경

**Pad dome** (`contact_interface.PadProfile(crown_wid_mm=…)`):
- 길이 방향 1 mm crown과 폭 방향 1 mm crown을 합했다. 볼록하다.
- 두께: 중앙 3 mm, 변 중점 2 mm, 모서리 1 mm.
- 격자는 11×5에 바닥 모서리 4개를 더해 59 vertex다. Newton hull 한도 64 이내다.
- thumb pad에도 적용했다.
- 옛 rotate_cube task는 1D crown 그대로다.

**PIP servo collider** (`assets.builders.write_usd`, 기본 `servo_colliders=("pip",)`):
- 모든 hand USD에 들어간다. 옛 asset을 재현하려면 `CODE_ROTATE_CUBE_SERVO_COLLIDERS=0`을 쓴다.
- root customData에 `code:servo_colliders`로 기록된다.
- 질량과 관성은 바뀌지 않는다 (Newton model로 확인).

## 4. 결정 (2026-11-24, 사용자 승인)

- **접촉 모델** `pad13_condim4_v1` (`code_isaaclab/contact_model.py`). 새 task에만 적용하고 옛 rotate_cube는 그대로 둔다. finalize 전에 builder에 값을 넣고, 만든 model에서 값을 확인했다.

  | 항목 | 값 |
  |---|---|
  | μ | pad 1.3, 큐브와 나머지 손 0.5 |
  | condim, torsional | pad와 큐브 condim 4, torsional 0.003 m |
  | impratio | 1000 (elliptic) |
  | solimp | (0.95, 0.99, 0.001, 0.5, 2) |
  | solref | tc 0.012 s, dampratio 1 (ke 6944, kd 167) |
  | njmax | ×1.6 |

  - 근거: 기존 모델에서 held-out grasp의 pad 관통이 p90 3.6 mm였고(pad 두께 3 mm보다 깊음), impratio 10에서 creep이 있었다. 제안 모델에서는 관통 p90이 0.9 mm이고, 0.04 N·m 비틀림에서 8개 grasp 중 미끄러진 것이 없었다.
  - OmniReset의 마찰 DR은 뺐다. MuJoCo는 큰 계수를 쓰므로 그 범위가 pad 1.3 / 링크 0.5 설정을 덮어쓴다.
  - 질량 DR과 gain DR은 유지했다.
- **reset 수집 마찰:** OmniReset처럼 모든 손과 큐브 shape를 μ 0.3으로 두고 수집한다 (`--friction 0.3`). stable grasp 수용률은 0.3에서 7%, 기본 마찰에서 19%였다.
- **PIP servo collider:** 큐브, 이웃 손가락과 물리 접촉으로 둔다. 새 데이터는 모두 collider를 켠 상태로 만든다.
- **held-out:** `logs/rotate_cube/heldout_omnireset_v1` (`gen_grasps.py`, seed 76000, 1024 grasp, 수용률 0.37).
  - 생성 조건: `CODE_ROTATE_CUBE_CROWN_LAYOUT=dome`, `CODE_ROTATE_CUBE_CONTACT=pad13_condim4_v1`. 자산은 새 task와 같다.
  - 옛 `heldout_crown1_final4`는 그대로 둔다.
  - model_5595는 옛 자산과 접촉 모델로 학습했다. 새 cache에서의 수치는 참고용이다.
- **γ** = 0.997: OmniReset의 10 s horizon을 30 Hz에 맞춘 값이다.
