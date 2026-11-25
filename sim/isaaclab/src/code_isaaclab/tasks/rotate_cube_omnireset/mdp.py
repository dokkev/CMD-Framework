# Copyright (c) 2026, CoDe Hand.
# SPDX-License-Identifier: BSD-3-Clause

"""MDP terms of cube rotation with OmniReset's recipe (Yin et al., ICLR 2026; ``UW-Lab/UWLab`` at 2b40afb,
``source/uwlab_tasks/uwlab_tasks/manager_based/manipulation/omnireset``, cited below as ``omnireset/``). See
``docs/experimental/omnireset_recipe.md`` for the recipe and the mapping.

- reset: :class:`OmniResetStates` (``omnireset/mdp/events.py`` ``MultiResetManager``): each reset picks a reset type
  with the configured probabilities, then a state uniformly from that type's cached dataset, and writes it whole
  (joints, joint targets, cube pose and velocity). In generation mode it samples the type's raw states instead
  (``scripts/rotate_cube/gen_omnireset_resets.py`` settles and filters them);
- goal: :class:`GoalContext` (``omnireset/mdp/rewards.py`` ``ProgressContext``): a moving yaw goal ``k * goal_step``
  ahead of the episode's start, the cube's rest position on the pads; success, distances, and the goal advance;
- rewards (``omnireset/config/.../rl_state_cfg.py`` ``RewardsCfg``): :func:`success_reward`,
  :func:`dense_success_reward`, :func:`fingertip_cube_distance_tanh`, :func:`action_l2_clamped`,
  :func:`action_rate_l2_clamped`, :func:`joint_vel_l2_clamped`, :func:`abnormal_robot_state`;
- observations: the cube's pose and its pose relative to the goal (:func:`cube_pose`, :func:`cube_goal_error`);
- action: :class:`OmniResetFlexionAction`, the rotate_cube relative joint targets with OmniReset's per-reset
  log-uniform gain randomisation on the PIP servos.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from pathlib import Path

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import ManagerTermBase, SceneEntityCfg
from isaaclab.utils import configclass

from ..rotate_cube.rotate_cube_mdp import RelativeFlexionAction, RelativeFlexionActionCfg, _t

OMNIRESET_TYPES = ("ObjectAnywhereHandAnywhere", "ObjectRestingHandNear", "ObjectAnywhereHandGrasped",
                   "ObjectNearGoalHandGrasped")  # fmt: skip
"""OmniReset's four reset types for the hand (``omnireset/config/.../reset_states_cfg.py``): reaching
(``ObjectAnywhereEEAnywhere``), near-object (``ObjectRestingEEGrasped``), stable grasp (``ObjectAnywhereEEGrasped``)
and near-goal (``ObjectPartiallyAssembledEEGrasped``)."""
MIDTURN = "ObjectMidTurnHandWound"
"""v2's mid-turn type (Khandate et al., RSS 2023): states harvested from policy roll-outs with the cube 30-80 deg into
a turn or a servo near its limit, half of them with one finger lifted, the goal 10-60 deg ahead
(``scripts/rotate_cube/harvest_omnireset_midturn.py``, ``gen_omnireset_resets.py --type ObjectMidTurnHandWound``)."""
RESET_TYPES = OMNIRESET_TYPES + (MIDTURN,)
DATASET_KEYS = {"q": 2, "qd": 2, "target": 2, "pos": 2, "quat": 2, "vel": 2, "goal_offset": 1}


def reset_term(env) -> OmniResetStates:
    return env.event_manager.get_term_cfg("reset_grasp").func


def goal_context(env) -> GoalContext:
    return env.reward_manager.get_term_cfg("goal_context").func


# --- action -------------------------------------------------------------------------------------------------------


class OmniResetFlexionAction(RelativeFlexionAction):
    """:class:`~..rotate_cube.rotate_cube_mdp.RelativeFlexionAction` with the PIP servos' scheduled stiffness and
    damping scaled per reset by log-uniform factors in ``gain_log_range`` (OmniReset's
    ``randomize_gripper_actuator_parameters``, ``log_uniform`` (0.5, 2.0); the other servos get the same through the
    ``randomize_actuator_gains`` event)."""

    cfg: OmniResetFlexionActionCfg

    def reset(self, env_ids=None) -> None:
        super().reset(env_ids)
        lo, hi = self.cfg.gain_log_range
        if (lo, hi) == (1.0, 1.0):
            return
        ids = slice(None) if env_ids is None else env_ids
        if not hasattr(self, "_gain_scale"):
            self._gain_scale = torch.ones(self.num_envs, len(self._pip), 2, device=self.device)
        u = torch.rand_like(self._gain_scale[ids])
        self._gain_scale[ids] = torch.exp(math.log(lo) + u * (math.log(hi) - math.log(lo)))


@configclass
class OmniResetFlexionActionCfg(RelativeFlexionActionCfg):
    class_type: type = OmniResetFlexionAction
    gain_log_range: tuple[float, float] = (1.0, 1.0)


# --- reset --------------------------------------------------------------------------------------------------------


def load_reset_dataset(path: str | Path, joint_names: Sequence[str], goal_step: float) -> dict[str, torch.Tensor]:
    """A reset dataset (``gen_omnireset_resets.py``: ``q``, ``qd``, ``target``, ``pos``, ``quat``, ``vel``,
    ``goal_offset``) or a rotate_cube grasp cache (``q``, ``pos``, ``quat``, optional ``target``; at rest, the goal one
    ``goal_step`` ahead), checked, on the CPU."""
    data = torch.load(Path(path), map_location="cpu", weights_only=True)
    if list(data["joint_names"]) != list(joint_names):
        raise ValueError(f"{path}: joints differ from the robot's")
    n = len(data["q"])
    data.setdefault("target", data["q"])
    data.setdefault("qd", torch.zeros_like(data["q"]))
    data.setdefault("vel", torch.zeros(n, 6))
    data.setdefault("goal_offset", torch.full((n,), float(goal_step)))
    out = {k: data[k].to(torch.float32) for k in ("progress",) if k in data}
    for key, ndim in DATASET_KEYS.items():
        v = data[key].to(torch.float32)
        if n == 0 or v.shape[0] != n or v.ndim != ndim or not torch.isfinite(v).all():
            raise ValueError(f"{path}: {key} must hold {n} > 0 finite rows")
        out[key] = v
    return out


class OmniResetStates(ManagerTermBase):
    """Reset event (named ``reset_grasp``, the name the rotate_cube evaluation reads). Parameters:

    - ``datasets``: ``{reset type: path}``; ``probs``: their probabilities (normalised; OmniReset: 0.25 each). A
      reset draws a type per environment (``torch.multinomial``), then a row uniformly in that type's dataset;
    - ``cache`` instead of ``datasets``: one rotate_cube grasp cache (the evaluation's held-out grasps), all at rest
      with the goal ``goal_step`` ahead;
    - ``generate``: a reset type: sample its raw states (:meth:`_generate`) for the generation script;
    - ``actuated``, ``four_bar_tables``, ``goal_step`` [rad], ``rest_pos`` and ``rest_quat`` (the cube's canonical rest
      on the pads, env frame, quaternion ``xyzw``), ``grasp_rows`` (generation: the stable-grasp dataset the
      near-object and near-goal types start from), ``gen`` (generation ranges).

    Exposes what the rotate_cube terms and evaluation read: ``grasp_q`` and ``grasp_target`` (actuated, PIP in
    flexion), ``grasp_pos`` (the cube's reset position, env frame), ``grasp_cache_index`` (row, or -1 when
    generated), ``phase_offset`` (0); and ``task_id`` and ``goal_offset`` for the goal. At each reset it logs, per
    type, the goals reached in the episodes that just ended (``Metrics/<type>/goals_per_episode``; OmniReset logs each
    type's success rate the same way)."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        p = cfg.params
        self.robot, self.obj = env.scene["robot"], env.scene["object"]
        n, dev = env.num_envs, env.device
        self.actuated = list(p["actuated"])
        self._act = [self.robot.joint_names.index(j) for j in self.actuated]
        self.grasp_q = torch.zeros(n, len(self.actuated), device=dev)
        self.grasp_target = torch.zeros_like(self.grasp_q)
        self.grasp_pos = torch.zeros(n, 3, device=dev)
        self.grasp_cache_index = torch.full((n,), -1, dtype=torch.long, device=dev)
        self.phase_offset = torch.zeros(n, dtype=torch.long, device=dev)
        self.task_id = torch.zeros(n, dtype=torch.long, device=dev)
        self.goal_offset = torch.full((n,), float(p["goal_step"]), device=dev)
        self.rest_pos = torch.tensor(p["rest_pos"], device=dev, dtype=torch.float32)
        self.rest_quat = torch.tensor(p["rest_quat"], device=dev, dtype=torch.float32)
        from code_isaaclab.flexion_control import FlexionTables
        from code_isaaclab.loops import FourBarTables

        self._four_bar = FourBarTables(p["four_bar_tables"], self.robot.joint_names, device=dev)
        self._flex = FlexionTables(p["four_bar_tables"], device=dev)
        self.names: list[str] = []
        self.datasets: list[dict[str, torch.Tensor]] = []
        self.generate = p.get("generate")
        if self.generate:
            if self.generate not in RESET_TYPES:
                raise ValueError(f"generate must be one of {RESET_TYPES}")
            self.names = [self.generate]
            self.grasp_rows = None
            if p.get("grasp_rows"):
                rows = load_reset_dataset(p["grasp_rows"], self.robot.joint_names, p["goal_step"])
                self.grasp_rows = {k: v.to(dev) for k, v in rows.items()}
            elif self.generate in ("ObjectRestingHandNear", "ObjectNearGoalHandGrasped", MIDTURN):
                raise ValueError(f"{self.generate} starts from source states: set grasp_rows")
        elif p.get("cache"):
            self.names = ["cache"]
            self.datasets = [load_reset_dataset(p["cache"], self.robot.joint_names, p["goal_step"])]
        else:
            for name, path in p["datasets"].items():
                if name not in RESET_TYPES:
                    raise ValueError(f"unknown reset type {name}")
                if not Path(path).is_file():
                    raise FileNotFoundError(f"No reset dataset at {path}: scripts/rotate_cube/gen_omnireset_resets.py")
                self.names.append(name)
                self.datasets.append(load_reset_dataset(path, self.robot.joint_names, p["goal_step"]))
        self.datasets = [{k: v.to(dev) for k, v in d.items()} for d in self.datasets]
        self.goal_offsets = p.get("goal_offsets") or {}
        """Per type, how a reset sets the goal instead of the dataset's stored offset: ``["fixed", rad]``,
        ``["near", inside, outside]`` (half within +-inside, half inside..outside ahead: OmniReset's
        ``assembly_success_prob`` 0.5), ``["uniform", lo, hi]``; types not listed keep the stored offset (v1)."""
        probs = p.get("probs") or [1.0] * len(self.names)
        if len(probs) != len(self.names):
            raise ValueError("probs must match the reset types")
        self.probs = torch.tensor(probs, device=dev, dtype=torch.float32)
        self.probs /= self.probs.sum()

    # helpers ------------------------------------------------------------------------------------------------------

    def _to_flexion(self, q_act: torch.Tensor) -> torch.Tensor:
        out = q_act.clone()
        for k, name in enumerate(self.actuated):
            if name.endswith("_pip_servo"):
                out[:, k] = self._flex.flexion(name, q_act[:, k])
        return out

    def _from_flexion(self, q_act: torch.Tensor) -> torch.Tensor:
        out = q_act.clone()
        for k, name in enumerate(self.actuated):
            if name.endswith("_pip_servo"):
                out[:, k] = self._flex.servo(name, q_act[:, k])
        return out

    def flexion_limits(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Actuated joints' limits, PIP in flexion (0, 120 deg, the action's range) [rad], ``(num_actuated,)``."""
        lim = _t(self.robot.data.soft_joint_pos_limits)[0, self._act]
        lo, hi = lim[:, 0].clone(), lim[:, 1].clone()
        for k, name in enumerate(self.actuated):
            if name.endswith("_pip_servo"):
                lo[k], hi[k] = 0.0, math.radians(120.0)
        return lo, hi

    def write(self, ids: torch.Tensor, row: dict[str, torch.Tensor]) -> None:
        """Write full states (joints, targets, cube pose in the env frame and velocity) to environments ``ids``."""
        origins = self._env.scene.env_origins[ids]
        self.robot.write_joint_position_to_sim_index(position=row["q"], env_ids=ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=row["qd"], env_ids=ids)
        self.robot.set_joint_position_target_index(target=row["target"], env_ids=ids)
        pose = torch.cat((row["pos"] + origins, row["quat"]), dim=1)
        self.obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
        self.obj.write_root_velocity_to_sim_index(root_velocity=row["vel"], env_ids=ids)
        self.grasp_q[ids] = self._to_flexion(row["q"][:, self._act])
        self.grasp_target[ids] = self._to_flexion(row["target"][:, self._act])
        self.grasp_pos[ids] = row["pos"]
        self.goal_offset[ids] = row["goal_offset"]

    # reset --------------------------------------------------------------------------------------------------------

    def __call__(
        self, env, env_ids, actuated, four_bar_tables, goal_step, rest_pos, rest_quat, gen, datasets=None, probs=None,
        cache=None, generate=None, grasp_rows=None, goal_offsets=None,
    ):  # fmt: skip
        if env_ids is None or isinstance(env_ids, slice):
            ids = torch.arange(env.num_envs, device=env.device)
        else:
            ids = torch.as_tensor(env_ids, device=env.device, dtype=torch.long)
        self._log(env, ids)
        if hasattr(env, "termination_manager") and "non_finite_state" in env.termination_manager.active_terms:
            bad = env.termination_manager.get_term_cfg("non_finite_state").func.last_bad[ids]
            recover_worlds(env, ids[bad])
        self.phase_offset[ids] = 0
        if self.generate:
            self.task_id[ids] = 0
            self.grasp_cache_index[ids] = -1
            self.write(ids, self._generate(len(ids)))
            return
        pick_type = torch.multinomial(self.probs, len(ids), replacement=True)
        self.task_id[ids] = pick_type
        for t, data in enumerate(self.datasets):
            mask = pick_type == t
            if not mask.any():
                continue
            sub = ids[mask]
            rows = torch.randint(0, data["q"].shape[0], (len(sub),), device=env.device)
            self.grasp_cache_index[sub] = rows
            self.write(sub, {k: v[rows] for k, v in data.items() if k in DATASET_KEYS})
            spec = self.goal_offsets.get(self.names[t])
            if spec:
                self.goal_offset[sub] = self._sample_goal(spec, len(sub))

    def _sample_goal(self, spec, k: int) -> torch.Tensor:
        dev = self._env.device
        u = torch.rand(k, device=dev)
        if spec[0] == "fixed":
            return torch.full((k,), float(spec[1]), device=dev)
        if spec[0] == "uniform":
            return spec[1] + (spec[2] - spec[1]) * u
        if spec[0] == "near":
            inside = torch.rand(k, device=dev) < 0.5
            return torch.where(inside, (2 * u - 1) * spec[1], spec[1] + (spec[2] - spec[1]) * u)
        raise ValueError(f"unknown goal offset spec {spec}")

    def _log(self, env, ids: torch.Tensor) -> None:
        if not hasattr(env, "reward_manager") or "goal_context" not in env.reward_manager.active_terms:
            return
        context = goal_context(env)
        ended = ids[context.steps[ids] > 0]
        if len(ended) == 0:
            return
        log = env.extras.setdefault("log", {})
        for t, name in enumerate(self.names):
            sel = ended[self.task_id[ended] == t]
            if len(sel):
                log[f"Metrics/{name}/goals_per_episode"] = float(context.goals_reached[sel].float().mean())

    # generation ---------------------------------------------------------------------------------------------------

    def _generate(self, k: int) -> dict[str, torch.Tensor]:
        """Raw states of the generated type (unsettled; see ``gen_omnireset_resets.py`` for the ranges' sources)."""
        g, dev = self.cfg.params["gen"], self._env.device
        lo, hi = self.flexion_limits()
        names = self.actuated
        default = _t(self.robot.data.default_joint_pos)[0]
        cradle = self._to_flexion(default[self._act][None])[0]

        def uniform(a, b):
            return a + (b - a) * torch.rand(k, *a.shape[-1:], device=dev) if torch.is_tensor(a) else \
                a + (b - a) * torch.rand(k, device=dev)

        def window(centre, half):  # uniform over [centre - half, centre + half] within the limits: no pile-up
            a = torch.maximum(centre - half, lo)
            b = torch.minimum(centre + half, hi)
            return a + (b - a) * torch.rand(k, len(names), device=dev)

        def col(name):
            return names.index(name)

        flex_cols = [col(n) for n in names if n.endswith(("_fe", "_pip_servo"))]
        kind = self.generate
        goal = torch.full((k,), float(self.cfg.params["goal_step"]), device=dev)
        vel = torch.zeros(k, 6, device=dev)
        if kind == "ObjectAnywhereHandAnywhere":
            # the hand anywhere in its joint range, the cube dropped over the pads at a uniform random orientation
            act = lo + (hi - lo) * torch.rand(k, len(names), device=dev)
            target = act.clone()
            pos = self.rest_pos + torch.stack(
                [uniform(-g["drop_xy_m"], g["drop_xy_m"]), uniform(-g["drop_xy_m"], g["drop_xy_m"]),
                 uniform(*g["drop_lift_m"])], dim=1)  # fmt: skip
            quat = math_utils.random_orientation(k, device=dev)
        elif kind == "ObjectAnywhereHandGrasped":
            # a pinch around the cradle with wide per-joint windows (thumb rotation uniform over its window, not piled
            # at its limit), some fingers lifted off (regrasp-like), the fingers closing onto the cube from slightly
            # open; the cube anywhere on the pads, any yaw, a small tilt
            target = window(cradle, torch.tensor([g["window_rad"].get(n.split("_", 1)[1], 0.3) for n in names],
                                                 device=dev))  # fmt: skip
            for f in ("index", "middle", "ring"):
                lift = torch.rand(k, device=dev) < g["lift_prob"]
                c = col(f"{f}_fe")
                lifted = lo[c] + (cradle[c] - math.radians(15.0) - lo[c]) * torch.rand(k, device=dev)
                target[:, c] = torch.where(lift, lifted, target[:, c])
            act = target.clone()
            act[:, flex_cols] -= math.radians(g["close_deg"]) * torch.rand(k, len(flex_cols), device=dev)
            act = torch.maximum(torch.minimum(act, hi), lo)
            pos = self.rest_pos + torch.stack(
                [uniform(-g["grasp_xy_m"], g["grasp_xy_m"]), uniform(-g["grasp_xy_m"], g["grasp_xy_m"]),
                 uniform(*g["grasp_lift_m"])], dim=1)  # fmt: skip
            quat = self._yaw_tilt(k, g["grasp_tilt_rad"])
        else:
            if self.grasp_rows is None:
                raise ValueError(f"{kind} needs grasp_rows")
            rows = torch.randint(0, self.grasp_rows["q"].shape[0], (k,), device=dev)
            src = {key: v[rows] for key, v in self.grasp_rows.items()}
            act = self._to_flexion(src["q"][:, self._act])
            target = self._to_flexion(src["target"][:, self._act])
            pos, quat = src["pos"], src["quat"]
            self.gen_progress = src.get("progress")
            if kind == MIDTURN:
                # the harvested state, jittered by +-1 deg; half with one digit (a finger, or the thumb) opened by
                # 0-20 deg; the goal 10-60 deg ahead
                jitter = math.radians(1.0) * (2 * torch.rand(k, len(names), device=dev) - 1)
                act, target = act + jitter, target + jitter
                digits = [[col(f"{f}_fe"), col(f"{f}_pip_servo")] for f in ("index", "middle", "ring")]
                digits.append([col("thumb_mcp"), col("thumb_ip")])
                which = torch.randint(0, len(digits), (k,), device=dev)
                lift = (torch.rand(k, device=dev) < 0.5)[:, None]
                sign = torch.tensor(g["open_sign"], device=dev)
                for d, cols in enumerate(digits):
                    mask = torch.zeros(len(names), device=dev)
                    mask[cols] = 1.0
                    amount = math.radians(g["midturn_lift_deg"]) * torch.rand(k, 1, device=dev)
                    opened = lift & (which == d)[:, None]
                    act = act + opened * amount * sign * mask
                    target = target + opened * amount * sign * mask
                self.gen_lifted = lift[:, 0] & True
                goal = uniform(*g["midturn_goal_rad"])
            elif kind == "ObjectRestingHandNear":
                # the hand opened off the grasp by a few degrees (OmniReset: +-2 cm, +-pi/16 off a grasp pose)
                open_ = math.radians(g["near_open_deg"]) * torch.rand(k, len(names), device=dev)
                sign = torch.tensor(g["open_sign"], device=dev)
                act = act + sign * open_
                target = act.clone()
            else:  # ObjectNearGoalHandGrasped: a small perturbation (OmniReset: +-1 cm, +-pi/32) and a goal near
                act = act + math.radians(g["goal_jitter_deg"]) * (2 * torch.rand(k, len(names), device=dev) - 1)
                target = target + (act - self._to_flexion(src["q"][:, self._act]))
                inside = torch.rand(k, device=dev) < 0.5  # OmniReset's assembly_success_prob 0.5
                near = g["goal_inside_rad"]
                goal = torch.where(inside, uniform(-near, near), uniform(near, g["goal_outside_rad"]))
            act = torch.maximum(torch.minimum(act, hi), lo)
            target = torch.maximum(torch.minimum(target, hi), lo)
        q = default.expand(k, -1).clone()
        q[:, self._act] = self._from_flexion(act)
        q = self._four_bar.close_from_servo(q)
        full_target = q.clone()
        full_target[:, self._act] = self._from_flexion(target)
        full_target = self._four_bar.close_from_servo(full_target)
        return {"q": q, "qd": torch.zeros_like(q), "target": full_target, "pos": pos, "quat": quat, "vel": vel,
                "goal_offset": goal}  # fmt: skip

    def _yaw_tilt(self, k: int, tilt_rad: float) -> torch.Tensor:
        """The rest orientation turned about world z by a uniform yaw and tilted about a random horizontal axis by up
        to ``tilt_rad``."""
        dev = self._env.device
        z = torch.zeros(k, 3, device=dev)
        z[:, 2] = 1.0
        yaw = math_utils.quat_from_angle_axis((2 * torch.rand(k, device=dev) - 1) * math.pi, z)
        phi = 2 * math.pi * torch.rand(k, device=dev)
        axis = torch.stack((phi.cos(), phi.sin(), torch.zeros_like(phi)), dim=1)
        tilt = math_utils.quat_from_angle_axis(tilt_rad * torch.rand(k, device=dev), axis)
        return math_utils.quat_mul(tilt, math_utils.quat_mul(yaw, self.rest_quat.expand(k, 4)))


# --- goal ---------------------------------------------------------------------------------------------------------


def cube_tilt(quat_xyzw: torch.Tensor) -> torch.Tensor:
    """The angle between world z and the cube's body axis nearest to it [rad] (0: lying on a face; symmetric)."""
    r = math_utils.matrix_from_quat(quat_xyzw)  # Isaac Lab 3: xyzw throughout
    return r[:, 2, :].abs().max(dim=1).values.clamp(max=1.0).acos()


class GoalContext(ManagerTermBase):
    """Reward term returning 0 that keeps the goal state (OmniReset's ``ProgressContext``; it runs first).

    The goal: the cube on its rest position on the pads, lying on a face, turned ``goal`` about world z from the
    episode's start. ``goal`` starts at the reset's ``goal_offset`` (``goal_step`` ahead, or near for near-goal
    states); progress is the integrated world-z rotation of the cube (the evaluation's ``yaw``). Each step:
    ``yaw_error = goal - progress``, ``orientation_distance = |yaw_error| + tilt`` (OmniReset: the summed roll and
    pitch errors), ``distance = |p - p_rest|``; success while ``distance < success_distance_m`` and
    ``orientation_distance < success_orientation_rad`` (OmniReset's metadata thresholds). After ``hold_steps``
    consecutive successes the goal advances by ``goal_step`` and ``goals_reached`` counts it."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        p = cfg.params
        self.obj = env.scene["object"]
        n, dev = env.num_envs, env.device
        self.rest_pos = torch.tensor(p["rest_pos"], device=dev, dtype=torch.float32)
        self.prev_quat = _t(self.obj.data.root_quat_w).clone()
        z = torch.zeros(n, device=dev)
        self.progress, self.goal, self.yaw_error, self.tilt, self.distance = (z.clone() for _ in range(5))
        self.orientation_distance = z.clone()
        self.success = torch.zeros(n, dtype=torch.bool, device=dev)
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)
        self.goals_reached = torch.zeros(n, dtype=torch.long, device=dev)
        self.steps = torch.zeros(n, dtype=torch.long, device=dev)
        self.advanced = torch.zeros(n, dtype=torch.bool, device=dev)

    def reset(self, env_ids: Sequence[int] | None = None):
        ids = slice(None) if env_ids is None else env_ids
        log = self._env.extras.setdefault("log", {})
        if bool((self.steps[ids] > 0).any()):
            done = self.steps[ids] > 0
            log["Metrics/goals_per_episode"] = float(self.goals_reached[ids][done].float().mean())
        self.prev_quat[ids] = _t(self.obj.data.root_quat_w)[ids]
        self.progress[ids] = 0.0
        self.goal[ids] = reset_term(self._env).goal_offset[ids]
        self.hold[ids] = 0
        self.goals_reached[ids] = 0
        self.steps[ids] = 0
        self._measure()

    def _measure(self) -> None:
        quat = _t(self.obj.data.root_quat_w)
        pos = _t(self.obj.data.root_pos_w) - self._env.scene.env_origins
        self.yaw_error[:] = self.goal - self.progress
        self.tilt[:] = cube_tilt(quat)
        self.orientation_distance[:] = self.yaw_error.abs() + self.tilt
        self.distance[:] = (pos - self.rest_pos).norm(dim=1)

    def __call__(self, env, rest_pos, goal_step: float, success_distance_m: float, success_orientation_rad: float,
                 hold_steps: int) -> torch.Tensor:  # fmt: skip
        quat = _t(self.obj.data.root_quat_w)
        dq = math_utils.axis_angle_from_quat(math_utils.quat_mul(quat, math_utils.quat_conjugate(self.prev_quat)))
        self.prev_quat[:] = quat
        self.progress += dq[:, 2]
        self.steps += 1
        self._measure()
        self.success[:] = (self.distance < success_distance_m) & (self.orientation_distance < success_orientation_rad)
        self.hold[:] = torch.where(self.success, self.hold + 1, 0)
        advance = self.hold >= hold_steps
        self.advanced[:] = advance
        self.goal += torch.where(advance, goal_step, 0.0)
        self.goals_reached += advance.long()
        self.hold[advance] = 0
        return torch.zeros(env.num_envs, device=env.device)


# --- rewards (omnireset/mdp/rewards.py) ----------------------------------------------------------------------------


def success_reward(env) -> torch.Tensor:
    """1 while the cube is at the goal (OmniReset's ``success_reward``)."""
    return goal_context(env).success.float()


def goal_advance_bonus(env) -> torch.Tensor:
    """``1 / step_dt`` on the step a goal is reached and the goal advances, else 0: with weight ``w`` the reward manager
    pays ``w`` once per goal (it scales every term by the step duration). Dactyl's goal bonus (+5, Andrychowicz et al.
    2018, section 4.3) and AnyRotate's (10, Yang et al. 2024, section 3.1); OmniReset has none because its success
    persists while the object stays assembled, where a chained goal pays success only for its hold."""
    return goal_context(env).advanced.float() / env.step_dt


def dense_success_reward(env, std: float) -> torch.Tensor:
    """``mean(exp(-orientation_distance / std), exp(-distance / std))`` (OmniReset's ``dense_success_reward``)."""
    c = goal_context(env)
    return _finite(0.5 * (torch.exp(-c.orientation_distance / std) + torch.exp(-c.distance / std)))


def fingertip_cube_distance_tanh(env, std: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """``1 - tanh(d / std)``, ``d`` the mean distance from the fingertip links (``asset_cfg.body_ids``) to the cube's
    centre (OmniReset's ``ee_asset_distance_tanh`` from the gripper's grasp point to the object)."""
    robot, obj = env.scene[asset_cfg.name], env.scene["object"]
    tips = _t(robot.data.body_link_pos_w)[:, asset_cfg.body_ids]
    d = (tips - _t(obj.data.root_pos_w)[:, None]).norm(dim=-1).mean(dim=1)
    return _finite(1.0 - torch.tanh(d / std))


def action_l2_clamped(env) -> torch.Tensor:
    return _finite(torch.clamp(torch.sum(torch.square(env.action_manager.action), dim=1), 0, 1e4))


def action_rate_l2_clamped(env) -> torch.Tensor:
    return _finite(torch.clamp(
        torch.sum(torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1), 0, 1e4
    ))


def joint_vel_l2_clamped(env, asset_cfg: SceneEntityCfg, four_bar_tables: dict) -> torch.Tensor:
    """``sum qd^2`` over the actuated joints, the PIP as flexion rate (OmniReset: the arm joints)."""
    from code_isaaclab.flexion_control import flexion_joint_vel

    return _finite(torch.clamp(flexion_joint_vel(env, asset_cfg, four_bar_tables).square().sum(dim=1), 0, 1e4))


def joint_speed_abnormal(env, factor: float = 2.0) -> torch.Tensor:
    """Any joint faster than ``factor`` times its velocity limit (``uwlab/envs/mdp/terminations.py``
    ``abnormal_robot_state``: 2). PhysX enforces the limit, so there 2x means a diverged simulation; MuJoCo-Warp
    ignores the implicit servo model's velocity limit, so here 2x is reached by ordinary fast moves and a larger factor
    keeps the test to divergence."""
    robot = env.scene["robot"]
    return (_t(robot.data.joint_vel).abs() > factor * _t(robot.data.joint_vel_limits)).any(dim=1)


def _world_rows(n: int):
    """The MuJoCo-Warp data arrays with one row per world, as torch views, with their names."""
    import warp as wp

    from isaaclab_newton.physics import NewtonManager

    data = NewtonManager._solver.mjw_data
    for name in sorted(vars(data)):
        arr = getattr(data, name, None)
        if isinstance(arr, wp.array) and arr.ndim >= 1 and arr.shape[0] == n:
            try:
                t = wp.to_torch(arr)
            except Exception:  # noqa: BLE001 (types without a torch view)
                continue
            if torch.is_floating_point(t):
                yield name, t


def _newton_state_arrays():
    import warp as wp

    from isaaclab_newton.physics import NewtonManager

    backend = NewtonManager.backend
    for state_name in ("state_0", "state_1"):
        state = getattr(backend, state_name, None)
        if state is None:
            continue
        for name in ("joint_q", "joint_qd", "body_q", "body_qd", "body_f", "joint_f"):
            arr = getattr(state, name, None)
            if isinstance(arr, wp.array) and arr.shape[0] > 0:
                yield f"{state_name}.{name}", wp.to_torch(arr)


def recover_worlds(env, ids: torch.Tensor) -> None:
    """Make diverged worlds finite again before their reset state is written: zero the non-finite entries of their
    MuJoCo-Warp data rows and of Newton's state arrays, then reset the solver for those worlds with the joint state
    (``SolverMuJoCo.reset``, mask-aware, ``JOINT_Q | JOINT_QD``: model defaults pushed into ``qpos``/``qvel`` at once,
    which matters with ``update_data_interval`` 2). Isaac Lab's own reset clears only the warm-start and applied inputs;
    a world was seen to stay non-finite through every later reset (v2c segment 1). The reset term then writes the
    episode's state as usual."""
    if len(ids) == 0:
        return
    import newton
    import warp as wp

    from isaaclab_newton.physics import NewtonManager

    for _, t in _world_rows(env.num_envs):
        t[ids] = torch.nan_to_num(t[ids], nan=0.0, posinf=0.0, neginf=0.0)
    for _, t in _newton_state_arrays():
        torch.nan_to_num_(t, nan=0.0, posinf=0.0, neginf=0.0)
    mask = torch.zeros(NewtonManager.backend.model.world_count + 1, dtype=torch.bool, device=env.device)
    mask[ids] = True
    NewtonManager._solver.reset(NewtonManager.backend.state_0, world_mask=wp.from_torch(mask),
                                flags=int(newton.StateFlags.JOINT_Q) | int(newton.StateFlags.JOINT_QD))


class NonFiniteState(ManagerTermBase):
    """Termination: a non-finite joint or cube state (a diverged world; NaN compares false, so the speed test misses
    it). Logs, since the start of the run, ``Metrics/non_finite_per_10k_env_steps`` (every flagged step),
    ``Metrics/non_finite_onsets_per_10k_env_steps`` (a world flagged after a finite step) and
    ``Metrics/non_finite_stuck_steps`` (flagged again on the step after its reset: recovery failed). On an onset it
    appends which MuJoCo-Warp and Newton arrays are non-finite for that world to ``nan_onsets.jsonl`` in
    ``CODE_ROTATE_CUBE_LOG_DIR`` (or prints it). The reset term calls :func:`recover_worlds` for flagged worlds."""

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.events = 0
        self.onsets = 0
        self.stuck = 0
        self.env_steps = 0
        self.last_bad = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def __call__(self, env) -> torch.Tensor:
        robot, obj = env.scene["robot"], env.scene["object"]
        states = (_t(robot.data.joint_pos), _t(robot.data.joint_vel), _t(obj.data.root_pose_w),
                  _t(obj.data.root_vel_w))  # fmt: skip
        bad = ~torch.stack([torch.isfinite(x).all(dim=1) for x in states], dim=1).all(dim=1)
        onset = bad & ~self.last_bad
        self.events += int(bad.sum())
        self.onsets += int(onset.sum())
        self.stuck += int((bad & self.last_bad).sum())
        self.env_steps += env.num_envs
        if onset.any():
            self._dump(env, onset.nonzero().flatten())
        self.last_bad = bad.clone()
        log = env.extras.setdefault("log", {})
        log["Metrics/non_finite_per_10k_env_steps"] = 1e4 * self.events / self.env_steps
        log["Metrics/non_finite_onsets_per_10k_env_steps"] = 1e4 * self.onsets / self.env_steps
        log["Metrics/non_finite_stuck_steps"] = float(self.stuck)
        return bad

    def _dump(self, env, ids: torch.Tensor) -> None:
        import json
        import os

        rows = {name: [int(e) for e in ids.tolist() if not torch.isfinite(t[e]).all()]
                for name, t in _world_rows(env.num_envs)}  # fmt: skip
        record = {
            "env_steps": self.env_steps, "worlds": ids.tolist(),
            "episode_step": env.episode_length_buf[ids].tolist(),
            "reset_type": [reset_term(env).names[int(k)] for k in reset_term(env).task_id[ids].tolist()],
            "mjw_data_nonfinite": {k: v for k, v in rows.items() if v},
            "newton_state_nonfinite_entries": {
                name: int((~torch.isfinite(t)).sum())
                for name, t in _newton_state_arrays()
                if not torch.isfinite(t).all()
            },
        }  # fmt: skip
        log_dir = os.environ.get("CODE_ROTATE_CUBE_LOG_DIR")
        line = json.dumps(record)
        if log_dir:
            with open(os.path.join(log_dir, "nan_onsets.jsonl"), "a") as f:
                f.write(line + "\n")
        print(f"non-finite onset: {line}", flush=True)


def abnormal_robot_state(env) -> torch.Tensor:
    """The abnormal-state penalty's condition: a runaway joint or a non-finite state (both end the episode)."""
    tm = env.termination_manager
    return tm.get_term("abnormal_robot") | tm.get_term("non_finite_state")


def _finite(x: torch.Tensor) -> torch.Tensor:
    """Zero for a diverged world's terms (it is terminated as abnormal in the same step)."""
    return torch.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0)


# --- terminations -------------------------------------------------------------------------------------------------


def cube_out_of_reach(env, rest_pos, max_distance: float) -> torch.Tensor:
    """The cube more than ``max_distance`` from its rest position on the pads: off the hand, beyond any regrasp (the
    hand cannot reach it, where OmniReset's arm can always reach the table)."""
    pos = _t(env.scene["object"].data.root_pos_w) - env.scene.env_origins
    return (pos - torch.tensor(rest_pos, device=env.device)).norm(dim=1) > max_distance


# --- observations -------------------------------------------------------------------------------------------------


def cube_pose(env, rest_pos) -> torch.Tensor:
    """The cube's position from its rest position and its orientation as an axis-angle vector, env frame (the hand's
    base is fixed in it; OmniReset's object pose in the wrist frame), ``(num_envs, 6)``."""
    obj = env.scene["object"]
    pos = _t(obj.data.root_pos_w) - env.scene.env_origins - torch.tensor(rest_pos, device=env.device)
    rot = math_utils.axis_angle_from_quat(_t(obj.data.root_quat_w))
    return _finite(torch.cat((pos, rot), dim=1))


def cube_goal_error(env, rest_pos) -> torch.Tensor:
    """The cube relative to the goal (OmniReset's object pose in the goal frame): position error [m] (3), the tilt as
    the horizontal part of the up-most body axis (2), and the yaw error to the goal [rad] (1, clipped to +-pi)."""
    if not hasattr(env, "reward_manager"):  # observation shapes are inferred before the reward manager exists
        return torch.zeros(env.num_envs, 6, device=env.device)
    c = goal_context(env)
    obj = env.scene["object"]
    pos = _t(obj.data.root_pos_w) - env.scene.env_origins - c.rest_pos
    r = math_utils.matrix_from_quat(_t(obj.data.root_quat_w))
    k = r[:, 2, :].abs().argmax(dim=1)
    up = r.gather(2, k[:, None, None].expand(-1, 3, 1))[..., 0]  # the body axis nearest world z
    up = torch.where(up[:, 2:3] < 0, -up, up)
    return _finite(torch.cat((pos, up[:, :2], c.yaw_error.clamp(-math.pi, math.pi)[:, None]), dim=1))


def time_left(env) -> torch.Tensor:
    return (1.0 - env.episode_length_buf / env.max_episode_length)[:, None]


__all__ = [
    "MIDTURN",
    "OMNIRESET_TYPES",
    "RESET_TYPES",
    "GoalContext",
    "OmniResetFlexionAction",
    "OmniResetFlexionActionCfg",
    "OmniResetStates",
    "NonFiniteState",
    "abnormal_robot_state",
    "joint_speed_abnormal",
    "action_l2_clamped",
    "action_rate_l2_clamped",
    "cube_goal_error",
    "cube_out_of_reach",
    "cube_pose",
    "cube_tilt",
    "dense_success_reward",
    "fingertip_cube_distance_tanh",
    "goal_advance_bonus",
    "goal_context",
    "joint_vel_l2_clamped",
    "load_reset_dataset",
    "recover_worlds",
    "reset_term",
    "success_reward",
    "time_left",
]
