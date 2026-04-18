"""Scripted expert policy for the assemble-brick task."""

from collections.abc import Coroutine, Sequence

import isaaclab.utils.math as math_utils
import torch
from isaaclab.envs.mdp.actions.task_space_actions import (
    DifferentialInverseKinematicsAction,
)

from bricksim.core import compute_connection_transform
from bricksim.utils.debug_draw import DebugDraw, acquire_debug_draw

BRICK_UNIT_LENGTH = 0.0080
PLATE_UNIT_HEIGHT = 0.0032
TCP_TO_FINGER_TIP = 0.0090

PRE_GRASP_OFFSET = 0.10
LIFT_HEIGHT = 0.10
ASSEMBLY_HEIGHT_OFFSET = 0.04
PRESS_DEPTH = 0.003
PRESS_HOLD_S = 1.0
RETREAT_HEIGHT = 0.05

PRE_GRASP_POS_TOL = 0.03
PRE_GRASP_ROT_TOL = 0.75
GRASP_POS_TOL = 0.004
GRASP_ROT_TOL = 0.50
LIFT_POS_TOL = 0.03
LIFT_ROT_TOL = 0.75
PRE_ASSEMBLY_POS_TOL = 0.01
PRE_ASSEMBLY_ROT_TOL = 0.25
PRESS_POS_TOL = 0.006
PRESS_ROT_TOL = 0.50
RETREAT_POS_TOL = 0.03
RETREAT_ROT_TOL = 0.75

PRE_GRASP_TIMEOUT_S = 10.0
GRASP_TIMEOUT_S = 10.0
LIFT_TIMEOUT_S = 10.0
PRE_ASSEMBLY_TIMEOUT_S = 10.0
PRESS_TIMEOUT_S = 1.5
RETREAT_TIMEOUT_S = 10.0
GRIPPER_TIMEOUT_S = 3.0

GRIPPER_POS_TOL = 0.003
GRIPPER_VEL_TOL = 0.0001
GRIPPER_STALL_TIME_S = 1.0
GRIPPER_GRASP_WIDTH_TOL = 0.002
GRIPPER_WIDTH_DELTA_TOL = 0.0005
GRIPPER_WIDTH_STABLE_TIME_S = 0.2
OPEN_GRIPPER_EXTRA_WIDTH = 0.015
MAX_FRANKA_GRIPPER_WIDTH = 0.08

_STEP_REQUEST = object()
ExpertCoroutine = Coroutine[object, None, None]


class _NextStepAwaitable:
    def __await__(self):
        yield _STEP_REQUEST
        return None


class AssembleBrickExpert:
    """Coroutine-driven scripted controller for assembling one LEGO brick."""

    def __init__(
        self,
        env,
        *,
        stud_if: int,
        hole_if: int,
        target_offset: tuple[int, int],
        target_yaw: int,
        enable_debug_draw: bool = True,
    ):
        """Initialize the expert for a fixed target connection."""
        self.env = env
        self._stud_if = stud_if
        self._hole_if = hole_if
        self._target_offset = target_offset
        self._target_yaw = target_yaw

        self._robot = self.env.scene["robot"]
        self._brick = self.env.scene["lego_brick"]
        self._baseplate = self.env.scene["lego_baseplate"]
        arm_action_term = self.env.action_manager.get_term("arm_action")
        assert isinstance(arm_action_term, DifferentialInverseKinematicsAction)
        self._arm_action_term = arm_action_term
        self._action_scale = float(self.env.cfg.actions.arm_action.scale)
        self._step_dt = float(self.env.step_dt)
        self._dtype = self.env.scene.env_origins.dtype
        self._num_envs = self.env.num_envs
        self._action_dim = self.env.action_manager.total_action_dim
        self._gripper_joint_ids, _ = self._robot.find_joints(
            self.env.cfg.gripper_joint_names
        )
        self._brick_paths = self._brick.root_physx_view.prim_paths
        self._baseplate_paths = self._baseplate.root_physx_view.prim_paths

        self._step_actions = torch.zeros(
            (self._num_envs, self._action_dim),
            device=self.env.device,
            dtype=self._dtype,
        )
        self._captured_hole_to_eef_pos = torch.zeros(
            (self._num_envs, 3), device=self.env.device, dtype=self._dtype
        )
        self._captured_hole_to_eef_quat = torch.zeros(
            (self._num_envs, 4), device=self.env.device, dtype=self._dtype
        )
        self._captured_valid = torch.zeros(
            (self._num_envs,), device=self.env.device, dtype=torch.bool
        )
        self._coroutines: list[ExpertCoroutine | None] = [None] * self._num_envs
        self._debug_draw: DebugDraw | None = (
            acquire_debug_draw() if enable_debug_draw else None
        )
        self._debug_draw_env_id = 0
        self._debug_print_env_id = 0

        self._configure_grasp_geometry()

    def reset(self, env_ids: Sequence[int]) -> None:
        """Reset per-environment expert coroutine and captured-pose state."""
        for env_id in env_ids:
            self._coroutines[int(env_id)] = None
        self._captured_hole_to_eef_pos[env_ids] = 0.0
        self._captured_hole_to_eef_quat[env_ids] = 0.0
        self._captured_valid[env_ids] = False

    def compute_actions(self) -> torch.Tensor:
        """Advance each expert coroutine and return the current action batch.

        Returns:
            Action tensor with shape ``(num_envs, action_dim)``.
        """
        self._step_actions.zero_()
        for env_id in range(self._num_envs):
            self._advance_coroutine(env_id)
        return self._step_actions

    def _configure_grasp_geometry(self) -> None:
        length, width, height = self.env.cfg.scene.lego_brick.spawn.dimensions
        grasp_depth = 0.002 if length == 1 or width == 1 else 0.001

        self._grasp_width = float(min(length, width) * BRICK_UNIT_LENGTH)
        self._grasp_tcp_pos_h = torch.tensor(
            [0.0, 0.0, height * PLATE_UNIT_HEIGHT + TCP_TO_FINGER_TIP - grasp_depth],
            device=self.env.device,
            dtype=self._dtype,
        )

        if width <= length:
            grasp_rot_h = torch.tensor(
                [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
                device=self.env.device,
                dtype=self._dtype,
            )
        else:
            grasp_rot_h = torch.tensor(
                [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
                device=self.env.device,
                dtype=self._dtype,
            )
        flip_about_tool_z = torch.tensor(
            [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
            device=self.env.device,
            dtype=self._dtype,
        )
        flipped_grasp_rot_h = grasp_rot_h @ flip_about_tool_z
        self._grasp_tcp_quat_h_nominal = math_utils.quat_from_matrix(
            grasp_rot_h.unsqueeze(0)
        )[0]
        self._grasp_tcp_quat_h_flipped = math_utils.quat_from_matrix(
            flipped_grasp_rot_h.unsqueeze(0)
        )[0]
        self._unit_z = torch.tensor(
            [0.0, 0.0, 1.0], device=self.env.device, dtype=self._dtype
        )

    def _debug_log(self, env_id: int, message: str) -> None:
        if env_id == self._debug_print_env_id:
            print(message)

    def _advance_coroutine(self, env_id: int) -> None:
        coroutine = self._coroutines[env_id]
        if coroutine is None:
            coroutine = self._run_policy(env_id)
            self._coroutines[env_id] = coroutine

        while True:
            try:
                yielded = coroutine.send(None)
            except StopIteration:
                coroutine = self._idle_forever(env_id)
                self._coroutines[env_id] = coroutine
                continue

            if yielded is _STEP_REQUEST:
                return
            raise RuntimeError(
                f"Unexpected awaitable yielded by assemble-brick expert: {yielded!r}"
            )

    async def _run_policy(self, env_id: int) -> None:
        grasp_success = await self._grasp_lego_part(env_id)
        if grasp_success:
            await self._assemble_lego_part(env_id)
        await self._idle_forever(env_id)

    async def _idle_forever(self, env_id: int) -> None:
        while True:
            self._set_idle_action(env_id)
            await _NextStepAwaitable()

    async def _grasp_lego_part(self, env_id: int) -> bool:
        self._debug_log(
            env_id, f"--- Attempting to grasp brick: {self._brick_paths[env_id]} ---"
        )
        brick_pos_w = self._brick.data.root_pos_w[env_id].clone()
        brick_quat_w = self._brick.data.root_quat_w[env_id].clone()
        grasp_pos_w, grasp_quat_w = self._select_grasp_pose(
            env_id, brick_pos_w, brick_quat_w
        )
        approach_vector_w = math_utils.quat_apply(
            brick_quat_w.unsqueeze(0), self._unit_z.unsqueeze(0)
        )[0]

        pre_grasp_pos_w = grasp_pos_w + PRE_GRASP_OFFSET * approach_vector_w
        post_grasp_pos_w = pre_grasp_pos_w.clone()
        post_grasp_pos_w[2] += LIFT_HEIGHT

        open_width = min(
            self._grasp_width + OPEN_GRIPPER_EXTRA_WIDTH, MAX_FRANKA_GRIPPER_WIDTH
        )
        self._debug_log(
            env_id,
            f"-> Opening gripper to width: {open_width:.3f}m "
            f"(Grasp width: {self._grasp_width:.3f}m)",
        )
        if not await self._set_gripper(env_id, target_width=open_width):
            return False

        self._debug_log(env_id, "-> Moving to pre-grasp pose.")
        if not await self._move_ee_to(
            env_id,
            pre_grasp_pos_w,
            grasp_quat_w,
            pos_tol=PRE_GRASP_POS_TOL,
            rot_tol=PRE_GRASP_ROT_TOL,
            vel_tol=None,
            timeout=PRE_GRASP_TIMEOUT_S,
            settle_time=0.0,
            close_gripper=False,
        ):
            return False

        self._debug_log(env_id, "-> Moving to grasp pose.")
        if not await self._move_ee_to(
            env_id,
            grasp_pos_w,
            grasp_quat_w,
            pos_tol=GRASP_POS_TOL,
            rot_tol=GRASP_ROT_TOL,
            vel_tol=0.10,
            timeout=GRASP_TIMEOUT_S,
            settle_time=0.0,
            close_gripper=False,
        ):
            return False

        self._debug_log(env_id, "-> Closing gripper.")
        if not await self._set_gripper(env_id, target_width=0.0):
            self._debug_log(
                env_id, "Warning: Gripper closing sequence reported timeout."
            )

        self._debug_log(env_id, "-> Lifting brick.")
        success = await self._move_ee_to(
            env_id,
            post_grasp_pos_w,
            grasp_quat_w,
            pos_tol=LIFT_POS_TOL,
            rot_tol=LIFT_ROT_TOL,
            vel_tol=None,
            timeout=LIFT_TIMEOUT_S,
            settle_time=0.0,
            close_gripper=True,
        )
        self._debug_log(env_id, "--- Grasp sequence finished. ---")
        return success

    def _select_grasp_pose(
        self,
        env_id: int,
        brick_pos_w: torch.Tensor,
        brick_quat_w: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        grasp_pos_w, nominal_grasp_quat_w = self._combine_pose(
            brick_pos_w,
            brick_quat_w,
            self._grasp_tcp_pos_h,
            self._grasp_tcp_quat_h_nominal,
        )
        _, flipped_grasp_quat_w = self._combine_pose(
            brick_pos_w,
            brick_quat_w,
            self._grasp_tcp_pos_h,
            self._grasp_tcp_quat_h_flipped,
        )

        _, control_quat_w = self._get_control_frame_pose_w(env_id)
        nominal_ang_err = self._measure_orientation_error(
            control_quat_w, nominal_grasp_quat_w
        )
        flipped_ang_err = self._measure_orientation_error(
            control_quat_w, flipped_grasp_quat_w
        )

        if nominal_ang_err <= flipped_ang_err:
            selected_name = "nominal"
            selected_grasp_quat_w = nominal_grasp_quat_w
        else:
            selected_name = "flipped_180deg"
            selected_grasp_quat_w = flipped_grasp_quat_w

        self._debug_log(
            env_id,
            f"-> Selected grasp orientation: {selected_name} "
            f"(nominal_ang_err={nominal_ang_err:.4f}, "
            f"flipped_ang_err={flipped_ang_err:.4f})",
        )
        return grasp_pos_w, selected_grasp_quat_w

    async def _assemble_lego_part(self, env_id: int) -> bool:
        self._debug_log(
            env_id,
            f"--- Attempting to assemble {self._brick_paths[env_id]} "
            f"onto {self._baseplate_paths[env_id]} ---",
        )
        self._debug_log(
            env_id, f"-> Offset: {self._target_offset}, Yaw Index: {self._target_yaw}"
        )
        connection_quat, connection_pos = compute_connection_transform(
            stud_path=self._baseplate_paths[env_id],
            stud_if=self._stud_if,
            hole_path=self._brick_paths[env_id],
            hole_if=self._hole_if,
            offset=self._target_offset,
            yaw=self._target_yaw,
        )
        t_s_h_pos = torch.tensor(
            connection_pos, device=self.env.device, dtype=self._dtype
        )
        t_s_h_quat = torch.tensor(
            connection_quat, device=self.env.device, dtype=self._dtype
        )

        baseplate_pos_w = self._baseplate.data.root_pos_w[env_id].clone()
        baseplate_quat_w = self._baseplate.data.root_quat_w[env_id].clone()
        t_w_h_pos, t_w_h_quat = self._combine_pose(
            baseplate_pos_w, baseplate_quat_w, t_s_h_pos, t_s_h_quat
        )

        self._debug_log(env_id, "-> Calculating actual grasp transform T_B_G.")
        t_h_g_pos, t_h_g_quat = self._measure_hole_to_eef(env_id)
        self._captured_hole_to_eef_pos[env_id] = t_h_g_pos
        self._captured_hole_to_eef_quat[env_id] = t_h_g_quat
        self._captured_valid[env_id] = True

        target_pos_w, target_quat_w = self._combine_pose(
            t_w_h_pos, t_w_h_quat, t_h_g_pos, t_h_g_quat
        )
        approach_vector_w = math_utils.quat_apply(
            baseplate_quat_w.unsqueeze(0), self._unit_z.unsqueeze(0)
        )[0]

        pre_assembly_pos_w = target_pos_w + ASSEMBLY_HEIGHT_OFFSET * approach_vector_w
        press_pos_w = target_pos_w - PRESS_DEPTH * approach_vector_w
        retreat_pos_w = pre_assembly_pos_w.clone()
        retreat_pos_w[2] += RETREAT_HEIGHT

        self._debug_log(env_id, "-> Moving to pre-assembly pose.")
        success = await self._move_ee_to(
            env_id,
            pre_assembly_pos_w,
            target_quat_w,
            pos_tol=PRE_ASSEMBLY_POS_TOL,
            rot_tol=PRE_ASSEMBLY_ROT_TOL,
            vel_tol=None,
            timeout=PRE_ASSEMBLY_TIMEOUT_S,
            settle_time=0.0,
            close_gripper=True,
        )
        if not success:
            self._debug_log(env_id, "Failed to reach pre-assembly pose.")

        self._debug_log(
            env_id,
            f"-> Moving to assembly pose and pressing "
            f"(depth={PRESS_DEPTH * 1000:.1f}mm, "
            f"duration={PRESS_HOLD_S:.1f}s).",
        )
        success = await self._move_ee_to(
            env_id,
            press_pos_w,
            target_quat_w,
            pos_tol=PRESS_POS_TOL,
            rot_tol=PRESS_ROT_TOL,
            vel_tol=0.30,
            timeout=PRESS_TIMEOUT_S,
            settle_time=PRESS_HOLD_S,
            close_gripper=True,
        )
        if not success:
            self._debug_log(
                env_id,
                "Note: Pressing sequence reported timeout or tolerance failure "
                "(may be expected due to contact).",
            )

        self._debug_log(env_id, "-> Releasing brick.")
        if not await self._set_gripper(env_id, delta_width=OPEN_GRIPPER_EXTRA_WIDTH):
            self._debug_log(
                env_id, "Warning: Gripper opening sequence reported timeout."
            )

        self._debug_log(env_id, "-> Retreating.")
        success = await self._move_ee_to(
            env_id,
            retreat_pos_w,
            target_quat_w,
            pos_tol=RETREAT_POS_TOL,
            rot_tol=RETREAT_ROT_TOL,
            vel_tol=None,
            timeout=RETREAT_TIMEOUT_S,
            settle_time=0.0,
            close_gripper=False,
        )
        self._debug_log(env_id, "--- Assembly sequence finished. ---")
        return success

    async def _move_ee_to(
        self,
        env_id: int,
        target_pos_w: torch.Tensor,
        target_quat_w: torch.Tensor,
        *,
        pos_tol: float,
        rot_tol: float,
        vel_tol: float | None,
        timeout: float | None,
        settle_time: float,
        close_gripper: bool,
    ) -> bool:
        self._debug_log(
            env_id,
            f"Moving end-effector to "
            f"pos={target_pos_w.detach().cpu().tolist()}, "
            f"quat={target_quat_w.detach().cpu().tolist()}",
        )
        elapsed = 0.0
        time_reached = None

        while True:
            self._set_pose_action(
                env_id, target_pos_w, target_quat_w, close_gripper=close_gripper
            )
            await _NextStepAwaitable()
            elapsed += self._step_dt

            self._draw_target_and_current(env_id, target_pos_w)

            pos_err, rot_err = self._measure_pose_error(
                env_id, target_pos_w, target_quat_w
            )
            max_vel = self._measure_joint_velocity_max(env_id)
            is_within_tolerance = (
                pos_err < pos_tol
                and rot_err < rot_tol
                and (vel_tol is None or max_vel < vel_tol)
            )

            if is_within_tolerance:
                if time_reached is None:
                    self._debug_log(env_id, "Reached target tolerance.")
                    time_reached = elapsed
                if elapsed - time_reached >= settle_time:
                    self._debug_log(env_id, "Settled at target.")
                    return True
            else:
                time_reached = None

            if timeout is not None and elapsed > timeout:
                self._debug_log(
                    env_id,
                    f"Timeout reached: pos_err={pos_err:.4f}, "
                    f"ang_err={rot_err:.4f}, max_vel={max_vel:.4f}",
                )
                return False

    async def _set_gripper(
        self,
        env_id: int,
        target_width: float | None = None,
        *,
        delta_width: float | None = None,
        timeout: float = GRIPPER_TIMEOUT_S,
    ) -> bool:
        initial_width = self._measure_gripper_width(env_id)
        if target_width is None:
            if delta_width is None:
                raise ValueError(
                    "Either target_width or delta_width must be specified."
                )
            target_width = initial_width + delta_width

        is_closing = target_width < initial_width
        elapsed = 0.0
        stall_time = 0.0
        width_stable_time = 0.0
        best_width_err = abs(initial_width - target_width)
        best_grasp_width_err = abs(initial_width - self._grasp_width)
        min_width = initial_width
        min_max_vel = float("inf")
        min_width_delta = float("inf")
        max_stall_time = 0.0
        max_width_stable_time = 0.0
        prev_width = initial_width

        while True:
            self._set_gripper_action(env_id, close_gripper=is_closing)
            await _NextStepAwaitable()
            elapsed += self._step_dt

            joint_pos = self._robot.data.joint_pos[env_id, self._gripper_joint_ids]
            joint_vel = self._robot.data.joint_vel[env_id, self._gripper_joint_ids]
            current_width = float(torch.sum(joint_pos).item())
            max_vel = float(torch.max(torch.abs(joint_vel)).item())
            width_err = abs(current_width - target_width)
            grasp_width_err = abs(current_width - self._grasp_width)
            width_delta = abs(current_width - prev_width)

            best_width_err = min(best_width_err, width_err)
            best_grasp_width_err = min(best_grasp_width_err, grasp_width_err)
            min_width = min(min_width, current_width)
            min_max_vel = min(min_max_vel, max_vel)
            min_width_delta = min(min_width_delta, width_delta)

            if is_closing:
                if width_err < GRIPPER_POS_TOL:
                    self._debug_log(
                        env_id, f"Gripper reached target width: {current_width:.4f}"
                    )
                    return True
                if (
                    grasp_width_err < GRIPPER_GRASP_WIDTH_TOL
                    and width_delta < GRIPPER_WIDTH_DELTA_TOL
                ):
                    width_stable_time += self._step_dt
                    max_width_stable_time = max(
                        max_width_stable_time, width_stable_time
                    )
                    if width_stable_time > GRIPPER_WIDTH_STABLE_TIME_S:
                        self._debug_log(
                            env_id,
                            "Grasp detected (width stable near ideal). "
                            f"Width: {current_width:.4f}, "
                            f"ideal_width={self._grasp_width:.4f}, "
                            f"width_delta={width_delta:.6f}",
                        )
                        return True
                else:
                    width_stable_time = 0.0
                if max_vel < GRIPPER_VEL_TOL:
                    stall_time += self._step_dt
                    max_stall_time = max(max_stall_time, stall_time)
                    if (
                        width_err >= GRIPPER_POS_TOL
                        and stall_time > GRIPPER_STALL_TIME_S
                    ):
                        self._debug_log(
                            env_id,
                            "Grasp detected (velocity stall). "
                            f"Width: {current_width:.4f}",
                        )
                        return True
                else:
                    stall_time = 0.0
            else:
                # The env only exposes binary open/close gripper commands, so
                # opening is approximated by holding the open command until the
                # measured width is at least the demo target width.
                if current_width >= target_width - GRIPPER_POS_TOL:
                    self._debug_log(
                        env_id, f"Gripper reached target width: {current_width:.4f}"
                    )
                    return True

            if elapsed > timeout:
                finger_pos = [
                    float(value) for value in joint_pos.detach().cpu().tolist()
                ]
                finger_vel = [
                    float(value) for value in joint_vel.detach().cpu().tolist()
                ]
                reason_parts: list[str] = []
                if is_closing and best_width_err >= GRIPPER_POS_TOL:
                    reason_parts.append("target width never reached")
                    if best_grasp_width_err >= GRIPPER_GRASP_WIDTH_TOL:
                        reason_parts.append("gripper never got near ideal grasp width")
                    elif max_width_stable_time <= GRIPPER_WIDTH_STABLE_TIME_S:
                        reason_parts.append(
                            "width never stabilized near ideal grasp width"
                        )
                    if min_max_vel >= GRIPPER_VEL_TOL:
                        reason_parts.append(
                            "finger velocity never dropped below stall threshold"
                        )
                    elif max_stall_time <= GRIPPER_STALL_TIME_S:
                        reason_parts.append("stall windows kept resetting before 1.0s")
                if not is_closing and current_width < target_width - GRIPPER_POS_TOL:
                    reason_parts.append("gripper never opened wide enough")
                if not reason_parts:
                    reason_parts.append(
                        "timed out despite satisfying neither success condition"
                    )

                self._debug_log(
                    env_id,
                    "Gripper timeout. "
                    f"initial_width={initial_width:.4f}, "
                    f"current_width={current_width:.4f}, "
                    f"target_width={target_width:.4f}, width_err={width_err:.4f}, "
                    f"ideal_grasp_width={self._grasp_width:.4f}, "
                    f"grasp_width_err={grasp_width_err:.4f}, "
                    f"width_delta={width_delta:.6f}, "
                    f"min_width_delta={min_width_delta:.6f}, "
                    f"max_vel={max_vel:.6f}, min_max_vel={min_max_vel:.6f}, "
                    f"stall_time={stall_time:.3f}, "
                    f"max_stall_time={max_stall_time:.3f}, "
                    f"width_stable_time={width_stable_time:.3f}, "
                    f"max_width_stable_time={max_width_stable_time:.3f}, "
                    f"min_width={min_width:.4f}, finger_pos={finger_pos}, "
                    f"finger_vel={finger_vel}, "
                    f"reason={'; '.join(reason_parts)}",
                )
                return False

            prev_width = current_width

    def _set_idle_action(self, env_id: int) -> None:
        self._step_actions[env_id] = 0.0

    def _set_gripper_action(self, env_id: int, *, close_gripper: bool) -> None:
        self._step_actions[env_id] = 0.0
        self._step_actions[env_id, 6] = -1.0 if close_gripper else 1.0

    def _set_pose_action(
        self,
        env_id: int,
        target_pos_w: torch.Tensor,
        target_quat_w: torch.Tensor,
        *,
        close_gripper: bool,
    ) -> None:
        self._step_actions[env_id] = 0.0
        self._step_actions[env_id, :6] = self._actions_to_world_target(
            env_id, target_pos_w, target_quat_w
        )
        self._step_actions[env_id, 6] = -1.0 if close_gripper else 1.0

    def _draw_target_and_current(self, env_id: int, target_pos_w: torch.Tensor) -> None:
        debug_draw = self._debug_draw
        if debug_draw is None or env_id != self._debug_draw_env_id:
            return

        current_pos_w, _ = self._get_control_frame_pose_w(env_id)
        debug_draw.clear_points()
        debug_draw.draw_points(
            [tuple(target_pos_w.detach().cpu().tolist())],
            [(1.0, 0.0, 0.0, 1.0)],
            [10.0],
        )
        debug_draw.draw_points(
            [tuple(current_pos_w.detach().cpu().tolist())],
            [(0.0, 1.0, 0.0, 1.0)],
            [10.0],
        )

    def _measure_pose_error(
        self, env_id: int, target_pos_w: torch.Tensor, target_quat_w: torch.Tensor
    ) -> tuple[float, float]:
        control_pos_w, control_quat_w = self._get_control_frame_pose_w(env_id)
        pos_error, rot_error = math_utils.compute_pose_error(
            control_pos_w.unsqueeze(0),
            control_quat_w.unsqueeze(0),
            target_pos_w.unsqueeze(0),
            target_quat_w.unsqueeze(0),
            rot_error_type="axis_angle",
        )
        return (
            float(torch.linalg.vector_norm(pos_error, dim=1)[0].item()),
            float(torch.linalg.vector_norm(rot_error, dim=1)[0].item()),
        )

    def _measure_orientation_error(
        self, source_quat_w: torch.Tensor, target_quat_w: torch.Tensor
    ) -> float:
        return float(
            math_utils.quat_error_magnitude(
                source_quat_w.unsqueeze(0), target_quat_w.unsqueeze(0)
            )[0].item()
        )

    def _measure_joint_velocity_max(self, env_id: int) -> float:
        return float(torch.max(torch.abs(self._robot.data.joint_vel[env_id])).item())

    def _measure_gripper_width(self, env_id: int) -> float:
        return float(
            torch.sum(
                self._robot.data.joint_pos[env_id, self._gripper_joint_ids]
            ).item()
        )

    def _measure_gripper_velocity_max(self, env_id: int) -> float:
        return float(
            torch.max(
                torch.abs(self._robot.data.joint_vel[env_id, self._gripper_joint_ids])
            ).item()
        )

    def _measure_hole_to_eef(self, env_id: int) -> tuple[torch.Tensor, torch.Tensor]:
        control_pos_w, control_quat_w = self._get_control_frame_pose_w(env_id)
        captured_pos, captured_quat = math_utils.subtract_frame_transforms(
            self._brick.data.root_pos_w[env_id : env_id + 1],
            self._brick.data.root_quat_w[env_id : env_id + 1],
            control_pos_w.unsqueeze(0),
            control_quat_w.unsqueeze(0),
        )
        return captured_pos[0], captured_quat[0]

    def _actions_to_world_target(
        self, env_id: int, target_pos_w: torch.Tensor, target_quat_w: torch.Tensor
    ) -> torch.Tensor:
        control_pos_b, control_quat_b = self._get_control_frame_pose_b(env_id)
        target_pos_b, target_quat_b = self._world_pose_to_robot_root(
            env_id, target_pos_w, target_quat_w
        )
        pos_error, rot_error = math_utils.compute_pose_error(
            control_pos_b.unsqueeze(0),
            control_quat_b.unsqueeze(0),
            target_pos_b.unsqueeze(0),
            target_quat_b.unsqueeze(0),
            rot_error_type="axis_angle",
        )
        delta_pose = torch.cat((pos_error, rot_error), dim=1)
        return torch.clamp(delta_pose / self._action_scale, -1.0, 1.0)[0]

    def _get_control_frame_pose_w(
        self, env_id: int
    ) -> tuple[torch.Tensor, torch.Tensor]:
        body_pos_w = self._arm_action_term._asset.data.body_pos_w[
            env_id : env_id + 1, self._arm_action_term._body_idx
        ]
        body_quat_w = self._arm_action_term._asset.data.body_quat_w[
            env_id : env_id + 1, self._arm_action_term._body_idx
        ]
        if self._arm_action_term.cfg.body_offset is None:
            return body_pos_w[0], body_quat_w[0]
        offset_pos = self._arm_action_term._offset_pos
        offset_rot = self._arm_action_term._offset_rot
        assert offset_pos is not None
        assert offset_rot is not None
        control_pos_w, control_quat_w = math_utils.combine_frame_transforms(
            body_pos_w,
            body_quat_w,
            offset_pos[env_id : env_id + 1],
            offset_rot[env_id : env_id + 1],
        )
        return control_pos_w[0], control_quat_w[0]

    def _get_control_frame_pose_b(
        self, env_id: int
    ) -> tuple[torch.Tensor, torch.Tensor]:
        control_pos_w, control_quat_w = self._get_control_frame_pose_w(env_id)
        return self._world_pose_to_robot_root(env_id, control_pos_w, control_quat_w)

    def _world_pose_to_robot_root(
        self,
        env_id: int,
        target_pos_w: torch.Tensor,
        target_quat_w: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        robot_root_pos_w = self._robot.data.root_pos_w[env_id : env_id + 1]
        robot_root_quat_w = self._robot.data.root_quat_w[env_id : env_id + 1]
        target_pos_b, target_quat_b = math_utils.subtract_frame_transforms(
            robot_root_pos_w,
            robot_root_quat_w,
            target_pos_w.unsqueeze(0),
            target_quat_w.unsqueeze(0),
        )
        return target_pos_b[0], target_quat_b[0]

    def _combine_pose(
        self,
        parent_pos: torch.Tensor,
        parent_quat: torch.Tensor,
        child_pos: torch.Tensor,
        child_quat: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        pos_w, quat_w = math_utils.combine_frame_transforms(
            parent_pos.unsqueeze(0),
            parent_quat.unsqueeze(0),
            child_pos.unsqueeze(0),
            child_quat.unsqueeze(0),
        )
        return pos_w[0], quat_w[0]
