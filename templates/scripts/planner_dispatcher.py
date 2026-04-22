#!/usr/bin/env python3
"""Planner dispatcher: cuMotion primary with OMPL fallback.

Robot-agnostic reference implementation of the production pattern described
in docs/06_ompl_fallback.md. Drop-in for any MoveIt2-based robot.

Usage:
    from planner_dispatcher import PlannerDispatcher, PlannerConfig

    dispatcher = PlannerDispatcher(node, PlannerConfig(
        planning_group="manipulator",
        planning_attempts=3,
        planning_time_regular=2.0,
        planning_time_increased=5.0,
    ))
    success, trajectory = dispatcher.plan_to_joint_goal(current_state, goal_joints)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MoveItErrorCodes,
    MotionPlanRequest,
    PlanningOptions,
    RobotState,
    JointConstraint,
    Constraints,
)
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


@dataclass
class PlannerConfig:
    planning_group: str
    arm_joint_names: List[str] = field(default_factory=list)

    # cuMotion config
    cumotion_pipeline_id: str = "isaac_ros_cumotion"
    cumotion_action_topic: str = "/cumotion/move_group"  # or "/move_action" for plugin
    cumotion_planner_id: str = ""
    planning_attempts: int = 3

    # OMPL config
    ompl_pipeline_id: str = "ompl"
    ompl_action_topic: str = "/move_action"
    ompl_planner_id: str = "RRTConnectkConfigDefault"
    planning_time_regular: float = 2.0
    planning_time_increased: float = 5.0

    # Fallback triggers — error codes that cause immediate OMPL fallback
    fallback_error_codes: List[int] = field(default_factory=lambda: [
        MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE,
        MoveItErrorCodes.START_STATE_IN_COLLISION,
    ])

    # Post-validation
    enable_post_validation: bool = True
    validity_service: str = "/check_state_validity"


class PlannerDispatcher:
    """Thin wrapper dispatching plan requests to cuMotion with OMPL fallback.

    Lifecycle:
        dispatcher = PlannerDispatcher(node, config)
        success, traj = dispatcher.plan_to_joint_goal(start_state, joint_goals)
    """

    def __init__(self, node: Node, config: PlannerConfig):
        self._node = node
        self._config = config
        self._cumotion_client = ActionClient(node, MoveGroup, config.cumotion_action_topic)
        self._ompl_client = ActionClient(node, MoveGroup, config.ompl_action_topic)
        self._validity_client = (
            node.create_client(GetStateValidity, config.validity_service)
            if config.enable_post_validation
            else None
        )
        self._last_cumotion_error: Optional[int] = None
        self._move_group_pid: Optional[int] = None  # tracked for crash detection

    # ── Public API ──────────────────────────────────────────────────────────

    def plan_to_joint_goal(
        self,
        start_state: RobotState,
        goal_joints: List[float],
    ) -> Tuple[bool, Optional[JointTrajectory]]:
        """Plan a joint-space motion with cuMotion, falling back to OMPL.

        Returns (success, trajectory). Trajectory is None on failure.
        """
        ompl_fallback = False
        for attempt in range(self._config.planning_attempts):
            active_pipeline = (
                self._config.ompl_pipeline_id if ompl_fallback
                else self._config.cumotion_pipeline_id
            )

            if active_pipeline == self._config.cumotion_pipeline_id:
                success, traj = self._plan_with_cumotion(start_state, goal_joints)
                if not success:
                    err = self._last_cumotion_error
                    # Immediate fallback triggers
                    if err in self._config.fallback_error_codes:
                        self._node.get_logger().warn(
                            f"cuMotion error {err} — falling back to OMPL"
                        )
                        ompl_fallback = True
                        success, traj = self._plan_with_ompl(
                            start_state, goal_joints,
                            self._config.planning_time_regular,
                        )
                    # Permanent fallback: move_group crashed (FCL race)
                    elif self._move_group_crashed():
                        self._node.get_logger().warn(
                            "move_group crashed during cuMotion — switching to OMPL permanently"
                        )
                        ompl_fallback = True
                        success, traj = self._plan_with_ompl(
                            start_state, goal_joints,
                            self._config.planning_time_increased,
                        )
            else:
                # Already in OMPL fallback mode
                planning_time = (
                    self._config.planning_time_regular if attempt == 0
                    else self._config.planning_time_increased
                )
                success, traj = self._plan_with_ompl(start_state, goal_joints, planning_time)

            if success and traj and self._post_validate(traj):
                self._node.get_logger().info(
                    f"Planning succeeded on attempt {attempt + 1} "
                    f"({'OMPL' if ompl_fallback else 'cuMotion'})"
                )
                return True, traj

        # Last resort: cuMotion exhausted attempts without triggering fallback
        if (
            self._config.cumotion_pipeline_id == self._config.cumotion_pipeline_id
            and not ompl_fallback
        ):
            self._node.get_logger().warn(
                f"cuMotion exhausted all {self._config.planning_attempts} attempts. "
                "Trying OMPL last-resort."
            )
            success, traj = self._plan_with_ompl(
                start_state, goal_joints,
                self._config.planning_time_increased,
            )
            if success and traj and self._post_validate(traj):
                self._node.get_logger().info("OMPL last-resort succeeded")
                return True, traj

        self._node.get_logger().error("All planning attempts failed")
        return False, None

    # ── Internal planning ──────────────────────────────────────────────────

    def _plan_with_cumotion(
        self,
        start_state: RobotState,
        goal_joints: List[float],
    ) -> Tuple[bool, Optional[JointTrajectory]]:
        self._move_group_pid = self._get_move_group_pid()
        req = self._build_request(
            start_state, goal_joints,
            pipeline_id=self._config.cumotion_pipeline_id,
            planner_id=self._config.cumotion_planner_id,
            planning_time=self._config.planning_time_regular,
        )
        result = self._send_goal(self._cumotion_client, req)
        if result is None:
            self._last_cumotion_error = MoveItErrorCodes.FAILURE
            return False, None
        self._last_cumotion_error = result.error_code.val
        traj = result.planned_trajectory.joint_trajectory
        success = (
            result.error_code.val == MoveItErrorCodes.SUCCESS
            and len(traj.points) > 0
        )
        return success, traj if success else None

    def _plan_with_ompl(
        self,
        start_state: RobotState,
        goal_joints: List[float],
        planning_time: float,
    ) -> Tuple[bool, Optional[JointTrajectory]]:
        req = self._build_request(
            start_state, goal_joints,
            pipeline_id=self._config.ompl_pipeline_id,
            planner_id=self._config.ompl_planner_id,
            planning_time=planning_time,
        )
        result = self._send_goal(self._ompl_client, req)
        if result is None:
            return False, None
        traj = result.planned_trajectory.joint_trajectory
        success = (
            result.error_code.val == MoveItErrorCodes.SUCCESS
            and len(traj.points) > 0
        )
        return success, traj if success else None

    def _build_request(
        self,
        start_state: RobotState,
        goal_joints: List[float],
        pipeline_id: str,
        planner_id: str,
        planning_time: float,
    ) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = self._config.planning_group
        req.pipeline_id = pipeline_id
        req.planner_id = planner_id
        req.allowed_planning_time = planning_time
        req.num_planning_attempts = 1
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0

        req.start_state = start_state

        goal = Constraints()
        for name, val in zip(self._config.arm_joint_names, goal_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        req.goal_constraints.append(goal)

        return req

    def _send_goal(self, client: ActionClient, req: MotionPlanRequest):
        """Synchronous-style: wait for accept and result."""
        if not client.wait_for_server(timeout_sec=10.0):
            self._node.get_logger().error(f"Action server {client._action_name} unavailable")
            return None
        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = True

        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=30.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self._node, result_future,
            timeout_sec=req.allowed_planning_time + 5.0,
        )
        if not result_future.done():
            return None
        return result_future.result().result

    # ── Post-validation ────────────────────────────────────────────────────

    def _post_validate(self, traj: JointTrajectory) -> bool:
        """Check every waypoint against FCL. Returns True if all valid."""
        if self._validity_client is None:
            return True
        if not self._validity_client.service_is_ready():
            self._node.get_logger().warn("validity service not ready; skipping post-validation")
            return True

        for i, pt in enumerate(traj.points):
            js = JointState()
            js.header.stamp = self._node.get_clock().now().to_msg()
            js.name = list(traj.joint_names)
            js.position = list(pt.positions)
            rs = RobotState()
            rs.joint_state = js

            req = GetStateValidity.Request()
            req.robot_state = rs
            req.group_name = self._config.planning_group

            fut = self._validity_client.call_async(req)
            rclpy.spin_until_future_complete(self._node, fut, timeout_sec=2.0)
            if not fut.done() or fut.result() is None:
                self._node.get_logger().warn(f"post-validation at waypoint {i} timed out")
                continue
            if not fut.result().valid:
                contacts = [
                    f"{c.contact_body_1}×{c.contact_body_2}"
                    for c in fut.result().contacts
                ]
                self._node.get_logger().warn(
                    f"post-validation FAIL at waypoint {i}: {contacts}"
                )
                return False
        return True

    # ── Crash detection ────────────────────────────────────────────────────

    def _get_move_group_pid(self) -> Optional[int]:
        """Read the current move_group pid via psutil (optional dependency)."""
        try:
            import psutil
        except ImportError:
            return None
        for proc in psutil.process_iter(["name", "cmdline"]):
            try:
                cmd = " ".join(proc.info["cmdline"] or [])
                if "move_group" in cmd:
                    return proc.pid
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None

    def _move_group_crashed(self) -> bool:
        """Return True if move_group's pid changed (crash → respawn)."""
        if self._move_group_pid is None:
            return False
        current = self._get_move_group_pid()
        return current is not None and current != self._move_group_pid
