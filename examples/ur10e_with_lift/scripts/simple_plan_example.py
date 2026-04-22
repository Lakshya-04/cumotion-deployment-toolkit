#!/usr/bin/env python3
"""Minimal 7-DOF planning example: UR10e + Lift."""

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

TEMPLATES_DIR = Path(__file__).resolve().parents[3] / "templates" / "scripts"
sys.path.insert(0, str(TEMPLATES_DIR))

from planner_dispatcher import PlannerDispatcher, PlannerConfig  # noqa: E402


JOINT_NAMES = [
    "lift",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def make_start_state(positions) -> RobotState:
    js = JointState()
    js.name = list(JOINT_NAMES)
    js.position = list(positions)
    rs = RobotState()
    rs.joint_state = js
    return rs


def main() -> int:
    rclpy.init()
    node = Node("ur10e_lift_plan_example")

    dispatcher = PlannerDispatcher(
        node,
        PlannerConfig(
            planning_group="arm_with_lift",
            arm_joint_names=JOINT_NAMES,
            planning_attempts=3,
            planning_time_regular=3.0,
            planning_time_increased=8.0,
        ),
    )

    # Low lift, neutral arm → High lift, stretched arm
    start = [0.2, 0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
    goal  = [1.1, 1.0, -0.8, 1.2, -0.5, 1.0, 0.5]

    start_state = make_start_state(start)

    node.get_logger().info(f"7-DOF plan {start} → {goal}")
    success, traj = dispatcher.plan_to_joint_goal(start_state, goal)

    if success and traj:
        node.get_logger().info(f"SUCCESS: {len(traj.points)} waypoints")
        return 0
    node.get_logger().error("FAILED")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
