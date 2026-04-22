#!/usr/bin/env python3
"""Minimal Python client showing the dispatcher in action on UR10e.

Prereqs:
  - move_group running (use ur_moveit_config)
  - cumotion_planner_node running (use fallback.launch.py in this example)

Usage:
    python3 simple_plan_example.py
"""

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

# Use the dispatcher from templates/
TEMPLATES_DIR = Path(__file__).resolve().parents[3] / "templates" / "scripts"
sys.path.insert(0, str(TEMPLATES_DIR))

from planner_dispatcher import PlannerDispatcher, PlannerConfig  # noqa: E402


UR10E_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def make_start_state(joint_positions) -> RobotState:
    js = JointState()
    js.name = list(UR10E_ARM_JOINTS)
    js.position = list(joint_positions)
    rs = RobotState()
    rs.joint_state = js
    return rs


def main() -> int:
    rclpy.init()
    node = Node("ur10e_plan_example")

    dispatcher = PlannerDispatcher(
        node,
        PlannerConfig(
            planning_group="manipulator",
            arm_joint_names=UR10E_ARM_JOINTS,
            planning_attempts=3,
            planning_time_regular=2.0,
            planning_time_increased=5.0,
        ),
    )

    # Home pose → a random reachable pose
    start = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]
    goal = [1.0, -1.2, 1.8, -0.5, 1.2, 0.5]

    start_state = make_start_state(start)

    node.get_logger().info(f"Planning {start} → {goal}")
    success, traj = dispatcher.plan_to_joint_goal(start_state, goal)

    if success and traj:
        node.get_logger().info(f"SUCCESS: {len(traj.points)} waypoints, "
                               f"duration {traj.points[-1].time_from_start.sec}s")
        return 0
    else:
        node.get_logger().error("FAILED: no valid trajectory found")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
