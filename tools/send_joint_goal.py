#!/usr/bin/env python3
"""Generic joint-space goal sender for any-DOF robot.

Sends a joint goal through the cuMotion + OMPL fallback dispatcher and
optionally replays the resulting trajectory to /joint_states so you can
watch it in RViz.

Three input modes:
  1) --pair-id <id> --dataset <pairs.json>
        Send one specific pair from a dataset JSON.
  2) --start <j1 j2 ...> --goal <j1 j2 ...>
        Pass start and goal joint values directly on the command line.
  3) --from-current --goal <j1 j2 ...>
        Use the robot's current /joint_states (first matching snapshot)
        as start, goal from command line.

Works for any DOF — pass as many joint names and values as your robot has.

Usage:
    # 6-DOF UR10e plan from home to a goal
    python3 send_joint_goal.py \\
        --planning-group manipulator \\
        --joint-names shoulder_pan_joint shoulder_lift_joint elbow_joint \\
                      wrist_1_joint wrist_2_joint wrist_3_joint \\
        --start 0 -1.57 1.57 0 1.57 0 \\
        --goal  1.0 -1.2 1.8 -0.5 1.2 0.5 \\
        --replay

    # 7-DOF arm + lift
    python3 send_joint_goal.py \\
        --planning-group arm_with_lift \\
        --joint-names lift shoulder_pan_joint shoulder_lift_joint elbow_joint \\
                      wrist_1_joint wrist_2_joint wrist_3_joint \\
        --from-current \\
        --goal 1.0 1.0 -0.8 1.2 -0.5 1.0 0.5 \\
        --replay

    # Reproduce a specific pair from a saved dataset
    python3 send_joint_goal.py \\
        --planning-group manipulator --joint-names <...> \\
        --dataset pairs.json --pair-id pair_000042 --replay
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

TEMPLATES_DIR = Path(__file__).resolve().parent.parent / "templates" / "scripts"
sys.path.insert(0, str(TEMPLATES_DIR))
from planner_dispatcher import PlannerConfig, PlannerDispatcher  # noqa: E402


ANIM_HZ = 30.0


class GoalSender(Node):
    def __init__(self, args):
        super().__init__("send_joint_goal")
        self._args = args
        self._joint_names = args.joint_names

        # Optional: capture current robot state from /joint_states
        self._current_state: Optional[List[float]] = None
        if args.from_current:
            self.create_subscription(JointState, "/joint_states",
                                     self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        if self._current_state is not None:
            return
        # Extract in our joint-name order
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._current_state = [name_to_pos[n] for n in self._joint_names]
            self.get_logger().info(
                f"Captured current state: {[round(x, 3) for x in self._current_state]}"
            )
        except KeyError as e:
            self.get_logger().warn(f"joint missing from /joint_states: {e}")

    def resolve_start(self) -> List[float]:
        if self._args.from_current:
            self.get_logger().info("Waiting for /joint_states ...")
            for _ in range(100):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._current_state is not None:
                    return self._current_state
            self.get_logger().error("Timed out waiting for /joint_states")
            sys.exit(1)
        if self._args.dataset and self._args.pair_id:
            with open(self._args.dataset) as f:
                ds = json.load(f)
            for p in ds["pairs"]:
                if p["id"] == self._args.pair_id:
                    return list(p["start"])
            self.get_logger().error(f"pair_id {self._args.pair_id} not found")
            sys.exit(1)
        if self._args.start:
            return list(self._args.start)
        self.get_logger().error("Need one of: --from-current, --pair-id, --start")
        sys.exit(1)

    def resolve_goal(self) -> List[float]:
        if self._args.dataset and self._args.pair_id:
            with open(self._args.dataset) as f:
                ds = json.load(f)
            for p in ds["pairs"]:
                if p["id"] == self._args.pair_id:
                    return list(p["goal"])
        if self._args.goal:
            return list(self._args.goal)
        self.get_logger().error("Need --goal or --pair-id")
        sys.exit(1)


def replay_trajectory(node: Node, traj: JointTrajectory, loops: int, hold_s: float) -> None:
    """Publish the trajectory waypoints on /joint_states for RViz replay."""
    pub = node.create_publisher(JointState, "/joint_states", 10)
    duration = traj.points[-1].time_from_start.sec + \
        traj.points[-1].time_from_start.nanosec * 1e-9

    def _interp(t: float) -> List[float]:
        pts = traj.points
        if t <= 0 or len(pts) == 1:
            return list(pts[0].positions)
        if t >= duration:
            return list(pts[-1].positions)
        for i in range(1, len(pts)):
            t0 = pts[i - 1].time_from_start.sec + pts[i - 1].time_from_start.nanosec * 1e-9
            t1 = pts[i].time_from_start.sec + pts[i].time_from_start.nanosec * 1e-9
            if t <= t1:
                a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                p0 = pts[i - 1].positions
                p1 = pts[i].positions
                return [p0[j] + a * (p1[j] - p0[j]) for j in range(len(p0))]
        return list(pts[-1].positions)

    node.get_logger().info(
        f"Replaying {len(traj.points)} waypoints, duration {duration:.2f}s, "
        f"{loops} loop(s)"
    )
    dt = 1.0 / ANIM_HZ
    for loop in range(loops):
        t_start = time.time()
        while True:
            t = time.time() - t_start
            if t > duration + hold_s:
                break
            positions = _interp(min(t, duration))
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = list(traj.joint_names)
            msg.position = list(positions)
            pub.publish(msg)
            time.sleep(dt)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--planning-group", required=True)
    ap.add_argument("--joint-names", nargs="+", required=True,
                    help="Joint names in the order used by your planning group")

    # Three input modes (mutually exclusive-ish)
    ap.add_argument("--start", nargs="+", type=float, help="Start joint values (DOF floats)")
    ap.add_argument("--goal", nargs="+", type=float, help="Goal joint values (DOF floats)")
    ap.add_argument("--from-current", action="store_true",
                    help="Use /joint_states as start state")
    ap.add_argument("--dataset", type=Path, help="pairs.json file for --pair-id")
    ap.add_argument("--pair-id", help="Specific pair id from dataset")

    # Dispatcher knobs
    ap.add_argument("--cumotion-action", default="/cumotion/move_group")
    ap.add_argument("--ompl-action", default="/move_action")
    ap.add_argument("--planning-attempts", type=int, default=3)
    ap.add_argument("--planning-time-regular", type=float, default=2.0)
    ap.add_argument("--planning-time-increased", type=float, default=5.0)

    # Replay
    ap.add_argument("--replay", action="store_true", help="Replay trajectory on /joint_states")
    ap.add_argument("--loops", type=int, default=1)
    ap.add_argument("--hold-s", type=float, default=1.0)

    args = ap.parse_args()

    # Validate
    if args.start and len(args.start) != len(args.joint_names):
        sys.exit(f"--start needs {len(args.joint_names)} values (same as --joint-names)")
    if args.goal and len(args.goal) != len(args.joint_names):
        sys.exit(f"--goal needs {len(args.joint_names)} values (same as --joint-names)")

    rclpy.init()
    sender = GoalSender(args)
    start = sender.resolve_start()
    goal = sender.resolve_goal()

    sender.get_logger().info(
        f"Planning {len(args.joint_names)}-DOF\n  start: {[round(x, 3) for x in start]}"
        f"\n  goal:  {[round(x, 3) for x in goal]}"
    )

    dispatcher = PlannerDispatcher(
        sender,
        PlannerConfig(
            planning_group=args.planning_group,
            arm_joint_names=args.joint_names,
            cumotion_action_topic=args.cumotion_action,
            ompl_action_topic=args.ompl_action,
            planning_attempts=args.planning_attempts,
            planning_time_regular=args.planning_time_regular,
            planning_time_increased=args.planning_time_increased,
        ),
    )

    js = JointState()
    js.name = list(args.joint_names)
    js.position = list(start)
    rs = RobotState()
    rs.joint_state = js

    success, traj = dispatcher.plan_to_joint_goal(rs, goal)
    if not success or not traj:
        sender.get_logger().error("PLAN FAILED")
        sender.destroy_node()
        rclpy.shutdown()
        return 1

    duration = traj.points[-1].time_from_start.sec + \
        traj.points[-1].time_from_start.nanosec * 1e-9
    sender.get_logger().info(
        f"PLAN SUCCESS — {len(traj.points)} waypoints, duration {duration:.2f}s"
    )

    if args.replay:
        replay_trajectory(sender, traj, args.loops, args.hold_s)

    sender.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
