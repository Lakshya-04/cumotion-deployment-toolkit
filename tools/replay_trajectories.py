#!/usr/bin/env python3
"""Replay saved trajectories in RViz by publishing /joint_states.

Reads a run_*.json from validate_dataset.py --save-trajectories and
streams each trajectory at real time. Useful for visual review without
re-planning.

Usage:
    python3 replay_trajectories.py --run results/run_*.json
    python3 replay_trajectories.py --run results/run_*.json --pair-ids pair_0001 pair_0010
    python3 replay_trajectories.py --run results/run_*.json --loops 3 --hold-s 0.5
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


ANIM_HZ = 30.0


class TrajReplayer(Node):
    def __init__(self, trajectories: list, loops: int, hold_s: float):
        super().__init__("trajectory_replayer")
        self.declare_parameter("extra_joint_names", [""])
        self.declare_parameter("extra_joint_values", [0.0])
        extra_names = [x for x in self.get_parameter("extra_joint_names").value if x]
        extra_vals = list(self.get_parameter("extra_joint_values").value)
        self._extra = dict(zip(extra_names, extra_vals))

        self._trajs = trajectories
        self._loops = loops
        self._hold_s = hold_s
        self._idx = 0
        self._loop = 0
        self._start_ns = self.get_clock().now().nanoseconds
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / ANIM_HZ, self._tick)

    def _tick(self) -> None:
        if self._idx >= len(self._trajs):
            rclpy.shutdown()
            return

        entry = self._trajs[self._idx]
        traj = entry["trajectory"]
        jnames = entry.get("joint_names", [])
        duration = traj[-1]["t"]

        t_now = (self.get_clock().now().nanoseconds - self._start_ns) * 1e-9
        if t_now > duration + self._hold_s:
            self._loop += 1
            if self._loop >= self._loops:
                self.get_logger().info(f"done replaying {entry['id']}")
                self._idx += 1
                self._loop = 0
            self._start_ns = self.get_clock().now().nanoseconds
            return

        positions = self._interp(traj, min(t_now, duration))
        self._publish(jnames, positions)

    def _interp(self, traj: List[dict], t: float) -> List[float]:
        if t <= traj[0]["t"]:
            return list(traj[0]["positions"])
        if t >= traj[-1]["t"]:
            return list(traj[-1]["positions"])
        for i in range(1, len(traj)):
            t0, t1 = traj[i - 1]["t"], traj[i]["t"]
            if t <= t1:
                a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                p0, p1 = traj[i - 1]["positions"], traj[i]["positions"]
                return [p0[j] + a * (p1[j] - p0[j]) for j in range(len(p0))]
        return list(traj[-1]["positions"])

    def _publish(self, names: List[str], positions: List[float]) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        # Also include any extra joints (e.g. mimic joints, gripper) as static
        js.name = list(names) + list(self._extra.keys())
        js.position = list(positions) + list(self._extra.values())
        self._pub.publish(js)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--pair-ids", nargs="*", default=None, help="Specific pair IDs")
    ap.add_argument("--loops", type=int, default=1)
    ap.add_argument("--hold-s", type=float, default=1.0, help="Hold at each end")
    args = ap.parse_args()

    with open(args.run) as f:
        data = json.load(f)

    traj_entries = [p for p in data["per_pair"] if p.get("trajectory")]
    if args.pair_ids:
        traj_entries = [p for p in traj_entries if p["id"] in args.pair_ids]
    if not traj_entries:
        print("No trajectories to replay.")
        return 1

    print(f"Replaying {len(traj_entries)} trajectories, {args.loops} loops each")
    rclpy.init()
    try:
        rclpy.spin(TrajReplayer(traj_entries, args.loops, args.hold_s))
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
