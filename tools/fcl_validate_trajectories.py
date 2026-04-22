#!/usr/bin/env python3
"""Post-validate saved trajectories against FCL mesh-level collision.

Reads a run_*.json (output of validate_dataset.py with --save-trajectories),
iterates every waypoint of every success, calls /check_state_validity, and
reports which trajectories have hidden mesh collisions.

Useful for:
  - Ground-truthing cuMotion's sphere-model "successes"
  - Finding XRDF ignore list entries that are wrong (enabled pairs that
    actually DO collide in mesh)

Usage:
    python3 fcl_validate_trajectories.py --run results/run_cumotion_*.json \\
        --planning-group manipulator
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, Set, Tuple

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--planning-group", default="")
    ap.add_argument("--validity-service", default="/check_state_validity")
    ap.add_argument("--max-pairs", type=int, default=0)
    args = ap.parse_args()

    with open(args.run) as f:
        data = json.load(f)

    pairs = [p for p in data["per_pair"] if p.get("trajectory")]
    if args.max_pairs:
        pairs = pairs[: args.max_pairs]
    print(f"FCL-validating {len(pairs)} trajectories from {args.run.name}")

    rclpy.init()
    node = Node("fcl_validator")
    client = node.create_client(GetStateValidity, args.validity_service)
    if not client.wait_for_service(timeout_sec=30.0):
        node.get_logger().error(f"{args.validity_service} unavailable")
        return 1

    clean: int = 0
    dirty: int = 0
    collision_pairs: Dict[Tuple[str, str], int] = defaultdict(int)

    for i, pair in enumerate(pairs):
        traj = pair["trajectory"]
        jnames = pair.get("joint_names", [])
        pair_clean = True
        pair_collisions: Set[Tuple[str, str]] = set()

        for wp in traj:
            js = JointState()
            js.name = list(jnames)
            js.position = list(wp["positions"])
            rs = RobotState()
            rs.joint_state = js
            req = GetStateValidity.Request()
            req.robot_state = rs
            req.group_name = args.planning_group

            fut = client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            if not fut.done():
                continue
            resp = fut.result()
            if resp is None or resp.valid:
                continue

            pair_clean = False
            for c in resp.contacts:
                key = tuple(sorted([c.contact_body_1, c.contact_body_2]))
                pair_collisions.add(key)

        if pair_clean:
            clean += 1
            print(f"  [{i + 1}/{len(pairs)}] {pair['id']}  CLEAN")
        else:
            dirty += 1
            for k in pair_collisions:
                collision_pairs[k] += 1
            print(f"  [{i + 1}/{len(pairs)}] {pair['id']}  DIRTY  "
                  f"pairs: {sorted(pair_collisions)}")

    print("\n" + "=" * 60)
    print(f"Clean:  {clean}/{len(pairs)}")
    print(f"Dirty:  {dirty}/{len(pairs)}")
    if collision_pairs:
        print("\nCollision pairs (mesh-level, sphere missed):")
        for pair, n in sorted(collision_pairs.items(), key=lambda x: -x[1]):
            print(f"  {pair[0]} x {pair[1]}: {n}/{dirty} trajectories")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
