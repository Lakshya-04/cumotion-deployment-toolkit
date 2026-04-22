#!/usr/bin/env python3
"""Run cuMotion or OMPL over a dataset of start/goal pairs, record results.

Dataset format (JSON):
{
  "pairs": [
    {"id": "pair_0000", "start": [j1, j2, ...], "goal": [j1, j2, ...]},
    ...
  ]
}

Outputs a JSON run file with per-pair result codes, waypoint counts, planning
times, and (optionally) trajectories for replay.

Use the companion tools/fcl_validate_trajectories.py to post-validate the
saved trajectories against mesh geometry.

Usage:
    python3 validate_dataset.py \\
        --dataset pairs.json --out-dir results/ \\
        --planning-group manipulator \\
        --arm-joints shoulder_pan_joint shoulder_lift_joint elbow_joint \\
                     wrist_1_joint wrist_2_joint wrist_3_joint \\
        --pipeline cumotion \\
        --save-trajectories
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

TEMPLATES_DIR = Path(__file__).resolve().parent.parent / "templates" / "scripts"
sys.path.insert(0, str(TEMPLATES_DIR))
from planner_dispatcher import PlannerConfig, PlannerDispatcher  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", required=True, type=Path, help="pairs.json path")
    ap.add_argument("--out-dir", required=True, type=Path, help="where to write run JSON")
    ap.add_argument("--planning-group", required=True)
    ap.add_argument("--arm-joints", nargs="+", required=True)
    ap.add_argument(
        "--pipeline",
        default="cumotion",
        choices=["cumotion", "ompl", "fallback"],
        help="cumotion only, ompl only, or cumotion+OMPL fallback",
    )
    ap.add_argument("--max-pairs", type=int, default=0, help="Limit (0 = all)")
    ap.add_argument("--offset", type=int, default=0, help="Skip first N pairs")
    ap.add_argument("--planning-timeout", type=float, default=10.0)
    ap.add_argument("--save-trajectories", action="store_true",
                    help="Include full trajectory waypoints in output JSON")
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    with open(args.dataset) as f:
        data = json.load(f)
    pairs = data["pairs"]
    if args.offset:
        pairs = pairs[args.offset:]
    if args.max_pairs:
        pairs = pairs[: args.max_pairs]
    print(f"Running {len(pairs)} pairs with pipeline={args.pipeline}")

    rclpy.init()
    node = Node("dataset_validator")

    # Configure dispatcher based on chosen pipeline
    if args.pipeline == "cumotion":
        config = PlannerConfig(
            planning_group=args.planning_group,
            arm_joint_names=args.arm_joints,
            cumotion_pipeline_id="isaac_ros_cumotion",
            cumotion_action_topic="/cumotion/move_group",
            ompl_pipeline_id="isaac_ros_cumotion",  # disables fallback
            ompl_action_topic="/cumotion/move_group",
            planning_attempts=1,
            planning_time_regular=args.planning_timeout,
            planning_time_increased=args.planning_timeout,
        )
    elif args.pipeline == "ompl":
        config = PlannerConfig(
            planning_group=args.planning_group,
            arm_joint_names=args.arm_joints,
            cumotion_pipeline_id="ompl",
            cumotion_action_topic="/move_action",
            ompl_pipeline_id="ompl",
            ompl_action_topic="/move_action",
            planning_attempts=1,
            planning_time_regular=args.planning_timeout,
            planning_time_increased=args.planning_timeout,
        )
    else:  # fallback
        config = PlannerConfig(
            planning_group=args.planning_group,
            arm_joint_names=args.arm_joints,
            planning_attempts=3,
            planning_time_regular=min(2.0, args.planning_timeout),
            planning_time_increased=args.planning_timeout,
        )

    dispatcher = PlannerDispatcher(node, config)

    results = []
    t_start = time.time()
    for i, pair in enumerate(pairs):
        pid = pair.get("id", f"pair_{i:06d}")

        js = JointState()
        js.name = list(args.arm_joints)
        js.position = list(pair["start"])
        rs = RobotState()
        rs.joint_state = js

        t0 = time.time()
        success, traj = dispatcher.plan_to_joint_goal(rs, pair["goal"])
        elapsed = time.time() - t0

        entry = {
            "id": pid,
            "success": success,
            "n_pts": len(traj.points) if traj else 0,
            "time_s": round(elapsed, 3),
        }
        if args.save_trajectories and success and traj:
            entry["joint_names"] = list(traj.joint_names)
            entry["trajectory"] = [
                {
                    "t": pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9,
                    "positions": list(pt.positions),
                }
                for pt in traj.points
            ]
        results.append(entry)

        if (i + 1) % 10 == 0:
            passed = sum(1 for r in results if r["success"])
            print(f"  [{i + 1}/{len(pairs)}]  {passed} passed  "
                  f"({elapsed:.2f}s last, {time.time() - t_start:.0f}s total)")

    # Write output
    n_success = sum(1 for r in results if r["success"])
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out_path = args.out_dir / f"run_{args.pipeline}_{ts}.json"
    with open(out_path, "w") as f:
        json.dump({
            "timestamp": ts,
            "pipeline": args.pipeline,
            "total_pairs": len(results),
            "passed": n_success,
            "success_rate_pct": round(100 * n_success / max(len(results), 1), 1),
            "total_runtime_s": round(time.time() - t_start, 1),
            "per_pair": results,
        }, f, indent=2)

    print(f"\nSummary: {n_success}/{len(results)} passed "
          f"({100 * n_success / max(len(results), 1):.1f}%)")
    print(f"Saved: {out_path}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
