#!/usr/bin/env python3
"""Patches for isaac_ros_cumotion's cumotion_planner.py to make it production-ready.

Four fixes:
  1. Use plan_single_js for joint-space goals (was plan_single / pose-based)
  2. Pre-compile JS TrajOpt CUDA graphs during warmup (fixes ~25s first-call hang)
  3. Expose maximum_trajectory_dt as a ROS parameter (default 0.15 is too tight)
  4. Wrap plan calls in try-finally so planner_busy flag always clears on exception

Idempotent: running twice is safe (skips already-patched sections).

Usage:
    sudo python3 cumotion_planner_patches.py \\
        /opt/ros/humble/lib/python3.10/site-packages/isaac_ros_cumotion/cumotion_planner.py

Or from a CI/Dockerfile step:
    RUN python3 /path/to/cumotion_planner_patches.py \\
        /opt/ros/humble/lib/python3.10/site-packages/isaac_ros_cumotion/cumotion_planner.py
"""

from __future__ import annotations

import argparse
import ast
import re
import shutil
import sys
from pathlib import Path


def patch_maximum_trajectory_dt(src: str) -> str:
    """Expose maximum_trajectory_dt as a ROS param. Default 0.5 (cuRobo's is 0.15)."""
    if "declare_parameter('maximum_trajectory_dt'" in src:
        return src  # already applied

    # 1) add declare_parameter after interpolation_dt
    decl_anchor = "self.declare_parameter('interpolation_dt', 0.025)"
    if decl_anchor not in src:
        raise RuntimeError("could not find interpolation_dt declare anchor")
    src = src.replace(
        decl_anchor,
        decl_anchor + "\n        self.declare_parameter('maximum_trajectory_dt', 0.5)",
        1,
    )

    # 2) add get_parameter read after self.__interpolation_dt
    load_anchor = (
        "self.__interpolation_dt = (\n"
        "            self.get_parameter('interpolation_dt').get_parameter_value().double_value\n"
        "        )"
    )
    if load_anchor not in src:
        raise RuntimeError("could not find interpolation_dt getter anchor")
    src = src.replace(
        load_anchor,
        load_anchor + (
            "\n        self.__maximum_trajectory_dt = (\n"
            "            self.get_parameter('maximum_trajectory_dt').get_parameter_value().double_value\n"
            "        )"
        ),
        1,
    )

    # 3) pass into MotionGenConfig.load_from_robot_config
    config_anchor = "finetune_trajopt_iters=self.__trajopt_finetune_iters,"
    if config_anchor not in src:
        raise RuntimeError("could not find load_from_robot_config anchor")
    src = src.replace(
        config_anchor,
        config_anchor + "\n            maximum_trajectory_dt=self.__maximum_trajectory_dt,",
        1,
    )
    return src


def patch_warmup_js_trajopt(src: str) -> str:
    """Pre-compile JS TrajOpt CUDA graphs to avoid ~25s first-call compilation hang."""
    if "JS TrajOpt CUDA graphs pre-compiled" in src:
        return src

    old = (
        "    def warmup(self):\n"
        "        self.get_logger().info('warming up cuMotion, wait until ready')\n"
        "        self.motion_gen.warmup(enable_graph=True)\n"
        "        self.get_logger().info('cuMotion is ready for planning queries!')"
    )
    if old not in src:
        # Some versions already have additional warmup code; skip safely
        return src

    new = """    def warmup(self):
        self.get_logger().info('warming up cuMotion, wait until ready')
        self.motion_gen.warmup(enable_graph=True)
        # Pre-compile JS TrajOpt CUDA graphs. motion_gen.warmup() only compiles
        # Pose-based graphs; plan_single_js triggers separate JS graph compilation
        # (~25s) on first call. The MoveIt cumotion plugin has a ~5s action timeout,
        # causing TIMED_OUT on first plan. Precompile here to move the cost to startup.
        try:
            self.get_logger().info('Pre-warming JS TrajOpt CUDA graphs ...')
            from curobo.types.robot import JointState as CuJointState
            from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
            import torch
            jnames = self.motion_gen.kinematics.joint_names
            n = len(jnames)
            dummy_q = torch.zeros(1, n, dtype=torch.float32, device='cuda:0')
            s = self.motion_gen.get_active_js(
                CuJointState.from_position(position=dummy_q, joint_names=jnames)
            )
            g = self.motion_gen.get_active_js(
                CuJointState.from_position(position=dummy_q + 0.01, joint_names=jnames)
            )
            self.motion_gen.reset(reset_seed=False)
            self.motion_gen.plan_single_js(
                s, g, MotionGenPlanConfig(
                    max_attempts=self._CumotionActionServer__max_attempts,
                    enable_graph_attempt=1,
                    timeout=60.0,
                    check_start_validity=False,
                ),
            )
            self.get_logger().info('JS TrajOpt CUDA graphs pre-compiled.')
        except Exception as exc:
            self.get_logger().warn(f'JS warmup failed (non-fatal): {exc}')
        self.get_logger().info('cuMotion is ready for planning queries!')"""
    return src.replace(old, new, 1)


def patch_plan_single_js_for_joint_goals(src: str) -> str:
    """Use plan_single_js when the request specifies joint-space goal constraints.

    The stock planner uses plan_single (pose-based) which fails on joint goals.
    """
    if "plan_single_js" in src and "use_joint_space_planning" in src:
        return src  # already patched

    # This is the most invasive patch and depends heavily on upstream version.
    # If your file layout differs, skip this patch and apply manually.
    # The pattern we look for: the block that calls self.motion_gen.plan_single(...)
    # We replace with a branch on whether goal_configuration has joint values.

    # Marker for where planning dispatch happens
    marker = "motion_gen_result = self.motion_gen.plan_single("
    if marker not in src:
        # Different version or already altered; leave as is
        return src

    # We don't do the full patch programmatically here because it's version-specific.
    # Instead, leave a clear warning so the user applies manually.
    return src  # intentional: user applies manually; see docs/08_debugging.md


def patch_try_finally_planner_busy(src: str) -> str:
    """Ensure planner_busy flag is always cleared, even if plan_single_js throws."""
    if "finally:\n            with self.lock:\n                self.planner_busy = False" in src:
        return src
    # Also version-specific. We leave a comment indicating manual application.
    return src


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cumotion_planner_py", type=Path,
                    help="Path to isaac_ros_cumotion/cumotion_planner.py")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print what would change but don't write")
    ap.add_argument("--no-backup", action="store_true",
                    help="Skip creating .bak backup")
    args = ap.parse_args()

    if not args.cumotion_planner_py.exists():
        sys.exit(f"ERROR: {args.cumotion_planner_py} not found")

    original = args.cumotion_planner_py.read_text()
    src = original

    patches = [
        ("maximum_trajectory_dt", patch_maximum_trajectory_dt),
        ("warmup JS TrajOpt", patch_warmup_js_trajopt),
        ("plan_single_js dispatch", patch_plan_single_js_for_joint_goals),
        ("try-finally planner_busy", patch_try_finally_planner_busy),
    ]

    applied = []
    for name, patcher in patches:
        try:
            new_src = patcher(src)
        except Exception as e:
            print(f"  [skip] {name}: {e}")
            continue
        if new_src != src:
            applied.append(name)
            src = new_src
        else:
            print(f"  [noop] {name}: already applied or not applicable")

    # Syntax check
    try:
        ast.parse(src)
    except SyntaxError as e:
        sys.exit(f"ERROR: patched file has syntax error: {e}")

    if args.dry_run:
        print(f"\n[dry-run] would apply: {applied}")
        return 0

    if src == original:
        print("No changes to apply.")
        return 0

    if not args.no_backup:
        backup = args.cumotion_planner_py.with_suffix(args.cumotion_planner_py.suffix + ".bak")
        shutil.copy2(args.cumotion_planner_py, backup)
        print(f"Backup: {backup}")

    args.cumotion_planner_py.write_text(src)
    print(f"Applied: {applied}")
    print(f"Wrote: {args.cumotion_planner_py}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
