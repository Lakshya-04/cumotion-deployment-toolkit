"""UR10e 6-DOF: visualization + cuMotion planner stack + goal ghost.

Launches:
  - robot_state_publisher (main arm)  → TF under default namespace
  - robot_state_publisher (goal ghost) → TF prefixed with `goal_`
  - joint_state_publisher_gui for the main arm
  - xrdf_sphere_publisher with the XRDF
  - cumotion_planner_node
  - rviz2 preconfigured with: RobotModel (current) + RobotModel (goal ghost,
    translucent) + MarkerArray (XRDF spheres) + TF for the goal EE pose

Requires MoveIt + move_group launched separately (use `ur_moveit_config`).

In another terminal, send a goal with --publish-goal-ghost to see it as a
translucent ghost in RViz:
    python3 tools/send_joint_goal.py \\
        --planning-group manipulator \\
        --joint-names shoulder_pan_joint shoulder_lift_joint elbow_joint \\
                      wrist_1_joint wrist_2_joint wrist_3_joint \\
        --start 0 -1.57 1.57 0 1.57 0 \\
        --goal  1.0 -1.2 1.8 -0.5 1.2 0.5 \\
        --publish-goal-ghost --replay

Ctrl-C cleanly terminates every child process (`emulate_tty=True` +
SIGTERM/SIGKILL timeouts + an OnShutdown handler).
"""

import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


# Wrap Node constructor to apply shutdown-friendly defaults.
def _node(**kwargs):
    kwargs.setdefault("emulate_tty", True)
    kwargs.setdefault("output", "screen")
    kwargs.setdefault("sigterm_timeout", "3")
    kwargs.setdefault("sigkill_timeout", "2")
    kwargs.setdefault("respawn", False)
    return Node(**kwargs)


def _launch(context):
    example_dir = Path(__file__).resolve().parent.parent
    toolkit_dir = example_dir.parent.parent

    try:
        ur_description_share = get_package_share_directory("ur_description")
    except Exception:
        raise RuntimeError(
            "ur_description not found. Install with:\n"
            "    sudo apt install ros-humble-ur-description ros-humble-ur-moveit-config"
        )

    urdf_xacro = os.path.join(ur_description_share, "urdf", "ur.urdf.xacro")
    robot_description = subprocess.check_output(
        ["xacro", urdf_xacro, "ur_type:=ur10e", "name:=ur10e"]
    ).decode()

    xrdf_path = str(example_dir / "xrdf" / "ur10e.xrdf")
    rviz_config = str(example_dir / "config" / "plan_and_visualize.rviz")
    cumotion_params = str(example_dir / "config" / "cumotion_params.yaml")
    publisher_script = str(toolkit_dir / "tools" / "xrdf_sphere_publisher.py")

    rsp = _node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Goal ghost: same URDF, frame_prefix="goal_", listens to /goal/joint_states
    goal_rsp = _node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="goal_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"frame_prefix": "goal_"},
        ],
        remappings=[
            ("/joint_states", "/goal/joint_states"),
            ("/robot_description", "/goal/robot_description"),
        ],
    )

    # Static TF anchoring the goal ghost at the main robot's base
    goal_anchor = _node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_goal_base",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "goal_base_link",
        ],
    )

    joint_gui = _node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    sphere_publisher = _node(
        executable=publisher_script,
        name="xrdf_sphere_publisher",
        parameters=[{"xrdf_path": xrdf_path}],
    )

    cumotion_node = _node(
        package="isaac_ros_cumotion",
        executable="cumotion_planner_node",
        name="cumotion_planner",
        parameters=[
            cumotion_params,
            {"robot": xrdf_path, "urdf_path": urdf_xacro},
        ],
    )

    rviz = _node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    )

    # If RViz exits (user closes window), shut everything down cleanly.
    rviz_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason="RViz closed by user"))],
        )
    )

    return [
        rsp,
        goal_rsp,
        goal_anchor,
        joint_gui,
        sphere_publisher,
        cumotion_node,
        rviz,
        rviz_shutdown_handler,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=_launch)])
