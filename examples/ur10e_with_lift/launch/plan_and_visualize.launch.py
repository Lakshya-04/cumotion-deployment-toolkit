"""UR10e + lift 7-DOF: visualization + cuMotion planner stack + goal ghost.

Same pattern as the 6-DOF example: current robot + translucent goal ghost +
XRDF spheres + cuMotion, with clean shutdown on Ctrl-C.

In another terminal:
    python3 tools/send_joint_goal.py \\
        --planning-group arm_with_lift \\
        --joint-names lift shoulder_pan_joint shoulder_lift_joint elbow_joint \\
                      wrist_1_joint wrist_2_joint wrist_3_joint \\
        --start 0.2 0 -1.57 1.57 0 1.57 0 \\
        --goal  1.1 1.0 -0.8 1.2 -0.5 1.0 0.5 \\
        --publish-goal-ghost --replay
"""

import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


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

    urdf_xacro = str(example_dir / "urdf" / "ur10e_with_lift.urdf.xacro")
    robot_description = subprocess.check_output(["xacro", urdf_xacro]).decode()

    xrdf_path = str(example_dir / "xrdf" / "ur10e_with_lift.xrdf")
    rviz_config = str(example_dir / "config" / "plan_and_visualize.rviz")
    cumotion_params = str(example_dir / "config" / "cumotion_params.yaml")
    publisher_script = str(toolkit_dir / "tools" / "xrdf_sphere_publisher.py")

    rsp = _node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
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
    gui = _node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )
    sphere_pub = _node(
        executable=publisher_script,
        name="xrdf_sphere_publisher",
        parameters=[{"xrdf_path": xrdf_path}],
    )
    cumotion = _node(
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

    shutdown_on_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason="RViz closed by user"))],
        )
    )

    return [rsp, goal_rsp, goal_anchor, gui, sphere_pub, cumotion, rviz, shutdown_on_rviz]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=_launch)])
