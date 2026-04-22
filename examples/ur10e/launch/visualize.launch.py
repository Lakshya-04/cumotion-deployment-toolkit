"""Visualize the UR10e 6-DOF stack in RViz: URDF model + XRDF spheres + joint sliders.

No planning — this just loads the robot and shows collision spheres so you
can confirm the XRDF aligns with the URDF meshes at every configuration.

Run:
    ros2 launch $PWD/examples/ur10e/launch/visualize.launch.py

Ctrl-C cleanly terminates every child process.
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
            "ur_description package not found. Install it with:\n"
            "    sudo apt install ros-humble-ur-description"
        )

    urdf_xacro = os.path.join(ur_description_share, "urdf", "ur.urdf.xacro")
    robot_description = subprocess.check_output(
        ["xacro", urdf_xacro, "ur_type:=ur10e", "name:=ur10e"]
    ).decode()

    xrdf_path = str(example_dir / "xrdf" / "ur10e.xrdf")
    rviz_config = str(example_dir / "config" / "visualize.rviz")
    publisher_script = str(toolkit_dir / "tools" / "xrdf_sphere_publisher.py")

    rsp = _node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
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
    rviz = _node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
    )

    # Close RViz → shut everything down.
    shutdown_on_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown(reason="RViz closed by user"))],
        )
    )

    return [rsp, gui, sphere_pub, rviz, shutdown_on_rviz]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=_launch)])
