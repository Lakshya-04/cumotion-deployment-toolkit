"""Visualize the UR10e + vertical lift 7-DOF stack in RViz.

Loads the composite URDF (UR10e on a prismatic lift), shows XRDF spheres,
and exposes all 7 joints via joint_state_publisher_gui so you can drive
the lift + arm and confirm the XRDF tracks.

Run:
    ros2 launch $PWD/examples/ur10e_with_lift/launch/visualize.launch.py
"""

import os
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _launch(context):
    example_dir = Path(__file__).resolve().parent.parent
    toolkit_dir = example_dir.parent.parent

    urdf_xacro = str(example_dir / "urdf" / "ur10e_with_lift.urdf.xacro")
    robot_description = subprocess.check_output(["xacro", urdf_xacro]).decode()

    xrdf_path = str(example_dir / "xrdf" / "ur10e_with_lift.xrdf")
    rviz_config = str(example_dir / "config" / "visualize.rviz")
    publisher_script = str(toolkit_dir / "tools" / "xrdf_sphere_publisher.py")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            executable=publisher_script,
            name="xrdf_sphere_publisher",
            output="screen",
            parameters=[{"xrdf_path": xrdf_path}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([OpaqueFunction(function=_launch)])
