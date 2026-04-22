"""Visualize the UR10e 6-DOF stack in RViz: URDF model + XRDF spheres + joint sliders.

No planning — this just loads the robot and shows collision spheres so you
can confirm the XRDF aligns with the URDF meshes at every configuration.

Drag the joint sliders in the joint_state_publisher_gui panel; the spheres
should track the meshes exactly.

Run:
    ros2 launch $PWD/examples/ur10e/launch/visualize.launch.py
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _launch(context):
    example_dir = Path(__file__).resolve().parent.parent
    toolkit_dir = example_dir.parent.parent

    # Public UR description — resolves URDF at runtime via xacro.
    # For simple visualization we use a pre-rendered URDF; if you want the full
    # xacro, pipe it through `xacro` first.
    # Minimal: if you don't have the URDF on disk, ament provides it.
    try:
        ur_description_share = get_package_share_directory("ur_description")
    except Exception:
        raise RuntimeError(
            "ur_description package not found. Install it with:\n"
            "    sudo apt install ros-humble-ur-description"
        )

    urdf_xacro = os.path.join(ur_description_share, "urdf", "ur.urdf.xacro")

    # Invoke xacro to produce the URDF string
    import subprocess
    robot_description = subprocess.check_output(
        ["xacro", urdf_xacro, "ur_type:=ur10e", "name:=ur10e"]
    ).decode()

    xrdf_path = str(example_dir / "xrdf" / "ur10e.xrdf")
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
