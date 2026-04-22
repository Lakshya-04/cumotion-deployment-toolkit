"""UR10e launch: cuMotion + OMPL fallback.

Loads both pipelines so the dispatcher can switch at runtime.

Requirements:
  apt install ros-humble-ur-description ros-humble-ur-moveit-config
  pip install isaac-ros-cumotion  (or build from source)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    example_dir = Path(__file__).resolve().parent.parent

    xrdf_arg = DeclareLaunchArgument(
        "xrdf_path",
        default_value=str(example_dir / "xrdf" / "ur10e.xrdf"),
        description="Path to UR10e XRDF",
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=os.path.join(
            get_package_share_directory("ur_description"),
            "urdf",
            "ur.urdf.xacro",
        ),
        description="Path to URDF",
    )

    # NOTE: This example assumes you have a companion package providing
    # full MoveIt config (URDF+SRDF+controllers) for UR10e. The public
    # `ur_moveit_config` package works. Adapt paths below for your setup.

    cumotion_params = str(example_dir / "config" / "cumotion_params.yaml")

    cumotion_node = Node(
        package="isaac_ros_cumotion",
        executable="cumotion_planner_node",
        name="cumotion_planner",
        output="screen",
        parameters=[
            cumotion_params,
            {
                "robot": LaunchConfiguration("xrdf_path"),
                "urdf_path": LaunchConfiguration("urdf_path"),
            },
        ],
    )

    # For move_group + OMPL pipeline, use the standard UR MoveIt config:
    # ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e \\
    #   use_sim_time:=false launch_rviz:=true
    #
    # Then run THIS launch file for the cumotion node alongside it.

    return LaunchDescription([
        xrdf_arg,
        urdf_arg,
        cumotion_node,
    ])
