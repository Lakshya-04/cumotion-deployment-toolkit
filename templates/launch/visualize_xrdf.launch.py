"""Visualize the cuMotion stack — URDF robot model + XRDF collision spheres — in RViz.

Launches:
  - robot_state_publisher (with your URDF)
  - joint_state_publisher_gui (GUI sliders to move joints)
  - xrdf_sphere_publisher (MarkerArray on /xrdf_spheres)
  - rviz2 (with a preconfigured layout showing both)

No cuMotion node, no MoveIt — purely for inspecting that your XRDF spheres
align with the URDF meshes. The joint sliders let you drive the whole arm
through its range to confirm spheres track correctly at all configurations.

Adapt the defaults below for your robot.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_rviz = (
        Path(__file__).resolve().parent.parent / "config" / "visualize_xrdf.rviz"
    )

    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        description="Absolute path to the URDF file",
    )
    xrdf_arg = DeclareLaunchArgument(
        "xrdf_path",
        description="Absolute path to the XRDF file",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=str(default_rviz),
        description="Path to RViz layout",
    )
    publisher_pkg_arg = DeclareLaunchArgument(
        "publisher_package",
        default_value="",
        description=(
            "ROS 2 package that provides xrdf_sphere_publisher. Leave empty to "
            "run the Python script directly from the toolkit's tools/ folder."
        ),
    )

    # robot_state_publisher — loads URDF into /robot_description, publishes TF
    # We read the URDF file content inline via a helper node since
    # robot_state_publisher wants the XML string as a param.
    def _rsp_node(context):
        urdf_path = LaunchConfiguration("urdf_path").perform(context)
        with open(urdf_path) as f:
            robot_description = f.read()
        return [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            )
        ]

    from launch.actions import OpaqueFunction

    rsp = OpaqueFunction(function=_rsp_node)

    joint_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # Try to run xrdf_sphere_publisher as either:
    #   - a ROS 2 package executable if publisher_package is set
    #   - the standalone Python script from the toolkit's tools/ folder otherwise
    def _sphere_publisher_node(context):
        pkg = LaunchConfiguration("publisher_package").perform(context)
        xrdf = LaunchConfiguration("xrdf_path").perform(context)
        if pkg:
            return [
                Node(
                    package=pkg,
                    executable="xrdf_sphere_publisher",
                    name="xrdf_sphere_publisher",
                    output="screen",
                    parameters=[{"xrdf_path": xrdf}],
                )
            ]
        # Fall back to the standalone Python script from the toolkit
        tool_path = (
            Path(__file__).resolve().parent.parent.parent
            / "tools"
            / "xrdf_sphere_publisher.py"
        )
        return [
            Node(
                executable=str(tool_path),
                name="xrdf_sphere_publisher",
                output="screen",
                parameters=[{"xrdf_path": xrdf}],
            )
        ]

    sphere_publisher = OpaqueFunction(function=_sphere_publisher_node)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription([
        urdf_arg,
        xrdf_arg,
        rviz_arg,
        publisher_pkg_arg,
        rsp,
        joint_gui,
        sphere_publisher,
        rviz,
    ])
