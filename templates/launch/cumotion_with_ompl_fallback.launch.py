"""Production launch: cuMotion + OMPL fallback pipeline.

Loads BOTH pipelines so your dispatcher can switch between them at runtime
without relaunching.

Replace the marked <TODO> sections with your robot specifics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# <TODO> pip install moveit_configs_utils (usually part of MoveIt2)
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    # ── Launch arguments ─────────────────────────────────────────────────
    robot_description_arg = DeclareLaunchArgument(
        "robot_description_package",
        default_value="<TODO_robot_description_package>",
        description="ROS package containing your URDF/SRDF",
    )
    xrdf_arg = DeclareLaunchArgument(
        "xrdf_path",
        default_value="<TODO_absolute_path_to_robot.xrdf>",
        description="Path to XRDF file",
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value="<TODO_absolute_path_to_robot.urdf>",
        description="Path to URDF file (for cuMotion)",
    )
    tool_frame_arg = DeclareLaunchArgument(
        "tool_frame", default_value="tool0", description="End-effector frame"
    )

    # ── MoveIt config with BOTH pipelines ────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="<TODO_robot_name>",
            package_name=LaunchConfiguration("robot_description_package"),
        )
        .robot_description(file_path="config/<TODO>.urdf.xacro")
        .robot_description_semantic(file_path="config/<TODO>.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["isaac_ros_cumotion", "ompl"],
            default_planning_pipeline="isaac_ros_cumotion",
        )
        .to_moveit_configs()
    )

    # ── move_group node (primary planner router) ─────────────────────────
    # Critical: use Bullet collision detector to avoid FCL thread-race SIGSEGV
    # in MoveIt2 Humble. If you're on MoveIt2 Iron+ this may not be needed.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            # <TODO> uncomment if you hit FCL segfaults on MoveIt2 Humble
            # {"collision_detector": "Bullet"},
        ],
    )

    # ── cuMotion action server ────────────────────────────────────────────
    # Serves /cumotion/move_group — direct action client target when you
    # want to skip the MoveIt plugin's 5s timeout.
    cumotion_node = Node(
        package="isaac_ros_cumotion",
        executable="cumotion_planner_node",
        name="cumotion_planner",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("<TODO_your_pkg>"),
                "config",
                "cumotion_params.yaml",
            ]),
            {
                "robot": LaunchConfiguration("xrdf_path"),
                "urdf_path": LaunchConfiguration("urdf_path"),
                "tool_frame": LaunchConfiguration("tool_frame"),
            },
        ],
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    return LaunchDescription([
        robot_description_arg,
        xrdf_arg,
        urdf_arg,
        tool_frame_arg,
        rsp_node,
        move_group_node,
        cumotion_node,
    ])
