from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="urdf_model",
            default_value=PathJoinSubstitution([
                FindPackageShare("mycobot_description"),
                "urdf/mycobot/mycobot_with_gripper_parallel.urdf",
            ]),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("mycobot_description"),
                "rviz/mycobot_280pi.rviz",
            ]),
        )
    )

    robot_description = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("urdf_model"),
        ])
    }

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return LaunchDescription(
        declared_arguments
        + [joint_state_publisher_gui_node, robot_state_publisher_node, rviz2_node]
    )
