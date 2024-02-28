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
from pymycobot import PI_BAUD, PI_PORT


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("port", default_value=PI_PORT, description="MyCobot Port")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baudrate", default_value=str(PI_BAUD), description="Baud rate"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "init_qpos",
            default_value=str([0.0] * 5 + [0.785398]),
            description="Initial joint angles in radians",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="urdf_model",
            default_value=PathJoinSubstitution([
                FindPackageShare("mycobot_description"),
                "urdf/mycobot/mycobot_with_gripper_parallel.urdf",
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

    joint_state_publisher_node = Node(
        package="mycobot_280pi",
        executable="joint_state_publisher",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "init_qpos": LaunchConfiguration("init_qpos"),
            }
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    driver_node = Node(
        package="mycobot_280pi",
        executable="driver_node",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
            }
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [joint_state_publisher_node, robot_state_publisher_node, driver_node]
    )
