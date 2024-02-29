from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pymycobot import PI_BAUD, PI_PORT


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("port", default_value=PI_PORT, description="MyCobot port")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baudrate", default_value=str(PI_BAUD), description="MyCobot baud rate"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "init_qpos",
            default_value=str([0.0] * 5 + [0.785398]),
            description="MyCobot initial joint angles in radians",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_frequency",
            default_value="30",
            description="MyCobot joint_states publishing frequency",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "with_camera",
            default_value="False",
            description="Whether or not to launch realsense2_camera node",
        )
    )

    # joint_state_publisher_node = Node(
    #     package="mycobot_280pi",
    #     executable="joint_state_publisher",
    #     parameters=[
    #         {
    #             "port": LaunchConfiguration("port"),
    #             "baudrate": LaunchConfiguration("baudrate"),
    #             "init_qpos": LaunchConfiguration("init_qpos"),
    #         }
    #     ],
    # )

    driver_node = Node(
        package="mycobot_280pi",
        executable="driver_node",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "init_qpos": LaunchConfiguration("init_qpos"),
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            }
        ],
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("realsense2_camera"),
                "launch/rs_launch.py",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("with_camera")),
    )

    return LaunchDescription(declared_arguments + [driver_node, camera_launch])
