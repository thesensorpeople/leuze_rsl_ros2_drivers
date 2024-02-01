import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory("leuze_description"))
    rviz_config_file = os.path.join(pkg_path, "cfg", "display.rviz")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("leuze_description"), "urdf", "rsl400_scanner.urdf.xacro"]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Pass the robot description into the robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Use Rviz to visualize the content
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    launch_actions = [rviz2]

    return LaunchDescription([
        TimerAction(
            period=1.0,
            actions=launch_actions
        ),
        robot_state_publisher,
    ])
