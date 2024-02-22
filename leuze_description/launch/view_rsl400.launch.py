import os

from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    # Set the paths to required files and folders.
    pkg_path = FindPackageShare(package='leuze_description').find('leuze_description')
    model_path = os.path.join(pkg_path, 'urdf', 'rsl400_scanner.urdf')
    rviz_config_file = os.path.join(pkg_path, "cfg", "display.rviz")

    # Get URDF via xacro
    robot_description_content = Command(['xacro ', model_path])

    # Launch configuration variables specific to simulation
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare a launch argument for use_rviz
    use_rviz_switch = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='If True, RViz starts automatically')

    # Declare a launch argument for use_sim_time
    use_sim_time_switch = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='If True, simulation clock instead of real time is used')

    # Pass the robot description into the robot state publisher to
    # subscribe to the joint states of the robot and publish the 3D pose of each link
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="state_publisher",
        parameters=[{'use_sim_time': use_sim_time,
                    'robot_description': robot_description_content}],
        arguments=[model_path])

    # Use Rviz to visualize the content
    rviz2 = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        use_rviz_switch,
        use_sim_time_switch,
        robot_state_publisher,
        rviz2
    ])
