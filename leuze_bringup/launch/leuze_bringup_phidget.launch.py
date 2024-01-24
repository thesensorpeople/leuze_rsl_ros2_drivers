import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config_file = os.path.join(
        get_package_share_directory('leuze_bringup'), 'rviz', 'rsl_view.rviz'
    )

    leuze_phidget_driver = Node(
        package='leuze_phidget_driver',
        executable='leuze_phidget_driver_node',
        output='screen'
    )

    # Launch RViz
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # launch_actions = [rviz2]   #use this instead of the next line to let rviz2 start automatically together with the laser scanner driver
    launch_actions = []

    return LaunchDescription([
        TimerAction(
            period=1.0,
            actions=launch_actions
        ),

        leuze_phidget_driver,
    ])


