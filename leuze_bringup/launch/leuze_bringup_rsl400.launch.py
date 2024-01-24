import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory('leuze_bringup'), 'config', 'params_rsl400.yaml'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('leuze_bringup'), 'rviz', 'rsl_view.rviz'
    )

    try:
        arg4 = sys.argv[4].split(':=')
        arg5 = sys.argv[5].split(':=')
        arg6 = sys.argv[6].split(':=')
        if arg4[0] == 'sensor_ip':
            udp_ip = arg4[1]
        if arg5[0] == 'port':
            udp_port = arg5[1]
        if arg6[0] == 'topic':
            topic = arg6[1]
    except:
        udp_ip = "192.168.10.1"
        udp_port = "9990"
        topic = "scan"
        print("Too few arguments => Setting udp_ip, udp_port and topic to default values (", udp_ip, udp_port, topic, ")")

    leuze_rsl400_driver = Node(
        package='leuze_rsl_driver',
        executable='leuze_rsl400_driver',
        output='screen',
        arguments=[udp_ip, udp_port, topic],
        parameters = [config_file_path]
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

        leuze_rsl400_driver,
    ])