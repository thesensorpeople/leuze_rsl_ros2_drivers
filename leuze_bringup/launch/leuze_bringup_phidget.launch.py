from launch.actions import TimerAction
from launch import LaunchDescription
from launch_ros.actions import Node

# IMPORTANT: The PHIDGET driver is deprecated and will be removed in the future!


def generate_launch_description():
    print("#IMPORTANT: The PHIDGET driver is deprecated and will be removed in the future!")

    # rviz_config_file = os.path.join(
    #     get_package_share_directory('leuze_bringup'), 'rviz', 'rsl_view.rviz'
    # )

    leuze_phidget_driver = Node(
        package='leuze_phidget_driver',
        executable='leuze_phidget_driver_node',
        output='screen'
    )

    # Launch RViz
    # rviz2 = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config_file],
    # )

    # Use this instead of the next line to let rviz2 start automatically
    # together with the laser scanner driver:
    # launch_actions = [rviz2]
    launch_actions = []

    return LaunchDescription([
        TimerAction(
            period=1.0,
            actions=launch_actions
        ),

        leuze_phidget_driver,
    ])
