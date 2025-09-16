from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lsm_localization',
            executable='lsm_localization_node',
            name='laser_scan_matcher',
            output='screen',
            parameters=[
                {'use_sim_time': False}  # Set to True if using Gazebo or bag files
            ]
        )
    ])
