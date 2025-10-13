from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    MAX_ALLOWED_LSM_SCAN_RANGE_arg = DeclareLaunchArgument(
            'MAX_ALLOWED_LSM_SCAN_RANGE',
            default_value='20.0'
    )

    LSM_SCAN_DOWNSAMPLE_RATE_arg = DeclareLaunchArgument(
            'LSM_SCAN_DOWNSAMPLE_RATE',
            default_value='4'
    )

    DEBUG_CSM_arg = DeclareLaunchArgument(
            'DEBUG_CSM',
            default_value='False'
    )

    PUB_DEBUG_TOPICS_arg = DeclareLaunchArgument(
            'PUB_DEBUG_TOPICS',
            default_value='False'
    )

    MAX_ALLOWED_LSM_SCAN_RANGE = LaunchConfiguration('MAX_ALLOWED_LSM_SCAN_RANGE')
    LSM_SCAN_DOWNSAMPLE_RATE = LaunchConfiguration('LSM_SCAN_DOWNSAMPLE_RATE')
    DEBUG_CSM = LaunchConfiguration('DEBUG_CSM')
    PUB_DEBUG_TOPICS = LaunchConfiguration('PUB_DEBUG_TOPICS')

    localization_node = Node(
            package='lsm_localization',
            executable='lsm_localization_node',
            name='laser_scan_matcher',
            output='screen',
            parameters=[
                {'fixed_frame': 'map'},
                {'base_frame': 'base_link'},
                {'publish_base_tf': False},
                {'publish_odom_tf': False},
                {'publish_pose': False},
                {'publish_pose_stamped': False},
                {'publish_pose_with_covariance_stamped': True},
                {'publish_predicted_pose': True},
                {'publish_measured_pose': True},
                {'use_map': True},
                {'publish_constructed_scan': True},
                {'do_compute_covariance': 1},
                {'initialpose_topic_name': 'lsm_localization/initialpose'},
                {'max_angular_correction_deg': 85.0},
                {'max_linear_correction': 2.0},
                {'max_iterations': 50},
                {'max_variance_trans': 1.0},
                {'max_variance_rot': 1.0},
                {'max_pose_delta_yaw': 1.5},
                {'max_correspondence_dist': 1.0},
                {'do_visibility_test': 1},
                {'restart': 0},
                {'sigma': 0.02},
                {'orientation_neighbourhood': 20},
                {'outliers_maxPerc': 0.99},
                {'outliers_adaptive_order': 0.95},
                {'use_corr_tricks': 1},
                {'max_allowed_range': MAX_ALLOWED_LSM_SCAN_RANGE},
                {'scan_downsample_rate': LSM_SCAN_DOWNSAMPLE_RATE},
                {'publish_debug': PUB_DEBUG_TOPICS},
                {'debug_csm': DEBUG_CSM}
            ]
        )

    return LaunchDescription([
        MAX_ALLOWED_LSM_SCAN_RANGE_arg,
        LSM_SCAN_DOWNSAMPLE_RATE_arg,
        PUB_DEBUG_TOPICS_arg,
        DEBUG_CSM_arg,
        localization_node
    ])
