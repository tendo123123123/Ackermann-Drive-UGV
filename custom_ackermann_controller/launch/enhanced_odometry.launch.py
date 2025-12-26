#!/usr/bin/env python3
"""
Enhanced Odometry Launch File
Launches Custom Laser Scan Matcher + EKF fusion for robust localization

This launch file starts:
1. Custom Laser Scan Matcher Node - estimates motion from laser scans using ICP
2. Robot Localization EKF Node - fuses wheel odom + IMU + laser odom

Architecture:
/odom (wheel encoders) + /imu + /odom_laser (laser scan matcher) → EKF → /odometry/filtered

Key Features:
- Pure laser-based odometry without TF publishing
- Configurable scan matching parameters
- Robust outlier rejection
- Real-time performance optimized
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_custom_ackermann = get_package_share_directory('custom_ackermann_controller')
    
    # Configuration files
    laser_matcher_config_file = os.path.join(pkg_custom_ackermann, 'config', 'adaptive_scan_matcher.yaml')
    ekf_config_file = os.path.join(pkg_custom_ackermann, 'config', 'robot_localization.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for all nodes'
    )
    
    use_laser_odom_arg = DeclareLaunchArgument(
        'use_laser_odom',
        default_value='true',
        description='Whether to use laser scan matcher for odometry'
    )
    
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', 
        default_value='true',
        description='Whether to use EKF for sensor fusion'
    )
    
    laser_topic_arg = DeclareLaunchArgument(
        'laser_topic',
        default_value='/scan',
        description='Laser scan topic for scan matcher'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output'
    )
    
    odom_laser_topic_arg = DeclareLaunchArgument(
        'odom_laser_topic',
        default_value='/odom_laser',
        description='Output topic for laser odometry'
    )
    
    # ===== CUSTOM LASER SCAN MATCHER NODE =====
    laser_scan_matcher_node = Node(
        package='custom_ackermann_controller',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[laser_matcher_config_file, {
            'subscribe_scan_topic': LaunchConfiguration('laser_topic'),
            'publish_odom_topic': LaunchConfiguration('odom_laser_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time')  # CRITICAL: Use simulation time
        }],
        condition=IfCondition(LaunchConfiguration('use_laser_odom')),
        respawn=True,
        respawn_delay=2.0
    )
    
    # ===== ROBOT LOCALIZATION EKF NODE =====
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {
            'print_diagnostics': LaunchConfiguration('debug'),
            'use_sim_time': LaunchConfiguration('use_sim_time')  # CRITICAL: Use simulation time
        }],
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        respawn=True,
        respawn_delay=2.0,
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,              # CRITICAL: Add use_sim_time argument
        use_laser_odom_arg,
        use_ekf_arg, 
        laser_topic_arg,
        odom_laser_topic_arg,
        debug_arg,
        
        # Core nodes
        laser_scan_matcher_node,
        robot_localization_node
    ])