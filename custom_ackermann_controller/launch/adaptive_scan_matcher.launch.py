import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package path
    pkg_share = FindPackageShare('custom_ackermann_controller')
    
    # Configuration file path
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'adaptive_scan_matcher.yaml'
    ])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Laser scan topic'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Robot base frame'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame'
    )
    
    # Adaptive scan matcher node
    adaptive_scan_matcher_node = Node(
        package='custom_ackermann_controller',
        executable='adaptive_scan_matcher',
        name='adaptive_scan_matcher',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
            }
        ],
        output='screen',
        remappings=[
            ('/scan', LaunchConfiguration('scan_topic')),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        scan_topic_arg,
        base_frame_arg,
        odom_frame_arg,
        adaptive_scan_matcher_node,
    ])