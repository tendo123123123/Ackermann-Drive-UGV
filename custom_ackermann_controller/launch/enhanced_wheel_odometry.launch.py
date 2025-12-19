#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('custom_ackermann_controller')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'enhanced_wheel_odometry.yaml'),
        description='Path to the enhanced wheel odometry configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Enhanced wheel odometry node
    enhanced_wheel_odometry_node = Node(
        package='custom_ackermann_controller',
        executable='enhanced_wheel_odometry',
        name='enhanced_wheel_odometry',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        enhanced_wheel_odometry_node,
    ])