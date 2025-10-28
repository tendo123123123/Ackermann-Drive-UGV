#!/usr/bin/env python3
"""
Smooth Velocity Controller Launch File

Launches the GUI-based smooth velocity controller that allows
simultaneous linear and angular velocity control without a joystick.

Usage:
    ros2 launch custom_ackermann_controller smooth_teleop.launch.py

Controls:
    - W/S: Increase/Decrease linear velocity (forward/backward)
    - A/D: Increase/Decrease angular velocity (turn left/right) 
    - Spacebar: Emergency stop
    - Q: Quit
    - Or use the GUI sliders for precise control
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_custom_ackermann = get_package_share_directory('custom_ackermann_controller')
    
    # Configuration file
    config_file = os.path.join(pkg_custom_ackermann, 'config', 'smooth_velocity_controller.yaml')
    
    # Smooth velocity controller node
    smooth_velocity_controller_node = Node(
        package='custom_ackermann_controller',
        executable='smooth_velocity_controller',
        name='smooth_velocity_controller',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ],
        # prefix='xterm -e',  # Run in separate terminal window so GUI works properly
    )
    
    return LaunchDescription([
        smooth_velocity_controller_node
    ])