#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class AckermannTwistController(Node):
    def __init__(self):
        super().__init__('ackermann_twist_controller')
        
        # Robot parameters 
        self.wheelbase = 0.9  # Distance between front and rear axles
        self.track_width = 0.67  # Distance between left and right wheels
        self.wheel_radius = 0.175  # Wheel radius in meters
        self.max_steering_angle = 0.2616  # 15 degrees in radians
        
        # Publishers for wheel velocities and steering angles
        self.front_left_wheel_pub = self.create_publisher(
            Float64MultiArray, 
            '/forward_velocity_controller/commands', 
            10
        )
        
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        
        # Subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Ackermann Twist Controller started')
        
    def cmd_vel_callback(self, msg):
        """Convert Twist message to Ackermann steering commands"""
        linear_vel = msg.linear.x  # Forward velocity (m/s)
        angular_vel = msg.angular.z  # Angular velocity (rad/s)
        
        # Calculate steering angle using Ackermann geometry
        if abs(linear_vel) < 0.001:
            # If not moving, set steering to zero
            steering_angle = 0.0
            left_wheel_vel = 0.0
            right_wheel_vel = 0.0
        else:
            if abs(angular_vel) < 0.001:
                # Going straight
                steering_angle = 0.0
                left_wheel_vel = linear_vel / self.wheel_radius
                right_wheel_vel = linear_vel / self.wheel_radius
            else:
                # Calculate turning radius
                turning_radius = linear_vel / angular_vel
                
                # Calculate steering angle (center of front axle)
                steering_angle = math.atan(self.wheelbase / turning_radius)
                
                # Clamp steering angle to limits
                steering_angle = max(-self.max_steering_angle, 
                                   min(self.max_steering_angle, steering_angle))
                
                # Calculate individual wheel velocities using Ackermann geometry
                # Inner and outer wheel have different turning radii
                if turning_radius > 0:  # Turning left
                    # Left wheels have smaller radius
                    left_radius = turning_radius - self.track_width / 2.0
                    right_radius = turning_radius + self.track_width / 2.0
                else:  # Turning right
                    left_radius = abs(turning_radius) + self.track_width / 2.0
                    right_radius = abs(turning_radius) - self.track_width / 2.0
                
                # Calculate wheel angular velocities
                left_wheel_vel = (angular_vel * left_radius) / self.wheel_radius
                right_wheel_vel = (angular_vel * right_radius) / self.wheel_radius
        
        # Calculate left and right steering angles (Ackermann geometry)
        if abs(steering_angle) > 0.001:
            # Inner wheel has larger steering angle
            cotangent = 1.0 / math.tan(abs(steering_angle))
            
            if steering_angle > 0:  # Left turn
                left_steering = math.atan(self.wheelbase / 
                                         (turning_radius - self.track_width / 2.0))
                right_steering = math.atan(self.wheelbase / 
                                          (turning_radius + self.track_width / 2.0))
            else:  # Right turn
                left_steering = -math.atan(self.wheelbase / 
                                          (abs(turning_radius) + self.track_width / 2.0))
                right_steering = -math.atan(self.wheelbase / 
                                           (abs(turning_radius) - self.track_width / 2.0))
        else:
            left_steering = 0.0
            right_steering = 0.0
        
        # Publish steering commands
        steering_msg = Float64MultiArray()
        steering_msg.data = [left_steering, right_steering]
        self.steering_pub.publish(steering_msg)
        
        # Publish wheel velocity commands (back wheels are driven)
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [left_wheel_vel, right_wheel_vel]
        self.front_left_wheel_pub.publish(velocity_msg)
        
        self.get_logger().debug(
            f'Linear: {linear_vel:.2f}, Angular: {angular_vel:.2f}, '
            f'Steering L/R: {left_steering:.2f}/{right_steering:.2f}, '
            f'Vel L/R: {left_wheel_vel:.2f}/{right_wheel_vel:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannTwistController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()