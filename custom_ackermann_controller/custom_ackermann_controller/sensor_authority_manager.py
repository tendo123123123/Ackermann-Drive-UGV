#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import yaml
from std_msgs.msg import Float64, Float64MultiArray, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from collections import deque
import time
import os

class SensorAuthorityManagerNode(Node):
    def __init__(self):
        super().__init__('sensor_authority_manager')
        
        # Parameters
        self.declare_parameter('config_file_path', '')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('min_confidence_threshold', 0.1)
        self.declare_parameter('max_confidence_threshold', 1.0)
        self.declare_parameter('wheel_odom_weight', 0.4)
        self.declare_parameter('scan_match_weight', 0.4)
        self.declare_parameter('imu_weight', 0.2)
        self.declare_parameter('enable_dynamic_weighting', True)
        self.declare_parameter('enable_ekf_reconfiguration', False)
        
        # Get parameters
        self.config_file_path = self.get_parameter('config_file_path').value
        self.update_rate = self.get_parameter('update_rate').value
        self.min_confidence = self.get_parameter('min_confidence_threshold').value
        self.max_confidence = self.get_parameter('max_confidence_threshold').value
        self.enable_dynamic_weighting = self.get_parameter('enable_dynamic_weighting').value
        self.enable_ekf_reconfig = self.get_parameter('enable_ekf_reconfiguration').value
        
        # Initial sensor weights
        self.wheel_odom_weight = self.get_parameter('wheel_odom_weight').value
        self.scan_match_weight = self.get_parameter('scan_match_weight').value
        self.imu_weight = self.get_parameter('imu_weight').value
        
        # Normalize weights
        total_weight = self.wheel_odom_weight + self.scan_match_weight + self.imu_weight
        self.wheel_odom_weight /= total_weight
        self.scan_match_weight /= total_weight
        self.imu_weight /= total_weight
        
        # Sensor quality metrics (moving averages)
        self.window_size = 20
        self.wheel_odom_confidence = deque(maxlen=self.window_size)
        self.wheel_slip_status = deque(maxlen=self.window_size)
        self.scan_match_quality = deque(maxlen=self.window_size)
        self.scan_match_confidence = deque(maxlen=self.window_size)
        self.imu_calibration_status = deque(maxlen=self.window_size)
        
        # Current authority levels (0.0 to 1.0)
        self.current_wheel_authority = self.wheel_odom_weight
        self.current_scan_authority = self.scan_match_weight
        self.current_imu_authority = self.imu_weight
        
        # Environmental state
        self.environment_type = "unknown"  # indoor, outdoor, mixed
        self.motion_state = "stationary"   # stationary, slow, fast, turning
        self.last_velocity = 0.0
        self.last_angular_velocity = 0.0
        
        # Authority adjustment factors
        self.indoor_bias = {"wheel": 1.2, "scan": 0.8, "imu": 1.0}
        self.outdoor_bias = {"wheel": 0.8, "scan": 1.2, "imu": 1.0}
        self.turning_bias = {"wheel": 0.7, "scan": 1.0, "imu": 1.3}
        self.fast_motion_bias = {"wheel": 1.3, "scan": 0.7, "imu": 1.0}
        
        # Publishers
        self.authority_status_pub = self.create_publisher(
            Float64MultiArray, '/sensor_authority/status', 10
        )
        self.wheel_authority_pub = self.create_publisher(
            Float64, '/sensor_authority/wheel_odometry', 10
        )
        self.scan_authority_pub = self.create_publisher(
            Float64, '/sensor_authority/scan_matching', 10
        )
        self.imu_authority_pub = self.create_publisher(
            Float64, '/sensor_authority/imu', 10
        )
        self.environment_pub = self.create_publisher(
            Float64MultiArray, '/sensor_authority/environment', 10
        )
        self.fused_odom_pub = self.create_publisher(
            Odometry, '/sensor_authority/fused_odometry', 10
        )
        
        # Subscribers
        self.wheel_confidence_sub = self.create_subscription(
            Float64, '/enhanced_wheel_odometry/confidence', self.wheel_confidence_callback, 10
        )
        self.wheel_slip_sub = self.create_subscription(
            Bool, '/enhanced_wheel_odometry/slip_detected', self.wheel_slip_callback, 10
        )
        self.scan_quality_sub = self.create_subscription(
            Float64, '/scan_matching/quality', self.scan_quality_callback, 10
        )
        self.scan_confidence_sub = self.create_subscription(
            Float64, '/scan_matching/confidence', self.scan_confidence_callback, 10
        )
        self.imu_calibration_sub = self.create_subscription(
            Float64MultiArray, '/imu/calibration_status', self.imu_calibration_callback, 10
        )
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_odometry_callback, 10
        )
        
        # Store latest odometry from each source for fusion
        self.latest_wheel_odom = None
        self.latest_scan_odom = None
        self.latest_imu_data = None
        
        self.wheel_odom_sub = self.create_subscription(
            Odometry, '/enhanced_wheel_odometry/odom', self.wheel_odom_callback, 10
        )
        self.scan_odom_sub = self.create_subscription(
            Odometry, '/scan_matching_odometry/odom', self.scan_odom_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_corrected', self.imu_callback, 10
        )
        
        # Timer for authority updates
        self.update_timer = self.create_timer(1.0 / self.update_rate, self.update_authority)
        
        self.get_logger().info('Sensor Authority Manager Node initialized')

    def wheel_confidence_callback(self, msg):
        """Handle wheel odometry confidence updates"""
        self.wheel_odom_confidence.append(msg.data)

    def wheel_slip_callback(self, msg):
        """Handle wheel slip detection updates"""
        self.wheel_slip_status.append(1.0 if msg.data else 0.0)

    def scan_quality_callback(self, msg):
        """Handle scan matching quality updates"""
        self.scan_match_quality.append(msg.data)

    def scan_confidence_callback(self, msg):
        """Handle scan matching confidence updates"""
        self.scan_match_confidence.append(msg.data)

    def imu_calibration_callback(self, msg):
        """Handle IMU calibration status updates"""
        if len(msg.data) >= 4:
            # Average of different calibration metrics
            calibration_score = np.mean(msg.data[:3])  # Exclude stationary flag
            self.imu_calibration_status.append(calibration_score)

    def wheel_odom_callback(self, msg):
        """Store latest wheel odometry"""
        self.latest_wheel_odom = msg

    def scan_odom_callback(self, msg):
        """Store latest scan matching odometry"""
        self.latest_scan_odom = msg

    def imu_callback(self, msg):
        """Store latest IMU data"""
        self.latest_imu_data = msg

    def ekf_odometry_callback(self, msg):
        """Handle EKF odometry for motion state analysis"""
        # Extract velocity information
        linear_vel = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        angular_vel = abs(msg.twist.twist.angular.z)
        
        self.last_velocity = linear_vel
        self.last_angular_velocity = angular_vel
        
        # Determine motion state
        if linear_vel < 0.1 and angular_vel < 0.1:
            self.motion_state = "stationary"
        elif angular_vel > 0.3:
            self.motion_state = "turning"
        elif linear_vel > 1.0:
            self.motion_state = "fast"
        else:
            self.motion_state = "slow"

    def detect_environment(self):
        """Detect environment type based on sensor patterns"""
        if not (self.scan_match_quality and self.wheel_odom_confidence):
            return "unknown"
        
        # Get recent averages
        avg_scan_quality = np.mean(list(self.scan_match_quality))
        avg_wheel_confidence = np.mean(list(self.wheel_odom_confidence))
        
        # Simple heuristics for environment detection
        if avg_scan_quality > 0.7:
            # High scan quality suggests structured environment (indoor)
            self.environment_type = "indoor"
        elif avg_scan_quality < 0.3:
            # Low scan quality suggests open environment (outdoor)
            self.environment_type = "outdoor"
        else:
            # Mixed quality suggests transitional environment
            self.environment_type = "mixed"
        
        return self.environment_type

    def calculate_sensor_authority(self):
        """Calculate dynamic sensor authority based on quality metrics"""
        if not self.enable_dynamic_weighting:
            return self.wheel_odom_weight, self.scan_match_weight, self.imu_weight
        
        # Get current quality metrics (with fallbacks)
        wheel_conf = np.mean(list(self.wheel_odom_confidence)) if self.wheel_odom_confidence else 0.5
        wheel_slip = np.mean(list(self.wheel_slip_status)) if self.wheel_slip_status else 0.0
        scan_qual = np.mean(list(self.scan_match_quality)) if self.scan_match_quality else 0.5
        scan_conf = np.mean(list(self.scan_match_confidence)) if self.scan_match_confidence else 0.5
        imu_calib = np.mean(list(self.imu_calibration_status)) if self.imu_calibration_status else 0.5
        
        # Calculate base authority from quality metrics
        wheel_base = wheel_conf * (1.0 - wheel_slip)  # Reduce authority if slipping
        scan_base = (scan_qual + scan_conf) / 2.0     # Average quality and confidence
        imu_base = imu_calib                          # Calibration status
        
        # Apply environmental biases
        env = self.detect_environment()
        env_bias = {"indoor": self.indoor_bias, "outdoor": self.outdoor_bias, "mixed": {"wheel": 1.0, "scan": 1.0, "imu": 1.0}}
        
        wheel_authority = wheel_base * env_bias.get(env, {"wheel": 1.0})["wheel"]
        scan_authority = scan_base * env_bias.get(env, {"scan": 1.0})["scan"]
        imu_authority = imu_base * env_bias.get(env, {"imu": 1.0})["imu"]
        
        # Apply motion state biases
        if self.motion_state == "turning":
            wheel_authority *= self.turning_bias["wheel"]
            scan_authority *= self.turning_bias["scan"]
            imu_authority *= self.turning_bias["imu"]
        elif self.motion_state == "fast":
            wheel_authority *= self.fast_motion_bias["wheel"]
            scan_authority *= self.fast_motion_bias["scan"]
            imu_authority *= self.fast_motion_bias["imu"]
        
        # Normalize authorities
        total_authority = wheel_authority + scan_authority + imu_authority
        if total_authority > 0:
            wheel_authority /= total_authority
            scan_authority /= total_authority
            imu_authority /= total_authority
        else:
            # Fallback to equal weights
            wheel_authority = scan_authority = imu_authority = 1.0/3.0
        
        # Apply min/max constraints
        wheel_authority = np.clip(wheel_authority, self.min_confidence, self.max_confidence)
        scan_authority = np.clip(scan_authority, self.min_confidence, self.max_confidence)
        imu_authority = np.clip(imu_authority, self.min_confidence, self.max_confidence)
        
        return wheel_authority, scan_authority, imu_authority

    def publish_fused_odometry(self, wheel_auth, scan_auth, imu_auth):
        """Publish sensor-authority-weighted fused odometry"""
        if not (self.latest_wheel_odom and self.latest_scan_odom):
            return
        
        # Create fused odometry message
        fused_msg = Odometry()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = "odom"
        fused_msg.child_frame_id = "base_footprint"
        
        # Weight position (primarily from scan matching)
        if scan_auth > 0.1:
            fused_msg.pose.pose.position = self.latest_scan_odom.pose.pose.position
            fused_msg.pose.pose.orientation = self.latest_scan_odom.pose.pose.orientation
        else:
            # Fallback to wheel odometry if scan matching is unreliable
            fused_msg.pose.pose.position = self.latest_wheel_odom.pose.pose.position
            fused_msg.pose.pose.orientation = self.latest_wheel_odom.pose.pose.orientation
        
        # Weight velocity (primarily from wheel odometry)
        if wheel_auth > 0.1:
            fused_msg.twist.twist = self.latest_wheel_odom.twist.twist
        else:
            # Fallback to zero velocity
            fused_msg.twist.twist.linear.x = 0.0
            fused_msg.twist.twist.linear.y = 0.0
            fused_msg.twist.twist.angular.z = 0.0
        
        # Weighted covariance (simplified approach)
        pose_var = 0.01 / max(scan_auth, 0.1)
        twist_var = 0.01 / max(wheel_auth, 0.1)
        
        # Set pose covariance
        fused_msg.pose.covariance = [0.0] * 36
        fused_msg.pose.covariance[0] = pose_var   # x
        fused_msg.pose.covariance[7] = pose_var   # y
        fused_msg.pose.covariance[35] = pose_var  # yaw
        
        # Set twist covariance
        fused_msg.twist.covariance = [0.0] * 36
        fused_msg.twist.covariance[0] = twist_var   # vx
        fused_msg.twist.covariance[7] = twist_var   # vy
        fused_msg.twist.covariance[35] = twist_var  # vyaw
        
        self.fused_odom_pub.publish(fused_msg)

    def update_authority(self):
        """Update sensor authority and publish status"""
        # Calculate new authority levels
        wheel_auth, scan_auth, imu_auth = self.calculate_sensor_authority()
        
        # Update current authority levels
        self.current_wheel_authority = wheel_auth
        self.current_scan_authority = scan_auth
        self.current_imu_authority = imu_auth
        
        # Publish individual authority levels
        wheel_msg = Float64()
        wheel_msg.data = wheel_auth
        self.wheel_authority_pub.publish(wheel_msg)
        
        scan_msg = Float64()
        scan_msg.data = scan_auth
        self.scan_authority_pub.publish(scan_msg)
        
        imu_msg = Float64()
        imu_msg.data = imu_auth
        self.imu_authority_pub.publish(imu_msg)
        
        # Publish combined status
        status_msg = Float64MultiArray()
        status_msg.data = [wheel_auth, scan_auth, imu_auth]
        self.authority_status_pub.publish(status_msg)
        
        # Publish environment information
        env_msg = Float64MultiArray()
        env_code = {"unknown": 0, "indoor": 1, "outdoor": 2, "mixed": 3}.get(self.environment_type, 0)
        motion_code = {"stationary": 0, "slow": 1, "fast": 2, "turning": 3}.get(self.motion_state, 0)
        env_msg.data = [env_code, motion_code, self.last_velocity, self.last_angular_velocity]
        self.environment_pub.publish(env_msg)
        
        # Publish fused odometry
        self.publish_fused_odometry(wheel_auth, scan_auth, imu_auth)
        
        # Log authority changes (if significant)
        if abs(wheel_auth - self.wheel_odom_weight) > 0.1:
            self.get_logger().info(
                f'Authority update - Wheel: {wheel_auth:.2f}, Scan: {scan_auth:.2f}, '
                f'IMU: {imu_auth:.2f}, Env: {self.environment_type}, Motion: {self.motion_state}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = SensorAuthorityManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()