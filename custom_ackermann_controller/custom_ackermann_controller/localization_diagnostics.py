#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time
import json
from collections import deque

class LocalizationDiagnosticsNode(Node):
    def __init__(self):
        super().__init__('localization_diagnostics')
        
        # Parameters
        self.declare_parameter('diagnostic_rate', 1.0)
        self.declare_parameter('window_size', 50)
        self.declare_parameter('log_to_file', False)
        self.declare_parameter('log_file_path', '/tmp/localization_diagnostics.log')
        
        # Get parameters
        self.diagnostic_rate = self.get_parameter('diagnostic_rate').value
        self.window_size = self.get_parameter('window_size').value
        self.log_to_file = self.get_parameter('log_to_file').value
        self.log_file_path = self.get_parameter('log_file_path').value
        
        # Data storage for analysis
        self.wheel_confidence_history = deque(maxlen=self.window_size)
        self.wheel_slip_history = deque(maxlen=self.window_size)
        self.scan_quality_history = deque(maxlen=self.window_size)
        self.scan_confidence_history = deque(maxlen=self.window_size)
        self.imu_calibration_history = deque(maxlen=self.window_size)
        self.authority_history = deque(maxlen=self.window_size)
        
        # Performance metrics
        self.odometry_timestamps = deque(maxlen=self.window_size)
        self.scan_timestamps = deque(maxlen=self.window_size)
        self.imu_timestamps = deque(maxlen=self.window_size)
        
        # System health indicators
        self.wheel_odom_health = "UNKNOWN"
        self.scan_match_health = "UNKNOWN"
        self.imu_health = "UNKNOWN"
        self.fusion_health = "UNKNOWN"
        self.overall_health = "UNKNOWN"
        
        # Alert conditions
        self.alerts = []
        self.performance_warnings = []
        
        # Publishers
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self.health_summary_pub = self.create_publisher(
            String, '/localization/health_summary', 10
        )
        self.performance_metrics_pub = self.create_publisher(
            Float64MultiArray, '/localization/performance_metrics', 10
        )
        self.alerts_pub = self.create_publisher(
            String, '/localization/alerts', 10
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
        self.authority_status_sub = self.create_subscription(
            Float64MultiArray, '/sensor_authority/status', self.authority_status_callback, 10
        )
        
        # Performance monitoring subscribers
        self.wheel_odom_sub = self.create_subscription(
            Odometry, '/enhanced_wheel_odometry/odom', self.wheel_odom_perf_callback, 10
        )
        self.scan_odom_sub = self.create_subscription(
            Odometry, '/scan_matching_odometry/odom', self.scan_odom_perf_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_corrected', self.imu_perf_callback, 10
        )
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_odom_callback, 10
        )
        
        # Timer for diagnostic updates
        self.diagnostic_timer = self.create_timer(
            1.0 / self.diagnostic_rate, self.publish_diagnostics
        )
        
        # Log file handle
        self.log_file = None
        if self.log_to_file:
            try:
                self.log_file = open(self.log_file_path, 'w')
                self.log_file.write("timestamp,wheel_conf,wheel_slip,scan_qual,scan_conf,imu_calib,overall_health\n")
            except Exception as e:
                self.get_logger().warn(f'Failed to open log file: {e}')
        
        self.get_logger().info('Localization Diagnostics Node initialized')

    def wheel_confidence_callback(self, msg):
        """Track wheel odometry confidence"""
        self.wheel_confidence_history.append(msg.data)

    def wheel_slip_callback(self, msg):
        """Track wheel slip events"""
        self.wheel_slip_history.append(1.0 if msg.data else 0.0)

    def scan_quality_callback(self, msg):
        """Track scan matching quality"""
        self.scan_quality_history.append(msg.data)

    def scan_confidence_callback(self, msg):
        """Track scan matching confidence"""
        self.scan_confidence_history.append(msg.data)

    def imu_calibration_callback(self, msg):
        """Track IMU calibration status"""
        if len(msg.data) >= 3:
            calibration_score = np.mean(msg.data[:3])
            self.imu_calibration_history.append(calibration_score)

    def authority_status_callback(self, msg):
        """Track sensor authority distribution"""
        if len(msg.data) >= 3:
            self.authority_history.append(msg.data[:3])

    def wheel_odom_perf_callback(self, msg):
        """Monitor wheel odometry performance"""
        current_time = time.time()
        self.odometry_timestamps.append(current_time)

    def scan_odom_perf_callback(self, msg):
        """Monitor scan matching performance"""
        current_time = time.time()
        self.scan_timestamps.append(current_time)

    def imu_perf_callback(self, msg):
        """Monitor IMU performance"""
        current_time = time.time()
        self.imu_timestamps.append(current_time)

    def ekf_odom_callback(self, msg):
        """Monitor EKF fusion output"""
        # Could add consistency checks here
        pass

    def calculate_data_rate(self, timestamps):
        """Calculate data rate from timestamp history"""
        if len(timestamps) < 2:
            return 0.0
        
        time_span = timestamps[-1] - timestamps[0]
        if time_span <= 0:
            return 0.0
        
        return (len(timestamps) - 1) / time_span

    def assess_component_health(self):
        """Assess health of each localization component"""
        # Wheel odometry health
        if self.wheel_confidence_history:
            avg_wheel_conf = np.mean(list(self.wheel_confidence_history))
            slip_rate = np.mean(list(self.wheel_slip_history)) if self.wheel_slip_history else 0.0
            
            if avg_wheel_conf > 0.7 and slip_rate < 0.1:
                self.wheel_odom_health = "OK"
            elif avg_wheel_conf > 0.5 and slip_rate < 0.3:
                self.wheel_odom_health = "WARN"
            else:
                self.wheel_odom_health = "ERROR"
        
        # Scan matching health
        if self.scan_quality_history and self.scan_confidence_history:
            avg_scan_qual = np.mean(list(self.scan_quality_history))
            avg_scan_conf = np.mean(list(self.scan_confidence_history))
            
            if avg_scan_qual > 0.6 and avg_scan_conf > 0.6:
                self.scan_match_health = "OK"
            elif avg_scan_qual > 0.3 and avg_scan_conf > 0.3:
                self.scan_match_health = "WARN"
            else:
                self.scan_match_health = "ERROR"
        
        # IMU health
        if self.imu_calibration_history:
            avg_imu_calib = np.mean(list(self.imu_calibration_history))
            
            if avg_imu_calib > 0.8:
                self.imu_health = "OK"
            elif avg_imu_calib > 0.5:
                self.imu_health = "WARN"
            else:
                self.imu_health = "ERROR"
        
        # Overall fusion health
        component_scores = {
            "OK": 2,
            "WARN": 1,
            "ERROR": 0,
            "UNKNOWN": 0
        }
        
        total_score = (component_scores[self.wheel_odom_health] +
                      component_scores[self.scan_match_health] +
                      component_scores[self.imu_health])
        
        if total_score >= 5:
            self.overall_health = "OK"
        elif total_score >= 3:
            self.overall_health = "WARN"
        else:
            self.overall_health = "ERROR"

    def generate_alerts(self):
        """Generate alerts based on system state"""
        self.alerts.clear()
        self.performance_warnings.clear()
        
        # Performance alerts
        wheel_rate = self.calculate_data_rate(list(self.odometry_timestamps))
        scan_rate = self.calculate_data_rate(list(self.scan_timestamps))
        imu_rate = self.calculate_data_rate(list(self.imu_timestamps))
        
        if wheel_rate < 10:  # Expected ~50 Hz
            self.performance_warnings.append(f"Low wheel odometry rate: {wheel_rate:.1f} Hz")
        
        if scan_rate < 5:   # Expected ~10 Hz
            self.performance_warnings.append(f"Low scan matching rate: {scan_rate:.1f} Hz")
        
        if imu_rate < 50:   # Expected ~100 Hz
            self.performance_warnings.append(f"Low IMU rate: {imu_rate:.1f} Hz")
        
        # Quality alerts
        if self.wheel_confidence_history:
            recent_wheel_conf = np.mean(list(self.wheel_confidence_history)[-10:])
            if recent_wheel_conf < 0.3:
                self.alerts.append("CRITICAL: Very low wheel odometry confidence")
        
        if self.scan_quality_history:
            recent_scan_qual = np.mean(list(self.scan_quality_history)[-10:])
            if recent_scan_qual < 0.2:
                self.alerts.append("WARNING: Poor scan matching quality")
        
        if self.wheel_slip_history:
            recent_slip_rate = np.mean(list(self.wheel_slip_history)[-20:])
            if recent_slip_rate > 0.5:
                self.alerts.append("WARNING: High wheel slip rate detected")
        
        # Authority distribution alerts
        if self.authority_history:
            recent_authority = np.array(list(self.authority_history)[-10:])
            if recent_authority.size > 0:
                avg_authority = np.mean(recent_authority, axis=0)
                if np.max(avg_authority) > 0.8:
                    dominant_sensor = ["Wheel", "Scan", "IMU"][np.argmax(avg_authority)]
                    self.alerts.append(f"INFO: {dominant_sensor} sensor dominant in fusion")

    def publish_diagnostics(self):
        """Publish comprehensive diagnostics"""
        # Assess system health
        self.assess_component_health()
        self.generate_alerts()
        
        # Create diagnostic array
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        diagnostic_array.header.frame_id = ""
        
        # Wheel odometry diagnostics
        wheel_status = DiagnosticStatus()
        wheel_status.name = "Enhanced Wheel Odometry"
        wheel_status.hardware_id = "wheel_encoders"
        
        if self.wheel_odom_health == "OK":
            wheel_status.level = DiagnosticStatus.OK
            wheel_status.message = "Wheel odometry operating normally"
        elif self.wheel_odom_health == "WARN":
            wheel_status.level = DiagnosticStatus.WARN
            wheel_status.message = "Wheel odometry performance degraded"
        else:
            wheel_status.level = DiagnosticStatus.ERROR
            wheel_status.message = "Wheel odometry unreliable"
        
        if self.wheel_confidence_history:
            avg_conf = np.mean(list(self.wheel_confidence_history))
            wheel_status.values.append(KeyValue(key="average_confidence", value=f"{avg_conf:.3f}"))
        
        if self.wheel_slip_history:
            slip_rate = np.mean(list(self.wheel_slip_history))
            wheel_status.values.append(KeyValue(key="slip_rate", value=f"{slip_rate:.3f}"))
        
        wheel_rate = self.calculate_data_rate(list(self.odometry_timestamps))
        wheel_status.values.append(KeyValue(key="data_rate_hz", value=f"{wheel_rate:.1f}"))
        
        diagnostic_array.status.append(wheel_status)
        
        # Scan matching diagnostics
        scan_status = DiagnosticStatus()
        scan_status.name = "Adaptive Scan Matching"
        scan_status.hardware_id = "lidar_sensor"
        
        if self.scan_match_health == "OK":
            scan_status.level = DiagnosticStatus.OK
            scan_status.message = "Scan matching operating normally"
        elif self.scan_match_health == "WARN":
            scan_status.level = DiagnosticStatus.WARN
            scan_status.message = "Scan matching quality reduced"
        else:
            scan_status.level = DiagnosticStatus.ERROR
            scan_status.message = "Scan matching unreliable"
        
        if self.scan_quality_history:
            avg_qual = np.mean(list(self.scan_quality_history))
            scan_status.values.append(KeyValue(key="average_quality", value=f"{avg_qual:.3f}"))
        
        if self.scan_confidence_history:
            avg_conf = np.mean(list(self.scan_confidence_history))
            scan_status.values.append(KeyValue(key="average_confidence", value=f"{avg_conf:.3f}"))
        
        scan_rate = self.calculate_data_rate(list(self.scan_timestamps))
        scan_status.values.append(KeyValue(key="data_rate_hz", value=f"{scan_rate:.1f}"))
        
        diagnostic_array.status.append(scan_status)
        
        # IMU diagnostics
        imu_status = DiagnosticStatus()
        imu_status.name = "Enhanced IMU Processing"
        imu_status.hardware_id = "imu_sensor"
        
        if self.imu_health == "OK":
            imu_status.level = DiagnosticStatus.OK
            imu_status.message = "IMU processing operating normally"
        elif self.imu_health == "WARN":
            imu_status.level = DiagnosticStatus.WARN
            imu_status.message = "IMU calibration needs attention"
        else:
            imu_status.level = DiagnosticStatus.ERROR
            imu_status.message = "IMU processing unreliable"
        
        if self.imu_calibration_history:
            avg_calib = np.mean(list(self.imu_calibration_history))
            imu_status.values.append(KeyValue(key="calibration_score", value=f"{avg_calib:.3f}"))
        
        imu_rate = self.calculate_data_rate(list(self.imu_timestamps))
        imu_status.values.append(KeyValue(key="data_rate_hz", value=f"{imu_rate:.1f}"))
        
        diagnostic_array.status.append(imu_status)
        
        # Overall system diagnostics
        system_status = DiagnosticStatus()
        system_status.name = "Localization System"
        system_status.hardware_id = "complete_system"
        
        if self.overall_health == "OK":
            system_status.level = DiagnosticStatus.OK
            system_status.message = "Localization system healthy"
        elif self.overall_health == "WARN":
            system_status.level = DiagnosticStatus.WARN
            system_status.message = "Localization system performance degraded"
        else:
            system_status.level = DiagnosticStatus.ERROR
            system_status.message = "Localization system unreliable"
        
        # Add authority distribution
        if self.authority_history:
            recent_auth = np.array(list(self.authority_history)[-5:])
            if recent_auth.size > 0:
                avg_auth = np.mean(recent_auth, axis=0)
                system_status.values.append(KeyValue(key="wheel_authority", value=f"{avg_auth[0]:.3f}"))
                system_status.values.append(KeyValue(key="scan_authority", value=f"{avg_auth[1]:.3f}"))
                system_status.values.append(KeyValue(key="imu_authority", value=f"{avg_auth[2]:.3f}"))
        
        diagnostic_array.status.append(system_status)
        
        # Publish diagnostics
        self.diagnostics_pub.publish(diagnostic_array)
        
        # Publish health summary
        health_summary = {
            "overall": self.overall_health,
            "wheel_odometry": self.wheel_odom_health,
            "scan_matching": self.scan_match_health,
            "imu_processing": self.imu_health,
            "alerts": self.alerts,
            "performance_warnings": self.performance_warnings
        }
        
        health_msg = String()
        health_msg.data = json.dumps(health_summary)
        self.health_summary_pub.publish(health_msg)
        
        # Publish alerts if any
        if self.alerts or self.performance_warnings:
            alert_msg = String()
            all_alerts = self.alerts + self.performance_warnings
            alert_msg.data = " | ".join(all_alerts)
            self.alerts_pub.publish(alert_msg)
        
        # Log to file if enabled
        if self.log_file:
            timestamp = time.time()
            wheel_conf = np.mean(list(self.wheel_confidence_history)) if self.wheel_confidence_history else 0
            wheel_slip = np.mean(list(self.wheel_slip_history)) if self.wheel_slip_history else 0
            scan_qual = np.mean(list(self.scan_quality_history)) if self.scan_quality_history else 0
            scan_conf = np.mean(list(self.scan_confidence_history)) if self.scan_confidence_history else 0
            imu_calib = np.mean(list(self.imu_calibration_history)) if self.imu_calibration_history else 0
            
            self.log_file.write(f"{timestamp},{wheel_conf:.3f},{wheel_slip:.3f},{scan_qual:.3f},{scan_conf:.3f},{imu_calib:.3f},{self.overall_health}\n")
            self.log_file.flush()

    def __del__(self):
        """Cleanup when node is destroyed"""
        if self.log_file:
            self.log_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationDiagnosticsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()