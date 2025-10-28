#!/usr/bin/env python3
"""
Smooth Velocity Controller GUI
A keyboard-based GUI that allows simultaneous linear and angular velocity control

Features:
- Smooth acceleration/deceleration
- Simultaneous control of linear and angular velocities
- Real-time velocity display
- Configurable max speeds
- Emergency stop (spacebar)

Controls:
- W/S: Increase/Decrease linear velocity
- A/D: Increase/Decrease angular velocity (turn left/right)
- Spacebar: Emergency stop (zero all velocities)
- Q: Quit

Author: Aditya Pachauri
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import threading
import time

# Try to import tkinter for GUI
try:
    import tkinter as tk
    from tkinter import ttk
    HAS_GUI = True
except ImportError:
    HAS_GUI = False
    print("Warning: tkinter not available, using terminal mode")


class SmoothVelocityController(Node):
    """
    Smooth velocity controller that publishes twist commands
    """
    
    def __init__(self):
        super().__init__('smooth_velocity_controller')
        
        # Declare parameters
        self.declare_parameter('max_linear_velocity', 2.0)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('linear_acceleration', 0.5)
        self.declare_parameter('angular_acceleration', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('velocity_increment', 0.1)
        self.declare_parameter('angular_increment', 0.2)
        
        # Get parameters with default fallbacks
        self.max_linear: float = float(self.get_parameter('max_linear_velocity').value or 2.0)
        self.max_angular: float = float(self.get_parameter('max_angular_velocity').value or 2.0)
        self.linear_accel: float = float(self.get_parameter('linear_acceleration').value or 0.5)
        self.angular_accel: float = float(self.get_parameter('angular_acceleration').value or 1.0)
        self.publish_rate: float = float(self.get_parameter('publish_rate').value or 20.0)
        self.vel_increment: float = float(self.get_parameter('velocity_increment').value or 0.1)
        self.ang_increment: float = float(self.get_parameter('angular_increment').value or 0.2)
        
        # Publisher
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current velocities
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Control flags
        self.running = True
        self.lock = threading.Lock()
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_velocity)
        
        self.get_logger().info("Smooth Velocity Controller started")
        self.get_logger().info(f"Max linear: {self.max_linear} m/s, Max angular: {self.max_angular} rad/s")
    
    def set_target_velocities(self, linear, angular):
        """Set target velocities"""
        with self.lock:
            self.target_linear = max(-self.max_linear, min(self.max_linear, linear))
            self.target_angular = max(-self.max_angular, min(self.max_angular, angular))
    
    def increase_linear(self, amount=None):
        """Increase linear velocity"""
        if amount is None:
            amount = self.vel_increment
        with self.lock:
            self.target_linear = min(self.max_linear, self.target_linear + amount)
    
    def decrease_linear(self, amount=None):
        """Decrease linear velocity"""
        if amount is None:
            amount = self.vel_increment
        with self.lock:
            self.target_linear = max(-self.max_linear, self.target_linear - amount)
    
    def increase_angular(self, amount=None):
        """Increase angular velocity (turn left)"""
        if amount is None:
            amount = self.ang_increment
        with self.lock:
            self.target_angular = min(self.max_angular, self.target_angular + amount)
    
    def decrease_angular(self, amount=None):
        """Decrease angular velocity (turn right)"""
        if amount is None:
            amount = self.ang_increment
        with self.lock:
            self.target_angular = max(-self.max_angular, self.target_angular - amount)
    
    def stop(self):
        """Emergency stop"""
        with self.lock:
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.current_linear = 0.0
            self.current_angular = 0.0
    
    def publish_velocity(self):
        """Publish smoothed velocity commands"""
        with self.lock:
            # Smooth acceleration/deceleration
            dt = 1.0 / self.publish_rate
            
            # Linear velocity smoothing
            linear_diff = self.target_linear - self.current_linear
            max_linear_change = self.linear_accel * dt
            
            if abs(linear_diff) > max_linear_change:
                self.current_linear += max_linear_change * (1 if linear_diff > 0 else -1)
            else:
                self.current_linear = self.target_linear
            
            # Angular velocity smoothing
            angular_diff = self.target_angular - self.current_angular
            max_angular_change = self.angular_accel * dt
            
            if abs(angular_diff) > max_angular_change:
                self.current_angular += max_angular_change * (1 if angular_diff > 0 else -1)
            else:
                self.current_angular = self.target_angular
            
            # Publish
            twist = Twist()
            twist.linear.x = self.current_linear
            twist.angular.z = self.current_angular
            self.vel_pub.publish(twist)
    
    def get_velocities(self):
        """Get current velocities for display"""
        with self.lock:
            return self.current_linear, self.current_angular, self.target_linear, self.target_angular


class VelocityControlGUI:
    """
    GUI for velocity control
    """
    
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("Smooth Velocity Controller")
        self.root.geometry("600x400")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="wens")
        
        # Title
        title = ttk.Label(main_frame, text="Robot Velocity Controller", font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Instructions
        instructions = """
Controls:
W / S : Increase / Decrease Linear Velocity (Forward/Backward)
A / D : Increase / Decrease Angular Velocity (Turn Left/Right)
SPACE : Emergency Stop
Q : Quit

Or use the sliders below for precise control
        """
        inst_label = ttk.Label(main_frame, text=instructions, justify=tk.LEFT, font=('Arial', 10))
        inst_label.grid(row=1, column=0, columnspan=2, pady=10)
        
        # Linear velocity slider
        ttk.Label(main_frame, text="Linear Velocity (m/s):", font=('Arial', 12)).grid(row=2, column=0, sticky=tk.W, pady=5)
        self.linear_slider = ttk.Scale(
            main_frame, 
            from_=-controller.max_linear, 
            to=controller.max_linear, 
            orient=tk.HORIZONTAL,
            length=400,
            command=self.on_linear_slider_change
        )
        self.linear_slider.set(0)
        self.linear_slider.grid(row=2, column=1, pady=5)
        
        # Angular velocity slider
        ttk.Label(main_frame, text="Angular Velocity (rad/s):", font=('Arial', 12)).grid(row=3, column=0, sticky=tk.W, pady=5)
        self.angular_slider = ttk.Scale(
            main_frame, 
            from_=-controller.max_angular, 
            to=controller.max_angular, 
            orient=tk.HORIZONTAL,
            length=400,
            command=self.on_angular_slider_change
        )
        self.angular_slider.set(0)
        self.angular_slider.grid(row=3, column=1, pady=5)
        
        # Current velocity display
        self.vel_display = ttk.Label(main_frame, text="", font=('Arial', 11, 'bold'))
        self.vel_display.grid(row=4, column=0, columnspan=2, pady=20)
        
        # Control buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=5, column=0, columnspan=2, pady=10)
        
        ttk.Button(button_frame, text="STOP", command=self.emergency_stop, width=15).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Quit", command=self.quit, width=15).pack(side=tk.LEFT, padx=5)
        
        # Keyboard bindings
        self.root.bind('<w>', lambda e: controller.increase_linear())
        self.root.bind('<W>', lambda e: controller.increase_linear())
        self.root.bind('<s>', lambda e: controller.decrease_linear())
        self.root.bind('<S>', lambda e: controller.decrease_linear())
        self.root.bind('<a>', lambda e: controller.increase_angular())
        self.root.bind('<A>', lambda e: controller.increase_angular())
        self.root.bind('<d>', lambda e: controller.decrease_angular())
        self.root.bind('<D>', lambda e: controller.decrease_angular())
        self.root.bind('<space>', lambda e: self.emergency_stop())
        self.root.bind('<q>', lambda e: self.quit())
        self.root.bind('<Q>', lambda e: self.quit())
        
        # Update display periodically
        self.update_display()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
    
    def on_linear_slider_change(self, value):
        """Handle linear slider change"""
        self.controller.set_target_velocities(float(value), self.controller.target_angular)
    
    def on_angular_slider_change(self, value):
        """Handle angular slider change"""
        self.controller.set_target_velocities(self.controller.target_linear, float(value))
    
    def emergency_stop(self):
        """Emergency stop"""
        self.controller.stop()
        self.linear_slider.set(0)
        self.angular_slider.set(0)
    
    def update_display(self):
        """Update velocity display"""
        current_lin, current_ang, target_lin, target_ang = self.controller.get_velocities()
        
        display_text = f"""
Current Velocities:
Linear: {current_lin:.2f} m/s (Target: {target_lin:.2f} m/s)
Angular: {current_ang:.2f} rad/s (Target: {target_ang:.2f} rad/s)
        """
        self.vel_display.config(text=display_text)
        
        # Schedule next update
        self.root.after(50, self.update_display)
    
    def quit(self):
        """Quit application"""
        self.controller.stop()
        self.controller.running = False
        self.root.quit()
        self.root.destroy()
    
    def run(self):
        """Run the GUI"""
        self.root.mainloop()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    controller = SmoothVelocityController()
    
    if HAS_GUI:
        # Run with GUI
        gui = VelocityControlGUI(controller)
        
        # Run ROS2 spinning in separate thread
        spin_thread = threading.Thread(target=lambda: rclpy.spin(controller), daemon=True)
        spin_thread.start()
        
        # Run GUI in main thread
        try:
            gui.run()
        except KeyboardInterrupt:
            pass
        finally:
            controller.stop()
            controller.destroy_node()
            rclpy.shutdown()
    else:
        # Fallback to terminal mode
        print("\n" + "="*50)
        print("Terminal Velocity Controller")
        print("="*50)
        print("\nControls:")
        print("w/s: Increase/Decrease linear velocity")
        print("a/d: Increase/Decrease angular velocity")
        print("x: Stop")
        print("q: Quit")
        print("="*50 + "\n")
        
        try:
            rclpy.spin(controller)
        except KeyboardInterrupt:
            pass
        finally:
            controller.stop()
            controller.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()