#!/usr/bin/env python3
"""
ROS 2 Node for interfacing with ODrive motor controllers.
This node handles communication with the ODrive and exposes ROS 2 interfaces
for controlling the motors and reading their state.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import odrive
from odrive.enums import *
import time
import math
import numpy as np
import threading
import sys

class ODriveNode(Node):
    """
    ROS 2 Node for interfacing with ODrive motor controllers.
    """
    
    def __init__(self):
        super().__init__('odrive_node')
        
        # Declare parameters
        self.declare_parameter('left_motor_index', 0)
        self.declare_parameter('right_motor_index', 1)
        self.declare_parameter('wheel_radius', 0.085)  # meters
        self.declare_parameter('track_width', 0.5)     # meters (distance between wheels)
        self.declare_parameter('encoder_cpr', 42)      # Counts per revolution
        self.declare_parameter('gear_ratio', 1.0)      # Motor to wheel gear ratio
        self.declare_parameter('max_speed', 2.0)       # Max speed in turns/second
        self.declare_parameter('control_mode', 'velocity')  # 'velocity' or 'position'
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('connect_on_startup', True)
        self.declare_parameter('calibrate_on_startup', False)
        
        # Get parameters
        self.left_motor_index = self.get_parameter('left_motor_index').value
        self.right_motor_index = self.get_parameter('right_motor_index').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.max_speed = self.get_parameter('max_speed').value
        self.control_mode = self.get_parameter('control_mode').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.connect_on_startup = self.get_parameter('connect_on_startup').value
        self.calibrate_on_startup = self.get_parameter('calibrate_on_startup').value
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odometry', sensor_qos)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', sensor_qos)
        self.left_vel_pub = self.create_publisher(Float64, 'left_wheel/velocity', 10)
        self.right_vel_pub = self.create_publisher(Float64, 'right_wheel/velocity', 10)
        self.left_pos_pub = self.create_publisher(Float64, 'left_wheel/position', 10)
        self.right_pos_pub = self.create_publisher(Float64, 'right_wheel/position', 10)
        self.connected_pub = self.create_publisher(Bool, 'connected', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Initialize ODrive
        self.odrive_device = None
        self.left_motor = None
        self.right_motor = None
        self.connected = False
        
        # Initialize state variables
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Connect to ODrive if requested
        if self.connect_on_startup:
            self.connect()
            
        # Start publishing thread
        self.stop_thread = False
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
        self.get_logger().info('ODrive node initialized')
    
    def connect(self):
        """Connect to ODrive device."""
        self.get_logger().info('Connecting to ODrive...')
        try:
            self.odrive_device = odrive.find_any()
            self.get_logger().info(f'Found ODrive: {self.odrive_device.serial_number}')
            
            # Get motor objects
            if self.left_motor_index == 0:
                self.left_motor = self.odrive_device.axis0
            else:
                self.left_motor = self.odrive_device.axis1
                
            if self.right_motor_index == 0:
                self.right_motor = self.odrive_device.axis0
            else:
                self.right_motor = self.odrive_device.axis1
            
            # Set control mode
            if self.control_mode == 'velocity':
                self.left_motor.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                self.right_motor.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            else:  # position mode
                self.left_motor.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                self.right_motor.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            
            # Calibrate if requested
            if self.calibrate_on_startup:
                self.calibrate()
            
            # Set to closed loop control
            self.left_motor.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.right_motor.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            
            self.connected = True
            self.publish_connected_status()
            self.get_logger().info('ODrive connected and configured')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ODrive: {str(e)}')
            self.connected = False
            self.publish_connected_status()
            return False
    
    def calibrate(self):
        """Calibrate the ODrive motors."""
        self.get_logger().info('Starting motor calibration...')
        
        # Motor calibration
        self.left_motor.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        self.right_motor.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        
        # Wait for calibration to complete
        time.sleep(10)
        
        # Check if calibration was successful
        if self.left_motor.motor.error != 0:
            self.get_logger().error(f'Left motor calibration failed with error: {self.left_motor.motor.error}')
        else:
            self.get_logger().info('Left motor calibration successful')
            
        if self.right_motor.motor.error != 0:
            self.get_logger().error(f'Right motor calibration failed with error: {self.right_motor.motor.error}')
        else:
            self.get_logger().info('Right motor calibration successful')
        
        # Encoder offset calibration
        self.left_motor.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.right_motor.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        
        # Wait for calibration to complete
        time.sleep(10)
        
        # Check if calibration was successful
        if self.left_motor.encoder.error != 0:
            self.get_logger().error(f'Left encoder calibration failed with error: {self.left_motor.encoder.error}')
        else:
            self.get_logger().info('Left encoder calibration successful')
            
        if self.right_motor.encoder.error != 0:
            self.get_logger().error(f'Right encoder calibration failed with error: {self.right_motor.encoder.error}')
        else:
            self.get_logger().info('Right encoder calibration successful')
    
    def cmd_vel_callback(self, msg):
        """
        Handle cmd_vel messages to control the robot.
        
        Args:
            msg (Twist): The velocity command message
        """
        if not self.connected:
            self.get_logger().warning('Received cmd_vel but ODrive is not connected')
            return
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate wheel velocities using differential drive kinematics
        left_wheel_vel = (linear_x - angular_z * self.track_width / 2.0) / self.wheel_radius
        right_wheel_vel = (linear_x + angular_z * self.track_width / 2.0) / self.wheel_radius
        
        # Convert to turns/second for ODrive
        left_turns_per_sec = left_wheel_vel / (2.0 * math.pi) * self.gear_ratio
        right_turns_per_sec = right_wheel_vel / (2.0 * math.pi) * self.gear_ratio
        
        # Limit to max speed
        left_turns_per_sec = max(min(left_turns_per_sec, self.max_speed), -self.max_speed)
        right_turns_per_sec = max(min(right_turns_per_sec, self.max_speed), -self.max_speed)
        
        # Invert right motor direction if needed
        # Uncomment the line below if your right motor needs to be inverted
        # right_turns_per_sec = -right_turns_per_sec
        
        try:
            # Send commands to ODrive
            if self.control_mode == 'velocity':
                self.left_motor.controller.input_vel = float(left_turns_per_sec)
                self.right_motor.controller.input_vel = float(right_turns_per_sec)
            else:  # position mode
                # In position mode, we integrate the velocity to get position
                current_time = self.get_clock().now()
                dt = (current_time - self.last_time).nanoseconds / 1e9
                self.last_time = current_time
                
                # Calculate new positions
                left_pos_increment = left_turns_per_sec * dt
                right_pos_increment = right_turns_per_sec * dt
                
                left_pos_target = self.left_motor.controller.input_pos + left_pos_increment
                right_pos_target = self.right_motor.controller.input_pos + right_pos_increment
                
                self.left_motor.controller.input_pos = float(left_pos_target)
                self.right_motor.controller.input_pos = float(right_pos_target)
                
        except Exception as e:
            self.get_logger().error(f'Failed to send command to ODrive: {str(e)}')
    
    def read_odrive_state(self):
        """Read the current state from the ODrive."""
        if not self.connected:
            return
        
        try:
            # Read encoder positions and velocities
            self.left_pos = self.left_motor.encoder.pos_estimate
            self.right_pos = self.right_motor.encoder.pos_estimate
            self.left_vel = self.left_motor.encoder.vel_estimate
            self.right_vel = self.right_motor.encoder.vel_estimate
            
            # Invert right motor readings if needed
            # Uncomment the lines below if your right motor is inverted
            # self.right_pos = -self.right_pos
            # self.right_vel = -self.right_vel
            
        except Exception as e:
            self.get_logger().error(f'Failed to read state from ODrive: {str(e)}')
            self.connected = False
            self.publish_connected_status()
    
    def update_odometry(self):
        """Update odometry based on wheel positions."""
        # Get current time
        current_time = self.get_clock().now()
        
        # Calculate time difference
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0:
            return
        
        # Convert from turns to meters
        left_wheel_dist = self.left_pos * 2.0 * math.pi * self.wheel_radius / self.gear_ratio
        right_wheel_dist = self.right_pos * 2.0 * math.pi * self.wheel_radius / self.gear_ratio
        
        # Calculate linear and angular displacement
        linear_disp = (right_wheel_dist + left_wheel_dist) / 2.0
        angular_disp = (right_wheel_dist - left_wheel_dist) / self.track_width
        
        # Update pose
        self.x += linear_disp * math.cos(self.theta)
        self.y += linear_disp * math.sin(self.theta)
        self.theta += angular_disp
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
    
    def publish_odometry(self):
        """Publish odometry message."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = sy
        odom_msg.pose.pose.orientation.w = cy
        
        # Set velocity
        linear_vel = (self.right_vel + self.left_vel) * math.pi * self.wheel_radius / self.gear_ratio
        angular_vel = (self.right_vel - self.left_vel) * math.pi * self.wheel_radius / (self.gear_ratio * self.track_width)
        
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)
    
    def publish_joint_states(self):
        """Publish joint states message."""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # Convert from turns to radians
        left_rad = self.left_pos * 2.0 * math.pi / self.gear_ratio
        right_rad = self.right_pos * 2.0 * math.pi / self.gear_ratio
        
        joint_msg.position = [left_rad, right_rad]
        
        # Convert from turns/s to rad/s
        left_vel_rad = self.left_vel * 2.0 * math.pi / self.gear_ratio
        right_vel_rad = self.right_vel * 2.0 * math.pi / self.gear_ratio
        
        joint_msg.velocity = [left_vel_rad, right_vel_rad]
        
        # Publish joint states
        self.joint_pub.publish(joint_msg)
    
    def publish_wheel_data(self):
        """Publish individual wheel data."""
        # Publish wheel velocities
        left_vel_msg = Float64()
        left_vel_msg.data = self.left_vel
        self.left_vel_pub.publish(left_vel_msg)
        
        right_vel_msg = Float64()
        right_vel_msg.data = self.right_vel
        self.right_vel_pub.publish(right_vel_msg)
        
        # Publish wheel positions
        left_pos_msg = Float64()
        left_pos_msg.data = self.left_pos
        self.left_pos_pub.publish(left_pos_msg)
        
        right_pos_msg = Float64()
        right_pos_msg.data = self.right_pos
        self.right_pos_pub.publish(right_pos_msg)
    
    def publish_connected_status(self):
        """Publish connection status."""
        connected_msg = Bool()
        connected_msg.data = self.connected
        self.connected_pub.publish(connected_msg)
    
    def publish_loop(self):
        """Main publishing loop."""
        rate = 1.0 / self.publish_rate
        
        while not self.stop_thread:
            if self.connected:
                try:
                    # Read current state from ODrive
                    self.read_odrive_state()
                    
                    # Update and publish odometry
                    self.update_odometry()
                    self.publish_odometry()
                    
                    # Publish other data
                    self.publish_joint_states()
                    self.publish_wheel_data()
                    
                except Exception as e:
                    self.get_logger().error(f'Error in publish loop: {str(e)}')
            
            # Sleep to maintain publish rate
            time.sleep(rate)
    
    def destroy_node(self):
        """Clean up when node is destroyed."""
        self.get_logger().info('Shutting down ODrive node...')
        
        # Stop publishing thread
        self.stop_thread = True
        if self.publish_thread.is_alive():
            self.publish_thread.join(timeout=1.0)
        
        # Stop motors
        if self.connected:
            try:
                self.left_motor.controller.input_vel = 0.0
                self.right_motor.controller.input_vel = 0.0
                self.left_motor.requested_state = AXIS_STATE_IDLE
                self.right_motor.requested_state = AXIS_STATE_IDLE
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = ODriveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
