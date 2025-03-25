#!/usr/bin/env python3
"""
ROS 2 Node for teleoperation of the tracked robot using a joystick.
This node subscribes to joy messages and publishes cmd_vel messages
for controlling the robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math

class TeleopNode(Node):
    """
    ROS 2 Node for teleoperation of the tracked robot using a joystick.
    """
    
    def __init__(self):
        super().__init__('teleop_node')
        
        # Declare parameters
        self.declare_parameter('linear_axis', 1)  # Left stick Y-axis
        self.declare_parameter('angular_axis', 0)  # Left stick X-axis
        self.declare_parameter('linear_scale', 1.0)  # m/s
        self.declare_parameter('angular_scale', 2.0)  # rad/s
        self.declare_parameter('deadzone', 0.1)  # Joystick deadzone
        self.declare_parameter('enable_button', 4)  # LB button
        self.declare_parameter('turbo_button', 5)  # RB button
        self.declare_parameter('turbo_scale', 2.0)  # Turbo mode multiplier
        
        # Get parameters
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        self.enable_button = self.get_parameter('enable_button').value
        self.turbo_button = self.get_parameter('turbo_button').value
        self.turbo_scale = self.get_parameter('turbo_scale').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        
        self.get_logger().info('Teleop node initialized')
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick value."""
        if abs(value) < self.deadzone:
            return 0.0
        
        # Scale the value to account for deadzone
        sign = 1.0 if value > 0.0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def joy_callback(self, msg):
        """
        Handle joy messages to control the robot.
        
        Args:
            msg (Joy): The joystick message
        """
        # Create Twist message
        cmd_vel = Twist()
        
        # Check if enable button is pressed
        if len(msg.buttons) > self.enable_button and msg.buttons[self.enable_button]:
            # Check axes array bounds
            if len(msg.axes) > max(self.linear_axis, self.angular_axis):
                # Get joystick values with deadzone applied
                linear_val = self.apply_deadzone(msg.axes[self.linear_axis])
                angular_val = self.apply_deadzone(msg.axes[self.angular_axis])
                
                # Apply scaling
                linear_scale = self.linear_scale
                angular_scale = self.angular_scale
                
                # Check if turbo button is pressed
                if len(msg.buttons) > self.turbo_button and msg.buttons[self.turbo_button]:
                    linear_scale *= self.turbo_scale
                    angular_scale *= self.turbo_scale
                
                # Set cmd_vel values
                cmd_vel.linear.x = linear_val * linear_scale
                cmd_vel.angular.z = angular_val * angular_scale
                
                self.get_logger().debug(f'Cmd vel: linear={cmd_vel.linear.x}, angular={cmd_vel.angular.z}')
        
        # Publish cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = TeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
