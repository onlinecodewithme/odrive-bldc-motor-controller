# ODrive Control for ROS 2 Humble on Jetson Orin NX

This package provides ROS 2 integration for controlling ODrive-powered tracked robots using a Jetson Orin NX board.

## Features

- Direct interface with ODrive V3.6 controller
- Differential steering control for tracked robots
- Joystick teleoperation support
- Odometry publishing for navigation
- ROS 2 parameter system for easy configuration

## Prerequisites

- NVIDIA Jetson Orin NX with JetPack 5.1.1 or later
- ROS 2 Humble installed on the Jetson
- ODrive v3.6 controller
- 2x MY1020 BLDC motors (48V, 1000W)
- Joystick controller (Xbox controller recommended)
- Python 3.8+

## Installation

### 1. Install Dependencies

```bash
# Install ODrive Python package
pip3 install --user odrive

# Install ROS 2 dependencies
sudo apt update
sudo apt install -y ros-humble-joy ros-humble-teleop-twist-joy
```

### 2. Clone and Build the Package

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/yourusername/odrive_control.git

# Build the package
cd ~/ros2_ws
colcon build --symlink-install --packages-select odrive_control

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Usage

### 1. Connect Hardware

- Connect the ODrive controller to the Jetson Orin NX via USB
- Connect the joystick controller to the Jetson
- Ensure the motors are properly connected to the ODrive as per the wiring guide

### 2. Launch the Control System

```bash
# Launch the ODrive control system with default parameters
ros2 launch odrive_control odrive_control.launch.py

# Launch with custom configuration
ros2 launch odrive_control odrive_control.launch.py config_file:=my_config.yaml joy_config:=my_joystick.yaml
```

### 3. Teleoperation

- Press and hold the LB button (button 4) on the controller to enable motor control
- Use the left stick to control the robot:
  - Y-axis: Forward/backward movement
  - X-axis: Left/right turning
- Press the RB button (button 5) while driving for "turbo" mode (increased speed)

## Configuration

### ODrive Parameters

Edit the `config/odrive_params.yaml` file to adjust:

- Motor configuration (indices, control mode)
- Robot physical parameters (wheel radius, track width)
- Control parameters (max speed, publish rate)
- Startup behavior

### Joystick Configuration

Edit the `config/xbox.yaml` file to adjust:

- Axis mappings
- Button assignments
- Speed scaling
- Deadzone values

## Nodes

### odrive_node

The main node that interfaces with the ODrive controller.

**Subscribed Topics:**
- `cmd_vel` (geometry_msgs/Twist): Velocity commands

**Published Topics:**
- `odometry` (nav_msgs/Odometry): Robot odometry
- `joint_states` (sensor_msgs/JointState): Wheel joint states
- `left_wheel/velocity` (std_msgs/Float64): Left wheel velocity
- `right_wheel/velocity` (std_msgs/Float64): Right wheel velocity
- `left_wheel/position` (std_msgs/Float64): Left wheel position
- `right_wheel/position` (std_msgs/Float64): Right wheel position
- `connected` (std_msgs/Bool): ODrive connection status

### teleop_node

Node for teleoperation using a joystick.

**Subscribed Topics:**
- `joy` (sensor_msgs/Joy): Joystick input

**Published Topics:**
- `cmd_vel` (geometry_msgs/Twist): Velocity commands

## Troubleshooting

### ODrive Not Detected

- Ensure the ODrive is properly connected via USB
- Check that the ODrive has power
- Verify USB permissions: `sudo chmod 666 /dev/ttyUSB*`
- Try reconnecting the USB cable

### Motors Not Responding

- Check the ODrive connection status topic: `ros2 topic echo /connected`
- Verify motor wiring
- Check for errors in the ODrive node output
- Ensure the joystick enable button is pressed during operation

### Erratic Movement

- Adjust the deadzone parameter in the joystick configuration
- Reduce the max_speed parameter in the ODrive configuration
- Check for loose connections

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ODrive Robotics for their excellent motor controller
- ROS 2 community for the framework and tools
