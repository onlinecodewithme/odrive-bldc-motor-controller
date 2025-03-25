# ROS 2 ODrive Implementation Guide for Jetson Orin NX

This guide provides detailed instructions for implementing the ODrive-controlled tracked robot system using ROS 2 Humble on a Jetson Orin NX board.

## System Overview

The system consists of:

- NVIDIA Jetson Orin NX as the main computing platform
- ROS 2 Humble as the robotics framework
- ODrive V3.6 controller for motor control
- 2x MY1020 BLDC motors (48V, 1000W)
- Joystick controller for teleoperation
- Power supply (48V for motors)

## Hardware Setup

### 1. Jetson Orin NX Setup

1. Install JetPack 5.1.1 or later on your Jetson Orin NX
2. Set up the Jetson for development:
   ```bash
   sudo apt update
   sudo apt upgrade
   ```
3. Configure power mode for better performance:
   ```bash
   sudo nvpmodel -m 0  # Set to maximum performance mode
   sudo jetson_clocks  # Set clocks to maximum
   ```

### 2. ODrive and Motor Connections

Follow the same wiring instructions as in the Arduino implementation, with these differences:

1. Connect the ODrive controller directly to the Jetson Orin NX via USB
2. No Arduino is needed as the Jetson will communicate directly with the ODrive

#### ODrive to Motors

| ODrive Connection | Motor Connection | Description       |
|-------------------|------------------|-------------------|
| M0A               | Left Motor A     | Left motor phase A|
| M0B               | Left Motor B     | Left motor phase B|
| M0C               | Left Motor C     | Left motor phase C|
| M1A               | Right Motor A    | Right motor phase A|
| M1B               | Right Motor B    | Right motor phase B|
| M1C               | Right Motor C    | Right motor phase C|

#### Hall Sensors

| ODrive Connection | Sensor Connection | Description       |
|-------------------|-------------------|-------------------|
| M0 Hall A         | Left Hall A       | Left hall sensor A|
| M0 Hall B         | Left Hall B       | Left hall sensor B|
| M0 Hall C         | Left Hall C       | Left hall sensor C|
| M0 5V             | Left 5V           | Left sensor power |
| M0 GND            | Left GND          | Left sensor ground|
| M1 Hall A         | Right Hall A      | Right hall sensor A|
| M1 Hall B         | Right Hall B      | Right hall sensor B|
| M1 Hall C         | Right Hall C      | Right hall sensor C|
| M1 5V             | Right 5V          | Right sensor power|
| M1 GND            | Right GND         | Right sensor ground|

### 3. Power Connections

1. Connect a 48V power supply to the ODrive DC+ and DC- terminals
2. Ensure the Jetson has its own power supply (do not power it from the ODrive)
3. Connect a common ground between the Jetson and ODrive power systems

### 4. Joystick Connection

1. Connect a USB joystick (Xbox controller recommended) to the Jetson Orin NX
2. Verify the joystick is detected:
   ```bash
   ls -l /dev/input/js*
   ```

## Software Installation

### 1. Install ROS 2 Humble

Follow the official ROS 2 Humble installation instructions for Ubuntu 20.04 on ARM64:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools
```

### 2. Install ODrive Python Package

```bash
pip3 install --user odrive
```

### 3. Install ROS 2 Dependencies

```bash
sudo apt install -y ros-humble-joy ros-humble-teleop-twist-joy
```

### 4. Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 5. Copy the ODrive Control Package

Copy the entire `odrive_control` package to your ROS 2 workspace:

```bash
cp -r /path/to/odrive_control ~/ros2_ws/src/
```

### 6. Build the Package

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select odrive_control
source install/setup.bash
```

## ODrive Configuration

### 1. Initial ODrive Setup

Connect to the ODrive using the ODrive Tool:

```bash
odrivetool
```

### 2. Configure Motor Parameters

```python
# For both motors (axis0 and axis1)
for axis in [odrv0.axis0, odrv0.axis1]:
    # Motor configuration
    axis.motor.config.pole_pairs = 7  # Check your motor specifications
    axis.motor.config.resistance_calib_max_voltage = 4
    axis.motor.config.requested_current_range = 25  # Adjust based on your motor
    axis.motor.config.current_control_bandwidth = 100
    
    # Set motor type to high-current gimbal motor
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    
    # Hall sensor configuration
    axis.encoder.config.mode = ENCODER_MODE_HALL
    axis.encoder.config.cpr = 6 * 7  # 6 states * pole pairs
    axis.encoder.config.calib_scan_distance = 150
    
    # Controller configuration
    axis.controller.config.vel_limit = 2  # Adjust based on your requirements
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.vel_ramp_rate = 0.5  # Acceleration limit (turns/s²)
    
    # Current limit
    axis.motor.config.current_lim = 20.0  # Adjust based on your motor specs

# Save configuration
odrv0.save_configuration()
```

### 3. Calibrate Motors

```python
# For both motors
for axis in [odrv0.axis0, odrv0.axis1]:
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    # Wait for calibration to complete
    time.sleep(10)
    
    # Check if calibration was successful
    if axis.motor.error != 0:
        print(f"Motor calibration failed with error: {axis.motor.error}")
        continue
    
    # Encoder offset calibration
    axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    
    # Wait for calibration to complete
    time.sleep(10)
    
    # Check if calibration was successful
    if axis.encoder.error != 0:
        print(f"Encoder calibration failed with error: {axis.encoder.error}")
        continue

# Save configuration
odrv0.save_configuration()
```

## ROS 2 Configuration

### 1. Configure ODrive Parameters

Edit the `config/odrive_params.yaml` file to match your robot's physical parameters:

```yaml
odrive_node:
  ros__parameters:
    # Motor configuration
    left_motor_index: 0  # M0 on ODrive
    right_motor_index: 1  # M1 on ODrive
    
    # Robot physical parameters
    wheel_radius: 0.085  # meters - adjust to your wheel size
    track_width: 0.5     # meters - adjust to your robot's track width
    encoder_cpr: 42      # Counts per revolution - adjust to your encoder
    gear_ratio: 1.0      # Motor to wheel gear ratio - adjust if using gears
    
    # Control parameters
    max_speed: 2.0       # Max speed in turns/second
    control_mode: "velocity"  # 'velocity' or 'position'
    publish_rate: 20.0   # Hz
```

### 2. Configure Joystick Parameters

Edit the `config/xbox.yaml` file to match your joystick:

```yaml
joy_node:
  ros__parameters:
    device_id: 0
    deadzone: 0.05
    autorepeat_rate: 20.0

teleop_node:
  ros__parameters:
    # Adjust these based on your joystick
    linear_axis: 1      # Y-axis
    angular_axis: 0     # X-axis
    linear_scale: 1.0   # m/s
    angular_scale: 2.0  # rad/s
    deadzone: 0.1
    enable_button: 4    # LB button
    turbo_button: 5     # RB button
```

## Running the System

### 1. Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

### 2. Launch the Control System

```bash
ros2 launch odrive_control odrive_control.launch.py
```

### 3. Verify Operation

Check that the nodes are running:

```bash
ros2 node list
```

You should see:
- `/joy_node`
- `/odrive_node`
- `/teleop_node`

Check published topics:

```bash
ros2 topic list
```

You should see topics including:
- `/cmd_vel`
- `/joy`
- `/odometry`
- `/connected`

### 4. Test Teleoperation

1. Press and hold the LB button (button 4) on the controller
2. Move the left stick to control the robot
3. Verify the motors respond correctly

## Troubleshooting

### USB Permission Issues

If you encounter permission issues with the ODrive USB device:

```bash
sudo chmod 666 /dev/ttyUSB*
```

For a permanent solution, create a udev rule:

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666"' | sudo tee /etc/udev/rules.d/50-odrive.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Joystick Not Detected

Check the joystick device:

```bash
ls -l /dev/input/js*
```

If not found, try reconnecting the joystick or check if it needs drivers.

### Motors Not Responding

1. Check ODrive connection:
   ```bash
   ros2 topic echo /connected
   ```

2. Check for errors in the ODrive node:
   ```bash
   ros2 topic echo /odrive_node/status
   ```

3. Verify the joystick is publishing commands:
   ```bash
   ros2 topic echo /cmd_vel
   ```

### ROS 2 Node Crashes

Check the node logs:

```bash
ros2 log level set odrive_node debug
```

Then restart the system and check the logs for detailed error messages.

## Advanced Configuration

### 1. Tuning Motor Control Parameters

For smoother operation, you can adjust the ODrive control parameters:

```python
# In odrivetool
for axis in [odrv0.axis0, odrv0.axis1]:
    # Position control
    axis.controller.config.pos_gain = 20.0
    axis.controller.config.vel_gain = 0.16
    axis.controller.config.vel_integrator_gain = 0.32
    
    # Velocity control
    axis.controller.config.vel_limit = 2.0
    axis.controller.config.vel_ramp_rate = 0.5
    
    # Current control
    axis.motor.config.current_lim = 20.0
    axis.motor.config.current_lim_margin = 5.0

odrv0.save_configuration()
```

### 2. Adding Navigation Support

To enable autonomous navigation, you can integrate the odometry with the ROS 2 navigation stack:

1. Install navigation packages:
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

2. Create a navigation launch file that includes your odrive_control launch file and the nav2 stack.

### 3. Adding a Mapping Sensor

For autonomous navigation, add a LiDAR or depth camera:

1. Install the appropriate ROS 2 driver for your sensor
2. Add the sensor node to your launch file
3. Configure the navigation stack to use the sensor data

## Maintenance

1. Regularly check motor temperature during operation
2. Keep the ODrive controller clean and well-ventilated
3. Update firmware and software as new versions become available
4. Periodically recalibrate the system if performance degrades
5. Check all connections for looseness or damage

## Next Steps

- Implement autonomous navigation using the ROS 2 Navigation2 stack
- Add a web interface for remote monitoring and control
- Integrate with a camera for computer vision applications
- Develop custom behaviors using ROS 2 actions and services
