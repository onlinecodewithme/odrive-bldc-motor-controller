# ROS 2 ODrive Control Project Structure

This document provides an overview of the project structure and the purpose of each file in the ROS 2 implementation.

```
ros2_odrive/
│
├── README.md                  # Project overview and getting started guide
│
├── src/                       # Source code directory
│   └── odrive_control/        # ROS 2 package
│       ├── package.xml        # Package manifest
│       ├── setup.py           # Package setup script
│       ├── resource/          # Package resources
│       │   └── odrive_control # Package marker file
│       │
│       ├── odrive_control/    # Python module
│       │   ├── __init__.py    # Module initialization
│       │   ├── odrive_node.py # Main ODrive interface node
│       │   └── teleop_node.py # Teleoperation node
│       │
│       ├── launch/            # Launch files
│       │   └── odrive_control.launch.py # Main launch file
│       │
│       ├── config/            # Configuration files
│       │   ├── odrive_params.yaml # ODrive parameters
│       │   └── xbox.yaml      # Joystick configuration
│       │
│       └── test/              # Test files
│
└── docs/                      # Documentation directory
    ├── implementation_guide.md # Implementation guide
    └── wiring_diagram.txt     # ASCII art wiring diagram
```

## Source Code Files

### odrive_node.py
The main ROS 2 node that interfaces with the ODrive controller:
- Communicates with the ODrive over USB
- Subscribes to cmd_vel messages for motor control
- Publishes odometry and joint state information
- Handles motor control and feedback
- Implements differential steering logic

### teleop_node.py
A ROS 2 node for teleoperation using a joystick:
- Subscribes to joy messages from the joystick
- Processes joystick input with deadzone handling
- Publishes cmd_vel messages for robot control
- Implements safety features like enable button and speed limits

## Configuration Files

### odrive_params.yaml
Configuration parameters for the ODrive controller:
- Motor indices and physical parameters
- Robot dimensions and characteristics
- Control parameters and limits
- Startup behavior settings

### xbox.yaml
Configuration for the joystick controller:
- Axis and button mappings
- Scaling factors for linear and angular velocity
- Deadzone settings
- Turbo mode configuration

## Launch Files

### odrive_control.launch.py
Main launch file that starts all required nodes:
- joy_node for joystick input
- teleop_node for converting joystick input to velocity commands
- odrive_node for interfacing with the ODrive controller
- Loads configuration parameters from YAML files

## Documentation Files

### implementation_guide.md
Detailed guide for implementing the system:
- Hardware setup instructions
- Software installation steps
- ODrive configuration
- ROS 2 configuration
- Running and testing the system
- Troubleshooting tips
- Advanced configuration options

### wiring_diagram.txt
ASCII art diagram showing:
- Physical connections between components
- ROS 2 node architecture and message flow

## Package Files

### package.xml
ROS 2 package manifest defining:
- Package metadata
- Dependencies
- Build type
- Export information

### setup.py
Python package setup script defining:
- Package structure
- Entry points for executables
- Data files for installation
- Dependencies

## Directory Structure

### src/odrive_control/
The main ROS 2 package directory containing all source code and configuration files.

### odrive_control/
The Python module directory containing the actual node implementations.

### launch/
Contains launch files for starting the system.

### config/
Contains configuration files for the nodes.

### resource/
Contains package resource files.

### test/
Directory for test files (to be implemented).

### docs/
Documentation directory with guides and diagrams.
