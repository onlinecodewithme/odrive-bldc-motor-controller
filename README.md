# ODrive Controlled Tracked Robot

Author: Randika Prasad  
Website: https://cladik.com

This project implements a tracked robot control system using an ODrive controller, Arduino, and RC remote control. The system features differential steering for zero-radius turns, allowing precise control of the robot's movement.

## Project Overview

This repository contains all the necessary code and documentation to build a tracked robot controlled by an ODrive motor controller. The system uses:

- Arduino as the interface between the RC receiver and ODrive controller
- ODrive V3.6 controller for precise BLDC motor control
- Jumper T12D RC transmitter and receiver for remote control
- Two MY1020 BLDC motors (48V, 1000W) for propulsion

## Features

- **Differential Steering**: Implements tank-style steering where one track can move forward while the other moves backward for zero-radius turns
- **Proportional Control**: Smooth control of speed and turning using the RC transmitter
- **High Power**: Supports high-power BLDC motors (1000W each) for excellent torque and speed
- **Closed-Loop Control**: Uses the ODrive's advanced motor control capabilities for precise movement

## Repository Contents

### Source Code

- `src/ODriveRCControl/ODriveRCControl.ino`: Main Arduino sketch for interfacing between RC receiver and ODrive
- `Arduino/libraries/ODriveArduino/src/ODriveArduino.h`: Header file for the ODrive Arduino library
- `Arduino/libraries/ODriveArduino/src/ODriveArduino.cpp`: Implementation of the ODrive Arduino library

### Python Scripts

- `check_odrive.py`: Script to check if the ODrive controller is connected
- `configure_odrive.py`: Script to configure the ODrive controller for the tracked robot
- `verify_arduino.sh`: Script to verify Arduino sketch compilation

### Documentation

- `docs/wiring_guide.md`: Detailed instructions for wiring all components
- `docs/implementation_guide.md`: Step-by-step guide for implementing and configuring the system
- `arduino_instructions.md`: Instructions for compiling and uploading the Arduino sketch

## Getting Started

For detailed installation instructions, see the [Installation Guide](INSTALLATION.md).

Quick start:

1. Review the hardware requirements in the [Wiring Guide](docs/wiring_guide.md)
2. Install the ODriveArduino library (see below)
3. Set up the Python environment for ODrive (see below)
4. Configure the ODrive controller using the provided scripts
5. Upload the Arduino code to your Arduino board
6. Connect all components according to the wiring guide
7. Test and calibrate the system

### Installing the ODriveArduino Library

The project includes an Arduino library for interfacing with the ODrive controller. You need to install this library before uploading the Arduino code.

#### Method 1: Using Arduino Library Manager (Recommended)

1. Open the Arduino IDE
2. Go to **Sketch > Include Library > Manage Libraries...**
3. In the Library Manager, search for "ODriveArduino"
4. Click "Install"

#### Method 2: Manual Installation

1. Copy the `Arduino/libraries/ODriveArduino` folder to your Arduino libraries folder:
   - Windows: `Documents\Arduino\libraries\`
   - Mac: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
2. Restart the Arduino IDE

For more details, see the [ODriveArduino Library README](Arduino/libraries/ODriveArduino/README.md).

### Setting Up the ODrive Python Environment

The project includes Python scripts for configuring and testing the ODrive controller. To use these scripts:

1. Create a Python virtual environment:
   ```bash
   python3 -m venv odrive_venv
   ```

2. Activate the virtual environment:
   ```bash
   source odrive_venv/bin/activate
   ```

3. Install the ODrive Python library:
   ```bash
   pip install odrive
   ```

4. Check if the ODrive is connected:
   ```bash
   python check_odrive.py
   ```

5. Configure the ODrive for the tracked robot:
   ```bash
   python configure_odrive.py
   ```

6. For motor calibration and testing:
   ```bash
   python configure_odrive.py --calibrate --test
   ```

The configuration script sets up the ODrive with appropriate parameters for the tracked robot, including motor type, encoder configuration, and control mode.

## Hardware Requirements

- Arduino (Uno/Mega/Nano)
- ODrive V3.6 controller
- 2x MY1020 BLDC motors (48V, 1000W)
- Jumper T12D RC transmitter and R12DS receiver
- 48V power supply capable of at least 42A (2100W)
- Appropriate wiring and connectors
- Tracked robot chassis

## Safety Considerations

This project involves high-power motors and electrical systems. Please observe these safety precautions:

1. Always disconnect power before making any wiring changes
2. Use appropriate wire gauge for high-current connections
3. Ensure proper cooling for motors and the ODrive controller
4. Start testing with lower power settings
5. Implement an emergency stop mechanism
6. Keep the robot on blocks during initial testing to prevent unexpected movement

## Customization

The code and configuration can be customized for different requirements:

- Adjust motor speed limits in the Arduino code
- Modify the differential steering algorithm for different turning behavior
- Add additional RC channels for features like speed modes or special functions
- Implement telemetry feedback using the ODrive's sensor capabilities

## Troubleshooting

See the [Implementation Guide](docs/implementation_guide.md) for detailed troubleshooting steps.

## License

This project is open-source and available for personal and educational use.

## Acknowledgments

- ODrive Robotics for their excellent motor controller and documentation
- Arduino community for libraries and examples
