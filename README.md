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

- `src/ODriveRCControl.ino`: Main Arduino sketch for interfacing between RC receiver and ODrive
- `src/ODriveArduino.h`: Header file for the ODrive Arduino library
- `src/ODriveArduino.cpp`: Implementation of the ODrive Arduino library

### Documentation

- `docs/wiring_guide.md`: Detailed instructions for wiring all components
- `docs/implementation_guide.md`: Step-by-step guide for implementing and configuring the system

## Getting Started

1. Review the hardware requirements in the [Wiring Guide](docs/wiring_guide.md)
2. Follow the wiring instructions to connect all components
3. Upload the Arduino code to your Arduino board
4. Configure the ODrive controller as described in the [Implementation Guide](docs/implementation_guide.md)
5. Configure your RC transmitter
6. Test and calibrate the system

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
