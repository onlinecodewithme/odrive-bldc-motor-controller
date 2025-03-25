# Project Structure

This document provides an overview of the project structure and the purpose of each file.

```
oDrive_controller/
│
├── README.md                  # Project overview and getting started guide
│
├── src/                       # Source code directory
│   ├── ODriveRCControl.ino    # Main Arduino sketch for RC to ODrive interface
│   ├── ODriveArduino.h        # ODrive Arduino library header
│   ├── ODriveArduino.cpp      # ODrive Arduino library implementation
│   └── odrive_config.py       # Python script for ODrive configuration
│
├── docs/                      # Documentation directory
│   ├── wiring_guide.md        # Detailed wiring instructions
│   ├── implementation_guide.md # Step-by-step implementation guide
│   ├── wiring_diagram.txt     # ASCII art wiring diagram
│   └── project_structure.md   # This file - project structure overview
│
└── resources/                 # Resource files directory
    ├── preview.webp           # BLDC motor specifications
    ├── preview (1).webp       # RC transmitter and receiver
    ├── preview (2).webp       # Motor controller kit
    ├── preview (3).webp       # ODrive V3.6 controller
    └── preview (4).webp       # BLDC motor close-up
```

## Source Code Files

### ODriveRCControl.ino
The main Arduino sketch that:
- Reads RC signals from the receiver
- Processes these signals to determine forward/backward movement and turning
- Implements differential steering logic
- Sends appropriate commands to the ODrive controller

### ODriveArduino.h and ODriveArduino.cpp
A library for interfacing with the ODrive controller from Arduino:
- Provides functions for setting motor velocity, position, and current
- Handles communication with the ODrive over serial
- Implements helper functions for motor control and state management

### odrive_config.py
A Python script for configuring the ODrive controller:
- Sets motor parameters for the MY1020 BLDC motors
- Configures hall sensor settings
- Performs motor and encoder calibration
- Tests basic motor functionality

## Documentation Files

### wiring_guide.md
Detailed instructions for wiring all components:
- Connection diagrams and tables
- Pin mappings between components
- Power considerations and safety precautions

### implementation_guide.md
Step-by-step guide for implementing the system:
- Hardware assembly instructions
- ODrive configuration steps
- Arduino setup
- RC transmitter configuration
- Testing and calibration procedures
- Troubleshooting tips

### wiring_diagram.txt
ASCII art diagram showing the connections between components:
- Visual representation of the wiring described in the wiring guide
- Shows connections between RC receiver, Arduino, ODrive, and motors

### project_structure.md
This file - provides an overview of the project structure and file purposes.

## Resource Files
Images showing the hardware components used in the project:
- BLDC motor specifications
- RC transmitter and receiver
- Motor controller kit
- ODrive V3.6 controller
- BLDC motor close-up
