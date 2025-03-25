# ODriveArduino Library

Author: Randika Prasad  
Website: https://cladik.com

An Arduino library for interfacing with ODrive motor controllers.

## Features

- Control ODrive motors in position, velocity, or current mode
- Read encoder position and velocity
- Support for motor calibration and state control
- Simple API for easy integration

## Installation

### Method 1: Using Arduino Library Manager (Recommended)

1. Open the Arduino IDE
2. Go to **Sketch > Include Library > Manage Libraries...**
3. In the Library Manager, search for "ODriveArduino"
4. Click "Install"

### Method 2: Manual Installation

1. Download this repository as a ZIP file
2. In the Arduino IDE, go to **Sketch > Include Library > Add .ZIP Library...**
3. Select the downloaded ZIP file

### Method 3: Manual Installation (Direct)

1. Copy the `Arduino/libraries/ODriveArduino` folder to your Arduino libraries folder:
   - Windows: `Documents\Arduino\libraries\`
   - Mac: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
2. Restart the Arduino IDE

## Usage

```cpp
#include <ODriveArduino.h>
#include <SoftwareSerial.h>

// Software serial pins for ODrive communication
#define ODRIVE_TX_PIN 10
#define ODRIVE_RX_PIN 11

// ODrive motor indices
#define LEFT_MOTOR  0  // M0 on ODrive
#define RIGHT_MOTOR 1  // M1 on ODrive

// Setup software serial for ODrive communication
SoftwareSerial odrive_serial(ODRIVE_RX_PIN, ODRIVE_TX_PIN);

// ODrive object
ODriveArduino odrive(odrive_serial);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  odrive_serial.begin(115200);
  
  Serial.println("ODrive Arduino Example");
}

void loop() {
  // Set motor velocities
  odrive.SetVelocity(LEFT_MOTOR, 1.0);  // 1 turn/s
  odrive.SetVelocity(RIGHT_MOTOR, 1.0); // 1 turn/s
  
  delay(2000);
  
  // Stop motors
  odrive.SetVelocity(LEFT_MOTOR, 0.0);
  odrive.SetVelocity(RIGHT_MOTOR, 0.0);
  
  delay(2000);
}
```

## API Reference

### Constructor

```cpp
ODriveArduino(Stream& serial)
```

### Commands

```cpp
// Position control
void SetPosition(int motor_number, float position)
void SetPosition(int motor_number, float position, float velocity_feedforward)
void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward)

// Velocity control
void SetVelocity(int motor_number, float velocity)
void SetVelocity(int motor_number, float velocity, float current_feedforward)

// Current control
void SetCurrent(int motor_number, float current)

// Trapezoidal trajectory
void TrapezoidalMove(int motor_number, float position)
```

### Getters

```cpp
float GetVelocity(int motor_number)
float GetPosition(int motor_number)
```

### State Control

```cpp
bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f)
```

## License

This library is open-source and available for personal and educational use.

## Acknowledgments

- ODrive Robotics for their excellent motor controller and documentation
