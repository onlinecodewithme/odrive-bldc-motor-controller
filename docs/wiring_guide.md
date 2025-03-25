# Wiring Guide for ODrive Controlled Tracked Robot

Author: Randika Prasad  
Website: https://cladik.com

This guide provides detailed wiring instructions for connecting the ODrive controller, Arduino, RC receiver, and motors for the tracked robot project.

## Components

- ODrive V3.6 controller
- Arduino Uno/Mega/Nano
- Jumper T12D RC transmitter and R12DS receiver
- 2x MY1020 BLDC motors (48V, 1000W)
- 48V power supply capable of at least 42A (2100W)
- Hall effect sensors (if not built into motors)
- Appropriate wiring and connectors

## ODrive Controller Pinout

The ODrive V3.6 controller has the following key connection points:

### Power Connections
- **DC+**: Positive terminal for DC power input (48V)
- **GND**: Ground terminal for DC power input
- **M0 A+/A-/B+/B-/C+/C-**: Motor 0 phase connections (3 phases)
- **M1 A+/A-/B+/B-/C+/C-**: Motor 1 phase connections (3 phases)

### GPIO and Communication Pins
- **GND**: Ground reference for GPIO and communication
- **3.3V**: 3.3V output for powering external devices (limited current)
- **5V**: 5V output for powering external devices (limited current)
- **GPIO1-8**: General purpose I/O pins (can be configured for UART communication)
- **SPI pins**: MISO, MOSI, SCK, CS

### Encoder Connections
- **M0 ENC A, B, Z**: Encoder inputs for Motor 0
- **M1 ENC A, B, Z**: Encoder inputs for Motor 1
- **GND**: Ground reference for encoders
- **5V**: 5V output for powering encoders

## Wiring Diagram

### 1. Power Connections

Connect the 48V power supply to the ODrive:
- Power supply positive (+) → ODrive **DC+** terminal
- Power supply negative (-) → ODrive **GND** terminal

### 2. Motor Connections

#### Left Motor (M0)
- Left motor phase A → ODrive **M0 A+/A-** terminals
- Left motor phase B → ODrive **M0 B+/B-** terminals
- Left motor phase C → ODrive **M0 C+/C-** terminals

#### Right Motor (M1)
- Right motor phase A → ODrive **M1 A+/A-** terminals
- Right motor phase B → ODrive **M1 B+/B-** terminals
- Right motor phase C → ODrive **M1 C+/C-** terminals

### 3. Hall Sensor Connections

#### Left Motor Hall Sensors (M0)
- Hall sensor A → ODrive **M0 ENC A** pin
- Hall sensor B → ODrive **M0 ENC B** pin
- Hall sensor C → ODrive **M0 ENC Z** pin
- Hall sensor GND → ODrive **GND** pin
- Hall sensor VCC → ODrive **5V** pin

#### Right Motor Hall Sensors (M1)
- Hall sensor A → ODrive **M1 ENC A** pin
- Hall sensor B → ODrive **M1 ENC B** pin
- Hall sensor C → ODrive **M1 ENC Z** pin
- Hall sensor GND → ODrive **GND** pin
- Hall sensor VCC → ODrive **5V** pin

### 4. Arduino to ODrive Connections

For ODrive V3.6 with firmware v0.5.1 (your current version), the UART pins are on the GPIO header:
- Arduino pin 10 (RX) → ODrive **GPIO1** (functions as UART_TX)
- Arduino pin 11 (TX) → ODrive **GPIO2** (functions as UART_RX)
- Arduino GND → ODrive **GND** pin

**Important Note on UART Communication:**
Your ODrive firmware version (v0.5.1) has GPIO1 and GPIO2 pre-configured for UART communication at 115200 baud rate. These pins are part of the GPIO header on your ODrive board.

For detailed information about locating these pins on your ODrive V3.6 controller, please refer to the [ODrive UART Pins Guide](odrive_uart_pins.md).

**Voltage Level Consideration:**
- ODrive GPIO pins operate at 3.3V logic level
- Arduino typically operates at 5V logic level
- You might need a logic level converter if your Arduino uses 5V logic
- Some Arduino boards (like Arduino Due) operate at 3.3V and can be directly connected

### 5. RC Receiver to Arduino Connections

Connect the RC receiver to the Arduino:
- RC receiver throttle channel → Arduino pin 2
- RC receiver steering channel → Arduino pin 3
- RC receiver GND → Arduino GND
- RC receiver VCC → Arduino 5V

## Wiring Notes

1. **Power Wiring**:
   - Use thick gauge wire (10 AWG or thicker) for the power connections to handle the high current
   - Keep power wires as short as possible to minimize voltage drop
   - Consider adding a power switch and fuse for safety

2. **Motor Wiring**:
   - Use thick gauge wire (12 AWG or thicker) for the motor connections
   - Keep motor wires as short as possible to minimize EMI
   - Twist motor phase wires together to reduce EMI
   - If motors run in the wrong direction, swap any two of the three phase wires

3. **Signal Wiring**:
   - Keep signal wires away from power and motor wires to avoid interference
   - Use shielded cable for encoder connections if possible
   - Verify that the Hall sensor connections match the motor phase connections

4. **Grounding**:
   - Ensure all grounds are connected together
   - Use a star grounding topology to avoid ground loops

## Testing the Wiring

After completing the wiring:

1. **Power Test**:
   - Double-check all connections before applying power
   - Power on the system without the motors connected
   - Verify that the ODrive and Arduino power up correctly

2. **Communication Test**:
   - Run the `check_odrive.py` script to verify ODrive communication
   - Upload a simple test sketch to the Arduino to verify RC signal reception

3. **Motor Test**:
   - Connect one motor at a time
   - Run the `configure_odrive.py --test` script to test basic motor control
   - Verify that the motor spins in the correct direction

## Troubleshooting

- **ODrive Not Powering Up**: Check power connections and voltage
- **Motors Not Spinning**: Check motor phase connections and encoder/Hall sensor connections
- **Erratic Motor Behavior**: Check for loose connections and proper grounding
- **RC Signal Issues**: Verify RC receiver connections and channel assignments

## Safety Warnings

- Always disconnect power before making any wiring changes
- Use appropriate wire gauge for high-current connections
- Ensure proper cooling for motors and the ODrive controller
- Keep the robot on blocks during initial testing to prevent unexpected movement
- Implement an emergency stop mechanism
