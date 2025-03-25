# Wiring Guide for ODrive Controlled Tracked Robot

This document provides detailed instructions for wiring the tracked robot system using an ODrive controller, Arduino, RC receiver, and BLDC motors.

## Components

- Arduino (Uno/Mega/Nano)
- ODrive V3.6 controller
- 2x MY1020 BLDC motors (48V, 1000W)
- Jumper T12D RC transmitter and R12DS receiver
- Power supply (48V for motors)
- Encoders for motors (if not already integrated)
- Miscellaneous wires, connectors, and mounting hardware

## Wiring Diagram Overview

```
                                  +---------------+
                                  |               |
                                  |    Arduino    |
                                  |               |
                                  +-------+-------+
                                          |
          +---------------------------+   |   +---------------------------+
          |                           |   |   |                           |
          |                           |   |   |                           |
+---------v----------+        +-------v---v---+        +-----------------v-+
|                    |        |               |        |                   |
|   RC Receiver      +------->+  ODrive V3.6  +<-------+  Power Supply    |
|   (R12DS)          |        |  Controller   |        |  (48V)           |
|                    |        |               |        |                   |
+--------------------+        +-------+-------+        +-------------------+
                                      |
                      +---------------+----------------+
                      |                                |
                      |                                |
              +-------v-------+                +-------v-------+
              |               |                |               |
              |  Left Motor   |                |  Right Motor  |
              |  (MY1020)     |                |  (MY1020)     |
              |               |                |               |
              +---------------+                +---------------+
```

## Detailed Connections

### 1. Arduino to ODrive Controller

Connect the Arduino to the ODrive using a UART serial connection:

| Arduino Pin | ODrive Pin | Description       |
|-------------|------------|-------------------|
| Pin 10 (TX) | RX         | Serial data to ODrive   |
| Pin 11 (RX) | TX         | Serial data from ODrive |
| GND         | GND        | Common ground     |

### 2. RC Receiver to Arduino

Connect the RC receiver channels to the Arduino:

| RC Receiver Channel | Arduino Pin | Description            |
|---------------------|-------------|------------------------|
| Channel 2 (Throttle)| Pin 2       | Forward/backward control |
| Channel 1 (Steering)| Pin 3       | Left/right steering    |
| 5V                  | 5V          | Power for receiver     |
| GND                 | GND         | Common ground          |

### 3. Power Supply to ODrive

Connect the power supply to the ODrive:

| Power Supply | ODrive      | Description       |
|--------------|-------------|-------------------|
| 48V+         | DC+         | Positive terminal |
| 48V-         | DC-         | Negative terminal |

**IMPORTANT**: Ensure proper wire gauge for the high current requirements of the motors (recommend 12 AWG or thicker).

### 4. Motors to ODrive

Connect each BLDC motor to the ODrive:

#### Left Motor (M0)

| Motor Wire      | ODrive Connection | Description       |
|-----------------|-------------------|-------------------|
| Phase A (Blue)  | M0A               | Motor phase A     |
| Phase B (Green) | M0B               | Motor phase B     |
| Phase C (Yellow)| M0C               | Motor phase C     |

#### Right Motor (M1)

| Motor Wire      | ODrive Connection | Description       |
|-----------------|-------------------|-------------------|
| Phase A (Blue)  | M1A               | Motor phase A     |
| Phase B (Green) | M1B               | Motor phase B     |
| Phase C (Yellow)| M1C               | Motor phase C     |

### 5. Hall Sensors to ODrive

Connect the hall sensors from each motor to the ODrive:

#### Left Motor Hall Sensors

| Hall Sensor Wire | ODrive Connection | Description       |
|------------------|-------------------|-------------------|
| Hall A           | M0 Hall A         | Hall sensor A     |
| Hall B           | M0 Hall B         | Hall sensor B     |
| Hall C           | M0 Hall C         | Hall sensor C     |
| 5V               | 5V                | Power for sensors |
| GND              | GND               | Common ground     |

#### Right Motor Hall Sensors

| Hall Sensor Wire | ODrive Connection | Description       |
|------------------|-------------------|-------------------|
| Hall A           | M1 Hall A         | Hall sensor A     |
| Hall B           | M1 Hall B         | Hall sensor B     |
| Hall C           | M1 Hall C         | Hall sensor C     |
| 5V               | 5V                | Power for sensors |
| GND              | GND               | Common ground     |

## Power Considerations

1. The MY1020 motors are rated for 1000W each at 48V, which means they can draw up to ~21A each at full load.
2. Ensure your power supply can provide at least 42A (2100W) for both motors.
3. Use appropriate wire gauge for high-current connections:
   - 12 AWG or thicker for motor phase wires
   - 10 AWG or thicker for main power connections

## Safety Precautions

1. Always disconnect power before making any wiring changes.
2. Double-check all connections before applying power.
3. Use proper insulation and heat shrink tubing on all connections.
4. Consider adding an emergency stop button in the main power line.
5. Start testing with lower voltage/current limits in the ODrive configuration.
