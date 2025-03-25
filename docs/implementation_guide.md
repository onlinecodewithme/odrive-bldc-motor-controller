# Implementation Guide for ODrive Controlled Tracked Robot

This guide provides step-by-step instructions for implementing the tracked robot system using an ODrive controller, Arduino, RC receiver, and BLDC motors.

## Prerequisites

Before starting, ensure you have:

1. All components listed in the [Wiring Guide](wiring_guide.md)
2. Arduino IDE installed on your computer
3. ODrive Tool installed (`pip install odrive`)
4. Basic soldering equipment and tools
5. Multimeter for testing connections

## Implementation Steps

### Step 1: Hardware Assembly

1. Mount the BLDC motors to your tracked robot chassis
2. Secure the ODrive controller in a location with adequate ventilation
3. Mount the Arduino in a protected location
4. Install the RC receiver in a location with good signal reception
5. Complete all wiring according to the [Wiring Guide](wiring_guide.md)

### Step 2: ODrive Configuration

The ODrive needs to be configured for your specific motors. Connect the ODrive to your computer via USB and use the following steps:

1. Install ODrive tools if you haven't already:
   ```
   pip install odrive
   ```

2. Open a terminal/command prompt and access the ODrive:
   ```
   odrivetool
   ```

3. Configure motor parameters for both motors:
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
   
   # Save configuration
   odrv0.save_configuration()
   ```

4. Calibrate the motors:
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
       
       # Set to closed loop control
       axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
   
   # Save configuration
   odrv0.save_configuration()
   ```

5. Test basic motor control:
   ```python
   # Test motor 0
   odrv0.axis0.controller.input_vel = 0.5  # Half speed forward
   time.sleep(2)
   odrv0.axis0.controller.input_vel = 0  # Stop
   
   # Test motor 1
   odrv0.axis1.controller.input_vel = 0.5  # Half speed forward
   time.sleep(2)
   odrv0.axis1.controller.input_vel = 0  # Stop
   ```

### Step 3: Arduino Setup

1. Install the required libraries in Arduino IDE:
   - Go to Sketch > Include Library > Manage Libraries
   - Search for and install "SoftwareSerial" if not already installed

2. Install the ODriveArduino library:

   #### Method 1: Using Arduino Library Manager (Recommended)
   - Go to Sketch > Include Library > Manage Libraries
   - In the Library Manager, search for "ODriveArduino"
   - Click "Install"

   #### Method 2: Manual Installation
   - Copy the `Arduino/libraries/ODriveArduino` folder to your Arduino libraries folder:
     - Windows: `Documents\Arduino\libraries\`
     - Mac: `~/Documents/Arduino/libraries/`
     - Linux: `~/Arduino/libraries/`
   - Restart the Arduino IDE

3. Upload the ODriveRCControl.ino sketch to your Arduino:
   - Open the src/ODriveRCControl/ODriveRCControl.ino file in Arduino IDE
   - Select your Arduino board type from Tools > Board
   - Select the correct port from Tools > Port
   - Click the Upload button

### Step 4: RC Transmitter Configuration

Configure your Jumper T12D transmitter for optimal control:

1. Turn on your transmitter and navigate to the Model Setup menu
2. Create a new model or select an existing one
3. Configure the channels:
   - Channel 1: Steering (left stick horizontal)
   - Channel 2: Throttle (right stick vertical)
4. Set appropriate end points (typically 100% is fine)
5. Configure expo settings if desired (20-30% recommended for smoother control)
6. Set failsafe values to center position for safety
7. Ensure the transmitter is bound to the receiver

### Step 5: Testing and Calibration

1. **Initial Power-Up Test**:
   - Double-check all connections
   - Power up the system with the motors disconnected first
   - Verify the Arduino and ODrive are receiving power
   - Check that the RC receiver is receiving signals from the transmitter

2. **RC Signal Test**:
   - Open the Arduino Serial Monitor (115200 baud)
   - Move the transmitter sticks and verify the Arduino is receiving the correct signals
   - The serial monitor should display throttle and steering values

3. **Motor Control Test**:
   - Connect the motors
   - Gradually increase throttle and verify both motors respond correctly
   - Test steering in both directions
   - Verify the differential steering works as expected (one motor forward, one backward during turns)

4. **Fine-Tuning**:
   - Adjust the `MAX_SPEED` parameter in the Arduino code if the motors are too fast or too slow
   - Adjust the `RC_DEADBAND` parameter if there's unwanted movement when the sticks are centered
   - If the motors are running in the wrong direction, modify the code to invert the motor direction

### Step 6: Troubleshooting

**Problem**: Motors don't respond to RC input
- Check Arduino serial output for RC values
- Verify ODrive is in closed-loop control mode
- Check all connections between Arduino and ODrive

**Problem**: Motors run in the wrong direction
- Uncomment the line `// rightMotorSpeed = -rightMotorSpeed;` in the Arduino code
- Alternatively, swap any two of the three motor phase wires

**Problem**: Erratic motor behavior
- Check for loose connections
- Verify hall sensor wiring
- Reduce max speed and acceleration in the ODrive configuration

**Problem**: RC signal issues
- Check receiver binding
- Ensure receiver has adequate power
- Move receiver away from sources of interference

## Advanced Configuration

### Speed and Acceleration Limits

For smoother operation, you can adjust the velocity and acceleration limits in the ODrive:

```python
for axis in [odrv0.axis0, odrv0.axis1]:
    axis.controller.config.vel_limit = 2.0  # Adjust as needed
    axis.controller.config.vel_ramp_rate = 0.5  # Acceleration limit (turns/s²)
```

### Current Limits

To protect your motors and ODrive, set appropriate current limits:

```python
for axis in [odrv0.axis0, odrv0.axis1]:
    axis.motor.config.current_lim = 20.0  # Adjust based on your motor specs
```

### Additional Control Features

You can modify the Arduino code to add features like:

1. **Speed Modes**: Use an additional channel on your transmitter to switch between different speed modes
2. **Emergency Stop**: Add a dedicated channel for emergency stop functionality
3. **Telemetry**: Send motor temperature and current data back to a display or telemetry system

## Maintenance

1. Regularly check all connections for looseness or damage
2. Monitor motor temperature during operation
3. Keep the ODrive controller clean and well-ventilated
4. Update firmware and software as new versions become available
5. Periodically recalibrate the system if performance degrades
