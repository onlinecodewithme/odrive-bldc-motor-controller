# Installation Guide for ODrive Controlled Tracked Robot

Author: Randika Prasad  
Website: https://cladik.com

This guide provides detailed installation instructions for setting up the ODrive controlled tracked robot project.

## Prerequisites

- Arduino IDE (latest version recommended)
- Python 3.8 or higher
- ODrive V3.6 controller
- Arduino Uno/Mega/Nano
- Jumper T12D RC transmitter and R12DS receiver
- 2x MY1020 BLDC motors (48V, 1000W)
- 48V power supply capable of at least 42A (2100W)
- Appropriate wiring and connectors
- Tracked robot chassis

## Step 1: Clone the Repository

```bash
git clone https://github.com/yourusername/odrive_controller.git
cd odrive_controller
```

## Step 2: Install the ODriveArduino Library

### Method 1: Using Arduino Library Manager (Recommended)

1. Open the Arduino IDE
2. Go to **Sketch > Include Library > Manage Libraries...**
3. In the Library Manager, search for "ODriveArduino"
4. Click "Install"

### Method 2: Manual Installation

1. Copy the `Arduino/libraries/ODriveArduino` folder to your Arduino libraries folder:
   - Windows: `Documents\Arduino\libraries\`
   - Mac: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
2. Restart the Arduino IDE

### Method 3: Using the ZIP File

1. In the Arduino IDE, go to **Sketch > Include Library > Add .ZIP Library...**
2. Select the `Arduino/ODriveArduino.zip` file from this repository

## Step 3: Set Up the Python Environment for ODrive

1. Create a Python virtual environment:
   ```bash
   python3 -m venv odrive_venv
   ```

2. Activate the virtual environment:
   ```bash
   source odrive_venv/bin/activate  # On Linux/Mac
   odrive_venv\Scripts\activate      # On Windows
   ```

3. Install the ODrive Python library:
   ```bash
   pip install odrive
   ```

## Step 4: Verify ODrive Connection

1. Connect the ODrive controller to your computer via USB
2. Make sure the ODrive has power
3. Run the check script to verify the connection:
   ```bash
   source odrive_venv/bin/activate  # If not already activated
   python check_odrive.py
   ```

4. If successful, you should see output like:
   ```
   ODrive library imported successfully
   Searching for ODrive controller...
   Found ODrive controller with serial number: 55320073810227
   Hardware version: 3.6
   Firmware version: 0.5.1
   ODrive controller is connected and working properly
   ```

## Step 5: Configure the ODrive Controller

1. Make sure the ODrive is connected and powered
2. Run the configuration script:
   ```bash
   source odrive_venv/bin/activate  # If not already activated
   python configure_odrive.py
   ```

3. For motor calibration and testing (WARNING: Motors will spin!):
   ```bash
   python configure_odrive.py --calibrate --test
   ```

4. The script will configure the ODrive with appropriate parameters for the tracked robot, including:
   - Motor type (high-current)
   - Encoder configuration (Hall sensor mode)
   - Control mode (velocity control)
   - Current limits and other safety parameters

## Step 6: Upload the Arduino Sketch

1. Open the Arduino IDE
2. Open the sketch file: `src/ODriveRCControl/ODriveRCControl.ino`
3. Select your Arduino board type from the Tools > Board menu (Arduino Uno)
4. Select the correct port from the Tools > Port menu
5. Click the "Upload" button (right arrow icon)

## Step 7: Connect the Hardware

Follow the wiring guide in `docs/wiring_guide.md` to connect:
1. The ODrive controller to the motors
2. The Arduino to the ODrive controller
3. The RC receiver to the Arduino
4. The power supply to the ODrive controller

## Step 8: Test the System

1. Power on the system
2. Use your RC transmitter to control the robot
3. The left stick controls forward/backward movement
4. The right stick controls left/right turning
5. The differential steering will allow zero-radius turns

## Troubleshooting

### Arduino Compilation Errors

- Make sure the ODriveArduino library is properly installed
- Check that the Arduino IDE can find the library (Sketch > Include Library menu should show ODriveArduino)
- Restart the Arduino IDE
- If needed, reinstall the library using one of the methods above

### ODrive Connection Issues

- Run `python check_odrive.py` to verify the connection
- Check USB connections and power supply
- Try a different USB port
- Check if the ODrive has any LED indicators lit

### Motor Control Issues

- Verify motor wiring according to the wiring guide
- Check encoder connections
- Run the configuration script with the `--test` flag to test basic motor control
- Make sure the motors are properly calibrated

## ROS 2 Installation (Optional)

If you want to use the ROS 2 implementation on a Jetson Orin NX:

1. Install ROS 2 Humble on the Jetson following the official instructions
2. Copy the `ros2_odrive` directory to your Jetson
3. Follow the instructions in `ros2_odrive/README.md` for setup and usage

## Additional Resources

- [ODrive Documentation](https://docs.odriverobotics.com/)
- [Arduino Documentation](https://www.arduino.cc/reference/en/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
