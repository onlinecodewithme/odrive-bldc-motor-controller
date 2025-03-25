#!/usr/bin/env python3
"""
Script to configure the ODrive controller for the tracked robot.
"""

import sys
import time
import argparse

try:
    import odrive
    from odrive.enums import *
    print("ODrive library imported successfully")
except ImportError:
    print("Error: ODrive library not installed")
    sys.exit(1)

def find_odrive():
    """Find and connect to ODrive."""
    print("Looking for ODrive...")
    print("Checking USB devices...")
    
    # Try to find any connected ODrives
    try:
        print("Searching for ODrive devices...")
        devices = odrive.find_any(timeout=20)  # Increase timeout to 20 seconds
        if devices:
            print(f"Found ODrive: {devices.serial_number}")
            print(f"Hardware version: {devices.hw_version_major}.{devices.hw_version_minor}")
            print(f"Firmware version: {devices.fw_version_major}.{devices.fw_version_minor}.{devices.fw_version_revision}")
            return devices
        else:
            print("No ODrive devices found within timeout period.")
    except Exception as e:
        print(f"Error finding ODrive: {str(e)}")
    
    # If we get here, no ODrive was found
    print("\nTROUBLESHOOTING STEPS:")
    print("1. Make sure the ODrive is connected to your computer via USB")
    print("2. Check that the ODrive has power (DC power LED should be on)")
    print("3. Try unplugging and reconnecting the USB cable")
    print("4. Try a different USB port or cable")
    print("5. Check if the ODrive is recognized by your operating system:")
    print("   - On Linux, run 'lsusb' to see if ODrive is listed")
    print("   - On macOS, check System Information > USB")
    print("   - On Windows, check Device Manager")
    
    # Ask user if they want to continue without ODrive
    response = input("\nNo ODrive found. Do you want to continue with simulation mode? (y/n): ")
    if response.lower() == 'y':
        print("Continuing in simulation mode (no actual ODrive connected)")
        # Create a mock ODrive object for simulation
        class MockODrive:
            def __init__(self):
                self.serial_number = "SIMULATION"
                self.hw_version_major = 0
                self.hw_version_minor = 0
                self.fw_version_major = 0
                self.fw_version_minor = 0
                self.fw_version_revision = 0
                self.config = type('obj', (object,), {
                    'gpio3_mode': 0,
                    'gpio4_mode': 0,
                    'uart_a_baudrate': 0
                })
                self.axis0 = type('obj', (object,), {
                    'motor': type('obj', (object,), {'config': type('obj', (object,), {'pole_pairs': 0, 'resistance_calib_max_voltage': 0, 'requested_current_range': 0, 'current_control_bandwidth': 0, 'motor_type': 0, 'current_lim': 0})}),
                    'encoder': type('obj', (object,), {'config': type('obj', (object,), {'mode': 0, 'cpr': 0, 'calib_scan_distance': 0})}),
                    'controller': type('obj', (object,), {'config': type('obj', (object,), {'vel_limit': 0, 'control_mode': 0, 'vel_ramp_rate': 0}), 'input_vel': 0}),
                    'requested_state': 0
                })
                self.axis1 = self.axis0
            
            def save_configuration(self):
                print("SIMULATION: Configuration saved")
        
        return MockODrive()
    else:
        print("Exiting script. Please connect an ODrive and try again.")
        sys.exit(1)

def configure_motor(axis, motor_name, axis_num, pole_pairs=7, current_lim=20.0):
    """Configure a motor axis."""
    print(f"Configuring {motor_name} (axis{axis_num})...")
    
    # Motor configuration
    print("Setting motor parameters...")
    axis.motor.config.pole_pairs = pole_pairs
    axis.motor.config.resistance_calib_max_voltage = 4.0
    axis.motor.config.requested_current_range = 25.0
    axis.motor.config.current_control_bandwidth = 100.0
    
    # Set motor type to high-current
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    
    # Hall sensor configuration
    print("Setting encoder parameters...")
    axis.encoder.config.mode = ENCODER_MODE_HALL
    axis.encoder.config.cpr = 6 * pole_pairs  # 6 states * pole pairs
    axis.encoder.config.calib_scan_distance = 150.0
    
    # Controller configuration
    print("Setting controller parameters...")
    axis.controller.config.vel_limit = 2.0
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.vel_ramp_rate = 0.5  # Acceleration limit (turns/s²)
    
    # Current limit
    axis.motor.config.current_lim = current_lim
    
    print(f"{motor_name} configuration complete.")

def calibrate_motor(axis, motor_name, axis_num=None):
    """Calibrate a motor axis."""
    axis_num_str = f"axis{axis_num}" if axis_num is not None else "axis"
    print(f"Calibrating {motor_name} ({axis_num_str})...")
    
    print("Starting motor calibration...")
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    # Wait for calibration to complete
    time.sleep(10)
    
    # Check if calibration was successful
    if axis.motor.error != 0:
        print(f"Motor calibration failed with error: {axis.motor.error}")
        return False
    
    print("Motor calibration successful.")
    print("Starting encoder offset calibration...")
    
    axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    
    # Wait for calibration to complete
    time.sleep(10)
    
    # Check if calibration was successful
    if axis.encoder.error != 0:
        print(f"Encoder calibration failed with error: {axis.encoder.error}")
        return False
    
    print("Encoder calibration successful.")
    return True

def test_motor(axis, motor_name, axis_num=None, test_velocity=0.5, test_duration=2.0):
    """Test a motor with basic movements."""
    axis_num_str = f"axis{axis_num}" if axis_num is not None else "axis"
    print(f"Testing {motor_name} ({axis_num_str})...")
    
    print("Setting to closed loop control...")
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    print(f"Testing forward motion ({test_velocity} turns/s)...")
    axis.controller.input_vel = test_velocity
    time.sleep(test_duration)
    
    print("Stopping...")
    axis.controller.input_vel = 0
    time.sleep(1)
    
    print(f"Testing reverse motion (-{test_velocity} turns/s)...")
    axis.controller.input_vel = -test_velocity
    time.sleep(test_duration)
    
    print("Stopping...")
    axis.controller.input_vel = 0
    
    print(f"{motor_name} test complete.")

def check_uart_communication(odrv):
    """Check UART communication for Arduino."""
    print("\nUART Communication Setup:")
    print("-------------------------")
    print("Your ODrive V3.6 with firmware v0.5.1 uses GPIO1 and GPIO2 pins")
    print("that are pre-configured for UART communication.")
    print("\nFor Arduino communication:")
    print("1. Connect Arduino pin 10 (RX) to ODrive GPIO1 (UART_TX)")
    print("2. Connect Arduino pin 11 (TX) to ODrive GPIO2 (UART_RX)")
    print("3. Connect Arduino GND to ODrive GND pin")
    print("\nThe GPIO pins are located on the GPIO header of your ODrive board.")
    print("For a detailed pinout diagram, see the docs/odrive_uart_pins.md file.")
    print("\nImportant Notes:")
    print("- The default baud rate is 115200, which matches the Arduino sketch settings")
    print("- ODrive GPIO pins operate at 3.3V logic level")
    print("- Arduino typically operates at 5V logic level")
    print("- You might need a logic level converter if your Arduino uses 5V logic")

def main():
    parser = argparse.ArgumentParser(description='Configure ODrive for tracked robot')
    parser.add_argument('--calibrate', action='store_true', help='Perform motor calibration')
    parser.add_argument('--test', action='store_true', help='Test motors after configuration')
    parser.add_argument('--pole-pairs', type=int, default=7, help='Number of pole pairs in the motor')
    parser.add_argument('--current-limit', type=float, default=20.0, help='Current limit in amps')
    args = parser.parse_args()

    print("ODrive Configuration for Tracked Robot")
    print("======================================")
    
    # Find ODrive
    odrv = find_odrive()
    
    # Check UART communication setup
    check_uart_communication(odrv)
    
    # Configure motors
    configure_motor(odrv.axis0, "Left Motor", 0, args.pole_pairs, args.current_limit)
    configure_motor(odrv.axis1, "Right Motor", 1, args.pole_pairs, args.current_limit)
    
    # Save configuration
    print("Saving configuration to ODrive...")
    try:
        odrv.save_configuration()
        print("Configuration saved successfully.")
    except Exception as e:
        print(f"Error saving configuration: {str(e)}")
    
    # Calibrate motors if requested
    if args.calibrate:
        print("\nWARNING: Motors will spin during calibration!")
        print("Ensure motors are securely mounted and free to rotate safely.")
        input("Press Enter to continue with calibration or Ctrl+C to cancel...")
        
        left_success = calibrate_motor(odrv.axis0, "Left Motor", 0)
        right_success = calibrate_motor(odrv.axis1, "Right Motor", 1)
        
        # Save configuration after calibration
        if left_success or right_success:
            print("Saving calibration results...")
            try:
                odrv.save_configuration()
                print("Calibration results saved successfully.")
            except Exception as e:
                print(f"Error saving calibration results: {str(e)}")
    
        # Test motors if calibration was successful and testing is requested
        if args.test and (left_success or right_success):
            print("\nWARNING: Motors will spin during testing!")
            print("Ensure motors are securely mounted and free to rotate safely.")
            input("Press Enter to continue with testing or Ctrl+C to cancel...")
            
            if left_success:
                test_motor(odrv.axis0, "Left Motor", 0)
            
            if right_success:
                test_motor(odrv.axis1, "Right Motor", 1)
    
    # Test motors if calibration is not requested but testing is
    elif args.test:
        print("\nWARNING: Motors will spin during testing!")
        print("Ensure motors are securely mounted and free to rotate safely.")
        input("Press Enter to continue with testing or Ctrl+C to cancel...")
        
        test_motor(odrv.axis0, "Left Motor", 0)
        test_motor(odrv.axis1, "Right Motor", 1)
    
    print("\nConfiguration Complete")
    print("======================")
    print("Your ODrive has been configured for the tracked robot application.")
    print("You can now use the Arduino code to control the motors via RC input.")
    
    # Final cleanup
    print("Setting motors to idle state...")
    odrv.axis0.requested_state = AXIS_STATE_IDLE
    odrv.axis1.requested_state = AXIS_STATE_IDLE
    
    print("Done!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
        sys.exit(0)
