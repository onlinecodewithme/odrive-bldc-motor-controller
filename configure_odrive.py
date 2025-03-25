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
    try:
        odrv = odrive.find_any()
        print(f"Found ODrive: {odrv.serial_number}")
        print(f"Hardware version: {odrv.hw_version_major}.{odrv.hw_version_minor}")
        print(f"Firmware version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        return odrv
    except Exception as e:
        print(f"No ODrive found: {str(e)}")
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
