#!/usr/bin/env python3
"""
ODrive Configuration Script for Tracked Robot

This script configures an ODrive V3.6 controller for use with
two MY1020 BLDC motors in a tracked robot application.

Usage:
    python odrive_config.py

Requirements:
    - ODrive Python package (pip install odrive)
    - ODrive connected via USB
"""

import odrive
from odrive.enums import *
import time
import sys

def print_banner(text):
    """Print a banner with the given text."""
    print("\n" + "=" * 50)
    print(f" {text}")
    print("=" * 50)

def wait_for_key():
    """Wait for user to press Enter."""
    input("\nPress Enter to continue...")

def find_odrive():
    """Find and connect to ODrive."""
    print("Looking for ODrive...")
    try:
        odrv = odrive.find_any()
        print(f"Found ODrive: {odrv.serial_number}")
        return odrv
    except:
        print("No ODrive found. Please check connection and try again.")
        sys.exit(1)

def configure_motor(axis, motor_name):
    """Configure a motor axis."""
    print_banner(f"Configuring {motor_name} (axis{axis.axis_num})")
    
    print("Setting motor parameters...")
    axis.motor.config.pole_pairs = 7  # Adjust based on your motor
    axis.motor.config.resistance_calib_max_voltage = 4.0
    axis.motor.config.requested_current_range = 25.0  # Adjust based on your motor
    axis.motor.config.current_control_bandwidth = 100.0
    axis.motor.config.torque_constant = 8.27 / 270
    
    # Set motor type to high-current
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    
    print("Setting encoder parameters...")
    axis.encoder.config.mode = ENCODER_MODE_HALL
    axis.encoder.config.cpr = 6 * 7  # 6 states * pole pairs
    axis.encoder.config.calib_scan_distance = 150.0
    
    print("Setting controller parameters...")
    axis.controller.config.vel_limit = 2.0  # Adjust based on your requirements
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.vel_ramp_rate = 0.5  # Acceleration limit (turns/s²)
    
    # Current limit
    axis.motor.config.current_lim = 20.0  # Adjust based on your motor specs
    
    print(f"{motor_name} configuration complete.")

def calibrate_motor(axis, motor_name):
    """Calibrate a motor axis."""
    print_banner(f"Calibrating {motor_name} (axis{axis.axis_num})")
    
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

def test_motor(axis, motor_name):
    """Test a motor with basic movements."""
    print_banner(f"Testing {motor_name} (axis{axis.axis_num})")
    
    print("Setting to closed loop control...")
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    print("Testing forward motion (0.5 turns/s)...")
    axis.controller.input_vel = 0.5
    time.sleep(2)
    
    print("Stopping...")
    axis.controller.input_vel = 0
    time.sleep(1)
    
    print("Testing reverse motion (-0.5 turns/s)...")
    axis.controller.input_vel = -0.5
    time.sleep(2)
    
    print("Stopping...")
    axis.controller.input_vel = 0
    
    print(f"{motor_name} test complete.")

def main():
    """Main function."""
    print_banner("ODrive Configuration for Tracked Robot")
    
    print("This script will configure your ODrive for use with two MY1020 BLDC motors.")
    print("Make sure your ODrive is connected via USB and powered on.")
    print("WARNING: Motors will spin during calibration and testing!")
    print("Ensure motors are securely mounted and free to rotate safely.")
    
    wait_for_key()
    
    # Find ODrive
    odrv = find_odrive()
    
    # Configure motors
    configure_motor(odrv.axis0, "Left Motor")
    configure_motor(odrv.axis1, "Right Motor")
    
    # Save configuration
    print_banner("Saving Configuration")
    print("Saving configuration to ODrive...")
    try:
        odrv.save_configuration()
        print("Configuration saved successfully.")
    except:
        print("Error saving configuration.")
    
    wait_for_key()
    
    # Calibrate motors
    print_banner("Motor Calibration")
    print("WARNING: Motors will spin during calibration!")
    print("Ensure motors are securely mounted and free to rotate safely.")
    
    wait_for_key()
    
    left_success = calibrate_motor(odrv.axis0, "Left Motor")
    right_success = calibrate_motor(odrv.axis1, "Right Motor")
    
    # Save configuration after calibration
    if left_success or right_success:
        print("Saving calibration results...")
        try:
            odrv.save_configuration()
            print("Calibration results saved successfully.")
        except:
            print("Error saving calibration results.")
    
    # Test motors if calibration was successful
    if left_success or right_success:
        print_banner("Motor Testing")
        print("WARNING: Motors will spin during testing!")
        print("Ensure motors are securely mounted and free to rotate safely.")
        
        wait_for_key()
        
        if left_success:
            test_motor(odrv.axis0, "Left Motor")
        
        if right_success:
            test_motor(odrv.axis1, "Right Motor")
    
    print_banner("Configuration Complete")
    print("Your ODrive has been configured for the tracked robot application.")
    print("You can now use the Arduino code to control the motors via RC input.")
    
    # Final cleanup
    print("Setting motors to idle state...")
    if left_success:
        odrv.axis0.requested_state = AXIS_STATE_IDLE
    if right_success:
        odrv.axis1.requested_state = AXIS_STATE_IDLE
    
    print("Done!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
        sys.exit(0)
