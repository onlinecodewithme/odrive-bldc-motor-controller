#!/usr/bin/env python3
"""
Script to check if an ODrive controller is connected.
"""

import sys
import time

try:
    import odrive
    from odrive.enums import *
    print("ODrive library imported successfully")
except ImportError:
    print("Error: ODrive library not installed")
    print("Please install it with: pip install odrive")
    sys.exit(1)

def main():
    print("Searching for ODrive controller...")
    print("Checking USB devices...")
    
    # Try to find any connected ODrives
    try:
        print("Searching for ODrive devices (timeout: 20 seconds)...")
        odrv = odrive.find_any(timeout=20)  # Increase timeout to 20 seconds
        
        if odrv:
            print(f"Found ODrive controller with serial number: {odrv.serial_number}")
            print(f"Hardware version: {odrv.hw_version_major}.{odrv.hw_version_minor}")
            print(f"Firmware version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
            
            # Check for any errors
            axis0_errors = False
            axis1_errors = False
            
            try:
                if odrv.axis0.error != 0:
                    print(f"WARNING: Axis 0 has error code: {odrv.axis0.error}")
                    axis0_errors = True
                if odrv.axis1.error != 0:
                    print(f"WARNING: Axis 1 has error code: {odrv.axis1.error}")
                    axis1_errors = True
                
                if not axis0_errors and not axis1_errors:
                    print("No errors detected on motor axes")
            except:
                print("Could not check axis errors")
            
            print("\nODrive controller is connected and working properly")
            print("You can now run configure_odrive.py to set up the controller for your tracked robot")
            return 0
        else:
            print("No ODrive devices found within timeout period.")
    except Exception as e:
        print(f"Error finding ODrive controller: {str(e)}")
    
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
    print("\nIf you continue to have issues, please visit the ODrive documentation:")
    print("https://docs.odriverobotics.com/")
    
    return 1

if __name__ == "__main__":
    sys.exit(main())
