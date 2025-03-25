#!/usr/bin/env python3
"""
Script to check if an ODrive controller is connected.
"""

import sys
import time

try:
    import odrive
    print("ODrive library imported successfully")
except ImportError:
    print("Error: ODrive library not installed")
    sys.exit(1)

print("Searching for ODrive controller...")
try:
    odrv = odrive.find_any(timeout=3)
    print(f"Found ODrive controller with serial number: {odrv.serial_number}")
    print(f"Hardware version: {odrv.hw_version_major}.{odrv.hw_version_minor}")
    print(f"Firmware version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
except Exception as e:
    print(f"No ODrive controller found: {str(e)}")
    sys.exit(1)

print("ODrive controller is connected and working properly")
