#!/bin/bash
# Script to verify Arduino sketch compilation

echo "Verifying Arduino sketch compilation..."
echo "Using local Arduino CLI to verify the sketch..."

# Path to local Arduino CLI
ARDUINO_CLI="./bin/arduino-cli"

# Check if Arduino CLI exists
if [ ! -f "$ARDUINO_CLI" ]; then
    echo "Arduino CLI not found at $ARDUINO_CLI"
    echo "Please make sure it's installed correctly."
    exit 1
fi

# First, update the index
echo "Updating Arduino CLI index..."
$ARDUINO_CLI core update-index

# Install Arduino AVR core if not already installed
echo "Checking if Arduino AVR core is installed..."
if ! $ARDUINO_CLI core list | grep -q "arduino:avr"; then
    echo "Installing Arduino AVR core..."
    $ARDUINO_CLI core install arduino:avr
fi

# Verify the sketch
echo "Compiling sketch..."
$ARDUINO_CLI compile --fqbn arduino:avr:uno src/ODriveRCControl/ODriveRCControl.ino

if [ $? -eq 0 ]; then
    echo "Sketch compiled successfully!"
    echo "You can now upload it to your Arduino using the Arduino IDE."
else
    echo "Compilation failed. Please check the error messages above."
fi
