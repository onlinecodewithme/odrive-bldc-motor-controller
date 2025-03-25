# Arduino Compilation Instructions

## Issue Fixed: Multiple Definition Errors

We've fixed the multiple definition errors by moving the local ODriveArduino files to a backup folder. The Arduino IDE will now use only the library version we installed in your Arduino libraries folder.

## Steps to Compile and Upload the Sketch

1. Open the Arduino IDE
2. Open the sketch file: `src/ODriveRCControl/ODriveRCControl.ino`
3. Select your Arduino board type from the Tools > Board menu (Arduino Uno)
4. Select the correct port from the Tools > Port menu (`/dev/tty.usbmodem11301`)
5. Click the "Verify" button (checkmark icon) to compile the sketch
6. If compilation is successful, click the "Upload" button (right arrow icon) to upload the sketch to your Arduino

## Troubleshooting

If you still encounter compilation errors:

1. Make sure the ODriveArduino library is properly installed in your Arduino libraries folder
2. Check that the Arduino IDE can find the library (Sketch > Include Library menu should show ODriveArduino)
3. Restart the Arduino IDE
4. If needed, you can reinstall the library by:
   - Going to Sketch > Include Library > Add .ZIP Library...
   - Selecting the `Arduino/ODriveArduino.zip` file we created

## Testing the System

Once the sketch is uploaded to your Arduino:

1. Make sure the ODrive controller is connected and configured using the `configure_odrive.py` script
2. Connect the RC receiver to the Arduino according to the wiring guide
3. Power on the system
4. Test the differential steering functionality using your RC transmitter
