/*
 * ODriveRCControl.ino
 * 
 * Control two BLDC motors using ODrive controller through RC receiver input
 * Implements differential steering for tracked robot
 * 
 * Hardware:
 * - Arduino (Uno/Mega/Nano)
 * - ODrive V3.6 controller
 * - Jumper T12D RC transmitter and receiver
 * - 2x MY1020 BLDC motors (48V, 1000W)
 * 
 * Created: 2025
 */

#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// RC receiver pins (connect to PWM outputs from receiver)
#define RC_THROTTLE_PIN 2  // Channel for forward/backward movement
#define RC_STEERING_PIN 3  // Channel for left/right steering

// Software serial pins for ODrive communication
#define ODRIVE_TX_PIN 10
#define ODRIVE_RX_PIN 11

// ODrive motor indices
#define LEFT_MOTOR  0  // M0 on ODrive
#define RIGHT_MOTOR 1  // M1 on ODrive

// RC signal parameters
#define RC_MIN 1000      // Minimum RC pulse width (μs)
#define RC_MAX 2000      // Maximum RC pulse width (μs)
#define RC_MID 1500      // Neutral RC pulse width (μs)
#define RC_DEADBAND 50   // Deadband around center position

// Motor speed parameters
#define MAX_SPEED 2.0    // Maximum motor speed in turns/second
#define MIN_SPEED 0.0    // Minimum motor speed

// Setup software serial for ODrive communication
SoftwareSerial odrive_serial(ODRIVE_RX_PIN, ODRIVE_TX_PIN);

// ODrive object
ODriveArduino odrive(odrive_serial);

// Variables to store RC values
int throttleValue = RC_MID;
int steeringValue = RC_MID;

// Variables to store motor speeds
float leftMotorSpeed = 0.0;
float rightMotorSpeed = 0.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  odrive_serial.begin(115200);
  
  Serial.println("ODrive RC Control - Tracked Robot");
  
  // Set RC pins as inputs
  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(RC_STEERING_PIN, INPUT);
  
  // Wait for ODrive to initialize
  delay(1000);
  
  // Set motors to velocity control mode
  Serial.println("Setting motors to velocity control mode...");
  odrive_serial.print("w axis0.controller.config.control_mode 2\n");
  odrive_serial.print("w axis1.controller.config.control_mode 2\n");
  
  // Set motors to closed loop control
  Serial.println("Setting motors to closed loop control...");
  odrive_serial.print("w axis0.requested_state 8\n");
  odrive_serial.print("w axis1.requested_state 8\n");
  
  Serial.println("Setup complete");
}

void loop() {
  // Read RC values
  throttleValue = pulseIn(RC_THROTTLE_PIN, HIGH, 25000);
  steeringValue = pulseIn(RC_STEERING_PIN, HIGH, 25000);
  
  // Check if valid RC signals were received
  if (throttleValue == 0 || steeringValue == 0) {
    // No valid signal - stop motors
    leftMotorSpeed = 0.0;
    rightMotorSpeed = 0.0;
    
    Serial.println("No RC signal detected - stopping motors");
  } else {
    // Map throttle and steering values to motor speeds
    calculateMotorSpeeds();
    
    // Print debug info
    Serial.print("Throttle: "); Serial.print(throttleValue);
    Serial.print(", Steering: "); Serial.print(steeringValue);
    Serial.print(" | Left Motor: "); Serial.print(leftMotorSpeed);
    Serial.print(", Right Motor: "); Serial.println(rightMotorSpeed);
  }
  
  // Send commands to ODrive
  odrive.SetVelocity(LEFT_MOTOR, leftMotorSpeed);
  odrive.SetVelocity(RIGHT_MOTOR, rightMotorSpeed);
  
  // Small delay
  delay(10);
}

// Calculate motor speeds based on throttle and steering inputs
void calculateMotorSpeeds() {
  // Normalize throttle and steering values to range -1.0 to 1.0
  float throttle = normalizeRCInput(throttleValue);
  float steering = normalizeRCInput(steeringValue);
  
  // Apply deadband to throttle and steering
  if (abs(throttle) < (float)RC_DEADBAND / (RC_MAX - RC_MIN))
    throttle = 0.0;
  
  if (abs(steering) < (float)RC_DEADBAND / (RC_MAX - RC_MIN))
    steering = 0.0;
  
  // Calculate left and right motor speeds using differential steering
  leftMotorSpeed = throttle + steering;
  rightMotorSpeed = throttle - steering;
  
  // Ensure motor speeds are within bounds (-1.0 to 1.0)
  leftMotorSpeed = constrain(leftMotorSpeed, -1.0, 1.0);
  rightMotorSpeed = constrain(rightMotorSpeed, -1.0, 1.0);
  
  // Scale to actual motor speeds
  leftMotorSpeed *= MAX_SPEED;
  rightMotorSpeed *= MAX_SPEED;
  
  // Invert right motor direction if needed for your robot configuration
  // Uncomment the line below if your right motor needs to be inverted
  // rightMotorSpeed = -rightMotorSpeed;
}

// Normalize RC input from pulse width to range -1.0 to 1.0
float normalizeRCInput(int pulseWidth) {
  // Handle invalid pulse width
  if (pulseWidth < RC_MIN || pulseWidth > RC_MAX) {
    return 0.0;
  }
  
  // Map to range -1.0 to 1.0
  return (float)(pulseWidth - RC_MID) / (float)(RC_MAX - RC_MIN) * 2.0;
}
