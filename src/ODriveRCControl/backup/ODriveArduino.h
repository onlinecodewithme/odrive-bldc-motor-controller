// ODriveArduino.h
//
// Arduino library for communicating with ODrive motor controllers
// using Arduino Serial

#ifndef ODriveArduino_h
#define ODriveArduino_h

#include <Arduino.h>
#include <Stream.h>

class ODriveArduino {
public:
    enum AxisState_t {
        AXIS_STATE_UNDEFINED = 0,           // default
        AXIS_STATE_IDLE = 1,                // disable motor PWM and controller
        AXIS_STATE_STARTUP_SEQUENCE = 2,    // not implemented
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   // run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,   // run motor calibration
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6, // run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,  // run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8  // run closed loop control
    };

    ODriveArduino(Stream& serial);

    // Commands
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);
    void SetCurrent(int motor_number, float current);
    void TrapezoidalMove(int motor_number, float position);
    
    // Getters
    float GetVelocity(int motor_number);
    float GetPosition(int motor_number);
    
    // General params
    float readFloat();
    int32_t readInt();

    // State helper
    bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f);
    
private:
    String readString();
    
    Stream& serial_;
};

#endif //ODriveArduino_h
