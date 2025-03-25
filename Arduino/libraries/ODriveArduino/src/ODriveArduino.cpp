#include "ODriveArduino.h"

// Constructor
ODriveArduino::ODriveArduino(Stream& serial)
    : serial_(serial) {}

// Commands
void ODriveArduino::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    serial_.print("p ");
    serial_.print(motor_number);
    serial_.print(" ");
    serial_.print(position);
    serial_.print(" ");
    serial_.print(velocity_feedforward);
    serial_.print(" ");
    serial_.print(current_feedforward);
    serial_.println();
}

void ODriveArduino::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    serial_.print("v ");
    serial_.print(motor_number);
    serial_.print(" ");
    serial_.print(velocity);
    serial_.print(" ");
    serial_.print(current_feedforward);
    serial_.println();
}

void ODriveArduino::SetCurrent(int motor_number, float current) {
    serial_.print("c ");
    serial_.print(motor_number);
    serial_.print(" ");
    serial_.print(current);
    serial_.println();
}

void ODriveArduino::TrapezoidalMove(int motor_number, float position) {
    serial_.print("t ");
    serial_.print(motor_number);
    serial_.print(" ");
    serial_.print(position);
    serial_.println();
}

// Getters
float ODriveArduino::GetVelocity(int motor_number) {
    serial_.print("r axis");
    serial_.print(motor_number);
    serial_.println(".encoder.vel_estimate");
    return readFloat();
}

float ODriveArduino::GetPosition(int motor_number) {
    serial_.print("r axis");
    serial_.print(motor_number);
    serial_.println(".encoder.pos_estimate");
    return readFloat();
}

// General params
float ODriveArduino::readFloat() {
    return readString().toFloat();
}

int32_t ODriveArduino::readInt() {
    return readString().toInt();
}

// State helper
bool ODriveArduino::run_state(int axis, int requested_state, bool wait_for_idle, float timeout) {
    int timeout_ms = (int)(timeout * 1000.0f);
    serial_.print("w axis");
    serial_.print(axis);
    serial_.print(".requested_state ");
    serial_.print(requested_state);
    serial_.println();
    
    if (wait_for_idle) {
        unsigned long start_time = millis();
        int reported_state;
        do {
            delay(100);
            serial_.print("r axis");
            serial_.print(axis);
            serial_.println(".current_state");
            reported_state = readInt();
            
            if (millis() - start_time > timeout_ms) {
                return false;
            }
        } while (reported_state != AXIS_STATE_IDLE);
    }
    
    return true;
}

// Private methods
String ODriveArduino::readString() {
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    
    while (!serial_.available()) {
        if (millis() - timeout_start >= timeout) {
            return str;
        }
    }
    
    while (serial_.available()) {
        char c = serial_.read();
        if (c == '\n')
            break;
        str += c;
    }
    
    return str;
}
