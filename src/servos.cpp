#include "servos.h"
#include <Servo.h>
#include <HID.h>

/* Servo configuration */
Servo left_wing;
Servo right_wing;
Servo motor;
const int PIN_MOTOR = 9;
const int PIN_LEFT_WING_SERVO = 10;
const int PIN_RIGHT_WING_SERVO = 11;

float pitch, yaw, roll = 0;

/* Responsible for pot change */
void handle_servo_change() {
    /*
    int pot_val = analogRead(PIN_AN_SERVO_POT);
    int val = map(pot_val, 0, 1023, 0, 180);
    left_wing.write(val);
    right_wing.write(val);
     */
}

void setup_servos() {
    motor.attach(PIN_MOTOR, 1000, 2000);
    left_wing.attach(PIN_LEFT_WING_SERVO);
    right_wing.attach(PIN_RIGHT_WING_SERVO);
}

bool control_in_range(float operation) {
    return operation < 180 && operation > 0;
}

void set_throttle(float throttle) {
    motor.write(throttle);
}

void set_pitch(long displacement) {
    displacement = map(displacement, 0, 1023, 0, 10) - 5;
    if(control_in_range(pitch + displacement)) {
        pitch += displacement;
    }
    pitch += displacement;
    right_wing.write(pitch);
    left_wing.write(pitch);
}

void set_roll(long displacement) {
    displacement = map(displacement, 0, 1023, 0, 10) - 5;
    if(control_in_range(roll + displacement)) {
        roll += displacement;
    }
    roll += displacement;
    right_wing.write(roll);
    left_wing.write(roll);
}


