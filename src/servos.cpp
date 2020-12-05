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

long safe_control(long operation) {
    if (operation > 180) {
        return 180;
    } else if (operation < 0) {
        return 0;
    }
    return operation;
}

void set_throttle(long throttle) {
    motor.write(throttle);
}

void set_pitch(long displacement) {
    displacement = map(displacement, 0, 1023, 0, 10) - 5;
    pitch = safe_control(pitch + displacement);
    Serial.print("Displacement: ");
    Serial.print(displacement);
    Serial.print(" | ");
    right_wing.write(pitch);
    left_wing.write(pitch);
}

void set_roll(long displacement) {
    displacement = map(displacement, 0, 1023, 0, 10) - 5;
    roll = safe_control(roll + displacement);
    right_wing.write(roll);
    left_wing.write(roll);
}
