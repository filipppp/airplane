#include "servos.h"
#include <Servo.h>

/* Servo configuration */
Servo left_wing;
Servo right_wing;
Servo motor;
const int PIN_MOTOR = 9;
const int PIN_LEFT_WING_SERVO = 10;
const int PIN_RIGHT_WING_SERVO = 11;

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

void set_throttle(float throttle) {
    motor.write(throttle);
}
