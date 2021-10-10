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
    float disp = (float) displacement;
    long d = round( disp / 127 * 5);
    pitch += d;
    pitch = pitch > 180 ? 180 : pitch;
    pitch = pitch < 0 ? 0 : pitch;
    right_wing.write(pitch);
    left_wing.write(pitch);
}

void set_roll(long displacement) {
    displacement = map(displacement, 0, 1023, 0, 10) - 5;
    roll = safe_control(roll + displacement);
    right_wing.write(roll);
    left_wing.write(roll);
}
