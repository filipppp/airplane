#ifndef AIRPLANE_SERVOS_H
#define AIRPLANE_SERVOS_H

void handle_servo_change();
void setup_servos();
void set_throttle(float throttle);
void set_pitch(long displacement);
void set_roll(long displacement);

#endif //AIRPLANE_SERVOS_H
