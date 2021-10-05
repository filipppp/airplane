//
// Created by filip on 07.11.20.
//

#ifndef AIRPLANE_HELPERS_H
#define AIRPLANE_HELPERS_H

void setup_pin_modes();
void set_power_led(int logic_level);
void set_update_led(int logic_level);
void buzz();
void stop_buzz();
void check_calibration(bool force = false);
void update_power_led(uint32_t last_update);

#endif //AIRPLANE_HELPERS_H
