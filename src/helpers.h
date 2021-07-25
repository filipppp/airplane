//
// Created by filip on 07.11.20.
//

#ifndef AIRPLANE_HELPERS_H
#define AIRPLANE_HELPERS_H

enum STATE {
    RECEIVING,
    TRANSMITTING,
    REST
};

void setup_leds();
void set_power_led(int logic_level);
void set_update_led(int logic_level);
void buzz();
void stop_buzz();
void update_power_led(uint32_t last_update);

#endif //AIRPLANE_HELPERS_H
