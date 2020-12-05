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
void set_finish(int logic_level);
void set_update(int logic_level);
void buzz();
void stop_buzz();

#endif //AIRPLANE_HELPERS_H
