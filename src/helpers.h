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
void show_finish();

#endif //AIRPLANE_HELPERS_H
