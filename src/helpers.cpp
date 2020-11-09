//
// Created by filip on 07.11.20.
//

#include <HID.h>
#include "helpers.h"

const int GREEN_LED_PIN = 4;

void show_finish() {
    digitalWrite(GREEN_LED_PIN, HIGH);
}

void setup_leds() {
    pinMode(GREEN_LED_PIN, OUTPUT);
}
