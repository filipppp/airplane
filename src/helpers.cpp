//
// Created by filip on 07.11.20.
//

#include <HID.h>
#include "helpers.h"

const int BUZZER_PIN = 3;
const int WORKING_PIN = 4;
const int UPDATE_PIN = 8;

void setup_leds() {
    pinMode(WORKING_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(UPDATE_PIN, OUTPUT);
}

void set_finish(int logic_level) {
    digitalWrite(WORKING_PIN, logic_level);
}

void set_update(int logic_level) {
    digitalWrite(UPDATE_PIN, logic_level);
}

void buzz() {
    tone(BUZZER_PIN, 500);
}

void stop_buzz() {
    noTone(BUZZER_PIN);
}